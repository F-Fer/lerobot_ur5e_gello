import logging
from collections import deque
from dataclasses import asdict, dataclass, field
from pprint import pformat
import numpy as np
import time
import os
import json

from lerobot.robots import RobotConfig, make_robot_from_config, Robot
from lerobot.configs import parser
from lerobot.utils.import_utils import register_third_party_devices
from lerobot.utils.utils import init_logging
from lerobot.utils.control_utils import is_headless
from lerobot.utils.robot_utils import busy_wait
from openpi_client.websocket_client_policy import WebsocketClientPolicy
# Ensure third-party devices are discoverable by lerobot
from lerobot_camera_zmq import ZMQCameraConfig  # noqa: F401
from lerobot_robot_ur5e import UR5EConfig  # noqa: F401
from lerobot_teleoperator_gello import GelloConfig  # noqa: F401
from pynput import keyboard

logger = logging.getLogger(__name__)

@dataclass
class EvalConfig:
    task: str # e.g. "task1"
    model_type: str # e.g. "lora" or "fpft"
    total_steps: int
    timeout: int = 60 # Seconds before rollout counts as failed
    home_pose: list[float] = field(default_factory=lambda: [0, -1.57, 1.57, -1.57, -1.57, -1.57]) # Position to return to after each rollout
    num_rollouts: int = 20

@dataclass
class InferenceConfig:
    ip: str
    port: int
    prompt: str
    robot: RobotConfig = field(default_factory=lambda: UR5EConfig(ip="192.168.1.10"))
    fps: int = 60
    eval: EvalConfig

@dataclass
class RolloutResult:
    timestamp: str
    success: bool
    duration: float
    steps_completed: int
    total_steps: int
    score: float

    def to_dict(self):
        return asdict(self)

def try_load_rollout_results_from_file(eval_config: EvalConfig) -> list[RolloutResult]:
    results_file = f"rollout_results_{eval_config.task}.json"
    if os.path.exists(results_file):
        with open(results_file, "r") as f:
            try:
                data = json.load(f)
                return [RolloutResult(**d) for d in data]
            except json.JSONDecodeError:
                return []
    return []

def save_rollout_results_to_file(eval_config: EvalConfig, rollout_results: list[RolloutResult]):
    results_file = f"rollout_results_{eval_config.task}_{eval_config.model_type}.json"
    with open(results_file, "w") as f:
        json.dump([r.to_dict() for r in rollout_results], f, indent=2)


def init_keyboard_listener():
    """
    Initializes a non-blocking keyboard listener for real-time user interaction.

    This function sets up a listener for specific keys (1, 2, right arrow, left arrow, escape) to control
    the program flow during execution, such as stopping evaluation or exiting loops. It gracefully
    handles headless environments where keyboard listening is not possible.

    The keys are:
    - Escape: Stop evaluation
    - up: Rollout was successful, go to home pose and wait for right arrow to continue
    - down: Rollout was unsuccessful, go to home pose and wait for right arrow to continue
    - Right arrow: Continue to next rollout
    - Left arrow: Rerecord the current rollout, got to home pose and wait for right arrow to continue

    Returns:
        A tuple containing:
        - The `pynput.keyboard.Listener` instance, or `None` if in a headless environment.
        - A dictionary of event flags (e.g., `exit_early`) that are set by key presses.
    """
    events = {}
    events["stop_evaluation"] = False
    events["next_rollout"] = False
    events["rerecord_rollout"] = False
    events["success_rollout"] = False
    events["failure_rollout"] = False

    if is_headless():
        logging.warning(
            "Headless environment detected. On-screen cameras display and keyboard inputs will not be available."
        )
        listener = None
        return listener, events

    def on_press(key):
        try:
            if key == keyboard.Key.esc:
                logger.info("Escape key pressed. Stopping ...")
                events["stop_evaluation"] = True
            elif key == keyboard.Key.right:
                events["next_rollout"] = True
            elif key == keyboard.Key.left:
                events["rerecord_rollout"] = True
            elif key == keyboard.Key.up:
                events["success_rollout"] = True
            elif key == keyboard.Key.down:
                events["failure_rollout"] = True
        except Exception as e:
            logger.error(f"Error handling key press: {e}")

    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    return listener, events

def move_to_home(robot: Robot, home_pose: list[float], duration: float = 3.0, fps: int = 60):
    logger.info("Moving to home pose...")
    
    # Get current state
    obs = robot.get_observation()
    current_joints = np.array([obs[f"joint_{i}"] for i in range(6)])
    target_joints = np.array(home_pose)
    
    # Calculate steps
    steps = int(duration * fps)
    
    for i in range(steps):
        # Linear interpolation (LERP)
        alpha = (i + 1) / steps
        interpolated_joints = current_joints + alpha * (target_joints - current_joints)
        
        action_dict = {f"joint_{j}": interpolated_joints[j] for j in range(6)}
        action_dict["gripper"] = 0 # Keep gripper open/neutral
        
        robot.send_action(action_dict)
        busy_wait(1 / fps)
        
    time.sleep(0.5) # Settle time


def evaluation_loop(client: WebsocketClientPolicy, robot: Robot, events: dict, inference_config: InferenceConfig):
    eval_config = inference_config.eval
    fps = inference_config.fps
    
    rollout_results = try_load_rollout_results_from_file(eval_config)
    
    # Ensure we continue from where we left off
    while len(rollout_results) < eval_config.num_rollouts:
        rollout_idx = len(rollout_results)
        logger.info(f"=== Starting Rollout {rollout_idx + 1}/{eval_config.num_rollouts} ===")
        
        # Reset events
        for k in events:
            events[k] = False
            
        move_to_home(robot, eval_config.home_pose)
        
        logger.info("Rollout started. Press UP for Success, DOWN for Failure, LEFT to Retry, RIGHT to Skip/Next.")
        
        action_queue = deque([])
        rollout_start_wall = time.time()
        robot_active_time = 0.0
        
        stop_rollout = False
        rollout_success = False
        
        while not stop_rollout:
            loop_start = time.perf_counter()
            
            # 1. Check Global Stop
            if events["stop_evaluation"]:
                logger.info("Stop evaluation requested.")
                return

            # 2. Check Rollout Outcomes
            if events["success_rollout"]:
                rollout_success = True
                stop_rollout = True
                logger.info("User marked SUCCESS.")
                break
            if events["failure_rollout"]:
                rollout_success = False
                stop_rollout = True
                logger.info("User marked FAILURE.")
                break
            if events["rerecord_rollout"]:
                logger.info("User requested RETRY. Discarding current rollout.")
                stop_rollout = True
                rollout_success = None # Signal to retry
                break
            if events["next_rollout"]:
                # mid-rollout -> ignore
                pass
                
            # 3. Check Timeout
            if time.time() - rollout_start_wall > eval_config.timeout:
                logger.info("Timeout reached. Marking as FAILURE.")
                rollout_success = False
                stop_rollout = True
                break
                
            # 4. Inference
            inference_duration = 0.0
            if len(action_queue) == 0:
                obs = robot.get_observation()
                
                # Mapping per remote_pi_inference.py
                obs_dict = {
                    "observation/joint_position": [obs[f"joint_{i}"] for i in range(6)],
                    "observation/gripper_position": obs["gripper"],
                    "prompt": inference_config.prompt,
                    "observation/exterior_image_1_left": obs["zed2i_left"],
                    "observation/wrist_image_left": obs["zedm_left"],
                    "observation/wrist_image_right": obs["zedm_right"],
                }
                
                t_infer_start = time.perf_counter()
                try:
                    result = client.infer(obs_dict)
                    action_chunk = result["actions"]
                except Exception as e:
                    logger.error(f"Inference error: {e}")
                    # Short sleep to avoid busy loop on error
                    time.sleep(0.1)
                    continue
                t_infer_end = time.perf_counter()
                inference_duration = t_infer_end - t_infer_start
                
                if not isinstance(action_chunk, np.ndarray):
                    action_chunk = np.array(action_chunk)
                if action_chunk.ndim == 1:
                    action_chunk = action_chunk.reshape(1, -1)
                action_queue.extend(action_chunk)

            # 5. Execute Action
            if action_queue:
                action = action_queue.popleft()
                action_dict = {
                    "joint_0": action[0],
                    "joint_1": action[1],
                    "joint_2": action[2],
                    "joint_3": action[3],
                    "joint_4": action[4],
                    "joint_5": action[5],
                    "gripper": action[6],
                }
                robot.send_action(action_dict)
                
                # Pacing
                loop_end = time.perf_counter()
                dt_s = loop_end - loop_start
                busy_wait(1 / fps - dt_s)
                
                # Calculate time for this step
                step_end = time.perf_counter()
                step_total = step_end - loop_start
                
                # Accumulate active time: Total step time MINUS the time we waited for inference
                robot_active_time += (step_total - inference_duration)
                
            else:
                # No actions available, just wait
                busy_wait(1/fps)

        # Rollout Loop Ended
        
        if rollout_success is None:
             # Retry requested
             events["rerecord_rollout"] = False
             continue 

        # Wait for user to input the number of completed steps
        total_steps = eval_config.total_steps
        steps_completed = 0
        for step in range(1, total_steps + 1):
            response = input(f"Did it pass step '{step}'? (y/n): ")
            if response.lower() == 'y':
                steps_completed += 1
            else:
                break
        score = steps_completed / total_steps
             
        # Save Result
        res = RolloutResult(
            timestamp=time.time(),
            success=rollout_success,
            duration=robot_active_time,
            steps_completed=steps_completed,
            total_steps=total_steps,
            score=score,
        )
        rollout_results.append(res)
        save_rollout_results_to_file(eval_config, rollout_results)
        
        logger.info(f"Rollout {rollout_idx + 1} Result: Success={rollout_success}, Time={robot_active_time:.3f}s")
        
        # Inter-rollout Wait
        logger.info("Waiting for user input to proceed to next rollout...")
        logger.info("  [RIGHT ARROW] -> Next Rollout")
        logger.info("  [LEFT ARROW]  -> Retry this rollout")
        logger.info("  [ESC]         -> Stop Evaluation")
        
        # Reset events before waiting
        for k in events: events[k] = False
        
        robot.move_to_home()
        while True:
            if events["next_rollout"]:
                break # Go to outer loop
            if events["rerecord_rollout"]:
                # Remove the just-added result and retry
                rollout_results.pop()
                save_rollout_results_to_file(eval_config, rollout_results)
                break # Go to outer loop (len is now same as before)
            if events["stop_evaluation"]:
                return
            time.sleep(0.05)

@parser.wrap()
def run_evaluation(cfg: InferenceConfig):
    init_logging()
    logging.info(pformat(asdict(cfg)))
    
    robot = make_robot_from_config(cfg.robot)
    robot.connect()
    
    client = None
    listener = None
    
    try:
        client = WebsocketClientPolicy(
            host=cfg.ip,
            port=cfg.port,
            api_key=None,
        )
        
        listener, events = init_keyboard_listener()
        
        evaluation_loop(client, robot, events, cfg)
        
    except KeyboardInterrupt:
        logger.info("Inference stopped by user.")
    except Exception as e:
        logger.error(f"Error during inference: {e}")
        raise e
    finally:
        if client:
            logger.info("Closing client connection...")
            client.close()

        logger.info("Disconnecting robot...")
        robot.disconnect()
        if not is_headless() and listener is not None:
            listener.stop()

def main():
    register_third_party_devices()
    run_evaluation()

if __name__ == "__main__":
    main()
