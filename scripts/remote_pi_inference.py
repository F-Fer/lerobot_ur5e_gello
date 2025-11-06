import logging
from collections import deque
from dataclasses import asdict, dataclass, field
from pprint import pformat
from typing import List
import numpy as np
import time

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

logger = logging.getLogger(__name__)

@dataclass
class InferenceConfig:
    ip: str
    port: int
    prompt: str
    robot: RobotConfig = field(default_factory=lambda: UR5EConfig(ip="192.168.1.10"))
    fps: int = 20

def init_keyboard_listener():
    """
    Initializes a non-blocking keyboard listener for real-time user interaction.

    This function sets up a listener for specific keys (right arrow, left arrow, escape) to control
    the program flow during execution, such as stopping recording or exiting loops. It gracefully
    handles headless environments where keyboard listening is not possible.

    Returns:
        A tuple containing:
        - The `pynput.keyboard.Listener` instance, or `None` if in a headless environment.
        - A dictionary of event flags (e.g., `exit_early`) that are set by key presses.
    """
    # Allow to exit early while recording an episode or resetting the environment,
    # by tapping the right arrow key '->'. This might require a sudo permission
    # to allow your terminal to monitor keyboard events.
    events = {}
    events["stop_inference"] = False

    if is_headless():
        logging.warning(
            "Headless environment detected. On-screen cameras display and keyboard inputs will not be available."
        )
        listener = None
        return listener, events

    # Only import pynput if not in a headless environment
    from pynput import keyboard

    def on_press(key):
        try:
            if key == keyboard.Key.esc:
                logger.info("Escape key pressed. Stopping inference...")
                events["stop_inference"] = True
        except Exception as e:
            logger.error(f"Error handling key press: {e}")

    listener = keyboard.Listener(on_press=on_press)
    listener.start()

    return listener, events

def remap_observation_keys(obs: dict) -> dict:
    return {
        "observation/joint_position": obs["joint_position"],
        "observation/gripper_position": obs["gripper_position"],
        "observation/prompt": obs["prompt"]
    }

def inference_loop(client: WebsocketClientPolicy, robot: Robot, events: dict, fps: int, prompt: str):
    action_queue = deque([])
    while True:
        start_time = time.perf_counter()
        if events["stop_inference"]:
            events["stop_inference"] = False
            break
        if len(action_queue) == 0:
            obs = robot.get_observation()

            # Remap observation keys to the required input keys
            obs_dict = {
                "observation/joint_position": [obs[f"joint_{i}"] for i in range(6)],
                "observation/gripper_position": obs["gripper"],
                "prompt": prompt,
                "observation/exterior_image_1_left": obs["zed2i_left"],
                "observation/wrist_image_left": obs["zedm_left"],
                "observation/wrist_image_right": obs["zedm_right"],
            }

            action_chunk = client.infer(obs_dict)["actions"]

            # Normalize actions to shape (T, action_dim)
            if not isinstance(action_chunk, np.ndarray):
                action_chunk = np.array(action_chunk)
            if action_chunk.ndim == 1:
                action_chunk = action_chunk.reshape(1, -1)
            action_queue.extend(action_chunk)

        # Execute one action
        action = action_queue.popleft()
        # Remap action (np.ndarray) to robot action dict
        action_dict = {
            "joint_0": action[0],
            "joint_1": action[1],
            "joint_2": action[2],
            "joint_3": action[3],
            "joint_4": action[4],
            "joint_5": action[5],
            "gripper": action[6],
        }

        # Send action to robot
        robot.send_action(action_dict)

        # Pace the loop
        dt_s = time.perf_counter() - start_time
        busy_wait(1 / fps - dt_s)
        


@parser.wrap()
def run_inference(cfg: InferenceConfig):
    init_logging()
    logging.info(pformat(asdict(cfg)))

    robot = make_robot_from_config(cfg.robot)
    robot.connect()

    # Instantiate the client
    try:
        client = WebsocketClientPolicy(
            host=cfg.ip,
            port=cfg.port,
            api_key=None,
        )
    except Exception as e:
        logger.error(f"Error instantiating the client: {e}")
        raise e

    listener, events = init_keyboard_listener()

    inference_loop(client, robot, events, cfg.fps, cfg.prompt)

    robot.disconnect()
    if not is_headless() and listener is not None:
        listener.stop()

def main():
    register_third_party_devices()
    run_inference()


if __name__ == "__main__":
    main()
