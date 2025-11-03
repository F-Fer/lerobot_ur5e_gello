from dataclasses import dataclass, field
from typing import Optional, Any
from lerobot.cameras.configs import ColorMode, Cv2Rotation
from lerobot.cameras import CameraConfig, make_cameras_from_configs
from lerobot.robots import RobotConfig, Robot
from lerobot.utils.errors import DeviceNotConnectedError
import rtde_control
import rtde_receive
from .robotiq_gripper import RobotiqGripper
from .config_ur5e import UR5EConfig
import numpy as np

class UR5E(Robot):
    config_class = UR5EConfig
    name = "ur5e"

    def __init__(self, config: UR5EConfig):
        super().__init__(config)
        
        self.cameras = make_cameras_from_configs(config.cameras)

        # RTDE fields (hardware side)
        self.robot_ip = config.ip
        self.rtde_ctrl: Optional[rtde_control.RTDEControlInterface] = None
        self.rtde_rec: Optional[rtde_receive.RTDEReceiveInterface] = None

        # servoJ streaming parameters
        # Use a finite period so the controller gracefully holds until next update
        # and does not require a perfect 125 Hz stream from Python/HTTP loop
        self.acc = 0.5
        self.speed = 0.5
        self.servoj_t = 1.0 / 500
        self.servoj_lookahead = 0.2
        self.servoj_gain = 100

        # Gripper command throttling
        self._last_gripper_pos: Optional[int] = None
        self._last_gripper_cmd_time: float = 0.0
        self._gripper_min_delta: int = 3        # minimum change (0-255 scale)
        self._gripper_min_period_s: float = 0.05

        # Gripper (Robotiq on UR controller tool comms)
        self.with_gripper = True
        self.gripper = RobotiqGripper()
        self.gripper_speed = 255
        self.gripper_force = 10

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {
            "joint_0": float,
            "joint_1": float,
            "joint_2": float,
            "joint_3": float,
            "joint_4": float,
            "joint_5": float,
            "gripper": float,
        }

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        return {
            cam: (self.cameras[cam].height, self.cameras[cam].width, 3) for cam in self.cameras
        }

    @property
    def observation_features(self) -> dict:
        return {**self._motors_ft, **self._cameras_ft}

    @property
    def action_features(self) -> dict:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        return (
            self.rtde_ctrl is not None
            and self.rtde_rec is not None
            and self.rtde_ctrl.isConnected()
            and self.rtde_rec.isConnected()
            and all(cam.is_connected for cam in self.cameras.values())
        )

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            return
        try:
            self.rtde_ctrl = rtde_control.RTDEControlInterface(self.robot_ip)
            self.rtde_rec = rtde_receive.RTDEReceiveInterface(self.robot_ip)
            self.gripper.connect(self.robot_ip, 63352)
            self.gripper.activate(auto_calibrate=True)
        except Exception as e:
            print(f"Error connecting to robot: {e}")
            return

        for cam in self.cameras.values():
            cam.connect()

        self.configure()

    def configure(self) -> None:
        pass

    def disconnect(self) -> None:
        if self.rtde_ctrl:
            self.rtde_ctrl.disconnect()
        if self.rtde_rec:
            self.rtde_rec.disconnect()
        
        for cam in self.cameras.values():
            cam.disconnect()

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def get_observation(self) -> dict[str, Any]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        # Read arm position
        joint_positions = self.rtde_rec.getActualQ()
        gripper_position = self.gripper.get_current_position() / 255.0 # Normalize to [0, 1]
        obs_dict = {f"joint_{i}": val for i, val in enumerate(joint_positions)}
        obs_dict["gripper"] = gripper_position

        # Capture images from cameras
        for cam_key, cam in self.cameras.items():
            obs_dict[cam_key] = cam.async_read()

        return obs_dict

    def send_action(self, action: dict[str, float]) -> dict[str, float]:
        # Check if action is valid
        if not all(key in self.action_features for key in action.keys()):
            raise ValueError(f"Invalid action: {action}, features: {self.action_features}")

        goal_joint_positions = [action[f"joint_{i}"] for i in range(6)]
        goal_gripper_position = np.clip(action["gripper"] * 255.0, 0, 255) # Denormalize to [0, 255]

        # Send goal position to the arm
        self.rtde_ctrl.servoJ(
            goal_joint_positions,
            self.acc,
            self.speed,
            self.servoj_t,
            self.servoj_lookahead,
            self.servoj_gain,
        )
        self.gripper.move(int(goal_gripper_position), self.gripper_speed, self.gripper_force)

        return action