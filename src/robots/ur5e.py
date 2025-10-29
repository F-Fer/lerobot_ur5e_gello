from dataclasses import dataclass, field
from typing import Optional, Any
from lerobot.cameras.configs import ColorMode, Cv2Rotation
from lerobot.cameras import CameraConfig, make_cameras_from_configs
from lerobot.robots import RobotConfig, Robot
from lerobot.utils.errors import DeviceNotConnectedError
import rtde_control
import rtde_receive
from cameras.configuration_zmq import ZMQCameraConfig
from third_party.robotiq_gripper import RobotiqGripper


@RobotConfig.register_subclass("ur5e")
@dataclass
class UR5EConfig(RobotConfig):
    ip: str
    cameras: dict[str, CameraConfig] = field(
        default_factory={
            "zed2i_left": ZMQCameraConfig(
                tcp_address=f"tcp://192.168.1.12:5555",
                topic="zed2i_left",
                color_mode=ColorMode.RGB,
                rotation=Cv2Rotation.NO_ROTATION,
                width=672,
                height=376
            ),
            "zed2i_right": ZMQCameraConfig(
                tcp_address=f"tcp://192.168.1.12:5555",
                topic="zed2i_right",
                color_mode=ColorMode.RGB,
                rotation=Cv2Rotation.NO_ROTATION,
                width=672,
                height=376
            ),
            "zedm_left": ZMQCameraConfig(
                tcp_address=f"tcp://192.168.1.12:5555",
                topic="zedm_left",
                color_mode=ColorMode.RGB,
                rotation=Cv2Rotation.NO_ROTATION,
                width=672,
                height=376
            ),
            "zedm_right": ZMQCameraConfig(
                tcp_address=f"tcp://192.168.1.12:5555",
                topic="zedm_right",
                color_mode=ColorMode.RGB,
                rotation=Cv2Rotation.NO_ROTATION,
                width=672,
                height=376
            )
        }
    )

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
        self.gripper_force = 255

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {
            "motor_1": float,
            "motor_2": float,
            "motor_3": float,
            "motor_4": float,
            "motor_5": float,
            "motor_6": float,
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

    def action_features(self) -> dict:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        return self.rtde_ctrl.isConnected() and self.rtde_rec.isConnected() and all(cam.is_connected for cam in self.cameras.values())

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
        joint_positions = self.rtde_rec.get_actual_Q()
        gripper_position = self.gripper.get_current_position() / 255.0 # Normalize to [0, 1]
        obs_dict = {f"motor_{i}": val for i, val in enumerate(joint_positions)}
        obs_dict["gripper"] = gripper_position

        # Capture images from cameras
        for cam_key, cam in self.cameras.items():
            obs_dict[cam_key] = cam.async_read()

        return obs_dict

    def send_action(self, action: dict[str, float]) -> dict[str, float]:
        # Check if action is valid
        if not all(key in self.action_features() for key in action.keys()):
            raise ValueError(f"Invalid action: {action}")

        goal_joint_positions = [action[f"motor_{i}"] for i in range(6)]
        goal_gripper_position = action["gripper"] * 255.0 # Denormalize to [0, 255]

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