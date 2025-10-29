from dataclasses import dataclass, field
from lerobot.cameras.configs import ColorMode, Cv2Rotation
from lerobot.cameras import CameraConfig
from lerobot.robots import RobotConfig
from lerobot_camera_zmq.lerobot_camera_zmq.config_zmq import ZMQCameraConfig

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