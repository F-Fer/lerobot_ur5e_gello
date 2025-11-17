from dataclasses import dataclass, field
from lerobot.cameras.configs import ColorMode, Cv2Rotation
from lerobot.cameras import CameraConfig
from lerobot.robots import RobotConfig
from lerobot_camera_zmq import ZMQCameraConfig

@RobotConfig.register_subclass("ur5e")
@dataclass
class UR5EConfig(RobotConfig):
    ip: str
    cameras: dict[str, CameraConfig] = field(
        default_factory=lambda: {
            "zed2i_left": ZMQCameraConfig(
                tcp_address="tcp://192.168.1.12:5555",
                topic="zed2i_left",
                fps=60,
                width=672,
                height=376,
                color_mode=ColorMode.RGB,
                rotation=Cv2Rotation.NO_ROTATION,
            ),
            "zed2i_right": ZMQCameraConfig(
                tcp_address="tcp://192.168.1.12:5555",
                topic="zed2i_right",
                fps=60,
                color_mode=ColorMode.RGB,
                rotation=Cv2Rotation.NO_ROTATION,
                width=672,
                height=376,
            ),
            "zedm_left": ZMQCameraConfig(
                tcp_address="tcp://192.168.1.12:5555",
                topic="zedm_left",
                fps=60,
                color_mode=ColorMode.RGB,
                rotation=Cv2Rotation.NO_ROTATION,
                width=672,
                height=376,
            ),
            "zedm_right": ZMQCameraConfig(
                tcp_address="tcp://192.168.1.12:5555",
                topic="zedm_right",
                fps=60,
                color_mode=ColorMode.RGB,
                rotation=Cv2Rotation.NO_ROTATION,
                width=672,
                height=376,
            ),
        }
    )