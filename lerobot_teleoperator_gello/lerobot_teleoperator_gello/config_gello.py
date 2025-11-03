from dataclasses import dataclass, field

from lerobot.teleoperators.config import TeleoperatorConfig

@TeleoperatorConfig.register_subclass("gello")
@dataclass
class GelloConfig(TeleoperatorConfig):
    # Port to connect to the arm
    port: str = "/dev/ttyUSB0"
    baudrate: int = 57_600
    calibration_position: list[float] = field(default_factory=lambda: [0, -1.57, 1.57, -1.57, -1.57, -1.57])
    joint_signs: list[int] = field(default_factory=lambda: [1, 1, -1, 1, 1, 1])
    gripper_travel_counts: int = 575