"""Configuration dataclass for the GELLO teleoperator plugin.

Defines GelloConfig with serial port settings, calibration position, joint signs,
and optional smoothing/async parameters.
"""

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

    # Smoothing factor for Exponential Moving Average (EMA).
    # Range [0, 1]. 1 means no smoothing (instant update), 0 means no update (freeze).
    # Lower values smooth out jitter but add latency.
    smoothing: float = 0.85
    # Whether to run device reading in a background thread.
    # This helps when USB communication is slow (e.g. long cables).
    use_async: bool = True
