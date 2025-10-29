from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig

@TeleoperatorConfig.register_subclass("gello")
@dataclass
class GelloConfig(TeleoperatorConfig):
    # Port to connect to the arm
    port: str