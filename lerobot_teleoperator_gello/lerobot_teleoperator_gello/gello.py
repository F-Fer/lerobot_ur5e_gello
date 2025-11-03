import logging
import json
import time
import numpy as np
from dataclasses import dataclass
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.dynamixel import (
    DriveMode,
    DynamixelMotorsBus,
    OperatingMode,
)
from lerobot.teleoperators import Teleoperator
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from .config_gello import GelloConfig
from lerobot.cameras import make_cameras_from_configs
from pathlib import Path
logger = logging.getLogger(__name__)

@dataclass
class GelloCalibration:
    joint_offsets: dict[str, int] # map from motor name to offset in counts
    gripper_open_position: int # motor counts for the open position
    gripper_closed_position: int # motor counts for the closed position

class Gello(Teleoperator):
    """
    This is the GELLO teleoperator for the ur5 robot.
    The hardware is from Phillip Wu: https://wuphilipp.github.io/gello_site/
    """

    config_class = GelloConfig
    name = "gello"
    RAD_PER_COUNT = 2 * np.pi / (4096 - 1)

    def __init__(self, config: GelloConfig):
        super().__init__(config)
        self.config = config
        self.calibration = None
        self.bus = DynamixelMotorsBus(
            port=self.config.port,
            motors={
                "base": Motor(1, "xl330-m288", MotorNormMode.RANGE_M100_100),
                "shoulder": Motor(2, "xl330-m288", MotorNormMode.RANGE_M100_100),
                "elbow": Motor(3, "xl330-m288", MotorNormMode.RANGE_M100_100),
                "wrist_1": Motor(4, "xl330-m288", MotorNormMode.RANGE_M100_100),
                "wrist_2": Motor(5, "xl330-m288", MotorNormMode.RANGE_M100_100),
                "wrist_3": Motor(6, "xl330-m288", MotorNormMode.RANGE_M100_100),
                "gripper": Motor(7, "xl330-m077", MotorNormMode.RANGE_0_100),
            }
        )

    @property
    def action_features(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in self.bus.motors}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected

    def connect(self, calibrate: bool = True) -> None:
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} already connected")

        self.bus.connect()
        self._load_calibration()
        if not self.is_calibrated and calibrate:
            logger.info(
                "Mismatch between calibration values in the motor and the calibration file or no calibration file found"
            )
            self.calibrate()

        self.configure()
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        return self.calibration is not None

    def calibrate(self) -> None:
        self.bus.disable_torque()
        if self.calibration:
            # Calibration exists exists, ask user whether to use it or run new calibration
            user_input = input(
                f"Press ENTER to use existing calibration, or type 'c' and press ENTER to run new calibration: "
            )
            if user_input.strip().lower() != "c":
                logger.info(f"Using existing calibration")
                return
        logger.info(f"\nRunning calibration of {self}")

        input(f"Move {self} to the home position and press ENTER....")
        joint_motors = ["base", "shoulder", "elbow", "wrist_1", "wrist_2", "wrist_3"]
        start_joints = self.bus.sync_read("Present_Position", normalize=False)
        calibration = GelloCalibration(
            joint_offsets={motor: start_joints[motor] for motor in joint_motors},
            gripper_open_position=start_joints["gripper"],
            gripper_closed_position=start_joints["gripper"] + self.config.gripper_travel_counts,
        )
        self.calibration = calibration
        # Save calibration to file
        with open(self.calibration_fpath, "w") as f:
            json.dump(calibration.__dict__, f)
        logger.info(f"Calibration saved to {self.calibration_fpath}")

    def configure(self) -> None:
        self.bus.disable_torque()
        self.bus.configure_motors()
        for motor in self.bus.motors:
            if motor != "gripper":
                # Use 'extended position mode' for all motors except gripper, because in joint mode the servos
                # can't rotate more than 360 degrees (from 0 to 4095) And some mistake can happen while
                # assembling the arm, you could end up with a servo with a position 0 or 4095 at a crucial
                # point
                self.bus.write("Operating_Mode", motor, OperatingMode.EXTENDED_POSITION.value)

        # Use 'position control current based' for gripper to be limited by the limit of the current.
        # For the follower gripper, it means it can grasp an object without forcing too much even tho,
        # its goal position is a complete grasp (both gripper fingers are ordered to join and reach a touch).
        # For the leader gripper, it means we can use it as a physical trigger, since we can force with our finger
        # to make it move, and it will move back to its original target position when we release the force.
        self.bus.write("Operating_Mode", "gripper", OperatingMode.CURRENT_POSITION.value)

    def setup_motors(self) -> None:
        for motor in reversed(self.bus.motors):
            input(f"Connect the controller board to the '{motor}' motor only and press enter.")
            self.bus.setup_motor(motor)
            print(f"'{motor}' motor id set to {self.bus.motors[motor].id}")

    def get_action(self) -> dict[str, float]:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        start = time.perf_counter()
        action = self.bus.sync_read("Present_Position", normalize=False)

        # Normalize joint positions to [-pi, pi] and gripper position to [0, 1]
        joint_motors = ["base", "shoulder", "elbow", "wrist_1", "wrist_2", "wrist_3"]
        result = {}
        for idx, motor in enumerate(joint_motors):
            offset = self.calibration.joint_offsets[motor]
            sign = self.config.joint_signs[idx]
            ref_pos_rad = self.config.calibration_position[idx]
            angle_rad = sign * (action[motor] - offset) * self.RAD_PER_COUNT + ref_pos_rad
            result[f"{motor}.pos"] = angle_rad
        result["gripper.pos"] = (action["gripper"] - self.calibration.gripper_open_position) / (self.calibration.gripper_closed_position - self.calibration.gripper_open_position)

        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} read action: {dt_ms:.1f}ms")
        return result

    def send_feedback(self, feedback: dict[str, float]) -> None:
        # TODO(rcadene, aliberts): Implement force feedback
        raise NotImplementedError

    def disconnect(self) -> None:
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        self.bus.disconnect()
        logger.info(f"{self} disconnected.")

    def _load_calibration(self, fpath: Path | None = None) -> None:
        if fpath is None:
            fpath = self.calibration_fpath
        if fpath.is_file():
            with open(fpath, "r") as f:
                self.calibration = GelloCalibration(**json.load(f))
            logger.info(f"Calibration loaded from {fpath}")
        else:
            logger.info(f"No calibration file found at {fpath}")
            self.calibration = None