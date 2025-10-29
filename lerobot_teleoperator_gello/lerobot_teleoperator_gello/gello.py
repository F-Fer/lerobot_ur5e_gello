import logging
import json
import time
import numpy as np
from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.dynamixel import (
    DriveMode,
    DynamixelMotorsBus,
    OperatingMode,
)
from lerobot.teleoperators import Teleoperator
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from .config_gello import GelloConfig

logger = logging.getLogger(__name__)

class Gello(Teleoperator):
    """
    This is the Gello teleoperator for the ur5 robot.
    The hardware is from Phillip Wu: https://wuphilipp.github.io/gello_site/
    """

    config_class = GelloConfig
    name = "gello"
    CALIBRATION_POSITION = np.array([0, -1.57, 1.57, -1.57, -1.57, -1.57]) # Should be the same as UR5e 
    RAD_PER_COUNT = 2 * np.pi / (4096 - 1)

    def __init__(self, config: GelloConfig):
        super().__init__(config)
        self.config = config
        self.zero_counts: dict[str, int] | None = None
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
            },
            calibration=self.calibration,
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
        self._load_zero_counts()
        if not self.is_calibrated and calibrate:
            logger.info(
                "Mismatch between calibration values in the motor and the calibration file or no calibration file found"
            )
            self.calibrate()

        self.configure()
        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        return self.bus.is_calibrated and self.zero_counts is not None

    def calibrate(self) -> None:
        self.bus.disable_torque()
        if self.calibration:
            # Calibration file exists, ask user whether to use it or run new calibration
            user_input = input(
                f"Press ENTER to use provided calibration file associated with the id {self.id}, or type 'c' and press ENTER to run calibration: "
            )
            if user_input.strip().lower() != "c":
                logger.info(f"Writing calibration file associated with the id {self.id} to the motors")
                self.bus.write_calibration(self.calibration)
                return
        logger.info(f"\nRunning calibration of {self}")
        for motor in self.bus.motors:
            self.bus.write("Operating_Mode", motor, OperatingMode.EXTENDED_POSITION.value)

        input(f"Move {self} to the middle of its range of motion and press ENTER....")
        homing_offsets = self.bus.set_half_turn_homings()
        present_counts = self.bus.sync_read("Present_Position", normalize=False)
        joint_motors = ["base", "shoulder", "elbow", "wrist_1", "wrist_2", "wrist_3"]
        self.zero_counts = {motor: present_counts[motor] for motor in joint_motors}

        # Save zero counts as json file
        zero_counts_file = self.calibration_dir / f"{self.id}_zero_counts.json"
        with open(zero_counts_file, "w") as f:
            json.dump(self.zero_counts, f, indent=4)
        logger.info(f"Zero counts saved to {zero_counts_file}")

        # Full turn motors calibration
        with self.bus.torque_disabled():
            range_mins, range_maxes = self.bus.record_ranges_of_motion(
                joint_motors,
                normalize=False,
            )

            input(f"Move the gripper to the open position and press ENTER....")
            gripper_open_pos = self.bus.read("Present_Position", "gripper", normalize=False)
            input(f"Move the gripper to the closed position and press ENTER....")
            gripper_closed_pos = self.bus.read("Present_Position", "gripper", normalize=False)
            range_mins["gripper"] = gripper_closed_pos
            range_maxes["gripper"] = gripper_open_pos

        # Drive modes ()
        drive_modes = {motor: 0 for motor in self.bus.motors}

        self.calibration = {}
        for motor, m in self.bus.motors.items():
            self.calibration[motor] = MotorCalibration(
                id=m.id,
                drive_mode=drive_modes[motor],
                homing_offset=homing_offsets[motor],
                range_min=range_mins[motor],
                range_max=range_maxes[motor],
            )

        self.bus.write_calibration(self.calibration)
        self._save_calibration()
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
            zero = self.zero_counts[motor]
            result[f"{motor}.pos"] = (
                (action[motor] - zero) * self.RAD_PER_COUNT + self.CALIBRATION_POSITION[idx]
            )
        result["gripper.pos"] = (action["gripper"] - self.calibration["gripper"].range_min) / (self.calibration["gripper"].range_max - self.calibration["gripper"].range_min)

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

    def _load_zero_counts(self) -> None:
        zero_counts_file = self.calibration_dir / f"{self.id}_zero_counts.json"
        if zero_counts_file.is_file():
            with open(zero_counts_file, "r") as f:
                self.zero_counts = json.load(f)
            logger.info(f"Zero counts loaded from {zero_counts_file}")
        else:
            logger.info(f"No zero counts file found at {zero_counts_file}")
            self.zero_counts = None