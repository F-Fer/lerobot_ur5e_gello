from __future__ import annotations

import logging
import time

from lerobot.processor import (
    RobotObservation,
    make_default_processors,
)
from lerobot.robots import make_robot_from_config
from lerobot.teleoperators import make_teleoperator_from_config
from lerobot.utils.errors import DeviceNotConnectedError
from lerobot.utils.import_utils import register_third_party_devices
from lerobot.utils.utils import init_logging

from lerobot_teleoperator_gello import GelloConfig
from lerobot_robot_ur5e import UR5EConfig


def main() -> None:
    init_logging()
    logging.info("Starting UR5e ↔︎ GELLO teleoperation example")

    register_third_party_devices()

    robot_cfg = UR5EConfig(ip="192.168.1.10")
    teleop_cfg = GelloConfig(port="/dev/ttyUSB0", id="gello_teleop")

    teleop = make_teleoperator_from_config(teleop_cfg)
    robot = make_robot_from_config(robot_cfg)

    teleop_action_processor, robot_action_processor, robot_observation_processor = make_default_processors()

    teleop.connect()
    try:
        robot.connect()
    except Exception as exc:  # noqa: BLE001 - report but keep running in open-loop
        logging.warning("Robot connection failed: %s", exc)

    loop_hz = 20
    loop_period = 1.0 / loop_hz

    try:
        while True:
            loop_start = time.perf_counter()

            obs: RobotObservation = {}
            if robot.is_connected:
                try:
                    obs = robot.get_observation()
                except DeviceNotConnectedError:
                    logging.warning("Robot disconnected while reading observation")
                    obs = {}

            raw_action = teleop.get_action()
            teleop_action = teleop_action_processor((raw_action, obs))
            robot_action = robot_action_processor((teleop_action, obs))

            if robot.is_connected:
                robot.send_action(robot_action)

            elapsed = time.perf_counter() - loop_start
            if elapsed < loop_period:
                time.sleep(loop_period - elapsed)

    except KeyboardInterrupt:
        logging.info("Teleoperation interrupted by user")
    finally:
        teleop.disconnect()
        if robot.is_connected:
            robot.disconnect()


if __name__ == "__main__":
    main()