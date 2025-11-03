import argparse
from pathlib import Path

from lerobot_teleoperator_gello import GelloConfig  # ensure plugin is imported
from lerobot.scripts.lerobot_calibrate import CalibrateConfig, calibrate


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Calibrate the GELLO teleoperator")
    parser.add_argument("--port", required=True, help="Serial device path for the Dynamixel bus")
    parser.add_argument("--id", default="gello_teleop", help="Identifier saved with the calibration")
    parser.add_argument(
        "--calibration-dir",
        type=Path,
        default=None,
        help="Optional directory where calibration data should be stored",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    teleop_cfg = GelloConfig(
        port=args.port,
        id=args.id,
        calibration_dir=args.calibration_dir,
    )

    cfg = CalibrateConfig(teleop=teleop_cfg)
    calibrate(cfg)


if __name__ == "__main__":
    main()