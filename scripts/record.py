from __future__ import annotations

import contextlib
from lerobot.configs.types import PipelineFeatureType, PolicyFeature
from lerobot.processor import RobotAction, RobotActionProcessorStep
from lerobot.scripts import lerobot_record as base_record
from lerobot.utils.import_utils import register_third_party_devices

# Ensure third-party devices are discoverable by lerobot
from lerobot_camera_zmq import ZMQCameraConfig  # noqa: F401
from lerobot_robot_ur5e import UR5EConfig  # noqa: F401
from lerobot_teleoperator_gello import GelloConfig  # noqa: F401


class _RemapTeleopToUR5ActionStep(RobotActionProcessorStep):
    def __init__(self, mapping: dict[str, str]) -> None:
        self._mapping = mapping

    def action(self, action: RobotAction) -> RobotAction:
        if missing := [src for src in self._mapping if src not in action]:
            raise KeyError(f"Teleop action missing keys: {missing}")

        remapped = {dst: action[src] for src, dst in self._mapping.items()}
        for key, value in action.items():
            if key not in self._mapping:
                remapped[key] = value

        return remapped

    def transform_features(
        self, features: dict[PipelineFeatureType, dict[str, PolicyFeature]]
    ) -> dict[PipelineFeatureType, dict[str, PolicyFeature]]:
        transformed = {key: dict(values) for key, values in features.items()}
        if PipelineFeatureType.ACTION not in transformed:
            return transformed

        action_specs = transformed[PipelineFeatureType.ACTION]
        for src, dst in self._mapping.items():
            if src in action_specs:
                action_specs[dst] = action_specs.pop(src)
        transformed[PipelineFeatureType.ACTION] = action_specs
        return transformed


JOINT_MAPPING = {
    "base.pos": "joint_0",
    "shoulder.pos": "joint_1",
    "elbow.pos": "joint_2",
    "wrist_1.pos": "joint_3",
    "wrist_2.pos": "joint_4",
    "wrist_3.pos": "joint_5",
    "gripper.pos": "gripper",
}


@contextlib.contextmanager
def _patched_make_default_processors():
    original_factory = base_record.make_default_processors

    def _patched():
        teleop_proc, robot_action_proc, robot_obs_proc = original_factory()
        teleop_proc.steps.append(_RemapTeleopToUR5ActionStep(JOINT_MAPPING))
        return teleop_proc, robot_action_proc, robot_obs_proc

    try:
        base_record.make_default_processors = _patched  # type: ignore[assignment]
        yield
    finally:
        base_record.make_default_processors = original_factory  # type: ignore[assignment]


def main() -> None:
    register_third_party_devices()
    with _patched_make_default_processors():
        base_record.main()


if __name__ == "__main__":
    main()
