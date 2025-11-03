## Install Everything

```bash
uv pip install lerobot -e ./lerobot_camera_zmq -e ./lerobot_robot_ur5e -e ./lerobot_teleoperator_gello
```

## Calibrate GELLO Teleop

1. Find Port
```bash
uv run lerobot-find-port
```

2. Calibration
```bash
uv run scripts/calibrate_gello_teleop.py --port /dev/ttyUSB0 --id gello_teleop
```
