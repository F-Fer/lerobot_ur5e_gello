## Install Everything

```bash
uv pip install lerobot -e ./lerobot_camera_zmq -e ./lerobot_robot_ur5e -e ./lerobot_teleoperator_gello
```

```bash
uv run --refresh python -m pip install lerobot -e ./lerobot_camera_zmq -e ./lerobot_robot_ur5e -e ./lerobot_teleoperator_gello
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

## Record Dataset

```bash
uv run scripts/record.py \
    --robot.type=ur5e \
    --robot.ip=192.168.1.10 \
    --dataset.repo_id=F-Fer/ur5e_gello_test_1 \
    --dataset.num_episodes=2 \
    --dataset.single_task="Test..." \
    --dataset.push_to_hub=True \
    --display_data=true \
    --teleop.type=gello \
    --teleop.port=/dev/ttyUSB0 \
    --teleop.id=gello_teleop
```

### Push to Hub
```bash
lerobot-edit-dataset \
    --repo_id F-Fer/ur5e_gello_test_3 \
    --push_to_hub=True
```

## Todo

- Inference with remote pi model
- Integrate pi_streamer for zmq streaming
- Recording fails when video_encoding_batch_size > 1
