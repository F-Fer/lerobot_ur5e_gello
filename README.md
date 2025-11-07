# LeRobot UR5e/Gello 

LeRobot setup for UR5e + GELLO: install the custom plugins, calibrate the teleop, record datasets, and hook up remote OpenPI policies—everything managed with the provided `uv run` commands.

![connection setup](./assets/images/connection-setup.png)

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
    --dataset.repo_id=F-Fer/ur_task1_0 \
    --dataset.num_episodes=50 \
    --dataset.single_task="Pick up the M8 bolt and insert it into the hole until fully inserted." \
    --dataset.push_to_hub=True \
    --display_data=true \
    --teleop.type=gello \
    --teleop.port=/dev/ttyUSB0 \
    --teleop.id=gello_teleop
```

### Push to Hub
```bash
uv run lerobot-edit-dataset \
    --repo_id=F-Fer/ur5e_gello_test_3 \
    --push_to_hub=True
```

### Delete Episode and Push to Hub
```bash
uv run lerobot-edit-dataset \
    --repo_id=F-Fer/ur_task1_0 \
    --operation.type delete_episodes \
    --operation.episode_indices "[0]" \
    --push_to_hub=True
```

## Run Inference with $\pi0$ on Server

1. Setup the server: 

    Download the model on the server:
    ```bash
    cd /workspace
    git pull
    source .venv/bin/activate
    huggingface-cli download F-Fer/pi0_ur5e_0 --include=25000/ --repo-type=model
    ```

    Start the policy server:
    ```bash
    uv run scripts/serve_policy.py policy:checkpoint --policy.config=pi0_ur5e --policy.dir=/workspace/.cache/huggingface/hub/models--F-Fer--pi0_ur5e_0/snapshots/1d0272087fefadca401469112aa54ce5bd9c3455/25000/
    ```

2. Run inference script:

    Locally run:
    ```bash
    uv run scripts/remote_pi_inference.py --ip=<ip> --port=<port> --prompt=<prompts>
    ```

## Todo

- Inference with remote pi model
    - Add rerun visualization to remote_pi_inference
- Recording fails when video_encoding_batch_size > 1
