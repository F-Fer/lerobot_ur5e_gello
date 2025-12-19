# LeRobot UR5e Robot

A [LeRobot](https://github.com/huggingface/lerobot) plugin for the Universal Robots UR5e.

## Overview

This package implements the `UR5E` robot class, enabling control and observation of a UR5e arm equipped with a Robotiq gripper. It uses the `ur_rtde` library for communication with the robot controller.

## Features

- **RTDE Integration**: Uses Real-Time Data Exchange (RTDE) control and feedback.
- **ServoJ Control**: Uses `servoJ` for joint trajectory following.
- **Robotiq Gripper Support**: Integrated control for Robotiq grippers via the UR controller's tool communication port.
- **Integrated Cameras**: Supports managing and reading from multiple cameras (e.g., ZMQ cameras) as part of the robot's observations.

## Requirements

- Universal Robots UR5e (e-Series).
- Robotiq Gripper (e.g., 2F-85 or 2F-140)
- `ur-rtde` Python library.

## Installation

```bash
uv pip install -e ./lerobot_robot_ur5e
```

## Configuration

Requires the robot's IP address and camera configurations. See `config_ur5e.py` for details.
