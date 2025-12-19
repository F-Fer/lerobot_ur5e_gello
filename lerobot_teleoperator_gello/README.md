# LeRobot GELLO Teleoperator

A [LeRobot](https://github.com/huggingface/lerobot) plugin for the GELLO teleoperation device.

GELLO is a general, low-cost, and intuitive teleoperation framework for robot manipulators, originally developed by [Philipp Wu et al.](https://wuphilipp.github.io/gello_site/).

## Overview

This package implements the `Gello` teleoperator class, which allows controlling robots using the GELLO hardware. It uses Dynamixel motors for joint tracking and gripper input.

## Features

- **Joint Tracking**: Maps GELLO joints to robot joint commands.
- **Gripper Control**: Maps the GELLO trigger/gripper to normalized robot gripper actions.
- **Calibration**: Includes a built-in calibration routine to define home positions and gripper limits.
- **Async Support**: Optional asynchronous reading of motor states for lower latency.

## Hardware Requirements

- GELLO hardware (6-DOF arm + gripper).
- Dynamixel XL330-M288 motors for joints.
- Dynamixel XL330-M077 motor for the gripper.
- U2D2 or similar Dynamixel-to-USB adapter.

## Installation

```bash
uv pip install -e ./lerobot_teleoperator_gello
```

## Configuration

See `config_gello.py` for available configuration options, including port, baudrate, and joint signs.
