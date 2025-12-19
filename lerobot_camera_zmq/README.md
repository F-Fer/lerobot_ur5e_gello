# LeRobot ZMQ Camera

A [LeRobot](https://github.com/huggingface/lerobot) plugin for cameras streaming over ZMQ.

## Overview

This package provides the `ZMQCamera` class, allowing LeRobot to consume image streams from remote or local ZMQ publishers. This is particularly useful for offloading camera processing or using cameras connected to different machines.

## Features

- **ZMQ SUB Protocol**: Connects to any ZMQ publisher using TCP.
- **Decoding**: Supports both JPEG compressed and raw frame encodings.
- **Post-processing**:
  - Image rotation (0, 90, 180, 270 degrees).
  - Color mode conversion (RGB/BGR).
  - Dimension validation.
- **Async Reading**: Background thread for continuous frame capture to minimize latency.

## Installation

```bash
uv pip install -e ./lerobot_camera_zmq
```

## Configuration

Configure the `tcp_address`, `topic`, and optional parameters like `width`, `height`, and `rotation` in the `ZMQCameraConfig`.
