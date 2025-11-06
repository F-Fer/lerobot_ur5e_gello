"""ZMQ camera streamer for remote Phosphobot cameras.

This script is intended to run on an edge device (e.g., Raspberry Pi 4)
connected to USB cameras. It captures frames via OpenCV and publishes them
to the Phosphobot workstation over ZeroMQ as multipart PUB/SUB messages.

Usage:
  uv run python streamer.py --config config.json

See `config.example.json` for a sample configuration.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import signal
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Optional, Tuple, List

import cv2
import numpy as np
import zmq
import zmq.asyncio
from pydantic import BaseModel, Field, ValidationError
from base64 import b64encode
from typing import Literal


class CameraConfig(BaseModel):
    device: str = Field(..., description="Path or index of the video device")
    topic: str = Field(..., description="ZMQ topic name")
    width: Optional[int] = Field(
        default=None, ge=16, le=7680, description="Capture width in pixels"
    )
    height: Optional[int] = Field(
        default=None, ge=16, le=4320, description="Capture height in pixels"
    )
    fps: Optional[int] = Field(
        default=None, ge=1, le=120, description="Requested frames per second"
    )
    fourcc: Optional[str] = Field(
        default=None,
        min_length=4,
        max_length=4,
        description="Optional fourcc (e.g., MJPG, YUYV). Auto if omitted.",
    )
    stereo: bool = Field(
        default=False,
        description="Whether the camera outputs a side-by-side stereo frame",
    )
    right_topic: Optional[str] = Field(
        default=None,
        description="Optional topic override for the right eye (stereo only)",
    )
    only_send_left_image: bool = Field(
        default=False,
        description="If true and stereo, only publish the left eye frame",
    )


class StreamerConfig(BaseModel):
    endpoint: str = Field(
        "tcp://0.0.0.0:5555", description="ZMQ PUB bind endpoint"
    )
    cameras: list[CameraConfig]
    jpeg_quality: int = Field(
        default=80, ge=10, le=100, description="JPEG quality for encoding"
    )
    reconnect_interval: float = Field(
        default=3.0, ge=0.5, le=30.0, description="Seconds between reconnects"
    )
    encoding: Literal["raw", "jpeg"] = Field(
        default="jpeg",
        description="Frame encoding: raw RGB or JPEG-compressed",
    )


@dataclass
class VideoCapture:
    config: CameraConfig
    capture: cv2.VideoCapture
    left_topic: str
    right_topic: Optional[str]


class CameraStreamer:
    def __init__(self, config: StreamerConfig, loop: asyncio.AbstractEventLoop):
        self.config = config
        self.loop = loop
        self.context = zmq.asyncio.Context.instance()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.setsockopt(zmq.SNDHWM, 3)
        self.socket.bind(self.config.endpoint)
        self.captures: list[VideoCapture] = []
        self._shutdown = asyncio.Event()

    async def initialize(self) -> None:
        for camera_cfg in self.config.cameras:
            capture = self._open_capture(camera_cfg)
            if capture is None:
                continue
            left_topic = camera_cfg.topic
            right_topic = None
            if camera_cfg.stereo:
                right_topic = (
                    camera_cfg.right_topic
                    if camera_cfg.right_topic
                    else f"{camera_cfg.topic}_right"
                )
            self.captures.append(
                VideoCapture(camera_cfg, capture, left_topic, right_topic)
            )

        if not self.captures:
            raise RuntimeError("No cameras could be opened; check configuration")

    def _open_capture(self, camera_cfg: CameraConfig) -> Optional[cv2.VideoCapture]:
        device = camera_cfg.device
        # Allow numeric strings to be treated as indices
        if device.isdigit():
            index = int(device)
            capture = cv2.VideoCapture(index, cv2.CAP_V4L2)
        else:
            capture = cv2.VideoCapture(device, cv2.CAP_V4L2)

        if not capture.isOpened():
            print(f"[WARN] Failed to open camera {device}")
            return None

        if camera_cfg.width:
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, camera_cfg.width)
        if camera_cfg.height:
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, camera_cfg.height)
        if camera_cfg.fps:
            capture.set(cv2.CAP_PROP_FPS, camera_cfg.fps)
        if camera_cfg.fourcc:
            code = cv2.VideoWriter.fourcc(*camera_cfg.fourcc)
            capture.set(cv2.CAP_PROP_FOURCC, code)

        # Run one read to confirm we get frames
        ok, frame = capture.read()
        if not ok or frame is None:
            print(
                f"[WARN] Camera {device} produced no frames after initialization"
            )
            capture.release()
            return None

        return capture

    async def start(self) -> None:
        print(
            f"[INFO] ZMQ streamer bound to {self.config.endpoint} for"
            f" {len(self.captures)} camera(s)"
        )
        tasks = [self._spawn_camera_task(vc) for vc in self.captures]
        await asyncio.gather(*tasks)

    def request_shutdown(self) -> None:
        if not self._shutdown.is_set():
            self._shutdown.set()

    async def _spawn_camera_task(self, video_capture: VideoCapture) -> None:
        cfg = video_capture.config
        capture = video_capture.capture
        left_topic_bytes = video_capture.left_topic.encode()
        right_topic_bytes = (
            video_capture.right_topic.encode() if video_capture.right_topic else None
        )
        target_fps = cfg.fps or capture.get(cv2.CAP_PROP_FPS) or 30.0
        if target_fps <= 0:
            target_fps = 30.0
        interval = 1.0 / target_fps
        next_tick = time.perf_counter()
        last_log = time.time()
        frames_sent = 0

        try:
            while not self._shutdown.is_set():
                frames_payloads = await asyncio.to_thread(
                    self._capture_and_encode_frames,
                    video_capture,
                    left_topic_bytes,
                    right_topic_bytes,
                )

                if frames_payloads is None:
                    print(
                        f"[WARN] Camera {cfg.device} read failed; retrying in"
                        f" {self.config.reconnect_interval}s"
                    )
                    await asyncio.sleep(self.config.reconnect_interval)
                    next_tick = time.perf_counter()
                    continue

                for topic_bytes, encoded_bytes, shape, dtype in frames_payloads:
                    message = {
                        "shape": shape,
                        "dtype": dtype,
                        "timestamp": time.time(),
                        "frame_bytes": b64encode(encoded_bytes).decode("ascii"),
                        "encoding": self.config.encoding,
                    }

                    try:
                        await self.socket.send_multipart(
                            [topic_bytes, json.dumps(message).encode("utf-8")],
                            flags=zmq.NOBLOCK,
                        )
                    except zmq.Again:
                        print(
                            f"[WARN] Camera {cfg.topic}: outbound queue full, dropping frame"
                        )
                        await asyncio.sleep(0.01)
                        continue

                frames_sent += 1
                now = time.time()
                if now - last_log > 5:
                    print(
                        f"[INFO] Camera {cfg.topic}: {frames_sent / (now - last_log):.1f} fps"
                    )
                    frames_sent = 0
                    last_log = now

                if self._shutdown.is_set():
                    break

                next_tick += interval
                sleep_for = next_tick - time.perf_counter()
                if sleep_for <= 0:
                    next_tick = time.perf_counter()
                    await asyncio.sleep(0)
                else:
                    await asyncio.sleep(sleep_for)
        finally:
            capture.release()
            print(f"[INFO] Camera {cfg.topic} capture closed")

    def _encode_frame(self, frame: np.ndarray) -> bytes:
        if self.config.encoding == "jpeg":
            params = [int(cv2.IMWRITE_JPEG_QUALITY), int(self.config.jpeg_quality)]
            ok, buffer = cv2.imencode(".jpg", frame, params)
            if not ok:
                raise RuntimeError("Failed to encode frame to JPEG")
            return buffer.tobytes()
        # raw
        return frame.tobytes()

    async def shutdown(self) -> None:
        self._shutdown.set()
        await asyncio.sleep(0.1)
        self.socket.close(linger=0)
        self.context.term()

    def _capture_and_encode_frames(
        self,
        video_capture: VideoCapture,
        left_topic_bytes: bytes,
        right_topic_bytes: Optional[bytes],
    ) -> Optional[List[Tuple[bytes, bytes, Tuple[int, int, int], str]]]:
        capture = video_capture.capture
        cfg = video_capture.config

        ok, frame = capture.read()
        if not ok or frame is None:
            return None

        frames_to_process: List[Tuple[bytes, np.ndarray]]
        if cfg.stereo:
            h, w, _ = frame.shape
            mid = w // 2
            left_frame = frame[:, :mid]
            frames_to_process = [(left_topic_bytes, left_frame)]
            if right_topic_bytes and not cfg.only_send_left_image:
                right_frame = frame[:, mid:]
                frames_to_process.append((right_topic_bytes, right_frame))
        else:
            frames_to_process = [(left_topic_bytes, frame)]

        frames_payloads: List[Tuple[bytes, bytes, Tuple[int, int, int], str]] = []
        for topic_bytes, payload_frame in frames_to_process:
            encoded_bytes = self._encode_frame(payload_frame)
            frames_payloads.append(
                (
                    topic_bytes,
                    encoded_bytes,
                    tuple(int(x) for x in payload_frame.shape),
                    str(payload_frame.dtype),
                )
            )

        return frames_payloads


def load_config(path: Path) -> StreamerConfig:
    raw = json.loads(path.read_text())
    try:
        return StreamerConfig.model_validate(raw)
    except ValidationError as exc:
        print("Invalid configuration:\n", exc)
        raise SystemExit(1)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Phosphobot ZMQ camera streamer")
    parser.add_argument(
        "--config",
        type=Path,
        required=True,
        help="Path to JSON configuration file",
    )
    return parser.parse_args()


async def main_async(args: argparse.Namespace) -> None:
    config = load_config(args.config)
    loop = asyncio.get_running_loop()
    streamer = CameraStreamer(config, loop)
    await streamer.initialize()

    stop_event = asyncio.Event()

    def _signal_handler(*_: Any) -> None:
        print("[INFO] Shutdown signal received")
        streamer.request_shutdown()
        stop_event.set()

    loop.add_signal_handler(signal.SIGINT, _signal_handler)
    loop.add_signal_handler(signal.SIGTERM, _signal_handler)

    await asyncio.gather(streamer.start(), stop_event.wait())
    await streamer.shutdown()


def main() -> None:
    args = parse_args()
    asyncio.run(main_async(args))


if __name__ == "__main__":
    main()


