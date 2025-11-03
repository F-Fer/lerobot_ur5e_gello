import zmq
from typing import Optional, Any
import base64
import json
import cv2
import numpy as np
from threading import Event, Lock, Thread
from numpy.typing import NDArray
import time
import logging
from lerobot.cameras.camera import Camera
from lerobot.utils.errors import DeviceAlreadyConnectedError, DeviceNotConnectedError
from .config_zmq_camera import ZMQCameraConfig
from lerobot.cameras.configs import ColorMode, Cv2Rotation

logger = logging.getLogger(__name__)


def get_cv2_rotation(rotation: Cv2Rotation) -> Optional[int]:
    """Convert Cv2Rotation enum to cv2 rotation constant."""
    if rotation == Cv2Rotation.NO_ROTATION:
        return None
    elif rotation == Cv2Rotation.ROTATE_90:
        return cv2.ROTATE_90_CLOCKWISE
    elif rotation == Cv2Rotation.ROTATE_180:
        return cv2.ROTATE_180
    elif rotation == Cv2Rotation.ROTATE_270:
        return cv2.ROTATE_90_COUNTERCLOCKWISE
    else:
        raise ValueError(f"Invalid rotation: {rotation}")

class ZMQCamera(Camera):
    def __init__(self, config: ZMQCameraConfig):
        super().__init__(config)
        self.config = config
        self.tcp_address = config.tcp_address
        self.topic = config.topic

        self.fps = config.fps
        self.color_mode = config.color_mode
        self.warmup_s = config.warmup_s

        self.stream_initialized: bool = False
        self.context: Optional[zmq.Context] = None
        self.socket: Optional[zmq.Socket] = None
        self.poller: Optional[zmq.Poller] = None

        self.thread: Thread | None = None
        self.stop_event: Event | None = None
        self.frame_lock: Lock = Lock()
        self.latest_frame: NDArray[Any] | None = None
        self.new_frame_event: Event = Event()
        self.rotation: int | None = get_cv2_rotation(config.rotation)
        if config.width is None or config.height is None:
            self.width = None
            self.height = None
        else:
            self.width = config.width
            self.height = config.height

    def __str__(self) -> str:
        return f"{self.__class__.__name__}({self.tcp_address}, {self.topic})"

    @property
    def is_connected(self):
        return self.stream_initialized and self.context is not None and self.socket is not None and self.poller is not None

    def find_cameras(self):
        pass

    def connect(self, warmup: bool = True) -> None:
        """
        Connects to the ZMQ publisher and subscribes to the topic.

        Initializes the ZMQ context, socket, and poller. Optionally performs
        warmup reading to ensure the stream is working.

        Args:
            warmup: Whether to perform warmup reading before returning.

        Raises:
            DeviceAlreadyConnectedError: If the camera is already connected.
        """
        if self.is_connected:
            raise DeviceAlreadyConnectedError(f"{self} is already connected.")

        # Initialize the ZMQ context and socket
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.setsockopt(zmq.RCVTIMEO, 2000)
        self.socket.connect(self.tcp_address)
        self.socket.setsockopt_string(zmq.SUBSCRIBE, self.topic)
        self.stream_initialized = True
        self.poller = zmq.Poller()
        self.poller.register(self.socket, zmq.POLLIN)

        if warmup:
            start_time = time.time()
            while time.time() - start_time < self.warmup_s:
                try:
                    self.read()
                    time.sleep(0.1)
                except Exception:
                    # Continue warmup even if individual reads fail
                    pass

    def disconnect(self) -> None:
        """
        Disconnects from the ZMQ publisher and cleans up resources.

        Stops the background read thread (if running) and closes the ZMQ socket and context.

        Raises:
            DeviceNotConnectedError: If the camera is already disconnected.
        """
        if not self.is_connected and self.thread is None:
            raise DeviceNotConnectedError(f"{self} not connected.")

        if self.thread is not None:
            self._stop_read_thread()

        if self.socket is not None:
            self.socket.close()
        if self.context is not None:
            self.context.destroy()

        self.stream_initialized = False
        self.context = None
        self.socket = None
        self.poller = None

    def _process_frame_data(self, data: dict) -> np.ndarray:
        """Helper function to decode a received data dictionary into a raw frame as a NumPy array."""
        encoding = data.get("encoding", "raw")
        frame_bytes = base64.b64decode(data["frame_bytes"])

        if encoding == "jpeg":
            # JPEG decoding gives BGR by default
            frame = cv2.imdecode(
                np.frombuffer(frame_bytes, dtype=np.uint8), cv2.IMREAD_COLOR
            )
            if frame is None:
                raise RuntimeError("Failed to decode JPEG frame")
            shape = frame.shape
        else:
            # Raw encoding - assume RGB format from data
            shape = data["shape"]
            dtype = np.dtype(data["dtype"])
            rgb_flat = np.frombuffer(frame_bytes, dtype=dtype)
            rgb_frame = rgb_flat.reshape(shape)
            # Convert RGB to BGR for consistent processing
            frame = cv2.cvtColor(rgb_frame, cv2.COLOR_RGB2BGR)
        
        # Set the width and height
        if self.width is None or self.height is None:
            self.width = shape[1]
            self.height = shape[0]

        return frame

    def _postprocess_image(self, image: np.ndarray, color_mode: ColorMode | None = None) -> np.ndarray:
        """
        Applies color conversion, dimension validation, and rotation to a raw frame.

        Args:
            image (np.ndarray): The raw image frame (expected BGR format from decoding).
            color_mode (Optional[ColorMode]): The target color mode (RGB or BGR). If None,
                                             uses the instance's default `self.color_mode`.

        Returns:
            np.ndarray: The processed image frame.

        Raises:
            ValueError: If the requested `color_mode` is invalid.
            RuntimeError: If the raw frame dimensions do not match the configured
                          `width` and `height`.
        """
        requested_color_mode = self.color_mode if color_mode is None else color_mode

        if requested_color_mode not in (ColorMode.RGB, ColorMode.BGR):
            raise ValueError(
                f"Invalid color mode '{requested_color_mode}'. Expected {ColorMode.RGB} or {ColorMode.BGR}."
            )

        h, w, c = image.shape

        if h != self.height or w != self.width:
            raise RuntimeError(
                f"{self} frame width={w} or height={h} do not match configured width={self.width} or height={self.height}."
            )

        if c != 3:
            raise RuntimeError(f"{self} frame channels={c} do not match expected 3 channels (RGB/BGR).")

        processed_image = image
        if requested_color_mode == ColorMode.RGB:
            processed_image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

        if self.rotation in [cv2.ROTATE_90_CLOCKWISE, cv2.ROTATE_90_COUNTERCLOCKWISE, cv2.ROTATE_180]:
            processed_image = cv2.rotate(processed_image, self.rotation)

        return processed_image

    def read(self, color_mode: ColorMode | None = None) -> np.ndarray:
        """
        Reads a single frame synchronously from the ZMQ stream.

        This is a blocking call. It waits for the next available frame from the
        ZMQ publisher and processes it according to configuration.

        Args:
            color_mode (Optional[ColorMode]): If specified, overrides the default
                color mode (`self.color_mode`) for this read operation (e.g.,
                request RGB even if default is BGR).

        Returns:
            np.ndarray: The captured frame as a NumPy array in the format
                       (height, width, channels), using the specified or default
                       color mode and applying any configured rotation.

        Raises:
            DeviceNotConnectedError: If the camera is not connected.
            RuntimeError: If reading the frame from the stream fails or if the
                          received frame dimensions don't match expectations.
            ValueError: If an invalid `color_mode` is requested.
            TimeoutError: If no message is received within the timeout period.
        """
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        # Wait for a message to be available
        if self.poller.poll(timeout=1000):
            message = self.socket.recv_multipart()
            # ZMQ PUB/SUB sends [topic, data]
            if len(message) != 2:
                raise RuntimeError(f"Invalid ZMQ message format: expected 2 parts, got {len(message)}")

            topic, data_bytes = message
            # Parse JSON data
            try:
                data = json.loads(data_bytes.decode('utf-8'))
            except json.JSONDecodeError as e:
                raise RuntimeError(f"Failed to parse ZMQ message as JSON: {e}")

            # Decode the frame
            raw_frame = self._process_frame_data(data)

            # Apply postprocessing (color conversion, validation, rotation)
            processed_frame = self._postprocess_image(raw_frame, color_mode)

            return processed_frame
        else:
            raise TimeoutError(f"{self} timed out waiting for a message.")

    def _read_loop(self) -> None:
        """
        Internal loop run by the background thread for asynchronous reading.

        On each iteration:
        1. Reads a color frame from ZMQ
        2. Stores result in latest_frame (thread-safe)
        3. Sets new_frame_event to notify listeners

        Stops on DeviceNotConnectedError, logs other errors and continues.
        """

        if self.stop_event is None:
            raise RuntimeError(f"{self}: stop_event is not initialized before starting read loop.")

        while not self.stop_event.is_set():
            try:
                color_image = self.read()

                with self.frame_lock:
                    self.latest_frame = color_image
                self.new_frame_event.set()

            except DeviceNotConnectedError:
                break
            except Exception as e:
                logger.warning(f"Error reading frame in background thread for {self}: {e}")

    def _start_read_thread(self) -> None:
        """Starts or restarts the background read thread if it's not running."""
        if self.thread is not None and self.thread.is_alive():
            self.thread.join(timeout=0.1)
        if self.stop_event is not None:
            self.stop_event.set()

        self.stop_event = Event()
        self.thread = Thread(target=self._read_loop, args=(), name=f"{self}_read_loop")
        self.thread.daemon = True
        self.thread.start()

    def _stop_read_thread(self) -> None:
        """Signals the background read thread to stop and waits for it to join."""
        if self.stop_event is not None:
            self.stop_event.set()

        if self.thread is not None and self.thread.is_alive():
            self.thread.join(timeout=2.0)

        self.thread = None
        self.stop_event = None

    def async_read(self, timeout_ms: float = 200) -> NDArray[Any]:
        """
        Reads the latest available frame asynchronously.

        This method retrieves the most recent frame captured by the background
        read thread. It does not block waiting for the ZMQ stream directly,
        but may wait up to timeout_ms for the background thread to provide a frame.

        Args:
            timeout_ms (float): Maximum time in milliseconds to wait for a frame
                to become available. Defaults to 200ms (0.2 seconds).

        Returns:
            np.ndarray: The latest captured frame as a NumPy array in the format
                       (height, width, channels), processed according to configuration.

        Raises:
            DeviceNotConnectedError: If the camera is not connected.
            TimeoutError: If no frame becomes available within the specified timeout.
            RuntimeError: If an unexpected error occurs.
        """
        if not self.is_connected:
            raise DeviceNotConnectedError(f"{self} is not connected.")

        if self.thread is None or not self.thread.is_alive():
            self._start_read_thread()

        if not self.new_frame_event.wait(timeout=timeout_ms / 1000.0):
            thread_alive = self.thread is not None and self.thread.is_alive()
            raise TimeoutError(
                f"Timed out waiting for frame from camera {self} after {timeout_ms} ms. "
                f"Read thread alive: {thread_alive}."
            )

        with self.frame_lock:
            frame = self.latest_frame
            self.new_frame_event.clear()

        if frame is None:
            raise RuntimeError(f"Internal error: Event set but no frame available for {self}.")

        return frame