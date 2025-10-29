import argparse
from lerobot.cameras.configs import ColorMode, Cv2Rotation
from cameras.configuration_zmq import ZMQCameraConfig
from cameras.zmq_camera import ZMQCamera

if __name__ == "__main__":
    # args
    parser = argparse.ArgumentParser()
    parser.add_argument("--tcp_address", type=str, default="tcp://192.168.1.12:5555")
    parser.add_argument("--topic", type=str, default="zed2i_left")
    args = parser.parse_args()

    config = ZMQCameraConfig(
        tcp_address=args.tcp_address,
        topic=args.topic,
        color_mode=ColorMode.RGB,
        rotation=Cv2Rotation.NO_ROTATION
    )

    print(f"Config: {config}")

    camera = ZMQCamera(config)
    camera.connect()

    try:
        for i in range(10):
            frame = camera.async_read(timeout_ms=200)
            print(f"Async frame {i} shape:", frame.shape)
        
    finally:
        camera.disconnect()