# Pi Streamer

Lightweight ZeroMQ PUB server that captures frames from USB cameras and sends
them to the Phosphobot workstation. Intended for deployment on a USB3-capable
edge device (e.g., Raspberry Pi 4, Jetson Nano) located near the cameras.

## Setup

1. Flash Raspberry Pi OS (64-bit recommended) and enable SSH.
2. Install dependencies:
   ```bash
   sudo apt update
   sudo apt install python3 python3-pip python3-opencv libatlas-base-dev ffmpeg pipx
   pipx install uv
   pipx ensurepath
   ```
   
3. Clone this repository and copy `pi_streamer/` to the device.
4. Create a virtual env using `uv`:
   ```bash
   cd pi_streamer
   uv sync
   ```
5. Copy `config.example.json` to `config.json` and adjust devices/topics.
   - Set `"stereo": true` for side-by-side cameras (e.g., ZED Mini). Optionally
     set `"right_topic"` to control the topic name for the right eye; otherwise
     `topic + "_right"` is used.
   - Prefer the stable `/dev/v4l/by-id/...` symlinks instead of `/dev/videoN`
     so USB reordering or reboots do not break the mapping. Example:
     `"device": "/dev/v4l/by-id/usb-Stereolabs_ZED-M_12345-video-index0"`.
     List the available symlinks with:
     ```bash
     ls -l /dev/v4l/by-id
     ```
   - `encoding` controls bandwidth: `"jpeg"` compresses frames, `"raw"` keeps full RGB.
   - Set `"only_send_left_image": true` to drop the right-eye payload while still
     splitting locally (useful if downstream only needs the left image).
6. Test locally:
   ```bash
   uv run python streamer.py --config config.json
   ```
7. On the Phosphobot workstation, use the Admin UI or POST `/cameras/add-zmq`
   to register each stream with the corresponding topic.

## Systemd Service (optional)

Create `/etc/systemd/system/pi-streamer.service`:

```
[Unit]
Description=Phosphobot Pi Streamer
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=pi
WorkingDirectory=/home/pi/phosphobot-ur5e/pi_streamer
Environment="PATH=/home/pi/phosphobot-ur5e/pi_streamer/.venv/bin:/usr/bin:/bin"
ExecStart=/home/pi/phosphobot-ur5e/pi_streamer/.venv/bin/python streamer.py --config config.json
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
```

Then enable and start:

```bash
sudo systemctl daemon-reload
sudo systemctl enable --now pi-streamer
```

### Detailed boot-start walkthrough

If you have never created a service before, follow these steps:

1. **Prepare the working copy**
   ```bash
   cd ~/phosphobot-ur5e/pi_streamer
   uv sync
   ```
2. **Create the service file**
   ```bash
   sudo nano /etc/systemd/system/pi-streamer.service
   ```
   Paste the block above, then save (`Ctrl+O`, `Enter`) and exit (`Ctrl+X`).
3. **Reload and enable**
   ```bash
   sudo systemctl daemon-reload
   sudo systemctl enable --now pi-streamer
   ```
4. **Verify it’s running**
   ```bash
   systemctl status pi-streamer
   ```
   You should see `active (running)`.
5. **Watch logs** (use `Ctrl+C` to exit):
   ```bash
   journalctl -u pi-streamer -f
   ```

Later, restart with `sudo systemctl restart pi-streamer`, or disable with
`sudo systemctl disable --now pi-streamer`.

## Notes

- Ensure the edge device provides USB3 bandwidth and power for the ZED Mini.
- Tune `jpeg_quality` and `fps` to match your network throughput.
- Monitor `journalctl -u pi-streamer` for runtime diagnostics.
```bash
sudo ip route del default via 1.1.1.1 dev eth0
```


