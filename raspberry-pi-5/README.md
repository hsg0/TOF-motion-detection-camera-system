# Raspberry Pi 5

This board is the vision computer. It does not read the IMU, the barometer, the GPS, or the distance sensor itself. Those run on the Uno Q and the Pico. The Pi takes their text over USB and adds the camera.

What sits on this board:

| Folder | Program | Runs as |
|---|---|---|
| `camera-imx500` | Picamera2 capture inside `watcher.py` | Process on the Pi |
| `ai-hat-hailo-10h` | Hailo driver and `yolov11m_h10.hef` | Kernel module plus the same process |
| `dashboard` | `oct-24/watcher.py` | `python3` on port 8080 |
| `unoq-bridge` | `oct-24/unoq_bridge.py` | `python3`, started by hand |
| `cellular-modem` | ModemManager and NetworkManager | System services |

The copies that actually run are under the Desktop `scripts/oct-24` folder on the Pi. The project copies live in `oct-24/` next to this `oct-25` tree.

Power is a 5 V, 5 A USB-C supply. A weaker supply undervolts the board when the camera starts.

These programs do not start on boot. After a reboot, start `unoq_bridge.py` and then `watcher.py`.
