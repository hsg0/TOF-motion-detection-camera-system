# Uno Q bridge

Source: `oct-24/unoq_bridge.py`
On the Pi: Desktop `scripts/oct-24/unoq_bridge.py`

This program is the USB link between the Pi and the Uno Q.

1. `adb` forwards local port 7500 to the Uno Q serial-monitor proxy.
2. It reads the MCU text from that port and writes the latest lines to `/tmp/unoq_telemetry.txt`. The dashboard reads that file.
3. About once a second it sends one byte: `C` when `watcher.py` is running, `c` when it is not. The OLED uses that for the camera line.

Start this before the dashboard if you want the camera line to be right as soon as the camera opens.
