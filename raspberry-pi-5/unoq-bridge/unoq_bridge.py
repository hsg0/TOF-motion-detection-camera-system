#!/usr/bin/env python3
"""Bridge between Raspberry Pi 5 (drone2) and Arduino Uno Q over USB/ADB.

1. Forwards localhost:7500 -> Uno Q 127.0.0.1:7500 (arduino-router MCU serial proxy).
2. Detects whether the Pi 5 IMX500 camera is currently active and sends 'C' (CAM: ON)
   or 'c' (CAM: OFF) to the Uno Q every second so Line 4 of the Adafruit 4650 OLED
   displays live camera state.
3. Writes the latest telemetry block from the Uno Q to /tmp/unoq_telemetry.txt.
"""

import socket
import subprocess
import time
from pathlib import Path

TELEMETRY_FILE = Path("/tmp/unoq_telemetry.txt")


def ensure_adb_forward():
    subprocess.run(
        ["adb", "forward", "tcp:7500", "tcp:7500"],
        capture_output=True,
        text=True,
        timeout=5,
        check=False,
    )


def is_pi_camera_active():
    res = subprocess.run(
        ["pgrep", "-f", "camera_view.py|watcher.py|rpicam-|libcamera-"],
        capture_output=True,
        text=True,
        timeout=2,
        check=False,
    )
    return res.returncode == 0 and bool(res.stdout.strip())


def main():
    while True:
        try:
            ensure_adb_forward()
            sock = socket.create_connection(("127.0.0.1", 7500), timeout=3)
            sock.settimeout(0.5)
            last_hb = 0.0
            lines = []
            while True:
                now = time.time()
                if now - last_hb >= 0.8:
                    cam_on = is_pi_camera_active()
                    sock.sendall(b"C" if cam_on else b"c")
                    last_hb = now
                try:
                    data = sock.recv(4096)
                    if not data:
                        break
                    text = data.decode("utf-8", errors="replace")
                    for line in text.splitlines():
                        line = line.strip()
                        if line:
                            lines.append(line)
                            if len(lines) > 12:
                                lines = lines[-12:]
                    TELEMETRY_FILE.write_text("\n".join(lines) + "\n", encoding="utf-8")
                except socket.timeout:
                    pass
        except Exception:
            time.sleep(2.0)


if __name__ == "__main__":
    main()
