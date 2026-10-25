# Dashboard

Source: `oct-24/watcher.py`
On the Pi: Desktop `scripts/oct-24/watcher.py`

This is the only program that combines the camera, the Hailo detector, and the Uno Q telemetry. It listens on port 8080.

- Left side: live IMX500 image and detection boxes.
- Right side: time, GPS, IMU, barometer, distance, the four OLED lines, and Pi power.

It reads telemetry from the text file the bridge writes. It does not open the Uno Q serial port itself.

A snapshot is mailed only when both of these are true: the detector sees a person, and the distance has stayed at or inside 30 cm for 2 seconds. Repeat mail waits 100 seconds. The mail account is in a local config file on the Pi and is not stored in this tree.

Ollama is installed on the Pi as a separate model runtime. The dashboard does not call it.
