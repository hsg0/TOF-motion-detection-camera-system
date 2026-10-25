# Sony IMX500

The camera is on the Pi CSI connector. Nothing about it runs on the Uno Q or the Pico.

`watcher.py` opens it with Picamera2 at 1280×720, RGB. The same process draws detection boxes and serves the live image on the dashboard. `libcamera` is the driver underneath Picamera2.

The OLED camera line is not computed here. `unoq_bridge.py` only checks whether this process is running, then tells the Uno Q.
