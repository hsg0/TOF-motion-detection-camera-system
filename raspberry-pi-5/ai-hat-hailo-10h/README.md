# AI HAT+ 2 (Hailo-10H)

The HAT sits on the Pi. The device node is `/dev/hailo0`. The stack is `hailo-h10-all` 5.1.1, including the kernel driver and `python3-h10-hailort`.

The model file is `yolov11m_h10.hef`. `watcher.py` runs it on the 1280×720 frames, resized to 640×640, a few times a second. People are boxed in green. Other classes are boxed in cyan.

The HAT has no sensor code of its own. It only classifies camera frames.
