# HAT setup check

Use this on another Raspberry Pi 5 with an AI HAT+ 2. Run the commands, then compare the output with the reference column. The reference machine is `drone2`: a Pi 5 Model B Rev 1.1 with 16 GB of RAM. An 8 GB Pi is the same setup. Memory size is the one difference that is expected.

Do not copy accounts, passwords, or network addresses from the reference machine.

## How to compare

Run each command on the Pi you are checking. A row passes when the value matches the reference, except the memory row. If a Hailo package is not 5.1.1, stop. Models compiled for a newer Hailo release will not load on the 5.1.1 firmware in this HAT.

`lsmod` showing `hailo1x_pci` with `0` in the third column is normal when the dashboard is stopped. The HAT is present and idle. When `watcher.py` is running, that count goes above 0. The YOLOv11m network itself runs on the HAT. The Pi CPU only captures the frame, resizes it, draws the boxes, and serves the page.

## Reference versions

Collected from `drone2` on 29 Oct 2026.

| Check | Command | Reference |
|---|---|---|
| Board | `cat /proc/device-tree/model` | Raspberry Pi 5 Model B Rev 1.1 |
| Memory | `free -h` | 16 GB on the reference. 8 GB is acceptable on the other Pi. |
| OS | `cat /etc/os-release` | Debian GNU/Linux 13 (trixie) |
| Kernel | `uname -r` | `6.18.50+rpt-rpi-2712` |
| HAT device | `ls -l /dev/hailo0` | Character device exists |
| HAT architecture | `hailortcli fw-control identify` | `HAILO10H` |
| HAT firmware | `hailortcli fw-control identify` | `5.1.1 (release,app)` |
| Hailo runtime | `dpkg -l h10-hailort` | `5.1.1` |
| Hailo PCIe driver package | `dpkg -l h10-hailort-pcie-driver` | `5.1.1` |
| Hailo bundle | `dpkg -l hailo-h10-all` | `5.1.1` |
| Hailo Python | `dpkg -l python3-h10-hailort` | `5.1.1-1` |
| Hailo Python import | `python3 -c 'import hailo_platform; print(hailo_platform.__version__)'` | `5.1.1` |
| Loaded kernel module | `lsmod \| awk '/hailo/'` | `hailo1x_pci` |
| TAPPAS core | `dpkg -l hailo-tappas-core` | `5.1.0` |
| Model package | `dpkg -l hailo-models` | `1.0.0-2` |
| Camera library | `dpkg -l libcamera0.7` | `0.7.2+rpt20261009-1` |
| Picamera2 | `dpkg -l python3-picamera2` | `0.3.38-1` |
| rpicam apps | `dpkg -l rpicam-apps` | `1.14.0-1` |
| IMX500 bundle | `dpkg -l imx500-all` | `1.14.0-1` |
| IMX500 firmware | `dpkg -l imx500-firmware` | `0.FF23+3` |
| Ollama | `ollama --version` | `0.40.10` |
| Python | `python3 --version` | `3.13.5` |

## Models that made this setup work

These are the models the running software uses. The first one runs on the HAT. The other two run on the Pi CPU through Ollama. They are not Hailo files, and they do not move to the HAT.

| Role | Name | Where it runs | Where it is |
|---|---|---|---|
| Person and object detection | YOLOv11m, `yolov11m_h10.hef` | Hailo-10H NPU | `/usr/share/hailo-models/yolov11m_h10.hef` (27 MB) |
| Vision-language questions | `moondream:1.8b` | Pi CPU, Ollama port 11434 | `ollama list` shows id `55fc3abd3867`, about 1.7 GB |
| Text questions | `qwen2.5:1.5b` | Pi CPU, Ollama port 11434 | `ollama list` shows id `65ec06548149`, about 986 MB |

Check them with:

```bash
ls -lh /usr/share/hailo-models/yolov11m_h10.hef
ollama list
```

`ollama ps` is empty when nobody has asked a model a question. That is normal. An empty `ollama ps` does not mean the models are missing. `ollama list` is the install check.

The dashboard program is `watcher.py`. It loads `yolov11m_h10.hef` through `hailo_platform` on `/dev/hailo0`. It does not call Ollama.

## Hailo-10H files installed but not used by this setup

The `hailo-models` package also put these Hailo-10H files on the reference Pi. Our programs do not load them. Files whose names end in `h8` or `h8l` are for older Hailo chips. Ignore those on this HAT.

| File | Task |
|---|---|
| `yolov8m_h10.hef` | Object detection |
| `yolov8m_pose_h10.hef` | Pose |
| `yolov8s_pose_h10.hef` | Pose, smaller |
| `yolov5n_seg_h10.hef` | Segmentation |
| `resnet_v1_50_h10.hef` | Image classification |

## Language models that belong on the HAT, and are not installed yet

The HAT firmware is 5.1.1, so only 5.1.1 compiles apply. The pair to add later is:

| Job | Model |
|---|---|
| Text | Qwen2.5-1.5B-Instruct, Hailo name `qwen2.5-instruct:1.5b` |
| Vision | Qwen2-VL-2B-Instruct, file `Qwen2-VL-2B-Instruct.hef`, the 5.1.1 compile |

Do not use the newer Qwen2-VL file built for Hailo 5.2 or later. It is a different compile and will not load on 5.1.1 firmware.

These other text models are in the same 5.1.1 set. They also run on this HAT. They are not the pair to use with the vision model: `qwen2:1.5b`, `qwen2.5-coder:1.5b`, `deepseek_r1_distill_qwen:1.5b`, `llama3.2:3b`.

Hailo serves those through `hailo-ollama` on port 8000. That is a different program from Ollama on port 11434. On the reference Pi, port 8000 is not listening, and those models are not installed. A missing port 8000 is a match with this setup, not a failure.

## One-shot check

Run this on the Pi being compared and keep the output next to the tables above.

```bash
echo "=== BOARD ==="
cat /proc/device-tree/model; echo
free -h | awk 'NR==2'
echo "=== OS ==="
. /etc/os-release; echo "$PRETTY_NAME"
uname -r
python3 --version
echo "=== HAT ==="
ls -l /dev/hailo0
hailortcli fw-control identify
lsmod | awk '/hailo/'
echo "=== PACKAGES ==="
dpkg -l h10-hailort h10-hailort-pcie-driver hailo-h10-all python3-h10-hailort hailo-tappas-core hailo-models python3-picamera2 rpicam-apps imx500-all imx500-firmware | awk '/^ii/ {print $2, $3}'
python3 -c 'import hailo_platform; print("hailo_platform", hailo_platform.__version__)'
echo "=== MODELS IN USE ==="
ls -lh /usr/share/hailo-models/yolov11m_h10.hef
ollama --version
ollama list
echo "=== HAT LANGUAGE MODELS ==="
ss -ltn | awk '/:8000/ {print}' || true
echo "(no line means hailo-ollama is not running, which matches the reference)"
```
