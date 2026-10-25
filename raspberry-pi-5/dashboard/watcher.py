#!/usr/bin/env python3
"""
drone2 Unified Live Camera, Uno Q + Pico Telemetry & Detection Email Dashboard
Serves on http://0.0.0.0:8080/

Features:
- Left pane: Large 1280x720 live IMX500 stream with real-time Hailo-10H YOLOv11m
  bounding boxes, person detection banner, and captured event snapshot gallery.
- Right pane: Full telemetry dashboard showing Time (local & GPS UTC), Location/GPS
  (lat, lon, satellites in use/view, SNR dB, raw NMEA), IMU & Magnetometer
  (accel X/Y/Z, sample rates, FIFO, attitude/heading health), Barometer & Pico 1
  VL53L0X ToF (hPa, temp C, altitude m, ToF cm & mm), Adafruit 4650 OLED 4-line
  mirror, Flight Controller capabilities, Pi 5 Power/5V rail/NPU health, and
  SMTP alert configuration and event log. The account is not stored in this file.
- Automatic Snapshot + Full Telemetry Email Alert when Hailo-10H YOLOv11m sees a
  person and the VL53L0X ToF stays inside the distance threshold.
"""

from datetime import datetime, timezone
from email.mime.image import MIMEImage
from email.mime.multipart import MIMEMultipart
from email.mime.text import MIMEText
from http import server
from pathlib import Path
from threading import Condition, Lock
import io
import json
import logging
import os
import re
import smtplib
import socketserver
import subprocess
import sys
import threading
import time
import urllib.parse

import cv2
import numpy as np
from hailo_platform import VDevice
from picamera2 import Picamera2

PORT = int(os.environ.get("PORT", "8080"))
HEF_PATH = "/usr/share/hailo-models/yolov11m_h10.hef"
TELEMETRY_FILE = Path("/tmp/unoq_telemetry.txt")
CAPTURES_DIR = Path("/home/drone2/Desktop/captures")
EMAIL_CONFIG_PATH = Path("/home/drone2/.watcher_email.json")

COCO_CLASSES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light",
    "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow",
    "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard",
    "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple",
    "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard",
    "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
    "scissors", "teddy bear", "hair drier", "toothbrush",
]

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(message)s")

CAPTURES_DIR.mkdir(parents=True, exist_ok=True)


class FrameStore:
    def __init__(self):
        self.condition = Condition()
        self.jpeg_bytes = None
        self.raw_rgb = None
        self.frame_seq = 0

    def update(self, rgb_arr, jpeg_bytes):
        with self.condition:
            self.raw_rgb = rgb_arr
            self.jpeg_bytes = jpeg_bytes
            self.frame_seq += 1
            self.condition.notify_all()


frame_store = FrameStore()
state_lock = Lock()

detection_state = {
    "people": [],
    "all_objects": [],
    "npu_ms": 0.0,
    "last_yolo_ts": 0.0,
    "camera_person_detected": False,
    "tof_person_detected": False,
    "user_detected": False,
    "trigger_reason": "None",
    "last_capture_file": "",
    "last_capture_time": "",
    "last_email_status": "No email sent yet",
    "last_email_time": "",
    "events": [],
}

default_config = {
    "smtp_host": os.environ.get("SMTP_HOST", ""),
    "smtp_port": int(os.environ.get("SMTP_PORT", "465")),
    "sender_email": os.environ.get("ALERT_SENDER", ""),
    "recipient_email": os.environ.get("ALERT_RECIPIENT", ""),
    "app_password": os.environ.get("SMTP_APP_PASSWORD", ""),
    "tof_threshold_cm": 30,
    "tof_hold_sec": 2.0,
    "yolo_score_min": 0.45,
    "cooldown_sec": 100,
    "email_enabled": True,
    "trigger_mode": "both",  # "either", "both", "camera_only", "tof_only"
}


def load_config():
    cfg = dict(default_config)
    if EMAIL_CONFIG_PATH.exists():
        try:
            saved = json.loads(EMAIL_CONFIG_PATH.read_text(encoding="utf-8"))
            cfg.update(saved)
        except Exception as exc:
            logging.warning("Failed to read %s: %s", EMAIL_CONFIG_PATH, exc)
    return cfg


def save_config(new_cfg):
    cfg = load_config()
    for k in (
        "smtp_host",
        "smtp_port",
        "sender_email",
        "recipient_email",
        "app_password",
        "tof_threshold_cm",
        "tof_hold_sec",
        "yolo_score_min",
        "cooldown_sec",
        "email_enabled",
        "trigger_mode",
    ):
        if k in new_cfg and new_cfg[k] is not None:
            if k == "app_password" and new_cfg[k] == "********":
                continue
            cfg[k] = new_cfg[k]
    EMAIL_CONFIG_PATH.write_text(json.dumps(cfg, indent=2), encoding="utf-8")
    try:
        os.chmod(EMAIL_CONFIG_PATH, 0o600)
    except Exception:
        pass
    return cfg


def run_cmd(args, timeout=2.0):
    try:
        res = subprocess.run(args, capture_output=True, text=True, timeout=timeout, check=False)
        return res.stdout.strip()
    except Exception:
        return ""


def get_pi_power_telemetry():
    throttled_raw = run_cmd(["vcgencmd", "get_throttled"])
    throttled_hex = throttled_raw.split("=")[-1] if "=" in throttled_raw else "0x0"
    try:
        throttled_val = int(throttled_hex, 16)
    except Exception:
        throttled_val = 0

    ext5v_raw = run_cmd(["vcgencmd", "pmic_read_adc", "EXT5V_V"])
    m_v = re.search(r"([0-9]+\.[0-9]+)V", ext5v_raw)
    ext5v = round(float(m_v.group(1)), 3) if m_v else None

    core_a_raw = run_cmd(["vcgencmd", "pmic_read_adc", "VDD_CORE_A"])
    m_a = re.search(r"([0-9]+\.[0-9]+)A", core_a_raw)
    core_a = round(float(m_a.group(1)), 2) if m_a else None

    temp_raw = run_cmd(["vcgencmd", "measure_temp"])
    m_t = re.search(r"([0-9]+\.[0-9]+)", temp_raw)
    temp_c = round(float(m_t.group(1)), 1) if m_t else None

    return {
        "throttled_hex": throttled_hex,
        "undervolt_now": bool(throttled_val & 0x1),
        "undervolt_history": bool(throttled_val & 0x10000),
        "ext5v_v": ext5v,
        "vdd_core_a": core_a,
        "temp_c": temp_c,
        "hailo_dev": os.path.exists("/dev/hailo0"),
    }


def parse_nmea_utc_and_coords(nmea_str):
    """Extracts UTC time and lat/lon (if present) from $GPGGA or $GPRMC."""
    info = {"utc_time": "--:--:--", "utc_date": "--", "nmea_type": "--"}
    if not nmea_str or not nmea_str.startswith("$"):
        return info
    parts = nmea_str.split("*")[0].split(",")
    info["nmea_type"] = parts[0]
    if len(parts) > 1 and len(parts[1]) >= 6 and parts[1][:6].isdigit():
        hh, mm, ss = parts[1][0:2], parts[1][2:4], parts[1][4:6]
        info["utc_time"] = f"{hh}:{mm}:{ss} UTC"
    if parts[0].endswith("RMC") and len(parts) > 9 and len(parts[9]) == 6 and parts[9].isdigit():
        dd, mo, yy = parts[9][0:2], parts[9][2:4], parts[9][4:6]
        info["utc_date"] = f"20{yy}-{mo}-{dd}"
    return info


def parse_unoq_telemetry():
    now_local = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    now_utc = datetime.now(timezone.utc).strftime("%Y-%m-%d %H:%M:%S UTC")

    parsed = {
        "time": {
            "pi_local": now_local,
            "pi_utc": now_utc,
            "gps_utc_time": "--:--:--",
            "gps_utc_date": "--",
        },
        "gps": {
            "bytes": 0,
            "ok_sentences": 0,
            "bad_sentences": 0,
            "sats_in_use": 0,
            "sats_in_view": 0,
            "snr_db": 0,
            "link": "--",
            "fix": "no",
            "lat_rounded": "0.00",
            "lon_rounded": "0.00",
            "location_label": "Indoor / Waiting for 4-sat lock (192.168.0.134)",
            "raw_nmea": "",
        },
        "imu": {
            "acc_hz": 0.0,
            "gyr_hz": 0.0,
            "mag_hz": 0.0,
            "fifo": 0,
            "att": "--",
            "hdg": "--",
            "err": 0,
            "ax": 0.0,
            "ay": 0.0,
            "az": 0.0,
            "raw_err_line": "",
        },
        "baro": {
            "address": "0x77",
            "pressure_hpa": 0.0,
            "temp_c": 0.0,
            "baseline": "0/100",
            "altitude_m": 0.0,
            "ready": "no",
        },
        "tof": {
            "pico_present": False,
            "distance_cm": None,
            "distance_mm": None,
            "fresh": False,
            "sensor_name": "VL53L0X (GP0/GP1 I2C0 0x29)",
        },
        "oled": {
            "address": "0x3C",
            "status": "--",
            "line1": "",
            "line2": "",
            "line3": "",
            "line4": "",
        },
        "ctrl": {
            "stop": "--",
            "move": "--",
            "att": "--",
            "hdg": "--",
            "alt": "--",
            "pos": "--",
            "rth": "--",
            "gps": "--",
            "luna": "--",
        },
        "raw_lines": [],
    }

    if not TELEMETRY_FILE.exists():
        return parsed

    try:
        lines = [ln.strip() for ln in TELEMETRY_FILE.read_text(encoding="utf-8", errors="replace").splitlines() if ln.strip()]
    except Exception:
        return parsed

    parsed["raw_lines"] = lines[-10:]

    for ln in lines:
        if ln.startswith("IMU acc "):
            m = re.search(
                r"IMU acc ([0-9.]+) gyr ([0-9.]+) mag ([0-9.]+) Hz \| fifo ([0-9]+) \| att (\S+) \| hdg (\S+) \| err ([0-9]+)",
                ln,
            )
            if m:
                parsed["imu"]["acc_hz"] = float(m.group(1))
                parsed["imu"]["gyr_hz"] = float(m.group(2))
                parsed["imu"]["mag_hz"] = float(m.group(3))
                parsed["imu"]["fifo"] = int(m.group(4))
                parsed["imu"]["att"] = m.group(5)
                parsed["imu"]["hdg"] = m.group(6)
                parsed["imu"]["err"] = int(m.group(7))
        elif ln.startswith("IMUERR "):
            parsed["imu"]["raw_err_line"] = ln
        elif ln.startswith("BARO "):
            m = re.search(
                r"BARO (0x[0-9A-Fa-f]+) ([0-9.\-]+) hPa \| ([0-9.\-]+) C \| base ([0-9/]+) \| alt ([0-9.\-]+) m \| ready (\S+)",
                ln,
            )
            if m:
                parsed["baro"]["address"] = m.group(1)
                parsed["baro"]["pressure_hpa"] = float(m.group(2))
                parsed["baro"]["temp_c"] = float(m.group(3))
                parsed["baro"]["baseline"] = m.group(4)
                parsed["baro"]["altitude_m"] = float(m.group(5))
                parsed["baro"]["ready"] = m.group(6)
        elif ln.startswith("GPS "):
            m = re.search(
                r"GPS bytes ([0-9]+) \| ok ([0-9]+) \| bad ([0-9]+) \| sats ([0-9]+)/([0-9]+) \| snr ([0-9]+)dB \| link (\S+) \| fix (\S+) \| (.*)",
                ln,
            )
            if m:
                parsed["gps"]["bytes"] = int(m.group(1))
                parsed["gps"]["ok_sentences"] = int(m.group(2))
                parsed["gps"]["bad_sentences"] = int(m.group(3))
                parsed["gps"]["sats_in_use"] = int(m.group(4))
                parsed["gps"]["sats_in_view"] = int(m.group(5))
                parsed["gps"]["snr_db"] = int(m.group(6))
                parsed["gps"]["link"] = m.group(7)
                parsed["gps"]["fix"] = m.group(8)
                raw_nmea = m.group(9).strip()
                parsed["gps"]["raw_nmea"] = raw_nmea
                nmea_info = parse_nmea_utc_and_coords(raw_nmea)
                if nmea_info["utc_time"] != "--:--:--":
                    parsed["time"]["gps_utc_time"] = nmea_info["utc_time"]
                if nmea_info["utc_date"] != "--":
                    parsed["time"]["gps_utc_date"] = nmea_info["utc_date"]
        elif ln.startswith("PICO1 "):
            if "fresh yes" in ln:
                m = re.search(r"PICO1 yes \| ([0-9]+) cm \| strength ([0-9]+) \| fresh yes", ln)
                if m:
                    parsed["tof"]["pico_present"] = True
                    parsed["tof"]["distance_cm"] = int(m.group(1))
                    parsed["tof"]["distance_mm"] = int(m.group(2))
                    parsed["tof"]["fresh"] = True
            elif ln.startswith("PICO1 yes"):
                parsed["tof"]["pico_present"] = True
                parsed["tof"]["fresh"] = False
        elif ln.startswith("OLED "):
            m = re.search(
                r"OLED (0x[0-9A-Fa-f]+) (\S+) \| L1:\[(.*?)\] L2:\[(.*?)\] L3:\[(.*?)\] L4:\[(.*?)\]",
                ln,
            )
            if m:
                parsed["oled"]["address"] = m.group(1)
                parsed["oled"]["status"] = m.group(2)
                parsed["oled"]["line1"] = m.group(3)
                parsed["oled"]["line2"] = m.group(4)
                parsed["oled"]["line3"] = m.group(5)
                parsed["oled"]["line4"] = m.group(6)
                # Extract lat/lon from L1 and ax/ay/az from L2
                m_l1 = re.search(r"GPS ([0-9.\-]+) ([0-9.\-]+)", m.group(3))
                if m_l1:
                    parsed["gps"]["lat_rounded"] = m_l1.group(1)
                    parsed["gps"]["lon_rounded"] = m_l1.group(2)
                    if parsed["gps"]["fix"] == "yes":
                        parsed["gps"]["location_label"] = f"GPS Fix ({m_l1.group(1)}, {m_l1.group(2)})"
                m_l2 = re.search(r"IMU ([0-9.\-]+) ([0-9.\-]+) ([0-9.\-]+)", m.group(4))
                if m_l2:
                    parsed["imu"]["ax"] = float(m_l2.group(1))
                    parsed["imu"]["ay"] = float(m_l2.group(2))
                    parsed["imu"]["az"] = float(m_l2.group(3))
        elif ln.startswith("CTRL "):
            for key in ("stop", "move", "att", "hdg", "alt", "pos", "rth", "gps", "luna"):
                mk = re.search(rf"\b{key}\s+(\S+)", ln)
                if mk:
                    parsed["ctrl"][key] = mk.group(1)

    return parsed


def parse_yolo_nms(out_buf, score_min):
    people = []
    all_objects = []
    idx = 0
    n_buf = len(out_buf)
    for cls_id in range(80):
        if idx >= n_buf:
            break
        count = int(round(float(out_buf[idx])))
        idx += 1
        for _ in range(count):
            if idx + 5 > n_buf:
                break
            ymin, xmin, ymax, xmax, score = [float(x) for x in out_buf[idx : idx + 5]]
            idx += 5
            if score >= score_min:
                label = COCO_CLASSES[cls_id] if cls_id < len(COCO_CLASSES) else f"class_{cls_id}"
                cx = 0.5 * (xmin + xmax)
                cy = 0.5 * (ymin + ymax)
                horiz = "left" if cx < 0.35 else ("right" if cx > 0.65 else "center")
                vert = "top" if cy < 0.35 else ("bottom" if cy > 0.65 else "middle")
                item = {
                    "label": label,
                    "score": round(score, 2),
                    "position": f"{vert}-{horiz}",
                    "bbox": [round(xmin, 3), round(ymin, 3), round(xmax, 3), round(ymax, 3)],
                }
                all_objects.append(item)
                if cls_id == 0:
                    people.append(item)
    people.sort(key=lambda d: d["score"], reverse=True)
    all_objects.sort(key=lambda d: d["score"], reverse=True)
    return people, all_objects


def annotate_frame_bgr(bgr, people, all_objects, telemetry, trigger_reason=None):
    h, w = bgr.shape[:2]
    canvas = bgr.copy()

    # Draw non-person objects in cyan, person detections in bright green
    for obj in all_objects:
        xmin, ymin, xmax, ymax = obj["bbox"]
        x1, y1 = int(xmin * w), int(ymin * h)
        x2, y2 = int(xmax * w), int(ymax * h)
        is_person = obj["label"] == "person"
        color = (0, 255, 80) if is_person else (255, 200, 0)
        thickness = 3 if is_person else 2
        cv2.rectangle(canvas, (x1, y1), (x2, y2), color, thickness)
        caption = f"{obj['label'].upper()} {int(obj['score']*100)}% ({obj['position']})"
        (tw, th), _ = cv2.getTextSize(caption, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)
        ty = max(y1 - 8, th + 8)
        cv2.rectangle(canvas, (x1, ty - th - 6), (x1 + tw + 10, ty + 4), color, -1)
        cv2.putText(canvas, caption, (x1 + 5, ty - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)

    # Top overlay bar with live telemetry summary
    overlay = canvas.copy()
    cv2.rectangle(overlay, (0, 0), (w, 42), (15, 20, 28), -1)
    cv2.addWeighted(overlay, 0.78, canvas, 0.22, 0, canvas)

    tof_cm = telemetry["tof"]["distance_cm"]
    tof_str = f"{tof_cm}cm ({telemetry['tof']['distance_mm']}mm)" if tof_cm is not None else "--"
    header_text = (
        f"drone2 | {telemetry['time']['pi_local']} | "
        f"GPS:{telemetry['gps']['lat_rounded']},{telemetry['gps']['lon_rounded']} (s{telemetry['gps']['sats_in_use']}/{telemetry['gps']['sats_in_view']}) | "
        f"IMU:{telemetry['imu']['ax']},{telemetry['imu']['ay']},{telemetry['imu']['az']} | "
        f"Baro:{telemetry['baro']['pressure_hpa']:.1f}hPa | ToF:{tof_str}"
    )
    cv2.putText(canvas, header_text, (12, 27), cv2.FONT_HERSHEY_SIMPLEX, 0.52, (230, 240, 255), 1)

    if trigger_reason:
        cv2.rectangle(canvas, (0, h - 38), (w, h), (20, 30, 180), -1)
        cv2.putText(
            canvas,
            f"USER DETECTED: {trigger_reason}",
            (14, h - 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
        )

    return canvas


def email_recipients(cfg):
    raw = cfg.get("recipient_email") or ""
    parts = raw if isinstance(raw, list) else str(raw).replace(";", ",").split(",")
    recipients = []
    for part in parts:
        addr = str(part).strip()
        if addr and addr not in recipients:
            recipients.append(addr)
    if not recipients:
        recipients = []
    return recipients


def send_detection_email(cfg, jpeg_bytes, filename, trigger_reason, telemetry, power, people, all_objects):
    sender = (cfg.get("sender_email") or "").strip()
    recipients = email_recipients(cfg)
    recipient_header = ", ".join(recipients)
    app_pw = (cfg.get("app_password") or "").strip().replace(" ", "")
    smtp_host = (cfg.get("smtp_host") or "").strip()
    smtp_port = int(cfg.get("smtp_port") or 465)

    if not app_pw or not sender or not recipients or not smtp_host:
        return False, "Saved snapshot locally (set the SMTP host, sender, recipient, and app password to send email)"

    msg = MIMEMultipart("related")
    msg["Subject"] = f"[drone2 ALERT] User Detected ({trigger_reason}) - {telemetry['time']['pi_local']}"
    msg["From"] = sender
    msg["To"] = recipient_header

    people_str = (
        ", ".join([f"person ({int(p['score']*100)}% at {p['position']})" for p in people])
        if people
        else "None (Triggered via ToF Proximity)"
    )
    objects_str = (
        ", ".join([f"{o['label']} ({int(o['score']*100)}%)" for o in all_objects])
        if all_objects
        else "None"
    )
    tof_cm = telemetry["tof"]["distance_cm"]
    tof_mm = telemetry["tof"]["distance_mm"]
    tof_display = f"{tof_cm} cm ({tof_mm} mm)" if tof_cm is not None else "N/A"

    html = f"""
    <html>
    <body style="font-family: Arial, sans-serif; background:#0d1117; color:#e6edf3; padding:18px;">
      <h2 style="color:#58a6ff; margin-top:0;">drone2 — User Detection Telemetry Report</h2>
      <p><strong>Trigger Source:</strong> {trigger_reason}</p>
      <p><strong>Snapshot File:</strong> {filename}</p>
      <table border="1" cellpadding="8" cellspacing="0" style="border-collapse:collapse; border-color:#30363d; width:100%; max-width:720px; font-size:14px;">
        <tr style="background:#161b22;"><th align="left">Category</th><th align="left">Telemetry Field</th><th align="left">Live Value</th></tr>
        <tr><td><strong>Time</strong></td><td>Pi Local / GPS UTC</td><td>{telemetry['time']['pi_local']} / {telemetry['time']['gps_utc_time']} ({telemetry['time']['gps_utc_date']})</td></tr>
        <tr><td><strong>Detection</strong></td><td>Person / Objects (Hailo-10H YOLOv11m)</td><td><strong>{people_str}</strong><br>All objects: {objects_str}</td></tr>
        <tr><td><strong>ToF Sensor</strong></td><td>Pico 1 VL53L0X (GP0/GP1)</td><td><strong>{tof_display}</strong> (Fresh: {telemetry['tof']['fresh']})</td></tr>
        <tr><td><strong>Location / GPS</strong></td><td>GT-U7 (Uno Q D0)</td><td>Lat: <strong>{telemetry['gps']['lat_rounded']}</strong>, Lon: <strong>{telemetry['gps']['lon_rounded']}</strong><br>Fix: {telemetry['gps']['fix']} | Sats (use/view): {telemetry['gps']['sats_in_use']}/{telemetry['gps']['sats_in_view']} | Best SNR: {telemetry['gps']['snr_db']} dB<br>Raw NMEA: <code>{telemetry['gps']['raw_nmea']}</code></td></tr>
        <tr><td><strong>IMU / Mag</strong></td><td>LSM6DSOX (0x6A) + LIS3MDL (0x1C)</td><td>Accel X/Y/Z: <strong>{telemetry['imu']['ax']}, {telemetry['imu']['ay']}, {telemetry['imu']['az']} m/s²</strong><br>Rates: acc {telemetry['imu']['acc_hz']} Hz, gyr {telemetry['imu']['gyr_hz']} Hz, mag {telemetry['imu']['mag_hz']} Hz | Att: {telemetry['imu']['att']} | Hdg: {telemetry['imu']['hdg']}</td></tr>
        <tr><td><strong>Barometer</strong></td><td>BME280 (0x77)</td><td>Pressure: <strong>{telemetry['baro']['pressure_hpa']} hPa</strong> | Temp: <strong>{telemetry['baro']['temp_c']} °C</strong> | Rel Altitude: <strong>{telemetry['baro']['altitude_m']} m</strong> (Base {telemetry['baro']['baseline']})</td></tr>
        <tr><td><strong>OLED Screen</strong></td><td>Adafruit 4650 (0x3C)</td><td>L1: <code>{telemetry['oled']['line1']}</code><br>L2: <code>{telemetry['oled']['line2']}</code><br>L3: <code>{telemetry['oled']['line3']}</code><br>L4: <code>{telemetry['oled']['line4']}</code></td></tr>
        <tr><td><strong>Flight Ctrl</strong></td><td>Uno Q STM32U585</td><td>Stop: {telemetry['ctrl']['stop']} | Move: {telemetry['ctrl']['move']} | Att: {telemetry['ctrl']['att']} | Hdg: {telemetry['ctrl']['hdg']} | Alt: {telemetry['ctrl']['alt']} | Luna: {telemetry['ctrl']['luna']}</td></tr>
        <tr><td><strong>Pi 5 Power</strong></td><td>PMIC / Hailo-10H NPU</td><td>5V Rail: <strong>{power.get('ext5v_v')} V</strong> | Throttle: <code>{power.get('throttled_hex')}</code> | CPU Temp: {power.get('temp_c')} °C | NPU: /dev/hailo0 Ready</td></tr>
      </table>
      <h3 style="margin-top:18px;">Captured Camera Image</h3>
      <p><img src="cid:capture_img" style="max-width:720px; width:100%; border:2px solid #238636; border-radius:8px;"></p>
    </body>
    </html>
    """

    alt = MIMEMultipart("alternative")
    msg.attach(alt)
    alt.attach(MIMEText(html, "html", "utf-8"))

    img_part = MIMEImage(jpeg_bytes, _subtype="jpeg")
    img_part.add_header("Content-ID", "<capture_img>")
    img_part.add_header("Content-Disposition", "inline", filename=filename)
    msg.attach(img_part)

    try:
        if smtp_port == 465:
            with smtplib.SMTP_SSL(smtp_host, smtp_port, timeout=15) as server_ssl:
                server_ssl.login(sender, app_pw)
                server_ssl.sendmail(sender, recipients, msg.as_string())
        else:
            with smtplib.SMTP(smtp_host, smtp_port, timeout=15) as server_tls:
                server_tls.ehlo()
                server_tls.starttls()
                server_tls.login(sender, app_pw)
                server_tls.sendmail(sender, recipients, msg.as_string())
        return True, f"Sent email with image & telemetry to {recipient_header}"
    except smtplib.SMTPResponseException as exc:
        err_txt = exc.smtp_error.decode("utf-8", errors="replace") if isinstance(exc.smtp_error, bytes) else str(exc.smtp_error)
        return False, f"SMTP {exc.smtp_code}: {err_txt}"
    except Exception as exc:
        return False, f"SMTP error: {exc}"


def trigger_capture_and_email(trigger_reason, force=False):
    cfg = load_config()
    with frame_store.condition:
        raw_rgb = frame_store.raw_rgb.copy() if frame_store.raw_rgb is not None else None

    if raw_rgb is None:
        return {"ok": False, "error": "No camera frame available yet"}

    with state_lock:
        people = list(detection_state["people"])
        all_objects = list(detection_state["all_objects"])

    telemetry = parse_unoq_telemetry()
    power = get_pi_power_telemetry()

    bgr = cv2.cvtColor(raw_rgb, cv2.COLOR_RGB2BGR)
    annotated = annotate_frame_bgr(bgr, people, all_objects, telemetry, trigger_reason=trigger_reason)
    ok_enc, enc_buf = cv2.imencode(".jpg", annotated, [cv2.IMWRITE_JPEG_QUALITY, 90])
    if not ok_enc:
        return {"ok": False, "error": "JPEG encode failed"}
    jpeg_bytes = enc_buf.tobytes()

    ts_slug = datetime.now().strftime("%Y%m%d_%H%M%S")
    img_name = f"detect_{ts_slug}.jpg"
    json_name = f"detect_{ts_slug}.json"
    img_path = CAPTURES_DIR / img_name
    json_path = CAPTURES_DIR / json_name

    img_path.write_bytes(jpeg_bytes)

    event_payload = {
        "timestamp": telemetry["time"]["pi_local"],
        "trigger_reason": trigger_reason,
        "image_file": img_name,
        "people": people,
        "all_objects": all_objects,
        "telemetry": telemetry,
        "power": power,
    }

    if cfg.get("email_enabled", True) or force:
        mail_ok, mail_msg = send_detection_email(
            cfg, jpeg_bytes, img_name, trigger_reason, telemetry, power, people, all_objects
        )
    else:
        mail_ok, mail_msg = False, "Email alerts paused in settings (snapshot saved locally)"

    event_payload["email_ok"] = mail_ok
    event_payload["email_status"] = mail_msg
    json_path.write_text(json.dumps(event_payload, indent=2), encoding="utf-8")

    with state_lock:
        detection_state["last_capture_file"] = img_name
        detection_state["last_capture_time"] = telemetry["time"]["pi_local"]
        detection_state["last_email_status"] = mail_msg
        detection_state["last_email_time"] = telemetry["time"]["pi_local"]
        detection_state["events"].insert(
            0,
            {
                "timestamp": telemetry["time"]["pi_local"],
                "trigger_reason": trigger_reason,
                "image_file": img_name,
                "people_count": len(people),
                "tof_cm": telemetry["tof"]["distance_cm"],
                "email_ok": mail_ok,
                "email_status": mail_msg,
            },
        )
        detection_state["events"] = detection_state["events"][:25]

    return {
        "ok": True,
        "image_file": img_name,
        "email_ok": mail_ok,
        "email_status": mail_msg,
        "event": event_payload,
    }


def camera_and_yolo_loop():
    logging.info("Starting Picamera2 (1280x720) + Hailo-10H YOLOv11m detector loop...")
    picam2 = Picamera2()
    video_cfg = picam2.create_video_configuration(main={"size": (1280, 720), "format": "RGB888"})
    picam2.configure(video_cfg)
    picam2.start()

    last_yolo_run = 0.0
    last_alert_ts = 0.0
    tof_close_since = None

    with VDevice() as vdevice:
        infer_model = vdevice.create_infer_model(HEF_PATH)
        infer_model.set_batch_size(1)
        with infer_model.configure() as configured_model:
            out_name = infer_model.output_names[0]
            out_shape = infer_model.output(out_name).shape

            while True:
                rgb = picam2.capture_array()
                now = time.time()
                cfg = load_config()
                score_min = float(cfg.get("yolo_score_min", 0.45))
                tof_thresh = int(cfg.get("tof_threshold_cm", 30))
                tof_hold_sec = float(cfg.get("tof_hold_sec", 2.0))
                cooldown_sec = float(cfg.get("cooldown_sec", 100))
                trigger_mode = cfg.get("trigger_mode", "both")

                if now - last_yolo_run >= 0.25:
                    t0 = time.time()
                    inp = cv2.resize(rgb, (640, 640), interpolation=cv2.INTER_LINEAR).astype(np.uint8)
                    out_buf = np.empty(out_shape, dtype=np.float32)
                    bindings = configured_model.create_bindings()
                    bindings.input().set_buffer(inp)
                    bindings.output(out_name).set_buffer(out_buf)
                    configured_model.run([bindings], 5000)
                    npu_ms = round((time.time() - t0) * 1000.0, 1)
                    people, all_objects = parse_yolo_nms(out_buf, score_min)
                    last_yolo_run = now

                    telemetry = parse_unoq_telemetry()
                    tof_cm = telemetry["tof"]["distance_cm"]
                    tof_fresh = telemetry["tof"]["fresh"]

                    cam_hit = len(people) > 0
                    tof_close = bool(tof_fresh and tof_cm is not None and 0 < tof_cm <= tof_thresh)
                    if tof_close:
                        if tof_close_since is None:
                            tof_close_since = now
                        tof_hit = (now - tof_close_since) >= tof_hold_sec
                    else:
                        tof_close_since = None
                        tof_hit = False

                    if trigger_mode == "both":
                        triggered = cam_hit and tof_hit
                    elif trigger_mode == "camera_only":
                        triggered = cam_hit
                    elif trigger_mode == "tof_only":
                        triggered = tof_hit
                    else:
                        triggered = cam_hit or tof_hit

                    reasons = []
                    if cam_hit:
                        top_p = people[0]
                        reasons.append(f"Camera Person ({int(top_p['score']*100)}% at {top_p['position']})")
                    if tof_hit:
                        reasons.append(f"ToF Proximity ({tof_cm} cm <= {tof_thresh} cm)")
                    reason_str = " + ".join(reasons) if reasons else "None"

                    with state_lock:
                        detection_state["people"] = people
                        detection_state["all_objects"] = all_objects
                        detection_state["npu_ms"] = npu_ms
                        detection_state["last_yolo_ts"] = now
                        detection_state["camera_person_detected"] = cam_hit
                        detection_state["tof_person_detected"] = tof_hit
                        detection_state["user_detected"] = triggered
                        detection_state["trigger_reason"] = reason_str

                with state_lock:
                    cur_people = list(detection_state["people"])
                    cur_objects = list(detection_state["all_objects"])
                    cur_triggered = detection_state["user_detected"]
                    cur_reason = detection_state["trigger_reason"]

                telemetry = parse_unoq_telemetry()
                bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                annotated_bgr = annotate_frame_bgr(
                    bgr,
                    cur_people,
                    cur_objects,
                    telemetry,
                    trigger_reason=cur_reason if cur_triggered else None,
                )
                ok_jpg, jpg_buf = cv2.imencode(".jpg", annotated_bgr, [cv2.IMWRITE_JPEG_QUALITY, 82])
                if ok_jpg:
                    frame_store.update(rgb, jpg_buf.tobytes())

                if cur_triggered and (now - last_alert_ts >= cooldown_sec):
                    last_alert_ts = now
                    threading.Thread(
                        target=trigger_capture_and_email,
                        args=(cur_reason, False),
                        daemon=True,
                    ).start()

                time.sleep(0.03)


PAGE_HTML = """<!doctype html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>drone2 — Live Camera &amp; Full Sensor Telemetry Dashboard</title>
<style>
  :root {
    --bg: #0b0f17;
    --panel: #151b26;
    --panel2: #1c2433;
    --border: #2d3748;
    --text: #e6edf3;
    --muted: #8b949e;
    --accent: #3b82f6;
    --ok: #22c55e;
    --warn: #f59e0b;
    --danger: #ef4444;
  }
  * { box-sizing: border-box; }
  html, body {
    margin: 0; padding: 0;
    background: var(--bg); color: var(--text);
    font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
    min-height: 100vh;
  }
  header {
    display: flex; flex-wrap: wrap; align-items: center; justify-content: space-between;
    gap: 10px; padding: 10px 18px;
    background: var(--panel); border-bottom: 1px solid var(--border);
  }
  .brand { font-weight: 700; font-size: 16px; display: flex; align-items: center; gap: 10px; }
  .pills { display: flex; flex-wrap: wrap; gap: 8px; align-items: center; }
  .pill {
    font-size: 12px; padding: 4px 10px; border-radius: 999px;
    background: var(--panel2); border: 1px solid var(--border); font-weight: 600;
  }
  .pill.ok { border-color: var(--ok); color: #4ade80; }
  .pill.warn { border-color: var(--warn); color: #fbbf24; }
  .pill.danger { border-color: var(--danger); color: #f87171; background: rgba(239,68,68,0.15); }
  .layout {
    display: grid;
    grid-template-columns: 1.35fr 1fr;
    gap: 14px;
    padding: 14px;
    align-items: start;
  }
  @media (max-width: 720px) {
    .layout { grid-template-columns: 1fr; }
  }
  .card {
    background: var(--panel); border: 1px solid var(--border);
    border-radius: 10px; overflow: hidden; margin-bottom: 14px;
  }
  .card-head {
    padding: 10px 14px; background: var(--panel2); border-bottom: 1px solid var(--border);
    display: flex; align-items: center; justify-content: space-between;
    font-size: 13px; font-weight: 700; letter-spacing: 0.2px;
  }
  .cam-box {
    background: #000; width: 100%; aspect-ratio: 16 / 9;
    display: flex; align-items: center; justify-content: center; overflow: hidden;
  }
  .cam-box img { width: 100%; height: 100%; object-fit: contain; display: block; }
  .alert-banner {
    padding: 12px 16px; font-weight: 700; font-size: 14px;
    display: flex; align-items: center; justify-content: space-between; gap: 10px;
    border-top: 1px solid var(--border);
  }
  .alert-banner.idle { background: #121822; color: var(--muted); }
  .alert-banner.active { background: rgba(239,68,68,0.22); color: #fca5a5; border-color: var(--danger); }
  .btn-row { display: flex; flex-wrap: wrap; gap: 8px; padding: 10px 14px; border-top: 1px solid var(--border); }
  button {
    background: var(--accent); color: #fff; border: none; border-radius: 6px;
    padding: 7px 12px; font-size: 12.5px; font-weight: 600; cursor: pointer;
  }
  button.sec { background: var(--panel2); border: 1px solid var(--border); color: var(--text); }
  button:disabled { opacity: 0.5; cursor: wait; }
  .grid-2 {
    display: grid; grid-template-columns: 1fr 1fr; gap: 8px; padding: 12px;
  }
  .kv {
    background: #0f141d; border: 1px solid var(--border); border-radius: 7px;
    padding: 8px 10px;
  }
  .kv .k { font-size: 11px; color: var(--muted); text-transform: uppercase; }
  .kv .v { font-size: 14px; font-weight: 700; margin-top: 3px; word-break: break-word; }
  .oled-screen {
    margin: 10px 12px; padding: 10px 14px; background: #05070a;
    border: 2px solid #3b82f6; border-radius: 8px;
    font-family: "Courier New", monospace; font-size: 14px; font-weight: 700;
    color: #f8fafc; line-height: 1.55;
  }
  .form-grid {
    display: grid; grid-template-columns: 1fr 1fr; gap: 8px; padding: 12px;
  }
  .form-grid label { font-size: 11.5px; color: var(--muted); display: flex; flex-direction: column; gap: 4px; }
  input, select {
    background: #0b0f17; color: var(--text); border: 1px solid var(--border);
    border-radius: 6px; padding: 6px 9px; font-size: 12.5px;
  }
  .events-list {
    max-height: 240px; overflow-y: auto; padding: 10px 12px;
    display: flex; flex-direction: column; gap: 8px;
  }
  .ev-item {
    display: flex; align-items: center; justify-content: space-between; gap: 10px;
    padding: 8px 10px; background: #0f141d; border: 1px solid var(--border); border-radius: 7px;
    font-size: 12px;
  }
  .ev-item img { width: 84px; height: 48px; object-fit: cover; border-radius: 4px; border: 1px solid var(--border); }
  code.raw {
    display: block; padding: 8px 12px; font-size: 11.5px; color: #93c5fd;
    background: #070a0f; border-top: 1px solid var(--border); overflow-x: auto; white-space: pre;
  }
</style>
</head>
<body>
<header>
  <div class="brand">
    <span>drone2 — AI Camera (IMX500) + Hailo-10H NPU + Uno Q &amp; Pico ToF Telemetry Dashboard</span>
  </div>
  <div class="pills">
    <span id="detPill" class="pill">Detection: Scanning</span>
    <span id="tofPill" class="pill">ToF: -- cm</span>
    <span id="gpsPill" class="pill">GPS: --</span>
    <span id="pwrPill" class="pill">5V Rail: --</span>
  </div>
</header>

<div class="layout">
  <!-- LEFT COLUMN: Large Live Camera + Detection Status + Captures -->
  <div>
    <div class="card">
      <div class="card-head">
        <span>Sony IMX500 Live Feed (1280x720) + Hailo-10H YOLOv11m Person Detection</span>
        <span id="npuSpeed" style="color:var(--muted);font-weight:500;">NPU: -- ms</span>
      </div>
      <div class="cam-box">
        <img id="liveStream" src="/stream.mjpg" alt="Live Camera Stream">
      </div>
      <div id="alertBanner" class="alert-banner idle">
        <span id="alertText">Monitoring ToF sensor &amp; AI Camera for person detection...</span>
        <span id="lastEmailBadge" style="font-size:12px;font-weight:500;"></span>
      </div>
      <div class="btn-row">
        <button onclick="manualCapture(true)">Capture &amp; Email Telemetry Now</button>
        <button class="sec" onclick="manualCapture(false)">Save Snapshot Only</button>
        <a href="/snap.jpg" target="_blank" style="margin-left:auto;color:var(--accent);font-size:12.5px;align-self:center;text-decoration:none;">Open Raw JPEG</a>
      </div>
    </div>

    <div class="card">
      <div class="card-head">
        <span>Captured Detection Snapshots &amp; Email Log</span>
        <span id="captureCount" style="color:var(--muted);">0 events</span>
      </div>
      <div id="eventsList" class="events-list">
        <div style="color:var(--muted);font-size:12.5px;">No detection snapshots yet. Step in front of the camera or ToF sensor, or click "Capture &amp; Email Telemetry Now".</div>
      </div>
    </div>
  </div>

  <!-- RIGHT COLUMN: All Telemetry Cards + Email Configuration -->
  <div>
    <!-- 1. Person Detection & Time / Location Summary -->
    <div class="card">
      <div class="card-head">
        <span>1. Time, Location &amp; Person Detection Telemetry</span>
        <span id="timeUtc" style="color:var(--muted);">--</span>
      </div>
      <div class="grid-2">
        <div class="kv"><div class="k">Pi Local Time</div><div class="v" id="tLocal">--</div></div>
        <div class="kv"><div class="k">GPS Atomic UTC</div><div class="v" id="tGpsUtc">--</div></div>
        <div class="kv"><div class="k">Camera Person Detection</div><div class="v" id="detCam">--</div></div>
        <div class="kv"><div class="k">ToF Proximity (VL53L0X)</div><div class="v" id="detTof">--</div></div>
        <div class="kv" style="grid-column: span 2;"><div class="k">All Detected Objects (Hailo-10H YOLOv11m)</div><div class="v" id="detObjects">None</div></div>
      </div>
    </div>

    <!-- 2. GPS Telemetry (GT-U7 on Uno Q D0) -->
    <div class="card">
      <div class="card-head">
        <span>2. GPS Telemetry — GT-U7 (Uno Q Serial1 / D0 @ 9600)</span>
        <span id="gpsLinkBadge" class="pill">--</span>
      </div>
      <div class="grid-2">
        <div class="kv"><div class="k">Latitude / Longitude</div><div class="v" id="gpsCoords">--</div></div>
        <div class="kv"><div class="k">Satellites (In Use / In View)</div><div class="v" id="gpsSats">--</div></div>
        <div class="kv"><div class="k">Peak Satellite SNR</div><div class="v" id="gpsSnr">--</div></div>
        <div class="kv"><div class="k">NMEA Checksums (OK / Bad)</div><div class="v" id="gpsNmeaStats">--</div></div>
      </div>
      <code class="raw" id="gpsRawNmea">Waiting for NMEA...</code>
    </div>

    <!-- 3. IMU + Barometer + Pico 1 VL53L0X ToF -->
    <div class="card">
      <div class="card-head">
        <span>3. IMU (LSM6DSOX + LIS3MDL), Barometer (BME280) &amp; Pico ToF (VL53L0X)</span>
        <span id="imuHealthBadge" class="pill">--</span>
      </div>
      <div class="grid-2">
        <div class="kv"><div class="k">IMU Accel X / Y / Z (m/s²)</div><div class="v" id="imuAcc">--</div></div>
        <div class="kv"><div class="k">IMU Sample Rates (Hz)</div><div class="v" id="imuRates">--</div></div>
        <div class="kv"><div class="k">Barometer Pressure &amp; Temp</div><div class="v" id="baroPT">--</div></div>
        <div class="kv"><div class="k">Baro Relative Altitude</div><div class="v" id="baroAlt">--</div></div>
        <div class="kv"><div class="k">Pico 1 VL53L0X ToF Distance</div><div class="v" id="tofDist">--</div></div>
        <div class="kv"><div class="k">Flight Controller Permissions</div><div class="v" id="ctrlPerms">--</div></div>
      </div>
    </div>

    <!-- 4. Physical Adafruit 4650 OLED Mirror & Pi 5 Power -->
    <div class="card">
      <div class="card-head">
        <span>4. Adafruit FeatherWing 4650 OLED (0x3C) Live Mirror &amp; Pi 5 Power</span>
        <span id="oledBadge" class="pill">--</span>
      </div>
      <div class="oled-screen" id="oledMirror">
        L1: --<br>L2: --<br>L3: --<br>L4: --
      </div>
      <div class="grid-2">
        <div class="kv"><div class="k">Pi 5 Input 5V Rail (EXT5V_V)</div><div class="v" id="pi5v">--</div></div>
        <div class="kv"><div class="k">Pi 5 Temp / Throttle / NPU</div><div class="v" id="piHealth">--</div></div>
      </div>
    </div>

    <!-- 5. Email Alert Configuration -->
    <div class="card">
      <div class="card-head">
        <span>5. Email Alert Settings</span>
        <span id="emailStatusShort" style="color:var(--muted);font-size:11.5px;">Ready</span>
      </div>
      <form class="form-grid" onsubmit="saveSettings(event)">
        <label>Alert recipients (comma-separated)
          <input type="text" id="cfgEmail" value="" required>
        </label>
        <label>SMTP app password
          <input type="password" id="cfgPassword" placeholder="SMTP app password">
        </label>
        <label>ToF Trigger Distance (cm)
          <input type="number" id="cfgTofCm" value="30" min="5" max="200">
        </label>
        <label>Alert Cooldown (seconds)
          <input type="number" id="cfgCooldown" value="100" min="5" max="3600">
        </label>
        <label>Trigger Rule
          <select id="cfgMode">
            <option value="either">Either ToF &lt;= Threshold OR Camera Person</option>
            <option value="both">Both ToF &lt;= Threshold AND Camera Person</option>
            <option value="camera_only">Camera Person Detection Only</option>
            <option value="tof_only">ToF Proximity Only</option>
          </select>
        </label>
        <label style="justify-content:flex-end;">
          <button type="submit" style="margin-top:auto;">Save Alert Settings</button>
        </label>
      </form>
    </div>
  </div>
</div>

<script>
let settingsLoaded = false;

async function pollDashboard() {
  try {
    const r = await fetch('/api/telemetry');
    const d = await r.json();
    const t = d.telemetry;
    const det = d.detection;
    const pwr = d.power;
    const cfg = d.config;

    if (!settingsLoaded && cfg) {
      document.getElementById('cfgEmail').value = cfg.recipient_email || '';
      document.getElementById('cfgPassword').value = cfg.has_password ? '********' : '';
      document.getElementById('cfgTofCm').value = cfg.tof_threshold_cm || 30;
      document.getElementById('cfgCooldown').value = cfg.cooldown_sec || 100;
      document.getElementById('cfgMode').value = cfg.trigger_mode || 'both';
      settingsLoaded = true;
    }

    // Header pills
    const detPill = document.getElementById('detPill');
    if (det.user_detected) {
      detPill.textContent = `USER DETECTED: ${det.trigger_reason}`;
      detPill.className = 'pill danger';
    } else {
      detPill.textContent = 'Detection: Clear';
      detPill.className = 'pill ok';
    }

    const tofPill = document.getElementById('tofPill');
    if (t.tof.distance_cm !== null) {
      tofPill.textContent = `ToF: ${t.tof.distance_cm} cm (${t.tof.distance_mm} mm)`;
      tofPill.className = det.tof_person_detected ? 'pill danger' : 'pill ok';
    } else {
      tofPill.textContent = 'ToF: -- cm';
      tofPill.className = 'pill warn';
    }

    const gpsPill = document.getElementById('gpsPill');
    gpsPill.textContent = `GPS: ${t.gps.lat_rounded}, ${t.gps.lon_rounded} (s${t.gps.sats_in_use}/${t.gps.sats_in_view})`;
    gpsPill.className = t.gps.fix === 'yes' ? 'pill ok' : 'pill warn';

    const pwrPill = document.getElementById('pwrPill');
    pwrPill.textContent = `5V: ${pwr.ext5v_v ? pwr.ext5v_v.toFixed(2) + 'V' : '--'} (${pwr.throttled_hex})`;
    pwrPill.className = pwr.undervolt_now ? 'pill danger' : 'pill ok';

    // Left banner
    document.getElementById('npuSpeed').textContent = `Hailo-10H YOLOv11m: ${det.npu_ms} ms`;
    const banner = document.getElementById('alertBanner');
    const alertText = document.getElementById('alertText');
    if (det.user_detected) {
      banner.className = 'alert-banner active';
      alertText.textContent = `USER DETECTED — ${det.trigger_reason}`;
    } else {
      banner.className = 'alert-banner idle';
      alertText.textContent = 'Monitoring ToF sensor & AI Camera for person detection...';
    }
    document.getElementById('lastEmailBadge').textContent = det.last_email_status || '';

    // Card 1: Time & Detection
    document.getElementById('tLocal').textContent = t.time.pi_local;
    document.getElementById('tGpsUtc').textContent = `${t.time.gps_utc_time} (${t.time.gps_utc_date})`;
    document.getElementById('timeUtc').textContent = t.time.pi_utc;
    document.getElementById('detCam').textContent = det.people.length > 0
      ? det.people.map(p => `PERSON ${Math.round(p.score*100)}% (${p.position})`).join(', ')
      : 'No person in frame';
    document.getElementById('detTof').textContent = t.tof.distance_cm !== null
      ? `${t.tof.distance_cm} cm / ${t.tof.distance_mm} mm (${det.tof_person_detected ? 'TRIGGERED' : 'Clear'})`
      : 'Waiting...';
    document.getElementById('detObjects').textContent = det.all_objects.length > 0
      ? det.all_objects.map(o => `${o.label} (${Math.round(o.score*100)}% @ ${o.position})`).join(', ')
      : 'None';

    // Card 2: GPS
    document.getElementById('gpsLinkBadge').textContent = `Link ${t.gps.link} | Fix ${t.gps.fix}`;
    document.getElementById('gpsLinkBadge').className = t.gps.fix === 'yes' ? 'pill ok' : 'pill warn';
    document.getElementById('gpsCoords').textContent = `${t.gps.lat_rounded}, ${t.gps.lon_rounded}`;
    document.getElementById('gpsSats').textContent = `${t.gps.sats_in_use} in use / ${t.gps.sats_in_view} in view`;
    document.getElementById('gpsSnr').textContent = `${t.gps.snr_db} dB-Hz`;
    document.getElementById('gpsNmeaStats').textContent = `${t.gps.ok_sentences} OK / ${t.gps.bad_sentences} bad (${t.gps.bytes} B)`;
    document.getElementById('gpsRawNmea').textContent = t.gps.raw_nmea || 'Waiting for NMEA...';

    // Card 3: IMU, Baro, ToF
    document.getElementById('imuHealthBadge').textContent = `Att ${t.imu.att} | Hdg ${t.imu.hdg}`;
    document.getElementById('imuHealthBadge').className = t.imu.att === 'ok' ? 'pill ok' : 'pill danger';
    document.getElementById('imuAcc').textContent = `X: ${t.imu.ax} | Y: ${t.imu.ay} | Z: ${t.imu.az}`;
    document.getElementById('imuRates').textContent = `Acc ${t.imu.acc_hz} | Gyr ${t.imu.gyr_hz} | Mag ${t.imu.mag_hz} Hz`;
    document.getElementById('baroPT').textContent = `${t.baro.pressure_hpa} hPa | ${t.baro.temp_c} °C`;
    document.getElementById('baroAlt').textContent = `${t.baro.altitude_m} m (Base ${t.baro.baseline})`;
    document.getElementById('tofDist').textContent = t.tof.distance_cm !== null
      ? `${t.tof.distance_cm} cm (${t.tof.distance_mm} mm)`
      : 'No sample';
    document.getElementById('ctrlPerms').textContent =
      `move:${t.ctrl.move} att:${t.ctrl.att} hdg:${t.ctrl.hdg} alt:${t.ctrl.alt} luna:${t.ctrl.luna}`;

    // Card 4: OLED & Pi Power
    document.getElementById('oledBadge').textContent = `OLED ${t.oled.address} ${t.oled.status}`;
    document.getElementById('oledBadge').className = t.oled.status === 'ok' ? 'pill ok' : 'pill warn';
    document.getElementById('oledMirror').innerHTML =
      `L1: ${t.oled.line1 || '--'}<br>L2: ${t.oled.line2 || '--'}<br>L3: ${t.oled.line3 || '--'}<br>L4: ${t.oled.line4 || '--'}`;
    document.getElementById('pi5v').textContent = `${pwr.ext5v_v || '--'} V (Core ${pwr.vdd_core_a || '--'} A)`;
    document.getElementById('piHealth').textContent = `${pwr.temp_c || '--'} °C | Throttle ${pwr.throttled_hex} | /dev/hailo0 OK`;

    // Events list
    const evList = document.getElementById('eventsList');
    document.getElementById('captureCount').textContent = `${det.events.length} events`;
    if (det.events.length > 0) {
      evList.innerHTML = det.events.map(ev => `
        <div class="ev-item">
          <a href="/captures/${ev.image_file}" target="_blank">
            <img src="/captures/${ev.image_file}" alt="snapshot">
          </a>
          <div style="flex:1;">
            <div style="font-weight:700;color:#f8fafc;">${ev.trigger_reason}</div>
            <div style="color:var(--muted);margin-top:2px;">${ev.timestamp} • ToF: ${ev.tof_cm !== null ? ev.tof_cm + 'cm' : '--'}</div>
            <div style="color:${ev.email_ok ? '#4ade80' : '#fbbf24'};margin-top:2px;">${ev.email_status}</div>
          </div>
        </div>
      `).join('');
    }
  } catch (e) {}
}

async function manualCapture(sendEmail) {
  const r = await fetch('/api/capture', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify({send_email: sendEmail, reason: 'Manual Dashboard Trigger'})
  });
  const d = await r.json();
  pollDashboard();
}

async function saveSettings(ev) {
  ev.preventDefault();
  const payload = {
    sender_email: document.getElementById('cfgEmail').value.trim(),
    recipient_email: document.getElementById('cfgEmail').value.trim(),
    app_password: document.getElementById('cfgPassword').value,
    tof_threshold_cm: parseInt(document.getElementById('cfgTofCm').value, 10) || 30,
    cooldown_sec: parseInt(document.getElementById('cfgCooldown').value, 10) || 45,
    trigger_mode: document.getElementById('cfgMode').value,
  };
  const r = await fetch('/api/settings', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(payload)
  });
  const d = await r.json();
  document.getElementById('emailStatusShort').textContent = d.ok ? 'Saved!' : 'Error';
}

pollDashboard();
setInterval(pollDashboard, 1000);
</script>
</body>
</html>
"""


class DashboardHandler(server.BaseHTTPRequestHandler):
    def _send_json(self, obj, status=200):
        raw = json.dumps(obj).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(raw)))
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        self.wfile.write(raw)

    def do_GET(self):
        parsed_path = urllib.parse.urlparse(self.path).path
        if parsed_path in ("/", "/index.html"):
            raw = PAGE_HTML.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)
        elif parsed_path == "/api/telemetry":
            cfg = load_config()
            with state_lock:
                det_copy = dict(detection_state)
            self._send_json(
                {
                    "ok": True,
                    "telemetry": parse_unoq_telemetry(),
                    "detection": det_copy,
                    "power": get_pi_power_telemetry(),
                    "config": {
                        "sender_email": cfg.get("sender_email"),
                        "recipient_email": cfg.get("recipient_email"),
                        "has_password": bool(cfg.get("app_password")),
                        "tof_threshold_cm": cfg.get("tof_threshold_cm"),
                        "cooldown_sec": cfg.get("cooldown_sec"),
                        "trigger_mode": cfg.get("trigger_mode"),
                    },
                }
            )
        elif parsed_path == "/stream.mjpg":
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
            self.end_headers()
            last_seq = -1
            try:
                while True:
                    with frame_store.condition:
                        while frame_store.frame_seq == last_seq:
                            frame_store.condition.wait(timeout=1.0)
                        frame = frame_store.jpeg_bytes
                        last_seq = frame_store.frame_seq
                    if not frame:
                        continue
                    self.wfile.write(b"--FRAME\r\n")
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(frame)))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
            except Exception:
                pass
        elif parsed_path == "/snap.jpg":
            with frame_store.condition:
                frame = frame_store.jpeg_bytes
            if not frame:
                self.send_error(503)
                return
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(frame)))
            self.end_headers()
            self.wfile.write(frame)
        elif parsed_path.startswith("/captures/"):
            fname = os.path.basename(parsed_path)
            fpath = CAPTURES_DIR / fname
            if not fpath.exists():
                self.send_error(404)
                return
            data = fpath.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)
        else:
            self.send_error(404)

    def do_POST(self):
        length = int(self.headers.get("Content-Length", "0"))
        body = json.loads(self.rfile.read(length).decode("utf-8")) if length > 0 else {}
        if self.path == "/api/settings":
            cfg = save_config(body)
            self._send_json({"ok": True, "has_password": bool(cfg.get("app_password"))})
        elif self.path == "/api/capture":
            reason = body.get("reason") or "Manual Capture"
            res = trigger_capture_and_email(reason, force=bool(body.get("send_email", True)))
            self._send_json(res)
        else:
            self.send_error(404)

    def log_message(self, fmt, *args):
        pass


class ThreadedHTTPServer(socketserver.ThreadingMixIn, server.HTTPServer):
    allow_reuse_address = True
    daemon_threads = True


def main():
    if "--once" in sys.argv:
        # Single-frame CLI check mode
        picam2 = Picamera2()
        cfg = picam2.create_still_configuration(main={"size": (1280, 720), "format": "RGB888"})
        picam2.configure(cfg)
        picam2.start()
        rgb = picam2.capture_array()
        picam2.stop()
        picam2.close()
        inp = cv2.resize(rgb, (640, 640), interpolation=cv2.INTER_LINEAR).astype(np.uint8)
        with VDevice() as vdevice:
            infer_model = vdevice.create_infer_model(HEF_PATH)
            infer_model.set_batch_size(1)
            with infer_model.configure() as configured_model:
                out_name = infer_model.output_names[0]
                out_buf = np.empty(infer_model.output(out_name).shape, dtype=np.float32)
                bindings = configured_model.create_bindings()
                bindings.input().set_buffer(inp)
                bindings.output(out_name).set_buffer(out_buf)
                configured_model.run([bindings], 5000)
        people, objs = parse_yolo_nms(out_buf, 0.4)
        print(f"people: {len(people)} | objects: {objs}")
        return 0

    worker = threading.Thread(target=camera_and_yolo_loop, daemon=True)
    worker.start()
    logging.info("Unified Camera + Telemetry Dashboard listening on http://0.0.0.0:%d", PORT)
    ThreadedHTTPServer(("0.0.0.0", PORT), DashboardHandler).serve_forever()
    return 0


if __name__ == "__main__":
    sys.exit(main())
