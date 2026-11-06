

import io
import json
import os
import tempfile
import threading
import time

from flask import Flask, Response, jsonify, request, render_template_string
from picamera2 import Picamera2
import libcamera

app = Flask(__name__)

picam2 = Picamera2()
config_lock = threading.Lock()

current_config = {
    "width": 640,
    "height": 480,
    "format": "RGB888",
}

manual_controls = {}

exposure_mode = {"name": "preview", "requested_exposure_us": None}
preview_controls = None

VALID_FORMATS = {"RGB888", "YUV420", "RGB565", "BGR888"}

def build_and_apply_config(width, height, fmt):

    cfg = picam2.create_video_configuration(
        main={"format": fmt, "size": (width, height)},
        raw={"size": (width, height)}
    )
    picam2.configure(cfg)

def start_camera():
    with config_lock:
        build_and_apply_config(
            current_config["width"], current_config["height"], current_config["format"]
        )
        picam2.start()
        if manual_controls:
            picam2.set_controls(manual_controls)

def reconfigure_camera(width, height, fmt):
    with config_lock:
        picam2.stop()
        build_and_apply_config(width, height, fmt)
        picam2.start()
        current_config.update({"width": width, "height": height, "format": fmt})

        if manual_controls:
            picam2.set_controls(manual_controls)

def mjpeg_generator():

    from PIL import Image

    while True:
        with config_lock:
            frame = picam2.capture_array()
            fmt = current_config["format"]

        if fmt in ("RGB888", "BGR888"):
            img = Image.fromarray(frame)
            if fmt == "BGR888":
                img = Image.fromarray(frame[:, :, ::-1])
        elif fmt == "YUV420":

            import cv2
            import numpy as np
            h = current_config["height"]
            w = current_config["width"]
            yuv = frame.reshape((h * 3 // 2, w))
            rgb = cv2.cvtColor(yuv, cv2.COLOR_YUV2RGB_I420)
            img = Image.fromarray(rgb)
        else:
            img = Image.fromarray(frame)

        buf = io.BytesIO()
        img.save(buf, format="JPEG", quality=80)
        jpg_bytes = buf.getvalue()

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" + jpg_bytes + b"\r\n"
        )

@app.route("/")
def index():
    return render_template_string(
        """
        <html>
          <head><title>Pi Camera</title></head>
          <body style="background:#111;color:#eee;font-family:sans-serif;text-align:center">
            <h2>Live Feed</h2>
            <img src="/video_feed" style="max-width:90%;border:2px solid #444"/>
            <p>Current config: <span id="cfg"></span></p>
            <script>
              fetch('/config').then(r => r.json()).then(d => {
                document.getElementById('cfg').innerText = JSON.stringify(d);
              });
            </script>
          </body>
        </html>
        """
    )

@app.route("/video_feed")
def video_feed():
    return Response(
        mjpeg_generator(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )

@app.route("/capture_raw", methods=["GET"])
def capture_raw():

    with config_lock:
        frame = picam2.capture_array("raw")
        stream_config = picam2.camera_configuration()["raw"]
        stride = stream_config["stride"]
        fmt = str(stream_config["format"])
        width = stream_config["size"][0]
        height = stream_config["size"][1]
        shape_str = ",".join(map(str, frame.shape))
        dtype_str = str(frame.dtype)

        raw_bytes = frame.tobytes()

    headers = {
        "X-Width": str(width),
        "X-Height": str(height),
        "X-Format": fmt,
        "X-Stride": str(stride),
        "X-Shape": shape_str,
        "X-Dtype": dtype_str,
        "Content-Type": "application/octet-stream",
        "Content-Disposition": "attachment; filename=frame.raw",
    }
    return Response(raw_bytes, headers=headers)

@app.route("/capture_dng", methods=["GET"])
def capture_dng():

    with config_lock:
        requested_us = exposure_mode["requested_exposure_us"]

        for _ in range(4):
            cam_request = picam2.capture_request()
            try:
                metadata = cam_request.get_metadata()
                actual_us = metadata.get("ExposureTime", 0)
                if requested_us is None or actual_us >= 0.9 * requested_us:
                    raw_buffer = cam_request.make_buffer("raw")
                    raw_stream_config = picam2.camera_configuration()["raw"]
                    break
            finally:
                cam_request.release()
        else:
            return jsonify({
                "error": "Camera did not reach the requested long exposure",
                "requested_exposure_us": requested_us,
                "last_actual_exposure_us": actual_us,
            }), 503

    fd, tmp_path = tempfile.mkstemp(suffix=".dng")
    os.close(fd)
    try:
        picam2.helpers.save_dng(raw_buffer, metadata, raw_stream_config, tmp_path)
        with open(tmp_path, "rb") as f:
            dng_bytes = f.read()
    finally:
        os.remove(tmp_path)

    headers = {
        "Content-Type": "image/x-adobe-dng",
        "Content-Disposition": "attachment; filename=frame.dng",
        "X-Exposure-Time-Us": str(metadata.get("ExposureTime", "")),
        "X-Frame-Duration-Us": str(metadata.get("FrameDuration", "")),
        "X-Sensor-Timestamp-Ns": str(metadata.get("SensorTimestamp", "")),
    }
    return Response(dng_bytes, headers=headers)

@app.route("/exposure_mode", methods=["GET", "POST"])
def set_exposure_mode():

    global preview_controls
    if request.method == "GET":
        return jsonify(exposure_mode)

    data = request.get_json(force=True, silent=True) or {}
    mode = data.get("mode")
    if mode not in ("long", "preview"):
        return jsonify({"error": "mode must be 'long' or 'preview'"}), 400

    with config_lock:
        try:
            if mode == "long":
                exposure_us = int(data.get("exposure_us", 1_000_000))
                gain = float(data.get("gain", 1.0))
                frame_us = exposure_us + 50_000
                if exposure_us <= 0 or gain <= 0:
                    return jsonify({"error": "exposure and gain must be positive"}), 400
                limits = picam2.camera_controls
                for name, value in (
                    ("ExposureTime", exposure_us),
                    ("FrameDurationLimits", frame_us),
                    ("AnalogueGain", gain),
                ):
                    minimum, maximum = limits[name][:2]
                    lower = min(minimum) if isinstance(minimum, (tuple, list)) else minimum
                    upper = max(maximum) if isinstance(maximum, (tuple, list)) else maximum
                    if not lower <= value <= upper:
                        return jsonify({"error": f"{name}={value} outside {lower}..{upper}"}), 400

                if exposure_mode["name"] != "long":
                    metadata = picam2.capture_metadata()
                    old_frame_us = int(metadata.get("FrameDuration", 33_333))
                    preview_controls = {
                        "FrameDurationLimits": (old_frame_us, old_frame_us),
                        "ExposureTime": int(metadata.get("ExposureTime", 30_000)),
                        "AnalogueGain": float(metadata.get("AnalogueGain", 1.0)),
                        "AeEnable": bool(manual_controls.get("AeEnable", False)),
                        "AwbEnable": bool(manual_controls.get("AwbEnable", True)),
                    }

                controls = {
                    "FrameDurationLimits": (frame_us, frame_us),
                    "ExposureTime": exposure_us,
                    "AnalogueGain": gain,
                    "AeEnable": False,
                    "AwbEnable": False,
                }
                picam2.set_controls(controls)
                manual_controls.update(controls)
                exposure_mode.update(name="long", requested_exposure_us=exposure_us)
            else:
                if preview_controls is None:
                    return jsonify({"error": "No previous preview settings to restore"}), 400
                picam2.set_controls(preview_controls)
                manual_controls.update(preview_controls)
                exposure_mode.update(name="preview", requested_exposure_us=None)
            return jsonify({"status": "applied", **exposure_mode})
        except Exception as error:
            return jsonify({"error": str(error)}), 500

@app.route("/controls", methods=["GET"])
def get_controls():

    with config_lock:
        controls_limits = picam2.camera_controls
        try:
            metadata = picam2.capture_metadata()
        except Exception:
            metadata = {}

    response_data = {}
    for name, limits in controls_limits.items():

        if name in manual_controls:
            curr_val = manual_controls[name]
        else:
            curr_val = metadata.get(name, limits[2] if len(limits) > 2 else None)
        try:
            min_val = limits[0]
            max_val = limits[1]
            def_val = limits[2] if len(limits) > 2 else None

            json.dumps(min_val)
            json.dumps(max_val)
            json.dumps(def_val)
            json.dumps(curr_val)

            response_data[name] = {
                "min": min_val,
                "max": max_val,
                "default": def_val,
                "current": curr_val,
            }
        except Exception:

            response_data[name] = {
                "min": str(limits[0]),
                "max": str(limits[1]),
                "default": str(limits[2]) if len(limits) > 2 else None,
                "current": str(curr_val) if curr_val is not None else None,
            }

    return jsonify(response_data)

@app.route("/controls", methods=["POST"])
def set_controls():

    data = request.get_json(force=True, silent=True)
    if not data:
        return jsonify({"error": "invalid or missing JSON body"}), 400

    print(f"Received controls request: {data}")
    with config_lock:
        try:
            controls_limits = picam2.camera_controls
            typed_data = {}
            for k, v in data.items():
                if k in controls_limits:
                    limit_val = controls_limits[k][2] if len(controls_limits[k]) > 2 else controls_limits[k][0]
                    if isinstance(limit_val, bool):
                        if isinstance(v, str):
                            typed_data[k] = v.lower() in ("true", "1")
                        else:
                            typed_data[k] = bool(v)
                    elif isinstance(limit_val, int):
                        typed_data[k] = int(v)
                    elif isinstance(limit_val, float):
                        typed_data[k] = float(v)
                    else:
                        typed_data[k] = v
                else:
                    typed_data[k] = v

            if "ExposureTime" in typed_data or "AnalogueGain" in typed_data:
                if "AeEnable" not in typed_data and "AeEnable" in controls_limits:
                    typed_data["AeEnable"] = False
                    print("Auto-disabling AeEnable for manual exposure/gain control")
            if "ColourGains" in typed_data:
                if "AwbEnable" not in typed_data and "AwbEnable" in controls_limits:
                    typed_data["AwbEnable"] = False
                    print("Auto-disabling AwbEnable for manual color gains control")

            print(f"Applying controls to picamera2: {typed_data}")
            picam2.set_controls(typed_data)
            manual_controls.update(typed_data)
            return jsonify({"status": "applied", "controls": {k: str(v) for k, v in typed_data.items()}})
        except Exception as e:
            print(f"Error setting controls: {e}")
            return jsonify({"error": str(e)}), 500

@app.route("/config", methods=["GET"])
def get_config():
    return jsonify(current_config)

@app.route("/config", methods=["POST"])
def set_config():
    data = request.get_json(force=True, silent=True)
    if not data:
        return jsonify({"error": "invalid or missing JSON body"}), 400

    width = data.get("width", current_config["width"])
    height = data.get("height", current_config["height"])
    fmt = data.get("format", current_config["format"])

    if fmt not in VALID_FORMATS:
        return jsonify({"error": f"format must be one of {sorted(VALID_FORMATS)}"}), 400
    if not (isinstance(width, int) and isinstance(height, int) and width > 0 and height > 0):
        return jsonify({"error": "width/height must be positive integers"}), 400

    try:
        reconfigure_camera(width, height, fmt)
    except Exception as e:
        return jsonify({"error": str(e)}), 500

    return jsonify({"status": "reconfigured", "config": current_config})

if __name__ == "__main__":
    start_camera()

    app.run(host="0.0.0.0", port=8000, threaded=True)
