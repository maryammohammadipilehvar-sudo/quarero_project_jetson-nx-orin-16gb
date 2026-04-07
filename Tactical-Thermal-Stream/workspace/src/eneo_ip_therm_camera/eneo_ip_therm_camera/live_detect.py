"""
Perimeter Robot — Live Person Detection for NVIDIA Jetson
==========================================================
Reads from AXIS F41 optical camera via RTSP.
Runs YOLOv8 person detection.
Streams annotated frames via MJPEG HTTP on port 8080
so the web app can display it like any other camera tab.

Usage:
    python3 live_detect.py                          # defaults
    python3 live_detect.py --confidence 0.4         # lower threshold
    python3 live_detect.py --no-track               # disable tracking
    python3 live_detect.py --export-engine          # export TensorRT (once)
    python3 live_detect.py --model yolov8n.engine   # use TensorRT engine

Requirements:
    python3 -m pip install ultralytics --break-system-packages --ignore-installed numpy
"""

import os
import cv2
import time
import json
import signal
import sys
import logging
import argparse
import threading
from datetime import datetime
from pathlib import Path
from collections import deque
from http.server import HTTPServer, BaseHTTPRequestHandler
from socketserver import ThreadingMixIn

# Fix Ultralytics config dir (container /root/.config may be read-only)
os.environ.setdefault('YOLO_CONFIG_DIR', '/tmp/Ultralytics')

# =====================================================================
# CONFIGURATION
# =====================================================================

PERSON_CLASS_ID = 0

DEFAULT_CONFIG = {
    # AXIS F41 RTSP stream
    "rtsp_url": (
        "rtsp://root:axis@192.168.10.174:554"
        "/axis-media/media.amp?camera=1&subtype=2&fps=10"
    ),

    # Model
    "model_path": "yolov8n.pt",
    "confidence_threshold": 0.5,
    "enable_tracking": True,

    # MJPEG web stream
    "mjpeg_port": 8080,
    "mjpeg_quality": 80,    # JPEG quality 1-100

    # Alerts
    "alert_cooldown_sec": 10,

    # Logging
    "save_detections": True,
    "detection_log_dir": "/tmp/detection_logs",
    "max_saved_frames": 500,
}


# =====================================================================
# MJPEG STREAMING SERVER
# =====================================================================

class MJPEGFrame:
    """Thread-safe latest-frame holder."""
    def __init__(self):
        self._frame = None
        self._lock = threading.Lock()
        self._event = threading.Event()

    def update(self, jpeg_bytes: bytes):
        with self._lock:
            self._frame = jpeg_bytes
        self._event.set()

    def get(self, timeout=1.0):
        self._event.wait(timeout)
        with self._lock:
            return self._frame


_mjpeg_frame = MJPEGFrame()


class MJPEGHandler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass  # Suppress HTTP access logs

    def do_GET(self):
        if self.path not in ('/', '/stream'):
            self.send_error(404)
            return

        self.send_response(200)
        self.send_header('Content-Type',
                         'multipart/x-mixed-replace; boundary=frame')
        self.send_header('Cache-Control', 'no-cache')
        self.send_header('Access-Control-Allow-Origin', '*')
        self.end_headers()

        try:
            while True:
                jpeg = _mjpeg_frame.get(timeout=2.0)
                if jpeg is None:
                    continue
                self.wfile.write(
                    b'--frame\r\n'
                    b'Content-Type: image/jpeg\r\n\r\n'
                    + jpeg + b'\r\n'
                )
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True


def start_mjpeg_server(port: int):
    """Start MJPEG server in background thread."""
    server = ThreadedHTTPServer(('0.0.0.0', port), MJPEGHandler)
    t = threading.Thread(target=server.serve_forever, daemon=True)
    t.start()
    logging.getLogger('mjpeg').info(
        f"MJPEG stream running at http://0.0.0.0:{port}/stream"
    )
    return server


# =====================================================================
# CAMERA SETUP
# =====================================================================

def setup_camera(rtsp_url: str):
    logger = logging.getLogger("camera")
    logger.info(f"Opening RTSP stream: {rtsp_url}")

    cap = cv2.VideoCapture(rtsp_url, cv2.CAP_FFMPEG)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    cap.set(cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 5000)

    if not cap.isOpened():
        raise RuntimeError(f"Failed to open RTSP stream: {rtsp_url}")

    w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    logger.info(f"Camera ready: {w}x{h}")
    return cap


# =====================================================================
# MODEL
# =====================================================================

def load_model(model_path: str):
    logger = logging.getLogger("model")
    logger.info(f"Loading model: {model_path}")

    from ultralytics import YOLO
    import numpy as np

    model = YOLO(model_path)

    # Warm up
    dummy = np.zeros((480, 640, 3), dtype=np.uint8)
    for _ in range(3):
        model(dummy, verbose=False)

    logger.info("Model loaded and warmed up.")
    return model


def export_tensorrt(model_path="yolov8n.pt", img_size=640):
    logger = logging.getLogger("model")
    logger.info("Exporting to TensorRT (5-15 min, done once)...")
    from ultralytics import YOLO
    model = YOLO(model_path)
    engine_path = model.export(format="engine", imgsz=img_size)
    logger.info(f"TensorRT engine saved: {engine_path}")
    return engine_path


# =====================================================================
# ALERT SYSTEM
# =====================================================================

class AlertSystem:
    def __init__(self, config):
        self.cooldown = config["alert_cooldown_sec"]
        self.last_alert_time = {}
        self.logger = logging.getLogger("alerts")

    def should_alert(self, track_id):
        now = time.time()
        key = track_id if track_id is not None else "unknown"
        if now - self.last_alert_time.get(key, 0) < self.cooldown:
            return False
        self.last_alert_time[key] = now
        return True

    def fire(self, detections):
        new = [d for d in detections if self.should_alert(d.get("track_id"))]
        if new:
            self.logger.warning(
                f"⚠️  PERSON DETECTED: {len(new)} person(s) in frame!"
            )


# =====================================================================
# DETECTION LOGGER
# =====================================================================

class DetectionLogger:
    def __init__(self, config):
        self.logger = logging.getLogger("det_log")
        self.log_dir = Path(config["detection_log_dir"])
        self.log_dir.mkdir(parents=True, exist_ok=True)
        self.frame_count = 0
        self.max_frames = config["max_saved_frames"]
        self.save = config["save_detections"]
        self.log_file = self.log_dir / f"detections_{datetime.now():%Y%m%d}.jsonl"
        self.logger.info(f"Detection log: {self.log_file}")

    def log(self, detections, frame=None):
        if not detections:
            return
        with open(self.log_file, "a") as f:
            f.write(json.dumps({
                "timestamp": datetime.now().isoformat(),
                "count": len(detections),
                "detections": detections,
            }) + "\n")
        if frame is not None and self.save and self.frame_count < self.max_frames:
            ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            cv2.imwrite(str(self.log_dir / f"det_{ts}.jpg"), frame,
                        [cv2.IMWRITE_JPEG_QUALITY, 80])
            self.frame_count += 1


# =====================================================================
# FPS TRACKER
# =====================================================================

class FPSTracker:
    def __init__(self, window=60):
        self.frame_times = deque(maxlen=window)
        self.inference_times = deque(maxlen=window)

    def tick(self, inference_ms=None):
        self.frame_times.append(time.time())
        if inference_ms is not None:
            self.inference_times.append(inference_ms)

    @property
    def fps(self):
        if len(self.frame_times) < 2:
            return 0.0
        elapsed = self.frame_times[-1] - self.frame_times[0]
        return (len(self.frame_times) - 1) / elapsed if elapsed > 0 else 0.0

    @property
    def avg_inference_ms(self):
        return (sum(self.inference_times) / len(self.inference_times)
                if self.inference_times else 0.0)


# =====================================================================
# MAIN DETECTION LOOP
# =====================================================================

def run_detection(config):
    logger = logging.getLogger("main")

    camera     = setup_camera(config["rtsp_url"])
    model      = load_model(config["model_path"])
    alerts     = AlertSystem(config)
    det_logger = DetectionLogger(config)
    fps_tracker = FPSTracker()

    start_mjpeg_server(config["mjpeg_port"])

    running = True
    def _stop(sig, frame):
        nonlocal running
        logger.info("Shutdown signal received.")
        running = False

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    logger.info("=" * 55)
    logger.info("  PERSON DETECTION STARTED")
    logger.info(f"  Model:      {config['model_path']}")
    logger.info(f"  RTSP:       {config['rtsp_url']}")
    logger.info(f"  Tracking:   {config['enable_tracking']}")
    logger.info(f"  Confidence: {config['confidence_threshold']}")
    logger.info(f"  MJPEG:      http://0.0.0.0:{config['mjpeg_port']}/stream")
    logger.info("=" * 55)

    total_frames = 0
    total_detections = 0

    try:
        while running:
            # Flush buffer — grab without decoding to get newest frame
            for _ in range(3):
                if not camera.grab():
                    break
            ret, frame = camera.retrieve()
            if not ret or frame is None:
                ret, frame = camera.read()
            if not ret or frame is None:
                logger.warning("Failed to read frame, retrying...")
                time.sleep(0.1)
                continue

            total_frames += 1
            t_start = time.time()

            # Run inference
            if config["enable_tracking"]:
                results = model.track(
                    frame, persist=True,
                    classes=[PERSON_CLASS_ID],
                    conf=config["confidence_threshold"],
                    verbose=False
                )
            else:
                results = model(
                    frame,
                    classes=[PERSON_CLASS_ID],
                    conf=config["confidence_threshold"],
                    verbose=False
                )

            inference_ms = (time.time() - t_start) * 1000
            fps_tracker.tick(inference_ms)

            # Parse detections
            detections = []
            for result in results:
                if result.boxes is None:
                    continue
                for box in result.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    conf = float(box.conf[0])
                    track_id = (int(box.id[0])
                                if config["enable_tracking"] and box.id is not None
                                else None)
                    detections.append({
                        "bbox": [x1, y1, x2, y2],
                        "confidence": round(conf, 3),
                        "track_id": track_id,
                    })

            if detections:
                total_detections += len(detections)
                alerts.fire(detections)
                det_logger.log(detections, frame)

            # Draw annotations
            annotated = frame.copy()
            for det in detections:
                x1, y1, x2, y2 = det["bbox"]
                conf = det["confidence"]
                tid  = det["track_id"]
                cv2.rectangle(annotated, (x1, y1), (x2, y2), (0, 0, 255), 2)
                label = f"ID:{tid} {conf:.0%}" if tid is not None else f"Person {conf:.0%}"
                lsz = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 2)[0]
                cv2.rectangle(annotated,
                              (x1, y1 - lsz[1] - 10), (x1 + lsz[0], y1),
                              (0, 0, 255), -1)
                cv2.putText(annotated, label, (x1, y1 - 5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

            # HUD overlay
            status_color = (0, 0, 255) if detections else (0, 200, 0)
            status_text  = f"⚠ PERSONS: {len(detections)}" if detections else "No persons"
            hud = [
                status_text,
                f"FPS: {fps_tracker.fps:.1f}",
                f"Inference: {fps_tracker.avg_inference_ms:.0f}ms",
                f"Total detections: {total_detections}",
            ]
            for i, line in enumerate(hud):
                color = status_color if i == 0 else (0, 200, 0)
                cv2.putText(annotated, line, (10, 30 + i * 28),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

            # Push to MJPEG stream
            ok, jpeg = cv2.imencode(
                '.jpg', annotated,
                [cv2.IMWRITE_JPEG_QUALITY, config["mjpeg_quality"]]
            )
            if ok:
                _mjpeg_frame.update(jpeg.tobytes())

            # Periodic console status
            if total_frames % 100 == 0:
                logger.info(
                    f"{fps_tracker.fps:.1f} FPS | "
                    f"{fps_tracker.avg_inference_ms:.0f}ms | "
                    f"detections: {total_detections}"
                )

    except Exception as e:
        logger.error(f"Detection loop error: {e}", exc_info=True)
    finally:
        logger.info("Shutting down...")
        camera.release()
        logger.info(
            f"Session: {total_frames} frames, {total_detections} detections"
        )


# =====================================================================
# CLI
# =====================================================================

def parse_args():
    parser = argparse.ArgumentParser(
        description="Person Detection — AXIS F41 + YOLOv8 + MJPEG web stream"
    )
    parser.add_argument('--rtsp-url', type=str,
                        default=DEFAULT_CONFIG["rtsp_url"],
                        help="RTSP URL (default: AXIS F41 optical ch1)")
    parser.add_argument('--model', type=str, default='yolov8n.pt')
    parser.add_argument('--confidence', type=float, default=0.5)
    parser.add_argument('--no-track', action='store_true',
                        help='Disable object tracking')
    parser.add_argument('--mjpeg-port', type=int, default=8080,
                        help='MJPEG HTTP server port (default: 8080)')
    parser.add_argument('--export-engine', action='store_true',
                        help='Export YOLOv8 to TensorRT and exit')
    return parser.parse_args()


def main():
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(name)s] %(levelname)s: %(message)s",
        handlers=[
            logging.StreamHandler(),
            logging.FileHandler("/tmp/live_detect.log"),
        ]
    )

    args = parse_args()

    if args.export_engine:
        export_tensorrt(args.model)
        return

    config = DEFAULT_CONFIG.copy()
    config.update({
        "rtsp_url":             args.rtsp_url,
        "model_path":           args.model,
        "confidence_threshold": args.confidence,
        "enable_tracking":      not args.no_track,
        "mjpeg_port":           args.mjpeg_port,
    })

    run_detection(config)


if __name__ == "__main__":
    main()