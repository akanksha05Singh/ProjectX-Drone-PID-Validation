#!/usr/bin/env python3
"""
Week 2 Perception Node — Reformerz Healthcare Robotics Internship
=================================================================
Subscribes : /camera/image_raw  (sensor_msgs/Image)
Publishes  : /detected_objects  (vision_msgs/Detection2DArray)

Performance targets (Intel CPU, ONNX Runtime backend):
  • 15–30 FPS on /detected_objects
  • < 80 ms end-to-end latency
  • No memory leak over 30-min run

Architecture:
  • Frame skipping  — process every Nth frame; skip the rest (zero-copy)
  • Async inference — ThreadPoolExecutor keeps ROS2 callback non-blocking
  • ONNX export     — yolov8n converted once to ONNX at 160×160, lower
                      overhead than OpenVINO in WSL2 CPU-only environment
"""

import os
import queue
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

# Limit ORT to 4 threads — avoids over-subscription on 12-thread i5-1235U
os.environ.setdefault("OMP_NUM_THREADS", "4")

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image
from vision_msgs.msg import (
    BoundingBox2D,
    Detection2D,
    Detection2DArray,
    ObjectHypothesisWithPose,
)

# ---------------------------------------------------------------------------
# Model paths — ONNX format, 128×128 input (fastest on WSL2 CPU)
# ---------------------------------------------------------------------------
_MODEL_PATH = Path.home() / ".ros" / "yolov8n_128.onnx"
_PT_PATH    = Path.home() / ".ros" / "yolov8n.pt"

# ---------------------------------------------------------------------------
# COCO class names (80 classes — YOLOv8 default)
# ---------------------------------------------------------------------------
COCO_NAMES: dict[int, str] = {
    0: "person",        1: "bicycle",       2: "car",           3: "motorcycle",
    4: "airplane",      5: "bus",           6: "train",         7: "truck",
    8: "boat",          9: "traffic light", 10: "fire hydrant", 11: "stop sign",
    12: "parking meter",13: "bench",        14: "bird",         15: "cat",
    16: "dog",          17: "horse",        18: "sheep",        19: "cow",
    20: "elephant",     21: "bear",         22: "zebra",        23: "giraffe",
    24: "backpack",     25: "umbrella",     26: "handbag",      27: "tie",
    28: "suitcase",     29: "frisbee",      30: "skis",         31: "snowboard",
    32: "sports ball",  33: "kite",         34: "baseball bat", 35: "baseball glove",
    36: "skateboard",   37: "surfboard",    38: "tennis racket",39: "bottle",
    40: "wine glass",   41: "cup",          42: "fork",         43: "knife",
    44: "spoon",        45: "bowl",         46: "banana",       47: "apple",
    48: "sandwich",     49: "orange",       50: "broccoli",     51: "carrot",
    52: "hot dog",      53: "pizza",        54: "donut",        55: "cake",
    56: "chair",        57: "couch",        58: "potted plant", 59: "bed",
    60: "dining table", 61: "toilet",       62: "tv",           63: "laptop",
    64: "mouse",        65: "remote",       66: "keyboard",     67: "cell phone",
    68: "microwave",    69: "oven",         70: "toaster",      71: "sink",
    72: "refrigerator", 73: "book",         74: "clock",        75: "vase",
    76: "scissors",     77: "teddy bear",   78: "hair drier",   79: "toothbrush",
}


# ---------------------------------------------------------------------------
# QoS helpers
# ---------------------------------------------------------------------------
def _sensor_qos(depth: int = 1) -> QoSProfile:
    """Best-effort, keep-last-1 — matches Gazebo / test_image_pub publisher."""
    return QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
    )


def _reliable_qos(depth: int = 10) -> QoSProfile:
    return QoSProfile(
        reliability=QoSReliabilityPolicy.RELIABLE,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
    )


# ---------------------------------------------------------------------------
# Perception Node
# ---------------------------------------------------------------------------
class PerceptionNode(Node):
    """YOLOv8n + ONNX Runtime perception node (single-worker, 160×160)."""

    def __init__(self) -> None:
        super().__init__("perception_node")

        # ── ROS2 parameters (tunable at launch without recompiling) ─────────
        self.declare_parameter("frame_skip",      1)
        self.declare_parameter("input_width",   128)   # ONNX export size
        self.declare_parameter("input_height",  128)
        self.declare_parameter("conf_threshold", 0.40)
        self.declare_parameter("camera_topic",  "/camera/image_raw")
        self.declare_parameter("output_topic",  "/detected_objects")
        self.declare_parameter("num_workers",     1)   # single worker avoids CPU contention

        # ── Stress Parameters (Week 4) ──────────────────────────────────────
        self.declare_parameter("camera_latency_ms", 0) # 50-150ms random delay

        self._frame_skip: int  = self.get_parameter("frame_skip").value
        self._w: int           = self.get_parameter("input_width").value
        self._h: int           = self.get_parameter("input_height").value
        self._conf: float      = self.get_parameter("conf_threshold").value
        cam_topic: str         = self.get_parameter("camera_topic").value
        out_topic: str         = self.get_parameter("output_topic").value
        num_workers: int       = self.get_parameter("num_workers").value

        # ── Model pool — one YOLO instance per worker (thread-safe) ─────────
        self.get_logger().info(f"Loading {num_workers} model instance(s)…")
        self._model_pool: queue.Queue = queue.Queue()
        for i in range(num_workers):
            label = "primary" if i == 0 else f"instance {i + 1}/{num_workers}"
            self.get_logger().info(f"  Loading {label}…")
            model = self._load_model(warmup=(i == 0))
            self._model_pool.put(model)

        # ── ROS2 I/O ────────────────────────────────────────────────────────
        self._bridge = CvBridge()
        self._pub = self.create_publisher(Detection2DArray, out_topic, _reliable_qos())
        self._sub = self.create_subscription(
            Image, cam_topic, self._image_callback, _sensor_qos()
        )

        # ── Thread pool ───────────────────────────────────────────────────────
        self._executor = ThreadPoolExecutor(max_workers=num_workers)

        # ── Frame counter ─────────────────────────────────────────────────────
        self._frame_count: int = 0

        # ── FPS / latency diagnostics ─────────────────────────────────────────
        self._fps_counter: int = 0
        self._fps_t0: float = time.monotonic()
        self._latency_window: deque[float] = deque(maxlen=50)
        self._fps_baseline: float | None = None
        self._fps_baseline_set_at: float = time.monotonic() + 15.0
        self.create_timer(5.0, self._log_fps)

        self.get_logger().info(
            f"PerceptionNode ready — model={_MODEL_PATH.name}, "
            f"skip=1/{self._frame_skip}, res={self._w}x{self._h}, "
            f"conf≥{self._conf}"
        )

    # ── Model loading ────────────────────────────────────────────────────────

    def _load_model(self, warmup: bool = True):
        """
        Load YOLOv8n with ONNX Runtime backend.

        Flow:
          1. If ONNX file exists → load directly (fast, ~50 ms per inference)
          2. Else if .pt exists  → export to ONNX at self._w × self._h
          3. Else                → download .pt, export, load
        """
        from ultralytics import YOLO

        if _MODEL_PATH.exists():
            self.get_logger().info(f"Loading ONNX model from {_MODEL_PATH}")
            model = YOLO(str(_MODEL_PATH))
        else:
            if not _PT_PATH.exists():
                self.get_logger().info("Downloading yolov8n.pt …")
                _PT_PATH.parent.mkdir(parents=True, exist_ok=True)
                model = YOLO("yolov8n.pt")   # downloads to cwd
                import shutil
                # ultralytics downloads to cwd; move to ~/.ros/
                cwd_pt = Path("yolov8n.pt")
                if cwd_pt.exists():
                    shutil.move(str(cwd_pt), str(_PT_PATH))

            self.get_logger().info(
                f"Exporting yolov8n.pt → ONNX at {self._w}×{self._h} "
                f"(one-time, ~20 s) …"
            )
            model = YOLO(str(_PT_PATH))
            model.export(format="onnx", half=False, dynamic=False,
                         imgsz=self._w)
            # ultralytics saves <name>.onnx beside the .pt file
            exported = _PT_PATH.with_suffix(".onnx")
            if not exported.exists():
                # fallback: cwd
                exported = Path("yolov8n.onnx")
            if exported.exists():
                import shutil
                shutil.move(str(exported), str(_MODEL_PATH))
            else:
                self.get_logger().error(
                    "ONNX export did not produce expected file — "
                    "falling back to PyTorch weights"
                )
                model = YOLO(str(_PT_PATH))
                return model

            model = YOLO(str(_MODEL_PATH))

        if not warmup:
            return model

        # ── Timed warm-up: confirms backend and measures baseline latency ─────
        self.get_logger().info("Running timed warm-up inference …")
        dummy = np.zeros((self._h, self._w, 3), dtype=np.uint8)
        times_ms = []
        for _ in range(3):
            t0 = time.monotonic()
            model.predict(dummy, verbose=False, conf=self._conf, imgsz=self._w)
            times_ms.append((time.monotonic() - t0) * 1000.0)
        median_ms = sorted(times_ms)[1]

        model_file = str(_MODEL_PATH) if _MODEL_PATH.exists() else str(_PT_PATH)
        backend = "ONNX-Runtime" if model_file.endswith(".onnx") else "PyTorch-CPU"

        if median_ms <= 60.0:
            status = f"✓ GOOD ({median_ms:.1f} ms)"
        elif median_ms <= 80.0:
            status = f"⚠ MARGINAL ({median_ms:.1f} ms)"
        else:
            status = f"✗ SLOW ({median_ms:.1f} ms — above 80 ms target)"

        self.get_logger().info(
            f"\n"
            f"  ╔══════════════════════════════════════════╗\n"
            f"  ║  Backend  : {backend:<29}║\n"
            f"  ║  Latency  : {status:<29}║\n"
            f"  ║  Model    : {Path(model_file).name:<29}║\n"
            f"  ╚══════════════════════════════════════════╝"
        )
        return model

    # ── ROS2 image callback (non-blocking) ───────────────────────────────────

    def _image_callback(self, msg: Image) -> None:
        self._frame_count += 1
        if self._frame_count % self._frame_skip != 0:
            return

        try:
            model = self._model_pool.get_nowait()
        except queue.Empty:
            return  # all workers busy; drop frame

        self._executor.submit(self._run_inference, model, msg)

    # ── Inference (runs in ThreadPoolExecutor) ───────────────────────────────

    def _run_inference(self, model, msg: Image) -> None:
        t_recv = time.monotonic()

        # ── Stress Injection: Camera Latency ────────────────────────────────
        latency_stress = self.get_parameter("camera_latency_ms").value
        if latency_stress > 0:
            import random
            # Prompt suggests random 50-150ms
            delay = random.uniform(0.050, 0.150)
            time.sleep(delay)

        try:
            # 1. Convert ROS2 Image → OpenCV BGR
            try:
                cv_img = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            except Exception as exc:
                self.get_logger().warning(f"cv_bridge error: {exc}")
                return

            # 2. Resize to target resolution
            if cv_img.shape[1] != self._w or cv_img.shape[0] != self._h:
                cv_img = cv2.resize(cv_img, (self._w, self._h),
                                    interpolation=cv2.INTER_LINEAR)

            # 3. YOLO inference
            results = model.predict(
                cv_img,
                verbose=False,
                conf=self._conf,
                imgsz=self._w,
            )

            # 4. Build Detection2DArray message
            det_array = Detection2DArray()
            det_array.header.stamp = msg.header.stamp
            det_array.header.frame_id = msg.header.frame_id

            label_summary: list[str] = []

            if results and results[0].boxes is not None:
                boxes = results[0].boxes
                for i in range(len(boxes)):
                    det = Detection2D()
                    det.header = det_array.header

                    x1, y1, x2, y2 = boxes.xyxy[i].tolist()
                    cx = (x1 + x2) / 2.0
                    cy = (y1 + y2) / 2.0

                    bbox = BoundingBox2D()
                    bbox.center.position.x = cx
                    bbox.center.position.y = cy
                    bbox.center.theta = 0.0
                    bbox.size_x = x2 - x1
                    bbox.size_y = y2 - y1
                    det.bbox = bbox

                    cls_id  = int(boxes.cls[i].item())
                    score   = float(boxes.conf[i].item())
                    cls_name = COCO_NAMES.get(cls_id, str(cls_id))

                    hyp = ObjectHypothesisWithPose()
                    hyp.hypothesis.class_id = cls_name
                    hyp.hypothesis.score    = score
                    det.results.append(hyp)

                    det_array.detections.append(det)
                    label_summary.append(f"{cls_name}:{score:.2f}")

            if label_summary:
                self.get_logger().info(
                    f"[DETECT] {len(label_summary)} object(s): {', '.join(label_summary)}"
                )

            # 5. Publish
            self._pub.publish(det_array)
            self._fps_counter += 1

            # 6. Latency tracking
            latency_ms = (time.monotonic() - t_recv) * 1000.0
            self._latency_window.append(latency_ms)
            if latency_ms > 80.0:
                self.get_logger().warning(
                    f"[LATENCY] {latency_ms:.1f} ms — above 80 ms target"
                )

        finally:
            self._model_pool.put(model)

    # ── FPS / thermal diagnostics timer ──────────────────────────────────────

    def _log_fps(self) -> None:
        now = time.monotonic()
        elapsed = now - self._fps_t0
        if elapsed <= 0:
            return

        fps = self._fps_counter / elapsed
        lat_avg = (
            sum(self._latency_window) / len(self._latency_window)
            if self._latency_window else float("nan")
        )

        if self._fps_baseline is None and now >= self._fps_baseline_set_at and fps > 0:
            self._fps_baseline = fps
            self.get_logger().info(f"[PERF] Baseline FPS established: {fps:.1f} Hz")

        throttle_warn = ""
        if self._fps_baseline is not None and fps < self._fps_baseline * 0.80:
            drop_pct = (1.0 - fps / self._fps_baseline) * 100.0
            throttle_warn = (
                f"  *** THERMAL THROTTLE: FPS dropped {drop_pct:.0f}% "
                f"from baseline {self._fps_baseline:.1f} Hz ***"
            )

        fps_status = "OK" if 15.0 <= fps <= 30.0 else "OUT-OF-RANGE"
        self.get_logger().info(
            f"[PERF] FPS={fps:.1f} [{fps_status}]  "
            f"lat_avg={lat_avg:.1f}ms  "
            f"frames={self._fps_counter}"
            + throttle_warn
        )

        self._fps_counter = 0
        self._fps_t0 = now

    # ── Clean shutdown ────────────────────────────────────────────────────────

    def destroy_node(self) -> None:
        self._executor.shutdown(wait=False)
        super().destroy_node()


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------
def main(args=None) -> None:
    rclpy.init(args=args)
    node = PerceptionNode()
    exe = MultiThreadedExecutor(num_threads=4)
    exe.add_node(node)
    try:
        exe.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
