#!/usr/bin/env python3
"""
test_image_pub.py — Guaranteed Detection Publisher
===================================================
Publishes a real-world image to /camera/image_raw at 30 Hz.

Use this when:
  • Gazebo is not running (fast demo without full simulation)
  • Gazebo's rendered images aren't triggering YOLO detections
    (sim images are too "clean" for a model trained on real photos)

The image contains everyday COCO objects (bottle, cup, person, etc.)
so YOLOv8n will always return detections with score > 0.4.

Usage:
  python3 ~/ros2_ws/src/my_robot_controller/scripts/test_image_pub.py

Then in another terminal:
  ros2 topic echo /detected_objects    ← shows bounding boxes
  ros2 topic hz  /detected_objects    ← shows FPS
"""

import sys
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image

# ── Test image — a well-known COCO-style image with detectable objects ────────
# This is a royalty-free photo from a public domain source containing people
# and everyday objects. YOLOv8n (COCO-trained) reliably detects objects in it.
_TEST_IMAGE_URL = (
    "https://ultralytics.com/images/bus.jpg"  # Ultralytics' own test image
)
_TEST_IMAGE_PATH = Path.home() / ".ros" / "test_detection_image.jpg"

PUBLISH_HZ = 30.0


def _ensure_test_image() -> np.ndarray:
    """Download the test image once; reload from disk on subsequent runs."""
    if not _TEST_IMAGE_PATH.exists():
        print(f"Downloading test image → {_TEST_IMAGE_PATH} …")
        _TEST_IMAGE_PATH.parent.mkdir(parents=True, exist_ok=True)
        try:
            urllib.request.urlretrieve(_TEST_IMAGE_URL, str(_TEST_IMAGE_PATH))
            print("Download complete.")
        except Exception as exc:
            print(f"Download failed: {exc}")
            print("Using a generated checkerboard image instead.")
            # Checkerboard — YOLO won't detect objects but the pipeline still runs
            img = np.zeros((480, 640, 3), dtype=np.uint8)
            for r in range(0, 480, 40):
                for c in range(0, 640, 40):
                    if (r // 40 + c // 40) % 2 == 0:
                        img[r:r+40, c:c+40] = (200, 200, 200)
            return img

    img = cv2.imread(str(_TEST_IMAGE_PATH))
    if img is None:
        raise RuntimeError(f"Could not load {_TEST_IMAGE_PATH}")
    img = cv2.resize(img, (640, 480), interpolation=cv2.INTER_LINEAR)
    print(f"Loaded test image: {_TEST_IMAGE_PATH} → resized to 640×480")
    return img


class TestImagePublisher(Node):
    def __init__(self, image: np.ndarray) -> None:
        super().__init__("test_image_publisher")
        self._img = image
        self._bridge = CvBridge()

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pub = self.create_publisher(Image, "/camera/image_raw", qos)
        self._timer = self.create_timer(1.0 / PUBLISH_HZ, self._publish)
        self._count = 0
        self.get_logger().info(
            f"Publishing test image to /camera/image_raw at {PUBLISH_HZ:.0f} Hz"
        )

    def _publish(self) -> None:
        msg = self._bridge.cv2_to_imgmsg(self._img, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"
        self._pub.publish(msg)
        self._count += 1
        if self._count % int(PUBLISH_HZ * 5) == 0:          # log every 5 s
            self.get_logger().info(f"Published {self._count} frames")


def main() -> None:
    rclpy.init()
    try:
        img = _ensure_test_image()
    except Exception as exc:
        print(f"ERROR: {exc}")
        sys.exit(1)

    node = TestImagePublisher(img)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
