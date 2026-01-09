"""
aruco_center_controller.py

UDP multicast + robust ArUco detection
Supports DICT_4X4_50 (ID=0 and others)
Detects ALL markers, uses the largest for control
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import cv2
import numpy as np
import subprocess
import time
import os
import signal

# ---------------- CONFIG ----------------
UDP_ADDR = "239.0.0.1"
CAM_PORT = 5000

WIDTH = 640
HEIGHT = 480
FPS = 20

# ?? CORRECT DICTIONARY
ARUCO_DICT = cv2.aruco.DICT_4X4_50

MARKER_REAL_SIZE = 295.0  # mm (used only for coarse distance estimate)

TARGET_DISTANCE_CM = 80.0
DEADZONE_PIXELS = 10

KP_ANG = 0.8
KP_LIN = 0.02
MAX_ANG = 0.8
MAX_LIN = 0.35

PUBLISH_RATE_HZ = 10.0
FRAME_SKIP = 0   # keep 0 until everything is stable

# ---------------- Distance model ----------------
def calibrated_model(x):
    return -3.366e-06 * (x ** 2) + 0.08133 * x + 106.3


# ---------------- ROS Node ----------------
class ArucoCenterController(Node):
    def __init__(self):
        super().__init__("aruco_center_controller")

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.get_logger().info("Starting rpicam-vid (UDP multicast)")
        self.cam_proc = subprocess.Popen([
            "rpicam-vid",
            "--width", str(WIDTH),
            "--height", str(HEIGHT),
            "--framerate", str(FPS),
            "--codec", "mjpeg",
            "--inline",
            "--nopreview",
            "--timeout", "0",
            "-o", f"udp://{UDP_ADDR}:{CAM_PORT}"
        ], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        time.sleep(1.0)

        self.cap = cv2.VideoCapture(
            f"udp://{UDP_ADDR}:{CAM_PORT}",
            cv2.CAP_FFMPEG
        )

        if not self.cap.isOpened():
            raise RuntimeError("Failed to open UDP stream")

        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.get_logger().info("UDP stream connected")

        # ArUco
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
        self.aruco_params = cv2.aruco.DetectorParameters_create()

        self.timer = self.create_timer(
            1.0 / PUBLISH_RATE_HZ,
            self.timer_cb
        )

    def timer_cb(self):
        for _ in range(FRAME_SKIP):
            self.cap.grab()

        ret, frame = self.cap.read()
        if not ret:
            return

        h, w = frame.shape[:2]
        cx_frame = w / 2.0

        # Robust grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)

        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params
        )

        twist = Twist()

        if ids is None or len(ids) == 0:
            self.cmd_pub.publish(twist)
            return

        # ---- LOG ALL DETECTIONS ----
        ids_flat = ids.flatten().tolist()
        self.get_logger().info(f"Detected markers: {ids_flat}")

        best = None

        for i, marker_id in enumerate(ids_flat):
            pts = corners[i].reshape(-1, 2)
            area = cv2.contourArea(np.int32(pts))
            if area <= 0:
                continue

            cx = float(np.mean(pts[:, 0]))
            x_metric = (MARKER_REAL_SIZE ** 2) / area * 100.0

            self.get_logger().debug(
                f"ID={marker_id} area={area:.1f} cx={cx:.1f}"
            )

            if best is None or area > best["area"]:
                best = {
                    "id": marker_id,
                    "area": area,
                    "cx": cx,
                    "x_metric": x_metric
                }

        if best is None:
            self.cmd_pub.publish(twist)
            return

        # ---- CONTROL USING LARGEST MARKER ----
        err_px = best["cx"] - cx_frame
        norm_err = err_px / cx_frame

        if abs(err_px) > DEADZONE_PIXELS:
            twist.angular.z = -KP_ANG * norm_err
            twist.angular.z = np.clip(
                twist.angular.z, -MAX_ANG, MAX_ANG
            )

        est_dist = calibrated_model(best["x_metric"])
        dist_err = est_dist - TARGET_DISTANCE_CM

        lin = KP_LIN * dist_err
        lin = np.clip(lin, -MAX_LIN, MAX_LIN)

        center_scale = max(0.0, 1.0 - abs(norm_err))
        twist.linear.x = lin * center_scale

        self.get_logger().info(
            f"FOLLOWING ID={best['id']} dist={est_dist:.1f}cm "
            f"lin={twist.linear.x:.2f} ang={twist.angular.z:.2f}"
        )

        self.cmd_pub.publish(twist)

    def destroy_node(self):
        self.get_logger().info("Shutting down")
        try:
            self.cap.release()
            os.kill(self.cam_proc.pid, signal.SIGINT)
        except Exception:
            pass
        super().destroy_node()


# ---------------- Main ----------------
def main(args=None):
    rclpy.init(args=args)
    node = ArucoCenterController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
