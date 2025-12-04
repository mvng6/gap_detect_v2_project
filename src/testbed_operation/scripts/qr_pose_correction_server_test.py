#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
QR pose correction server with QRAnalyzer wrapper.
- camera_load() is called exactly once at startup.
- Each request: capture via Zivid -> estimate ArUco pose -> return tvec(mm) and rotation matrix.
"""

import math
import numpy as np
import rospy

from testbed_operation.srv import QRPoseCorrection, QRPoseCorrectionResponse
from zivid_capture import camera_load, snapshot
from aruco_pose import get_pose

def rotation_matrix_to_euler_xyz_deg(R: np.ndarray):
    """Convert 3x3 rotation matrix to XYZ Euler angles (degrees)."""
    sy = math.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        x = math.atan2(R[2, 1], R[2, 2])
        y = math.atan2(-R[2, 0], sy)
        z = math.atan2(R[1, 0], R[0, 0])
    else:
        x = math.atan2(-R[1, 2], R[1, 1])
        y = math.atan2(-R[2, 0], sy)
        z = 0.0
    return np.degrees([x, y, z])


class QRAnalyzer:
    """Encapsulates capture + ArUco pose estimation."""

    def __init__(self, camera, settings):
        self.camera = camera
        self.settings = settings
        rospy.loginfo("[QRAnalyzer] Ready (camera already initialized once).")

    def analyze(self):
        """Capture frame, estimate pose; return dict or None on failure."""
        try:
            _, rgba, xyz, depth_norm, depth_raw, intrinsics, dist = snapshot(
                self.camera, self.settings
            )
        except Exception as exc:
            rospy.logerr("[QRAnalyzer] Zivid capture failed: %s", exc)
            return None

        rvec, tvec, R, _ = get_pose(rgba, intrinsics, dist)
        if rvec is None or tvec is None or R is None:
            rospy.logwarn("[QRAnalyzer] ArUco marker not detected")
            return None

        tvec = np.asarray(tvec, dtype=float).reshape(3)
        R = np.asarray(R, dtype=float).reshape(3, 3)

        tvec_mm = (tvec * 1000.0).tolist()
        euler_deg = rotation_matrix_to_euler_xyz_deg(R).tolist()

        return {
            "delta_x": tvec_mm[0],
            "delta_y": tvec_mm[1],
            "delta_z": tvec_mm[2],
            "delta_rx": euler_deg[0],
            "delta_ry": euler_deg[1],
            "delta_rz": euler_deg[2],
            "rotation_matrix": R.flatten(order="C").tolist(),
            "confidence": 0.95,  # TODO: replace with real detector score
            "position_mm": tvec_mm,
        }


class QRPoseCorrectionServer:
    def __init__(self):
        rospy.init_node("qr_pose_correction_server")

        # Initialize camera exactly once for the runtime.
        self.camera, self.settings = camera_load()
        rospy.loginfo("Zivid camera initialized once via camera_load().")

        self.analyzer = QRAnalyzer(self.camera, self.settings)

        self.service = rospy.Service(
            "/qr_pose_correction",
            QRPoseCorrection,
            self.handle_request,
        )

        rospy.loginfo("QR Pose Correction Service ready at /qr_pose_correction")

    def handle_request(self, req):
        resp = QRPoseCorrectionResponse()
        rospy.loginfo("Received request: point=%s, robot_ready=%s", req.measurement_point_id, req.robot_ready)

        if not req.robot_ready:
            resp.success = False
            resp.message = "Robot not ready"
            resp.qr_confidence = 0.0
            return resp

        result = self.analyzer.analyze()
        if result is None:
            resp.success = False
            resp.message = "Pose estimation failed (capture or marker missing)"
            resp.qr_confidence = 0.0
            return resp

        resp.success = True
        resp.message = "Pose computed"
        resp.delta_x = result["delta_x"]
        resp.delta_y = result["delta_y"]
        resp.delta_z = result["delta_z"]
        resp.delta_rx = result["delta_rx"]
        resp.delta_ry = result["delta_ry"]
        resp.delta_rz = result["delta_rz"]
        resp.rotation_matrix = result["rotation_matrix"]
        resp.qr_confidence = result["confidence"]
        resp.qr_position = result["position_mm"]

        rospy.loginfo(
            "Pose -> tvec(mm) = [%.2f, %.2f, %.2f], euler(deg) = [%.2f, %.2f, %.2f]",
            resp.delta_x, resp.delta_y, resp.delta_z,
            resp.delta_rx, resp.delta_ry, resp.delta_rz,
        )
        return resp

    def run(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        server = QRPoseCorrectionServer()
        server.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("QR Pose Correction Server shutdown")
