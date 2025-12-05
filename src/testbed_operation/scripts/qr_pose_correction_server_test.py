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


# import sys
# sys.path.append("/home/katech/robot_ws/src/testbed_operation/scripts/qr_vision")

import zivid
import zivid.experimental.calibration as calib
import datetime
import cv2
import numpy as np
# from PIL import Image
import matplotlib.pyplot as plt

from testbed_operation.srv import QRPoseCorrection, QRPoseCorrectionResponse
# from .qr_vision.zivid_capture import camera_load, snapshot
# from .qr_vision.aruco_pose import get_pose
# from zivid_capture import camera_load, snapshot
# from aruco_pose import get_pose

##########################################
# 1. ArUco 설정
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()

# 마커 한 변의 실제 길이 (단위: m) – 예: 0.03 = 3cm
MARKER_LENGTH_M = 0.02084

def estimate_pose_single_markers(corners, marker_length, intrinsics, dist_coeffs):
    """
    corners: detectMarkers가 리턴한 corners (list of (1,4,2) 배열)
    marker_length: 마커 한 변 길이 (월드 단위, 예: m or mm)
    camera_matrix: 카메라 내参 (3x3)
    dist_coeffs: 왜곡 계수 (5x1, 8x1 등)

    return:
        rvecs: (N, 3, 1)
        tvecs: (N, 3, 1)
    """

    # 마커 좌표계에서의 3D 코너 좌표 (Z=0 평면 위 정사각형)
    half = marker_length / 2.0
    # 꼭짓점 순서는 OpenCV/ArUco가 반환하는 corner 순서와 맞춰야 함
    # 보통: [왼-위, 오른-위, 오른-아래, 왼-아래]
    objp = np.array([
        [-half,  half, 0],
        [ half,  half, 0],
        [ half, -half, 0],
        [-half, -half, 0]
    ], dtype=np.float32)

    rvecs = []
    tvecs = []
    
    camera_matrix = np.array([[intrinsics[0], 0, intrinsics[2]],
                              [0, intrinsics[1], intrinsics[3]],
                              [0, 0, 1]])

    for c in corners:
        # c shape: (1, 4, 2) → (4, 2)
        img_pts = c.reshape(-1, 2).astype(np.float32)

        # solvePnP 호출
        success, rvec, tvec = cv2.solvePnP(
            objp,              # 3D points
            img_pts,           # 2D points
            camera_matrix,
            dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE
        )
        if success:
            rvecs.append(rvec)
            tvecs.append(tvec)
        else:
            rvecs.append(None)
            tvecs.append(None)

    return rvecs, tvecs


def get_pose(frame, intrinsics, dist):  # zivid_capture -> snapshot -> get_pose(frame)
    # 0) frame을 OpenCV가 좋아하는 형태로 변환
    frame = np.ascontiguousarray(frame)  # 레이아웃 연속화

    # dtype이 uint8이 아니면 변환 (Zivid는 보통 float32 0~1)
    if frame.dtype != np.uint8:
        f = frame.astype(np.float32)
        max_val = float(f.max()) if f.size > 0 else 1.0
        if max_val <= 1.0:
            f = (f * 255.0)
        frame = np.clip(f, 0, 255).astype(np.uint8)

    # 1) 그레이스케일로 변환
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    fx, fy, cx, cy = intrinsics
    camera_matrix = np.array([[fx, 0, cx],
                              [0, fy, cy],
                              [0,  0, 1]], dtype=np.float32)
    
    # 2) ArUco 마커 검출
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
    corners, ids, rejected = detector.detectMarkers(gray)

    rvec_out, tvec_out, R_out = None, None, None

    if ids is not None and len(ids) > 0:
        # 검출된 마커 테두리 그리기
        cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # 3) 포즈 추정 (solvePnP 래핑)
        rvecs, tvecs = estimate_pose_single_markers(
            corners,
            marker_length=MARKER_LENGTH_M,
            intrinsics=intrinsics,
            dist_coeffs=dist
        )

        for i in range(len(ids)):
            if rvecs[i] is None:
                continue


            # (3,1) 형태로 맞추기
            rvec_ = np.asarray(rvecs[i], dtype=np.float32).reshape(3, 1)
            tvec_ = np.asarray(tvecs[i], dtype=np.float32).reshape(3, 1)

            # 4) 축 그리기 (왜곡은 dist 인자 사용)
            cv2.drawFrameAxes(frame, camera_matrix, dist,
                              rvec_, tvec_, MARKER_LENGTH_M * 0.5)

            # 5) 회전벡터 -> 회전행렬
            R, _ = cv2.Rodrigues(rvec_)

            # 6) z축 거리 (스칼라로 꺼내기)
            z = float(tvec_[2, 0])
            text = f"ID {int(ids[i])} | z = {z:.3f} m"
            cv2.putText(frame, text, (10, 30 + 30 * i),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            print("========== Marker ID:", int(ids[i]), "==========")
            print("rvec shape:", rvec_.shape)
            print("tvec shape:", tvec_.shape)
            print("rvec (Rodrigues):", rvec_.ravel())
            print("tvec (x, y, z) [m]:", tvec_.ravel())
            print("Rotation matrix R:\n", R)
            print()

            rvec_out, tvec_out, R_out = rvec_, tvec_, R

    # 7) 화면 표시
    # cv2.imshow("ArUco Pose", frame)
    # key = cv2.waitKey(1) & 0xFF
    # if key == 27:  # ESC
    #     cv2.destroyAllWindows()

    return rvec_out, tvec_out, R_out, frame

def camera_load():
    # Zivid 애플리케이션 초기화
    app = zivid.Application()
    # 카메라 연결
    camera = app.connect_camera()

    # 이미지 캡처 설정
    settings = zivid.Settings()
    settings.acquisitions.append(
        zivid.Settings.Acquisition(
            aperture=3.36,  # 조리개 값 3.36 # 7~8 is appropriate when turned all the lights in the lab.
            # exposure_time=datetime.timedelta(milliseconds=100),  # 노출 시간 defalut(max) 100
            exposure_time=datetime.timedelta(milliseconds=10),  # 노출 시간 defalut(max) 100
            gain=1.0  # gain 설정
        )
    )
    return camera, settings

def _get_intrinsics(camera, frame, settings=None):
    
    # 1) experimental 경로 (구버전)
    try:
        from zivid.experimental import calibration as zcal  # old API
        intr = zcal.estimate_intrinsics(frame)
        return intr
    except Exception:
        pass

    # 2) 비 experimental 경로 (일부 신버전)
    try:
        from zivid import calibration as zcal  # e.g., zivid.calibration.estimate_intrinsics
        if hasattr(zcal, "estimate_intrinsics"):
            return zcal.estimate_intrinsics(frame)
    except Exception:
        pass

    # 3) 하드코드된/설정기반 intrinsics (fallback)
    #    - 포인트클라우드 해상도에 맞추려면 settings와 함께 intrinsics를 구하는 함수가 있는 버전도 있습니다.
    #    - 없으면 camera.intrinsics() 사용
    try:
        # 가장 단순한 fallback
        return camera.intrinsics()
    except Exception:
        pass

    try:
        from zivid import calibration as zcal
        if settings is not None and hasattr(zcal, "intrinsics"):
            return zcal.intrinsics(camera, settings)
    except Exception:
        pass

    raise RuntimeError("Zivid intrinsics를 얻을 수 없습니다. zivid-python / SDK 버전을 확인하세요.")



# 이미지 캡처
# For the Previous Version
# def snapshot(camera, settings):
#     frame = camera.capture(settings)
#     # frame.save("output.zdf")  # Zivid의 기본 형식으로 저장
#     # frame.image().save("output.jpg")
#     point_cloud = frame.point_cloud()

#     xyz = point_cloud.copy_data("xyz") # point cloud의 xyz좌표 추출
#     rgba = point_cloud.copy_data("rgba")[:,:,:3] # rgba image
#     # rgba = (rgba * 255).astype(np.uint8)

#     depth_map = point_cloud.copy_data("z")

#     # 무한대 및 결측값을 0으로 설정
#     depth_array = np.nan_to_num(depth_map, nan=0.0, posinf=0.0, neginf=0.0)

#     # mm 단위로 변환 (Blender 기본 단위가 meter이므로 1000을 곱함)
#     # depth_array_mm = (depth_array * 1000).astype(np.uint16)

#     # 정규화되지 않은 Depth 저장
#     # depth_image_org = Image.fromarray(depth_array_mm, mode="I;16")

#     # Depth 데이터를 0~255로 정규화
#     depth_min = np.min(depth_array)
#     depth_max = np.max(depth_array)

#     if depth_min == depth_max:
#         # print("Warning: All depth values are the same.")
#         depth_normalized = (depth_array * 1)#.astype(np.uint8)
#     else:
#         depth_normalized = ((depth_array - depth_min) / (depth_max - depth_min) * 1)#.astype(np.uint8)

#     # cv2.imwrite("rgba.png", rgba)
    
#     estimated_intrinsics = zivid.experimental.calibration.estimate_intrinsics(frame)

#     fx = estimated_intrinsics.camera_matrix.fx
#     fy = estimated_intrinsics.camera_matrix.fy
#     cx = estimated_intrinsics.camera_matrix.cx
#     cy = estimated_intrinsics.camera_matrix.cy

#     intrinsics = [fx, fy, cx, cy]

#     return frame, rgba, xyz, depth_normalized, depth_array, intrinsics

def snapshot(camera, settings):
    frame = camera.capture(settings)
    point_cloud = frame.point_cloud()

    xyz  = point_cloud.copy_data("xyz")
    rgba = point_cloud.copy_data("rgba")[:, :, :3]
    depth_map = point_cloud.copy_data("z")

    depth_array = np.nan_to_num(depth_map, nan=0.0, posinf=0.0, neginf=0.0)

    depth_min = np.min(depth_array)
    depth_max = np.max(depth_array)
    if depth_min == depth_max:
        depth_normalized = depth_array * 1.0
    else:
        depth_normalized = (depth_array - depth_min) / (depth_max - depth_min)

    # ✅ 버전 호환 intrinsics 획득
    intr = _get_intrinsics(camera, frame, settings=settings)

    fx = intr.camera_matrix.fx
    fy = intr.camera_matrix.fy
    cx = intr.camera_matrix.cx
    cy = intr.camera_matrix.cy
    intrinsics = [fx, fy, cx, cy]
    intrinsics_ = calib.intrinsics(camera)

    k1 = intrinsics_.distortion.k1
    k2 = intrinsics_.distortion.k2
    k3 = intrinsics_.distortion.k3
    p1 = intrinsics_.distortion.p1
    p2 = intrinsics_.distortion.p2

    dist = np.array([k1, k2, p1, p2, k3])

    return frame, rgba, xyz, depth_normalized, depth_array, intrinsics, dist



##########################################
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
'zivid_capture'