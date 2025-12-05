import cv2
import numpy as np
# import sys

# mtx = [[fx, 0, cx],
#        [0, fy, cy],
#        [0,  0,  1]]

# 왜곡계수 (k1, k2, p1, p2, k3)
# dist_coeffs = np.zeros((5, 1), dtype=np.float32)

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

# def get_pose(frame, intrinsics, dist): # zivid_capture -> snapshot -> get_pose(frame)
#     # 그레이스케일로 변환
#     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

#     fx = intrinsics[0]
#     fy = intrinsics[1]
#     cx = intrinsics[2]
#     cy = intrinsics[3]
#     camera_matrix = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]])
    
#     # 3. 마커 검출
#     # corners, ids, rejected = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
#     detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)
#     corners, ids, rejected = detector.detectMarkers(gray)


#     if ids is not None and len(ids) > 0:
#         # 검출된 마커 테두리 그리기
#         cv2.aruco.drawDetectedMarkers(frame, corners, ids)

#         # 4. 포즈 추정
#         # estimatePoseSingleMarkers는 각 마커마다 rvec, tvec을 반환
#         # rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
#         #     corners,
#         #     MARKER_LENGTH_M,
#         #     camera_matrix,
#         #     dist_coeffs
#         # )
#         rvecs, tvecs = estimate_pose_single_markers(corners,
#                                                     marker_length=MARKER_LENGTH_M,
#                                                     intrinsics=intrinsics,
#                                                     dist_coeffs=dist)

#         for i in range(len(ids)):
#             rvec = rvecs[i][0]
#             tvec = tvecs[i][0]
#             rvec_ = rvecs[i].reshape(3, 1)
#             tvec_ = np.asarray(tvecs[i], dtype=np.float32).reshape(3, 1)

#             # 축 그리기 (좌표축 길이: 마커 변 길이의 0.5배 정도)

#             cv2.drawFrameAxes(frame, camera_matrix, dist, rvecs[i], tvec_, MARKER_LENGTH_M * 0.5)
#             # 회전벡터(rvec)를 회전행렬로 변환
#             R, _ = cv2.Rodrigues(rvec_)
            
#             z = float(tvec_[2, 0])          # 가장 명확
#             # 또는
#             z = float(tvec_.ravel()[2])     # 1D로 펴서 2번째 인덱스


#             # 화면에 텍스트로 출력 (z축 거리만 간단히 표시 예시)
#             text = f"ID {int(ids[i])} | z = {z:.3f} m"

#             cv2.putText(frame, text, (10, 30 + 30 * i),
#                         cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

#             # 콘솔에 자세한 포즈 정보 출력
#             print("========== Marker ID:", int(ids[i]), "==========")
#             print("rvec (Rodrigues):", rvec_)
#             print("tvec (x, y, z) [m]:", tvec_)
#             print("Rotation matrix R:\n",
 
#             print()

#     # 화면 표시
#     cv2.imshow("ArUco Pose", frame)

#     key = cv2.waitKey(1) & 0xFF
#     if key == 27:  # ESC
#         cv2.destroyAllWindows()
#     return rvec_, tvec_, R, frame

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

