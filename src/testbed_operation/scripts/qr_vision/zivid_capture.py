# %%
import zivid
import zivid.experimental.calibration as calib
import datetime
import cv2
import numpy as np
# from PIL import Image
import matplotlib.pyplot as plt

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



# Example call
# %%
# camera, settings = camera_load()
# %%
# tic=time.time()
# frame, rgba, xyz, depth_normalized, depth, intrinsics \
    # = snapshot(camera=camera, settings=settings)

# rgb = rgba[...,0:3]
# plt.imshow(rgb)
# import os
# write_path = "C:\\Users\\LEGION\\Desktop\\python_codes_for_image\\zivid\\images"
# write_path = os.path.join(write_path, "110cm\\section4.npz")

# np.savez(write_path,
#          rgba=rgba, 
#          xyz=xyz,
#          depth=depth,
#          intrinsics=intrinsics)
# cv2.imwrite()
# %%
# data = np.load(write_path)
# img = data["rgba"][...,0:3]
# plt.imshow(img)
