#!/usr/bin/env python

from socket import timeout
import rospy
from std_msgs.msg import Float64
# from your_package.srv import MobilePositionTwist, MobilePositionTwistRequest  # 커스텀 srv 사용 시
# from dsr_msgs.srv import MoveJoint, MoveJointRequest

from testbed_operation.srv import MoveJoint, MoveJointRequest
from testbed_operation.srv import QRPoseCorrection, QRPoseCorrectionRequest
from testbed_operation.srv import MobilePositionTwist, MobilePositionTwistRequest

def mobile_move(distance=0.1):
    """
    모바일 로봇을 지정된 거리만큼 이동시키는 함수
    기본값: 0.1m
    """
    service_name = '/mobile_positiontwist'
    rospy.loginfo("Waiting for mobile service: %s", service_name)
    rospy.wait_for_service(service_name)

    try:
        move_mobile = rospy.ServiceProxy(service_name, MobilePositionTwist)
        req = MobilePositionTwistRequest()
        req.distance = distance

        rospy.loginfo("Sending mobile move command: %.2f m", distance)
        response = move_mobile(req)
        rospy.loginfo("Mobile move command sent successfully.")
        return True
    except rospy.ServiceException as e:
        rospy.logerr("Mobile service call failed: %s", e)
        return False

def dsr_move_joint(pos=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                   vel=30.0, acc=30.0, time=0.0,
                   radius=0.0, mode=0, blendType=0, syncType=0):
    """
    DSR 로봇을 사용자 정의 자세로 이동시키는 범용 함수
    원하는 관절 각도를 직접 지정하여 로봇을 제어
    """
    service_name = '/dsr01a0912/motion/move_joint'
    rospy.loginfo("Waiting for DSR service: %s", service_name)
    rospy.wait_for_service(service_name)

    try:
        move_joint = rospy.ServiceProxy(service_name, MoveJoint)
        req = MoveJointRequest()
        req.pos = pos
        req.vel = vel
        req.acc = acc
        req.time = time
        req.radius = radius
        req.mode = mode
        req.blendType = blendType
        req.syncType = syncType

        rospy.loginfo("Sending DSR move_joint command: %s", pos)
        resp = move_joint(req)
        if resp.success:
            rospy.loginfo("DSR move_joint succeeded.")
        else:
            rospy.logwarn("DSR move_joint failed: %s", resp.message if hasattr(resp, 'message') else "Unknown")
        return resp.success
    except rospy.ServiceException as e:
        rospy.logerr("DSR service call failed: %s", e)
        return False

def dsr_move_home(pos=[90.0, 0.0, 90.0, 0.0, 90.0, -90.0],
                  vel=30.0, acc=30.0, time=0.0,
                  radius=0.0, mode=0, blendType=0, syncType=0):
    """
    DSR 로봇을 홈 포지션으로 이동시키는 함수
    기본 자세: [90.0, 0.0, 90.0, 0.0, 90.0, -90.0]
    인자 없이 호출 시 기본 자세로 이동
    """
    service_name = '/dsr01a0912/motion/move_joint'  # move_joint 서비스 사용
    rospy.loginfo("Waiting for DSR service: %s", service_name)
    rospy.wait_for_service(service_name)

    try:
        move_joint = rospy.ServiceProxy(service_name, MoveJoint)
        req = MoveJointRequest()
        req.pos = pos
        req.vel = vel
        req.acc = acc
        req.time = time
        req.radius = radius
        req.mode = mode
        req.blendType = blendType
        req.syncType = syncType

        rospy.loginfo("Sending DSR move_home command: %s", pos)
        resp = move_joint(req)
        if resp.success:
            rospy.loginfo("DSR move_home succeeded.")
        else:
            rospy.logwarn("DSR move_home failed: %s", resp.message if hasattr(resp, 'message') else "Unknown")
        return resp.success
    except rospy.ServiceException as e:
        rospy.logerr("DSR service call failed: %s", e)
        return False

def request_qr_pose_correction(current_joint_pos, measurement_point_id="defalut"):
    f"""
    QR 코드 기반 위치/자세 보정 요청 함수

    Args:
        current_joint_pos (list): 현재 로봇의 관절 각도 [6개, deg]
        measurement_point_id (str): 측정 포인트 식별자
    
    Returns:
        dist: 보정값 딕셔너리 또는 None
            {
                'delta_x': float,      # mm
                'delta_y': float,      # mm
                'delta_z': float,      # mm
                'delta_rx': float,     # deg
                'delta_ry': float,     # deg
                'delta_rz': float,     # deg
                'rotation_matrix': list,  # 9개 요소
                'qr_confidence': float
            }
    """
    service_name = '/qr_pose_correction'
    rospy.loginfo("QR 위치 보정 서비스 대기중: %s", service_name)

    try:
        rospy.wait_for_service(service_name, timeout=10.0)
    except rospy.ROSException:
        rospy.logerr("QR 위치 보정 서비스 사용 불가 (timeout)")
        return None

    try:
        qr_correction = rospy.ServiceProxy(service_name, QRPoseCorrection)
        req = QRPoseCorrectionRequest()
        req.robot_ready = True
        req.current_joint_pos = current_joint_pos
        req.measurement_point_id = measurement_point_id

        rospy.loginfo("QR 위치 보정 요청 전송: %s", measurement_point_id)
        resp = qr_correction(req)

        if resp.success:
            rospy.loginfo("QR 위치 보정 성공 (신뢰도: %.2f)", resp.qr_confidence)
            return {
                'delta_x': resp.delta_x,
                'delta_y': resp.delta_y,
                'delta_z': resp.delta_z,
                'delta_rx': resp.delta_rx,
                'delta_ry': resp.delta_ry,
                'delta_rz': resp.delta_rz,
            }
        else:
            rospy.logwarn("QR 위치 보정 실패: %s", resp.message)
            return None

    except rospy.ServiceException as e:
        rospy.logerr("QR 위치 보정 서비스 호출 실패: %s", e)
        return None



def main():
    rospy.init_node('integrated_robot_client', anonymous=True)

    # DSR 로봇 속도, 가속도, 이동 시간 설정
    dsr_vel = 30.0
    dsr_acc = 30.0
    dsr_time = 3.0

    # 1. 홈 포지션 이동
    rospy.loginfo("=== Step 1: Moving to home position ===")
    if not dsr_move_home(vel=dsr_vel, acc=dsr_acc):
        rospy.logerr("Failed to move to home position.")
        return

    rospy.sleep(1.0)    # 안정화 대기

    # 2. QR 코드 측정 위치로 이동
    rospy.loginfo("=== Step 2: Moving to QR measurement position ===")
    qr_measurement_pos = [49.54, 31.27, 87.67, 0.0, 61.06, -130.46]  # A-point1
    if not dsr_move_joint(pos=qr_measurement_pos, vel=dsr_vel, acc=dsr_acc):
        rospy.logerr("Failed to move to QR measurement position.")
        return

    rospy.sleep(0.5)  # 안정화 대기

    # 3. QR 코드 기반 위치/자세 보정
    rospy.loginfo("=== Step 3: Requesting QR pose correction ===")    
    correction_data = request_qr_pose_correction(
            current_joint_pos=qr_measurement_pos,
            measurement_point_id="A-point"
    )
    if correction_data is None:
        rospy.logerr("QR pose correction failed. Aborting.")
        return
        
    # 4. 보정값 기반 로봇 자세 보정
    rospy.loginfo("=== Step 4: Applying pose correction ===")
    if not dsr_move_joint(pos=correction_data, vel=dsr_vel, acc=dsr_acc):
        rospy.logerr("Failed to move to pose correction position.")
        return

    # 5. 홈 포지션으로 복귀
    rospy.loginfo("=== Step 5: Returning to home position ===")
    if not dsr_move_home(vel=dsr_vel, acc=dsr_acc):
        rospy.logerr("Failed to return to home position.")
        return
    
    rospy.loginfo("All operations completed successfully.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted by user.")
