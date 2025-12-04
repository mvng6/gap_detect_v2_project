#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
# from your_package.srv import MobilePositionTwist, MobilePositionTwistRequest  # 커스텀 srv 사용 시
# from dsr_msgs.srv import MoveJoint, MoveJointRequest

from testbed_operation.srv import MoveJoint, MoveJointRequest
from testbed_operation.srv import MobilePositionTwist, MobilePositionTwistRequest

def mobile_move(distance=0.1):
    service_name = '/mobile_positiontwist'
    rospy.loginfo("Waiting for mobile service: %s", service_name)
    rospy.wait_for_service(service_name)

    try:
        # std_msgs/Float64 기반 서비스 가정
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


def main():
    rospy.init_node('integrated_robot_client', anonymous=True)
    
    dsr_vel = 30.0
    dsr_acc = 30.0
    dsr_time = 0.0

    for i in range(4):
        # 1. DSR로봇 측정 초기 위치 이동
        target_joint = [90.0, 0.0, 90.0, 0.0, 90.0, -90.0]  # 홈 포지션 예시
        if not dsr_move_joint(
            pos=target_joint,
            vel=dsr_vel,
            acc=dsr_acc,
            time=dsr_time,
            radius=0.0,
            mode=0,        # 1: 절대 위치
            blendType=0,
            syncType=0
        ):
            rospy.logerr("DSR motion failed.")
            return

        rospy.sleep(1.0)  # 안정적인 전이 대기

        # 2-1. DSR 로봇 A-point1 측정 위치
        target_joint = [49.54, 31.27, 87.67, 0.0, 61.06, -130.46]  # 홈 포지션 예시
        if not dsr_move_joint(
            pos=target_joint,
            vel=dsr_vel,
            acc=dsr_acc,
            time=dsr_time,
            radius=0.0,
            mode=0,        # 1: 절대 위치
            blendType=0,
            syncType=0
        ):
            rospy.logerr("DSR motion failed.")
            return

        rospy.sleep(1.0)  # 안정적인 전이 대기

        # 2-2. DSR 로봇 A-point2 측정 위치
        target_joint = [55.63, 42.22, 68.88, 0.0, 68.9, -124.37]  # 홈 포지션 예시
        if not dsr_move_joint(
            pos=target_joint,
            vel=dsr_vel,
            acc=dsr_acc,
            time=dsr_time,
            radius=0.0,
            mode=0,        # 1: 절대 위치
            blendType=0,
            syncType=0
        ):
            rospy.logerr("DSR motion failed.")
            return

        rospy.sleep(1.0)  # 안정적인 전이 대기

        # 2-3. DSR 로봇 A-point3 측정 위치
        target_joint = [60.42, 56.73, 41.99, 0.0, 81.28, -119.58]  # 홈 포지션 예시
        if not dsr_move_joint(
            pos=target_joint,
            vel=dsr_vel,
            acc=dsr_acc,
            time=dsr_time,
            radius=0.0,
            mode=0,        # 1: 절대 위치
            blendType=0,
            syncType=0
        ):
            rospy.logerr("DSR motion failed.")
            return

        rospy.sleep(1.0)  # 안정적인 전이 대기

        # 3. DSR 로봇 조인트 홈 이동
        target_joint = [90.0, 0.0, 90.0, 0.0, 90.0, -90.0]  # 홈 포지션 예시
        if not dsr_move_joint(
            pos=target_joint,
            vel=dsr_vel,
            acc=dsr_acc,
            time=dsr_time,
            radius=0.0,
            mode=0,        # 1: 절대 위치
            blendType=0,
            syncType=0
        ):
            rospy.logerr("DSR motion failed.")
            return
        
        rospy.sleep(1.0)  # 안정적인 전이 대기

        # 4. 모바일 로봇 이동
        mobile_distance = 0.3  # [m]
        if not mobile_move(mobile_distance):
            rospy.logerr("Aborting: Mobile robot failed to move.")
            return

        rospy.loginfo("Mobile robot movement completed. Waiting 1 second before DSR motion...")
        rospy.sleep(1.0)  # 안정적인 전이 대기

        # 5-1. DSR 로봇 B-point1 측정 위치
        target_joint = [62.64, 55.81, 37.25, 0.0, 86.94, -117.36]  # 홈 포지션 예시
        if not dsr_move_joint(
            pos=target_joint,
            vel=dsr_vel,
            acc=dsr_acc,
            time=dsr_time,
            radius=0.0,
            mode=0,        # 1: 절대 위치
            blendType=0,
            syncType=0
        ):
            rospy.logerr("DSR motion failed.")
            return

        rospy.sleep(1.0)  # 안정적인 전이 대기

        # 5-2. DSR 로봇 B-point2 측정 위치
        target_joint = [69.94, 48.67, 50.55, 0.0, 80.78, -110.06]  # 홈 포지션 예시
        if not dsr_move_joint(
            pos=target_joint,
            vel=dsr_vel,
            acc=dsr_acc,
            time=dsr_time,
            radius=0.0,
            mode=0,        # 1: 절대 위치
            blendType=0,
            syncType=0
        ):
            rospy.logerr("DSR motion failed.")
            return

        rospy.sleep(1.0)  # 안정적인 전이 대기


        # 5-3. DSR 로봇 B-point3 측정 위치
        target_joint = [80.45, 44.08, 58.82, 0.0, 77.11, -99.55]  # 홈 포지션 예시
        if not dsr_move_joint(
            pos=target_joint,
            vel=dsr_vel,
            acc=dsr_acc,
            time=dsr_time,
            radius=0.0,
            mode=0,        # 1: 절대 위치
            blendType=0,
            syncType=0
        ):
            rospy.logerr("DSR motion failed.")
            return

        rospy.sleep(1.0)  # 안정적인 전이 대기

        # 5-4. DSR 로봇 B-point4 측정 위치
        target_joint = [90.3, 43.41, 60.0, 0.0, 76.59, -89.7]  # 홈 포지션 예시
        if not dsr_move_joint(
            pos=target_joint,
            vel=dsr_vel,
            acc=dsr_acc,
            time=dsr_time,
            radius=0.0,
            mode=0,        # 1: 절대 위치
            blendType=0,
            syncType=0
        ):
            rospy.logerr("DSR motion failed.")
            return

        rospy.sleep(1.0)  # 안정적인 전이 대기

        # 6. DSR 로봇 조인트 홈 이동
        target_joint = [90.0, 0.0, 90.0, 0.0, 90.0, -90.0]  # 홈 포지션 예시
        if not dsr_move_joint(
            pos=target_joint,
            vel=dsr_vel,
            acc=dsr_acc,
            time=dsr_time,
            radius=0.0,
            mode=0,        # 1: 절대 위치
            blendType=0,
            syncType=0
        ):
            rospy.logerr("DSR motion failed.")
            return
        
        rospy.sleep(1.0)  # 안정적인 전이 대기

        # 7. 모바일 로봇 이동
        mobile_distance = 0.6  # [m]
        if not mobile_move(mobile_distance):
            rospy.logerr("Aborting: Mobile robot failed to move.")
            return

        rospy.loginfo("Mobile robot movement completed. Waiting 1 second before DSR motion...")
        rospy.sleep(1.0)  # 안정적인 전이 대기

        # 8-1. DSR 로봇 C-point1 측정 위치
        target_joint = [72.49,  47.05, 53.49, 0.0, 79.46, -107.51]  # 홈 포지션 예시
        if not dsr_move_joint(
            pos=target_joint,
            vel=dsr_vel,
            acc=dsr_acc,
            time=dsr_time,
            radius=0.0,
            mode=0,        # 1: 절대 위치
            blendType=0,
            syncType=0
        ):
            rospy.logerr("DSR motion failed.")
            return

        rospy.sleep(1.0)  # 안정적인 전이 대기

        # 8-2. DSR 로봇 C-point2 측정 위치
        target_joint = [76.66, 45.15, 56.92, 0.0, 77.94, -103.34]  # 홈 포지션 예시
        if not dsr_move_joint(
            pos=target_joint,
            vel=dsr_vel,
            acc=dsr_acc,
            time=dsr_time,
            radius=0.0,
            mode=0,        # 1: 절대 위치
            blendType=0,
            syncType=0
        ):
            rospy.logerr("DSR motion failed.")
            return

        rospy.sleep(1.0)  # 안정적인 전이 대기

        # 8-3. DSR 로봇 C-point3 측정 위치
        target_joint = [88.41, 43.26, 60.27, 0.0, 76.47, -91.59]  # 홈 포지션 예시
        if not dsr_move_joint(
            pos=target_joint,
            vel=dsr_vel,
            acc=dsr_acc,
            time=dsr_time,
            radius=0.0,
            mode=0,        # 1: 절대 위치
            blendType=0,
            syncType=0
        ):
            rospy.logerr("DSR motion failed.")
            return

        rospy.sleep(1.0)  # 안정적인 전이 대기

        # 8-4. DSR 로봇 C-point4 측정 위치
        target_joint = [99.88, 45.7, 55.92, 0.0, 78.38, -80.12]  # 홈 포지션 예시
        if not dsr_move_joint(
            pos=target_joint,
            vel=dsr_vel,
            acc=dsr_acc,
            time=dsr_time,
            radius=0.0,
            mode=0,        # 1: 절대 위치
            blendType=0,
            syncType=0
        ):
            rospy.logerr("DSR motion failed.")
            return

        rospy.sleep(1.0)  # 안정적인 전이 대기

        # 8-5. DSR 로봇 C-point5 측정 위치
        target_joint = [107.71, 50.37, 47.44, 0.0, 82.19, -72.29]  # 홈 포지션 예시
        if not dsr_move_joint(
            pos=target_joint,
            vel=dsr_vel,
            acc=dsr_acc,
            time=dsr_time,
            radius=0.0,
            mode=0,        # 1: 절대 위치
            blendType=0,
            syncType=0
        ):
            rospy.logerr("DSR motion failed.")
            return

        rospy.sleep(1.0)  # 안정적인 전이 대기

        # 9. DSR로봇 측정 초기 위치 이동
        target_joint = [90.0, 0.0, 90.0, 0.0, 90.0, -90.0]  # 홈 포지션 예시
        if not dsr_move_joint(
            pos=target_joint,
            vel=dsr_vel,
            acc=dsr_acc,
            time=dsr_time,
            radius=0.0,
            mode=0,        # 1: 절대 위치
            blendType=0,
            syncType=0
        ):
            rospy.logerr("DSR motion failed.")
            return

        rospy.sleep(1.0)  # 안정적인 전이 대기

        # 10. 모바일 로봇 이동 (초기 위치)
        mobile_distance = -0.9  # [m]
        if not mobile_move(mobile_distance):
            rospy.logerr("Aborting: Mobile robot failed to move.")
            return

        rospy.loginfo("Mobile robot movement completed. Waiting 1 second before DSR motion...")
        rospy.sleep(1.0)  # 안정적인 전이 대기

        rospy.loginfo("Gap Detection Sequence: %d",i)

    rospy.loginfo("Entire sequence completed successfully!")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass