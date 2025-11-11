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
                   vel=30.0, acc=30.0, time=5.0,
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

    # 1. 모바일 로봇 이동
    mobile_distance = 1.0  # [m]
    if not mobile_move(mobile_distance):
        rospy.logerr("Aborting: Mobile robot failed to move.")
        return

    rospy.loginfo("Mobile robot movement completed. Waiting 1 second before DSR motion...")
    rospy.sleep(1.0)  # 안정적인 전이 대기

    # 2. DSR 로봇 조인트 이동
    target_joint = [0.0, 15.0, 0.0, 0.0, 0.0, 0.0]  # 홈 포지션 예시
    if not dsr_move_joint(
        pos=target_joint,
        vel=30.0,
        acc=30.0,
        time=5.0,
        radius=0.0,
        mode=0,        # 1: 절대 위치
        blendType=0,
        syncType=0
    ):
        rospy.logerr("DSR motion failed.")
        return

    rospy.sleep(1.0)  # 안정적인 전이 대기

    # 3. DSR 로봇 조인트 이동
    target_joint = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 홈 포지션 예시
    if not dsr_move_joint(
        pos=target_joint,
        vel=30.0,
        acc=30.0,
        time=5.0,
        radius=0.0,
        mode=0,        # 1: 절대 위치
        blendType=0,
        syncType=0
    ):
        rospy.logerr("DSR motion failed.")
        return
    
    rospy.sleep(1.0)  # 안정적인 전이 대기

    # 4. 모바일 로봇 이동
    mobile_distance = -1.0  # [m]
    if not mobile_move(mobile_distance):
        rospy.logerr("Aborting: Mobile robot failed to move.")
        return

    rospy.loginfo("Mobile robot movement completed. Waiting 1 second before DSR motion...")
    rospy.sleep(1.0)  # 안정적인 전이 대기

    # 5. DSR 로봇 조인트 이동
    target_joint = [0.0, -15.0, 0.0, 0.0, 0.0, 0.0]  # 홈 포지션 예시
    if not dsr_move_joint(
        pos=target_joint,
        vel=30.0,
        acc=30.0,
        time=5.0,
        radius=0.0,
        mode=0,        # 1: 절대 위치
        blendType=0,
        syncType=0
    ):
        rospy.logerr("DSR motion failed.")
        return

    rospy.sleep(1.0)  # 안정적인 전이 대기

    # 6. DSR 로봇 조인트 이동
    target_joint = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 홈 포지션 예시
    if not dsr_move_joint(
        pos=target_joint,
        vel=30.0,
        acc=30.0,
        time=5.0,
        radius=0.0,
        mode=0,        # 1: 절대 위치
        blendType=0,
        syncType=0
    ):
        rospy.logerr("DSR motion failed.")
        return






    rospy.loginfo("Entire sequence completed successfully!")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass