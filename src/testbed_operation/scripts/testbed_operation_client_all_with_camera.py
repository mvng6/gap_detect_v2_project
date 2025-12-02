#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64
# from your_package.srv import MobilePositionTwist, MobilePositionTwistRequest  # 커스텀 srv 사용 시
# from dsr_msgs.srv import MoveJoint, MoveJointRequest

from testbed_operation.srv import MoveJoint, MoveJointRequest
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


def main():
    rospy.init_node('integrated_robot_client', anonymous=True)

    # DSR 로봇 속도, 가속도, 이동 시간 설정
    dsr_vel = 30.0
    dsr_acc = 30.0
    dsr_time = 3.0

    # dsr_move_home은 기본 자세로 이동 (인자 없이 호출 가능)
    # 필요시 vel, acc만 변경하여 호출
    if not dsr_move_home(vel=dsr_vel, acc=dsr_acc):
        rospy.logerr("DSR motion failed.")
        return

    rospy.loginfo("All operations completed successfully.")


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Program interrupted by user.")
