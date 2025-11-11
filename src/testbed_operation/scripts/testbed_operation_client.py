#!/usr/bin/env python

import rospy
from testbed_operation.srv import MoveJoint, MoveJointRequest

def move_joint_client():
    # 서비스 프록시 대기
    rospy.wait_for_service('/dsr01a0912/motion/move_joint')
    
    try:
        # 서비스 호출 프록시 생성
        move_joint = rospy.ServiceProxy('/dsr01a0912/motion/move_joint', MoveJoint)
        
        # 요청 메시지 생성
        req = MoveJointRequest()
        req.pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 목표 조인트 각도 (deg)
        req.vel = 30.0                            # 속도 (%)
        req.acc = 30.0                            # 가속도 (%)
        req.time = 2.0                            # 이동 시간 (sec)
        req.radius = 0.0                          # 블렌딩 반경
        req.mode = 0                              # 0: 상대, 1: 절대 (보통 1 사용)
        req.blendType = 0                         # 블렌딩 타입
        req.syncType = 0                          # 동기화 타입

        # 서비스 호출
        resp = move_joint(req)
        rospy.loginfo("Service call success: %s", resp.success)
        return resp.success

    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)
        return False

if __name__ == '__main__':
    rospy.init_node('move_joint_client')
    success = move_joint_client()
    if success:
        rospy.loginfo("Move joint command sent successfully.")
    else:
        rospy.logwarn("Failed to send move joint command.")