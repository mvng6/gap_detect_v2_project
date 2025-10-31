#!/usr/bin/env python

from __future__ import print_function

from mobile_robot_server.srv import MobilePosition,MobilePositionResponse
import rospy
import asyncio
import argparse
import math

# from woosh_robot import WooshRobot, CommuSettings

from woosh_robot import WooshRobot
from woosh_interface import CommuSettings, NO_PRINT

from woosh.proto.robot.robot_pb2 import RobotInfo, PoseSpeed
from woosh.proto.robot.robot_pack_pb2 import Twist

# def handle_mobile_position(req):
#     #서비스 명령을 받으면 움직이는 코드
#     print("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))
#     return MobilePositionResponse(req.a + req.b)

def mobile_position_server():
    rospy.init_node('mobile_position_server')
    # 모바일로봇 연결



    # s = rospy.Service('mobile_position', MobilePosition, handle_mobile_position)
    print("Mobile Robot Ready to move!!!")
    rospy.spin()

if __name__ == "__main__":
    mobile_position_server()