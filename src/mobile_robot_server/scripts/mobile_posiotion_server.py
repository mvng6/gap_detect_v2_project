#!/usr/bin/env python

from __future__ import print_function

from mobile_robot_server.srv import MobilePosition,MobilePositionResponse
import rospy
import asyncio
import argparse
import math
from datetime import datetime
import time
from google.protobuf.timestamp_pb2 import Timestamp
import time

from woosh.proto.ros.action_pb2 import AnyAction

# from woosh_robot import WooshRobot, CommuSettings

from woosh_robot import WooshRobot
from woosh_interface import CommuSettings, NO_PRINT

from woosh.proto.robot.robot_pb2 import RobotInfo, PoseSpeed
from woosh.proto.robot.robot_pack_pb2 import Twist

from woosh.proto.ros.action_pb2 import (
    StepControl,
    ControlAction,
)

from woosh.proto.ros.ros_pack_pb2 import (
    CallAction,
    Feedbacks,
)

#모바일로봇 클래스
class MobileRobotTwistController:
    """Twist 방식으로 모바일 로봇의 거리를 제어하는 클래스"""
    
    def __init__(self, verbose=False, init_node=True):
        """
        ROS 노드 및 파라미터 초기화
        
        Args:
            verbose: SDK 로그 상세 출력 여부
            init_node: rospy.init_node() 호출 여부 (다른 노드에서 래핑 시 False)
        """
 
        # 파라미터 로드
        self.robot_ip = rospy.get_param('~robot_ip', '169.254.128.2')
        self.robot_port = rospy.get_param('~robot_port', 5480)
        self.robot_identity = rospy.get_param('~robot_identity', 'twist_controller')
        self.verbose = verbose  # 상세 모드 플래그
        
        self.robot = None
        self.current_pose = None  # 현재 위치 저장
        
        if init_node:
            rospy.loginfo("🤖 Twist Controller 초기화 완료")
            rospy.loginfo(f"   연결 대상: {self.robot_ip}:{self.robot_port}")
    
    async def connect(self):
        """로봇에 연결"""
        # SDK 로거 레벨 설정
        import logging
        sdk_logger = logging.getLogger('twist_controller')
        
        if self.verbose:
            # 상세 모드: 모든 로그 출력
            sdk_logger.setLevel(logging.DEBUG)
        else:
            # 일반 모드: WARNING 이상만 출력 (INFO 숨김)
            sdk_logger.setLevel(logging.WARNING)
        
        settings = CommuSettings(
            addr=self.robot_ip,
            port=self.robot_port,
            identity=self.robot_identity,
            logger=sdk_logger  # 커스텀 로거 전달
        )
        
        self.robot = WooshRobot(settings)
        await self.robot.run()
        
        # 연결 검증
        info, ok, msg = await self.robot.robot_info_req(RobotInfo(), NO_PRINT, NO_PRINT)
        if not ok:
            rospy.logerr(f"❌ 로봇 연결 실패: {msg}")
            raise ConnectionError(f"Failed to connect: {msg}")
        
        rospy.loginfo("✅ 로봇 연결 성공!")
        rospy.loginfo(f"🔋 배터리 잔량: {info.battery.power}%")
        
    
    async def stop(self):
        """로봇 연결 종료"""
        if self.robot:
            rospy.loginfo("📋 로봇 연결 종료 중...")
            await self.robot.stop()
            rospy.loginfo("✅ 연결 종료 완료")


    async def stepmove(self):
        """로봇 이동"""
        input("\n엔터를 입력하여 스텝 제어(직진 0.5m)를 실행하세요...\n")
        step_control = StepControl()
        step = step_control.steps.add() # 이동할 스텝 추가
        step.mode = StepControl.Step.Mode.kStraight # 모드: 직진
        step.value = 0.1  # 값: 0.5 미터
        step.speed = 0.1 # 속도: 0.25 m/s
        step_control.action = ControlAction.kExecute # 동작: 실행

        # call_action = CallAction(step_control=step_control)

        # AnyAction 으로 감싸서 전송
        any_act = AnyAction(
            type="StepControl",               # 서버가 인식하는 타입 문자열
            value=step_control.SerializeToString()
        )

        call_action = CallAction(any_action=any_act)

        
        _, ok, msg = await self.robot.call_action_req(call_action, NO_PRINT, NO_PRINT)
        if ok:
            print("스텝 제어 요청 성공")
        else:
            print(f"스텝 제어 요청 실패, msg: {msg}")

        await asyncio.sleep(5)


        # stop_twist = Twist(linear=0.1, angular=0.0)
        # _, ok, msg = await self.robot.twist_req(stop_twist, NO_PRINT, NO_PRINT)
        # if ok:
        #     print(" 트위스트 제어 요청 성공")
        # else:
        #     print(f"트위스트 제어 요청 실패, msg: {msg}")

        # await asyncio.sleep(5)

        # stop_twist = Twist(linear=0.0, angular=0.0)
        # _, ok, msg = await self.robot.twist_req(stop_twist, NO_PRINT, NO_PRINT)
        # if ok:
        #     print(" 트위스트 제어 요청 성공")
        # else:
        #     print(f"트위스트 제어 요청 실패, msg: {msg}")



# def handle_mobile_position(req):
#     #서비스 명령을 받으면 움직이는 코드
#     print("Returning [%s + %s = %s]"%(req.a, req.b, (req.a + req.b)))
#     return MobilePositionResponse(req.a + req.b)

async def mobile_position_server():
    rospy.init_node('mobile_position_server')
    # 모바일로봇 연결

    controller = MobileRobotTwistController()
    await controller.connect()
    await controller.stepmove()



    # s = rospy.Service('mobile_position', MobilePosition, handle_mobile_position)
    print("Mobile Robot Ready to move!!!")


    





    rospy.spin()

    await controller.stop()

if __name__ == "__main__":
    asyncio.run(mobile_position_server())