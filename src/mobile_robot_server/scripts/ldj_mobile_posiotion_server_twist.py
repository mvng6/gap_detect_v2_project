#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import asyncio
import math
import numpy as np
import sys
import os
from queue import Queue, Empty
from threading import Thread

# === battery_check.py에서 배터리 출력 함수 가져오기 ===
# 현재 스크립트 디렉토리 기준으로 testbed_operation/scripts 경로 추가
script_dir = os.path.dirname(os.path.abspath(__file__))
battery_check_dir = os.path.join(script_dir, '../../testbed_operation/scripts')
sys.path.insert(0, os.path.abspath(battery_check_dir))

from battery_check import print_battery_status  # 배터리 상태 출력 함수

from mobile_robot_server.srv import MobilePositionTwist, MobilePositionTwistResponse
from woosh_robot import WooshRobot
from woosh_interface import CommuSettings, NO_PRINT
from woosh.proto.robot.robot_pack_pb2 import Twist
from woosh.proto.robot.robot_pb2 import RobotInfo, PoseSpeed, OperationState
from woosh.proto.robot.robot_pack_pb2 import SwitchMap, SetRobotPose, InitRobot
from woosh.proto.map.map_pack_pb2 import SceneList

class SmoothTwistController:
    def __init__(self):
        self.robot_ip = rospy.get_param('~robot_ip', '169.254.128.2')
        self.robot_port = rospy.get_param('~robot_port', 5480)
        self.robot_identity = rospy.get_param('~robot_identity', 'twist_ctrl')

        self.robot = None

        # === 제어 파라미터 (작은 이동에 최적화) ===
        self.max_speed = 0.12      # 최대 속도 (절대값) - 작게 조정
        self.accel = 0.25          # 가속도
        self.decel = 0.50          # 감속도 - 충분히 강하게
        self.control_hz = 50       # 제어 주기 (50Hz) - 더 정밀한 제어

        # === 상태 (매번 초기화) ===
        self.target_distance = 0.0
        self.estimated_distance = 0.0
        self.current_speed = 0.0   # 부호 포함 (음수 = 후진)
        self.is_moving = False

        self.command_queue = Queue()
        self.result_queue = Queue()

    async def connect(self):
        settings = CommuSettings(addr=self.robot_ip, port=self.robot_port, identity=self.robot_identity)
        self.robot = WooshRobot(settings)
        await self.robot.run()

        info, ok, _ = await self.robot.robot_info_req(RobotInfo(), NO_PRINT, NO_PRINT)
        if not ok:
            raise RuntimeError("로봇 연결 실패")
        
        # battery_check.py의 함수를 사용하여 배터리 상태 출력
        print_battery_status(info.battery.power)
        rospy.loginfo("로봇 연결 성공!")

    async def _move_exact_distance(self, distance):
        if abs(distance) < 0.005:
            await self.robot.twist_req(Twist(linear=0.0, angular=0.0), NO_PRINT, NO_PRINT)
            return True, f"완료: {distance:+.3f}m (너무 작음)"

        self.target_distance = distance
        self.estimated_distance = 0.0
        self.current_speed = 0.0
        self.is_moving = True

        period = 1.0 / self.control_hz
        last_time = asyncio.get_event_loop().time()

        direction = np.sign(distance)  # +1 or -1
        rospy.loginfo(f"{distance:+.3f}m 이동 시작 (방향: {'전진' if direction > 0 else '후진'})")

        while self.is_moving:
            now = asyncio.get_event_loop().time()
            dt = max(now - last_time, period)
            last_time = now

            # 남은 거리 (절대값)
            remaining = self.target_distance - self.estimated_distance
            abs_remaining = abs(remaining)

            # 현재 속도 절대값
            speed_abs = abs(self.current_speed)

            # === 정지 조건 강화 ===
            if abs_remaining < 0.03:  # 3cm 이내
                if speed_abs < 0.06:  # 속도가 6cm/s 이하면 강제 정지
                    self.current_speed = 0.0
                    self.is_moving = False
                else:
                    # 강제 감속 (70% 감속)
                    self.current_speed *= 0.7
            else:
                # 감속 거리 계산 (최소 1.5cm 보장)
                stop_dist = max((speed_abs ** 2) / (2 * self.decel), 0.015)

                # 목표 속도 결정
                if abs_remaining <= stop_dist + 0.01:
                    target_speed = 0.0
                else:
                    target_speed = self.max_speed * direction

                # 가속/감속 적용
                if target_speed > self.current_speed:
                    self.current_speed = min(self.current_speed + self.accel * dt, target_speed)
                else:
                    self.current_speed = max(self.current_speed - self.decel * dt, target_speed)

            # Twist 명령 전송
            await self.robot.twist_req(Twist(linear=self.current_speed, angular=0.0), NO_PRINT, NO_PRINT)

            # 추정 거리 누적
            self.estimated_distance += self.current_speed * dt

            await asyncio.sleep(period)

        # 최종 정지 명령
        await self.robot.twist_req(Twist(linear=0.0, angular=0.0), NO_PRINT, NO_PRINT)
        self.is_moving = False
        rospy.loginfo(f"이동 완료: 목표 {distance:+.3f}m → 추정 {self.estimated_distance:+.3f}m")
        return True, f"완료: {self.estimated_distance:+.3f}m"

    async def _control_loop(self):
        while True:
            try:
                distance = self.command_queue.get_nowait()
            except Empty:
                await asyncio.sleep(0.01)
                continue

            success, msg = await self._move_exact_distance(distance)
            self.result_queue.put((success, msg))

    async def run(self):
        await self.connect()
        await self._control_loop()


# 전역
controller = None


def service_handler(req):
    global controller
    if controller is None:
        return MobilePositionTwistResponse(False, "서버 초기화 중")

    controller.command_queue.put(req.distance)
    try:
        success, msg = controller.result_queue.get(timeout=15.0)
        return MobilePositionTwistResponse(success, msg)
    except Empty:
        return MobilePositionTwistResponse(False, "타임아웃")


def run_asyncio():
    global controller
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    controller = SmoothTwistController()

    async def main():
        try:
            await controller.run()
        except Exception as e:
            rospy.logerr(f"Asyncio 오류: {e}")

    try:
        loop.run_until_complete(main())
    except KeyboardInterrupt:
        pass
    finally:
        loop.close()


if __name__ == "__main__":
    rospy.init_node('mobile_positiontwist_server', anonymous=False)

    thread = Thread(target=run_asyncio, daemon=True)
    thread.start()

    rospy.Service('mobile_positiontwist', MobilePositionTwist, service_handler)

    rospy.loginfo("서버 시작됨! (정/역방향 정밀 제어 - 작은 이동 최적화)")
    rospy.loginfo("rosservice call /mobile_positiontwist \"{distance: 0.3}\"")
    rospy.loginfo("rosservice call /mobile_positiontwist \"{distance: -0.3}\"")
    rospy.spin()