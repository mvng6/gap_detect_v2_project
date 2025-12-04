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
from woosh_interface import CommuSettings, NO_PRINT, FULL_PRINT
from woosh.proto.robot.robot_pack_pb2 import Twist, ExecTask
from woosh.proto.robot.robot_pb2 import RobotInfo, PoseSpeed, OperationState, TaskProc
from woosh.proto.robot.robot_pack_pb2 import SwitchMap, SetRobotPose, InitRobot, SwitchControlMode
from woosh.proto.map.map_pack_pb2 import SceneList
from woosh.proto.util.task_pb2 import Type as TaskType, State as TaskState, Direction as TaskDirection
from woosh.proto.util.robot_pb2 import ControlMode

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

        await self._setup_map()
    
    async def _setup_map(self):
        """네비게이션 설정: 맵 로드 및 로컬라이제이션"""
        rospy.loginfo("=== 네비게이션 설정 시작 ===")

        # map_loaded 변수를 함수 시작 부분에서 초기화
        map_loaded = False

        # 1단계: 현재 상태 확인
        rospy.loginfo("1단계: 현재 상태 확인")
        pose_speed, ok, msg = await self.robot.robot_pose_speed_req(PoseSpeed(), NO_PRINT, NO_PRINT)
        if not ok:
            rospy.logwarn(f"위치 정보 요청 실패: {msg}")
            return False
        
        # 현재 맵 ID를 확인하여 맵 로드 여부 판단
        current_map_id = pose_speed.map_id if hasattr(pose_speed, 'map_id') else 0
        if current_map_id != 0:
            map_loaded = True
            rospy.loginfo(f"   현재 로드된 맵 ID: {current_map_id}")
        else:
            rospy.loginfo("   현재 로드된 맵이 없습니다.")

        # 2단계: 사용 가능한 맵 목록 확인
        rospy.loginfo("2단계: 사용 가능한 맵 목록 확인")
        scene_list_req = SceneList()
        scene_list, ok, msg = await self.robot.scene_list_req(scene_list_req, NO_PRINT, NO_PRINT)
        
        available_scenes = []
        if ok and scene_list and scene_list.scenes:
            for scene in scene_list.scenes:
                available_scenes.append(scene.name)
            rospy.loginfo(f"{len(available_scenes)}개의 장면을 찾았습니다:")
            for i, scene_name in enumerate(available_scenes, 1):
                rospy.loginfo(f"   {i}. {scene_name}")
        else:
            rospy.logwarn(f"맵 목록 확인 실패: {msg if not ok else '사용 가능한 맵이 없습니다.'}")

        # 3단계: 맵 로드 (맵이 로드되지 않은 경우)
        if not map_loaded and available_scenes:
            rospy.loginfo("3단계: 맵 로드")
            # 3번째 맵(인덱스 2)을 선택
            target_scene = available_scenes[2]
            rospy.loginfo(f"   맵 로드 시도: {target_scene}")
            
            switch_map = SwitchMap()
            switch_map.scene_name = target_scene
            result, ok, msg = await self.robot.switch_map_req(switch_map, NO_PRINT, NO_PRINT)
            
            if ok:
                rospy.loginfo(f"맵 '{target_scene}' 로드 요청 성공")
                # await asyncio.sleep(3)  # 맵 로드 완료 대기
                
                # 맵 로드 확인
                pose_speed, ok, _ = await self.robot.robot_pose_speed_req(PoseSpeed(), NO_PRINT, NO_PRINT)
                if ok and pose_speed.map_id != 0:
                    rospy.loginfo(f"맵 ID가 {pose_speed.map_id}로 업데이트되었습니다.")
                    map_loaded = True
                else:
                    rospy.loginfo("ℹ맵 로드 요청 성공 (로컬라이제이션 대기 중)")
                    map_loaded = True  # 요청 성공했으므로 4단계 진행
            else:
                rospy.logerr(f"맵 로드 실패: {msg}")
        elif map_loaded:
            rospy.loginfo("맵이 이미 로드되어 있어 맵 로드를 건너뜁니다.")
        
        # 4단계: 로봇 위치 설정 (로컬라이제이션)
        if map_loaded:
            rospy.loginfo("4단계: 로봇 위치 설정 (로컬라이제이션)")
            pose_speed, ok, _ = await self.robot.robot_pose_speed_req(PoseSpeed(), NO_PRINT, NO_PRINT)
            if ok:
                # 현재 위치를 맵 상의 위치로 설정
                set_pose = SetRobotPose()
                set_pose.pose.x = pose_speed.pose.x
                set_pose.pose.y = pose_speed.pose.y
                set_pose.pose.theta = pose_speed.pose.theta
                
                result, ok, msg = await self.robot.set_robot_pose_req(set_pose, NO_PRINT, NO_PRINT)
                if ok:
                    rospy.loginfo(f"로봇 위치 설정 성공: ({set_pose.pose.x:.2f}, {set_pose.pose.y:.2f}, {set_pose.pose.theta:.2f})")
                    await asyncio.sleep(2)
                else:
                    rospy.logwarn(f"로봇 위치 설정 실패: {msg}")

        # 5단계: 로봇 초기화
        rospy.loginfo("5단계: 로봇 초기화")
        pose_speed, ok, _ = await self.robot.robot_pose_speed_req(PoseSpeed(), NO_PRINT, NO_PRINT)
        if ok:
            init_robot = InitRobot()
            init_robot.is_record = False
            init_robot.pose.x = pose_speed.pose.x if pose_speed else 0.0
            init_robot.pose.y = pose_speed.pose.y if pose_speed else 0.0
            init_robot.pose.theta = pose_speed.pose.theta if pose_speed else 0.0
            
            result, ok, msg = await self.robot.init_robot_req(init_robot, NO_PRINT, NO_PRINT)
            if ok:
                rospy.loginfo(f"✅ 로봇 초기화 성공: ({init_robot.pose.x:.2f}, {init_robot.pose.y:.2f}, {init_robot.pose.theta:.2f})")
                await asyncio.sleep(2)
            else:
                rospy.logwarn(f"⚠️ 로봇 초기화 실패: {msg}")
        
        # 6단계: 제어 모드를 자동 모드로 설정
        rospy.loginfo("6단계: 제어 모드를 자동 모드로 설정")
        switch_mode = SwitchControlMode()
        switch_mode.mode = ControlMode.kAuto
        result, ok, msg = await self.robot.switch_control_mode_req(switch_mode, NO_PRINT, NO_PRINT)
        if ok:
            rospy.loginfo("✅ 자동 제어 모드 설정 성공")
            await asyncio.sleep(2)
        else:
            rospy.logwarn(f"⚠️ 제어 모드 설정 실패: {msg}")
        
        # 최종 상태 확인
        rospy.loginfo("최종 상태 확인")
        state, ok, msg = await self.robot.robot_operation_state_req(OperationState(), NO_PRINT, NO_PRINT)
        if ok:
            # 디버그: state.robot과 state.nav 값 직접 출력
            rospy.loginfo(f"[DEBUG] state.robot = {state.robot} (이진: {bin(state.robot)})")
            rospy.loginfo(f"[DEBUG] state.nav = {state.nav} (이진: {bin(state.nav)})")
            rospy.loginfo(f"[DEBUG] kTaskable 값 = {OperationState.RobotBit.kTaskable}")
            rospy.loginfo(f"[DEBUG] state.robot & kTaskable = {state.robot & OperationState.RobotBit.kTaskable}")
            
            if state.robot & OperationState.RobotBit.kTaskable:
                rospy.loginfo("✅ 로봇이 작업을 받을 수 있는 상태입니다.")
            else:
                rospy.loginfo("ℹ️ 로봇이 아직 Taskable 상태가 아니지만, 작업 수행은 가능할 수 있습니다.")
            
            if state.nav & OperationState.NavBit.kImpede:
                rospy.logwarn("⚠️ 장애물이 감지되었습니다.")
            else:
                rospy.loginfo("✅ 네비게이션 경로가 깨끗합니다.")
        
        rospy.loginfo("=== 네비게이션 설정 완료 ===")
        return True

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