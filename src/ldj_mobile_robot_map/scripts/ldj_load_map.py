#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
woosh 모바일 로봇(TR-200) 맵 로드 및 제어 스크립트

패키지: ldj_mobile_robot_map
작성자: LDJ (KATECH)
설명: 로봇 연결, 맵 로드, 로컬라이제이션, Twist/Navigation 제어
"""

import rospy
import asyncio
import math
import numpy as np
import sys
import os
from queue import Queue, Empty
from threading import Thread

# === battery_check.py에서 배터리 출력 함수 가져오기 ===
# testbed_operation 패키지의 battery_check.py 사용
script_dir = os.path.dirname(os.path.abspath(__file__))
battery_check_dir = os.path.join(script_dir, '../../testbed_operation/scripts')
sys.path.insert(0, os.path.abspath(battery_check_dir))

try:
    from battery_check import print_battery_status  # 배터리 상태 출력 함수
except ImportError:
    # battery_check를 찾을 수 없는 경우 간단한 대체 함수 사용
    def print_battery_status(battery_level):
        rospy.loginfo(f"🔋 배터리 잔량: {battery_level}%")

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
        self.robot_identity = rospy.get_param('~robot_identity','map_load_ctrl')
        
        self.robot = None
        
        # 네비게이션 작업 완료 이벤트
        self.task_finished_event = asyncio.Event()
        self.task_result = None  # 작업 결과 저장

    async def connect(self):
        """로봇 연결 및 기본 설정"""
        settings = CommuSettings(addr=self.robot_ip, port=self.robot_port, identity=self.robot_identity)
        self.robot = WooshRobot(settings)
        await self.robot.run()

        info, ok, _ = await self.robot.robot_info_req(RobotInfo(), NO_PRINT, NO_PRINT)
        if not ok:
            raise RuntimeError("로봇 연결 실패")
        
        # battery_check.py의 함수를 사용하여 배터리 상태 출력
        print_battery_status(info.battery.power)
        rospy.loginfo("로봇 연결 성공!")

        # 연결 후 맵 로드 및 초기화 수행
        await self._setup_navigation()

    async def _setup_navigation(self):
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

    def _task_proc_callback(self, info: TaskProc):
        """작업 진행 상황을 모니터링하는 콜백 함수"""
        state_name = TaskState.Name(info.state)
        rospy.loginfo(f"[작업 업데이트] ID={info.robot_task_id}, 상태={state_name}, 메시지='{info.msg}'")
        
        # 작업이 종료 상태에 도달했는지 확인
        if info.state in [TaskState.kCompleted, TaskState.kFailed, TaskState.kCanceled]:
            self.task_result = info.state
            rospy.loginfo(f"작업 종료: {state_name}")
            self.task_finished_event.set()

    def _pose_speed_callback(self, info: PoseSpeed):
        """로봇의 현재 위치를 주기적으로 출력하는 콜백 함수"""
        rospy.loginfo(f"[위치] X={info.pose.x:.2f}, Y={info.pose.y:.2f}, Theta={info.pose.theta:.2f}")

    async def navigate_to_goal(self, target_x: float, target_y: float, target_theta: float = 0.0, timeout: float = 120.0):
        """
        지정된 목표 위치로 네비게이션 이동
        
        Args:
            target_x: 목표 X 좌표 (m)
            target_y: 목표 Y 좌표 (m)
            target_theta: 목표 방향 (rad)
            timeout: 최대 대기 시간 (초)
        
        Returns:
            bool: 이동 성공 여부
        """
        rospy.loginfo("=== 네비게이션 이동 테스트 시작 ===")
        
        # 현재 위치 확인
        pose_speed, ok, _ = await self.robot.robot_pose_speed_req(PoseSpeed(), NO_PRINT, NO_PRINT)
        if ok:
            rospy.loginfo(f"현재 위치: X={pose_speed.pose.x:.2f}, Y={pose_speed.pose.y:.2f}, Theta={pose_speed.pose.theta:.2f}")
        
        rospy.loginfo(f"목표 위치: X={target_x:.2f}, Y={target_y:.2f}, Theta={target_theta:.2f}")
        
        # 이벤트 초기화
        self.task_finished_event.clear()
        self.task_result = None
        
        # 작업 진행 및 위치 정보 구독
        rospy.loginfo("작업 진행 및 위치 업데이트 구독 시작...")
        await self.robot.robot_task_process_sub(self._task_proc_callback)
        await self.robot.robot_pose_speed_sub(self._pose_speed_callback)
        
        # ExecTask 메시지 생성
        task_id = 88888  # 고유한 작업 ID
        nav_task = ExecTask(
            task_id=task_id,
            type=TaskType.kCarry,  # 특정 지점으로 이동하는 작업 유형
            direction=TaskDirection.kDirectionUndefined,  # 방향 미정의 (단순 이동)
        )
        nav_task.pose.x = target_x
        nav_task.pose.y = target_y
        nav_task.pose.theta = target_theta
        
        rospy.loginfo(f"작업 설정: ID={task_id}, type={TaskType.Name(nav_task.type)}")
        
        # 네비게이션 작업 전송 (FULL_PRINT로 상세 디버그 정보 출력)
        rospy.loginfo("🚀 네비게이션 작업 전송 중...")
        _, ok, msg = await self.robot.exec_task_req(nav_task, FULL_PRINT, FULL_PRINT)
        
        if not ok:
            rospy.logerr(f"❌ 네비게이션 작업 전송 실패: {msg}")
            return False
        
        rospy.loginfo("✅ 네비게이션 작업이 성공적으로 전송되었습니다.")
        rospy.loginfo(f"작업 완료 대기 중... (최대 {timeout}초)")
        
        # 작업 완료 대기
        try:
            await asyncio.wait_for(self.task_finished_event.wait(), timeout=timeout)
            
            if self.task_result == TaskState.kCompleted:
                rospy.loginfo("✅ 네비게이션 목표에 성공적으로 도달했습니다!")
                return True
            elif self.task_result == TaskState.kFailed:
                rospy.logwarn("⚠️ 네비게이션 작업이 실패했습니다.")
                return False
            elif self.task_result == TaskState.kCanceled:
                rospy.logwarn("⚠️ 네비게이션 작업이 취소되었습니다.")
                return False
                
        except asyncio.TimeoutError:
            rospy.logwarn(f"⚠️ 네비게이션 작업 타임아웃 ({timeout}초 초과)")
            return False
        
        return False

    async def run(self):
        await self.connect()
        
        # 현재 위치 확인
        pose_speed, ok, _ = await self.robot.robot_pose_speed_req(PoseSpeed(), NO_PRINT, NO_PRINT)
        if not ok:
            rospy.logerr("현재 위치를 가져올 수 없습니다.")
            return
        
        current_x = pose_speed.pose.x
        current_y = pose_speed.pose.y
        current_theta = pose_speed.pose.theta
        
        # ========================================
        # 상대 이동 거리 설정 (맵 좌표계 기준)
        # ========================================
        delta_x = 0.05   # X방향 이동 거리 (m) - 양수: +X방향
        delta_y = 0.0   # Y방향 이동 거리 (m) - 양수: +Y방향
        
        # 목표 좌표 계산 (현재 위치 + 상대 이동)
        target_x = current_x + delta_x
        target_y = current_y + delta_y
        target_theta = current_theta  # 방향은 유지
        
        rospy.loginfo(f"\n{'='*60}")
        rospy.loginfo(f"현재 위치: ({current_x:.2f}, {current_y:.2f}, {current_theta:.2f})")
        rospy.loginfo(f"상대 이동: (Δx={delta_x:.2f}, Δy={delta_y:.2f})")
        rospy.loginfo(f"목표 위치: ({target_x:.2f}, {target_y:.2f}, {target_theta:.2f})")
        rospy.loginfo(f"{'='*60}\n")
        
        success = await self.navigate_to_goal(target_x, target_y, target_theta)
        
        if success:
            rospy.loginfo("🎉 네비게이션 테스트 완료!")
        else:
            rospy.logwarn("네비게이션 테스트 실패 또는 타임아웃")        

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
    rospy.init_node('mobile_map_load', anonymous=False)

    thread = Thread(target=run_asyncio, daemon=True)
    thread.start()

    rospy.loginfo("로봇 연결 및 맵 로드 시작!")
    rospy.spin()