#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Twist Motion Controller Node

모바일 로봇의 Twist 명령 기반 정밀 이동 제어 노드입니다.
WooshRobot SDK를 사용하여 사다리꼴 속도 프로파일로 로봇을 이동시킵니다.
Phase 3에서 LiDAR 기반 보정 기능과 통합될 예정입니다.

Features:
    - 사다리꼴 속도 프로파일 (가속-정속-감속)
    - 비상 정지 기능
    - 속도/가속도 제한 설정
    - ROS 서비스 인터페이스

Author: KATECH Robotics Team
Date: 2025-12-04
"""

import rospy
import asyncio
import numpy as np
import sys
import os
from queue import Queue, Empty
from threading import Thread, Lock
from typing import Optional, Tuple
from enum import Enum

# ROS 메시지/서비스 타입
from std_srvs.srv import Empty as EmptySrv, EmptyResponse

# === WooshRobot SDK 경로 추가 ===
script_dir = os.path.dirname(os.path.abspath(__file__))
woosh_sdk_dir = os.path.join(script_dir, '../../woosh_robot_py')
sys.path.insert(0, os.path.abspath(woosh_sdk_dir))

# WooshRobot SDK import
try:
    from woosh_robot import WooshRobot
    from woosh_interface import CommuSettings, NO_PRINT, FULL_PRINT
    from woosh.proto.robot.robot_pack_pb2 import Twist
    from woosh.proto.robot.robot_pb2 import RobotInfo, PoseSpeed, OperationState
except ImportError as e:
    print(f"[ERROR] WooshRobot SDK를 찾을 수 없습니다: {e}")
    sys.exit(1)

# mobile_lidar_control 자체 서비스 import
try:
    from mobile_lidar_control.srv import MoveDistance, MoveDistanceResponse
    SERVICE_AVAILABLE = True
except ImportError:
    # 서비스를 찾을 수 없는 경우 (빌드 전)
    rospy.logwarn("mobile_lidar_control.srv를 찾을 수 없습니다. 먼저 catkin_make를 실행하세요.")
    SERVICE_AVAILABLE = False


class MotionState(Enum):
    """로봇 이동 상태"""
    IDLE = 0           # 대기 중
    ACCELERATING = 1   # 가속 중
    CRUISING = 2       # 정속 이동 중
    DECELERATING = 3   # 감속 중
    STOPPING = 4       # 정지 중
    EMERGENCY_STOP = 5 # 비상 정지


class TwistMotionController:
    """
    Twist 명령 기반 모션 컨트롤러
    
    사다리꼴 속도 프로파일을 사용하여 정밀한 거리 이동을 수행합니다.
    """
    
    def __init__(self):
        """초기화: ROS 파라미터 로드 및 설정"""
        # === ROS 파라미터 로드 ===
        self.robot_ip = rospy.get_param('~robot_ip', '169.254.128.2')
        self.robot_port = rospy.get_param('~robot_port', 5480)
        self.robot_identity = rospy.get_param('~robot_identity', 'twist_controller')
        
        # === 제어 파라미터 ===
        self.max_linear_vel = rospy.get_param('~max_linear_vel', 0.12)    # 최대 선속도 (m/s)
        self.min_linear_vel = rospy.get_param('~min_linear_vel', 0.03)    # 최소 선속도 (m/s)
        self.linear_accel = rospy.get_param('~linear_accel', 0.25)        # 선가속도 (m/s²)
        self.linear_decel = rospy.get_param('~linear_decel', 0.50)        # 선감속도 (m/s²)
        self.control_rate = rospy.get_param('~control_rate', 50.0)        # 제어 주기 (Hz)
        self.position_tolerance = rospy.get_param('~position_tolerance', 0.03)  # 위치 허용 오차 (m)
        
        # === 안전 파라미터 ===
        self.timeout = rospy.get_param('~timeout', 30.0)                  # 이동 타임아웃 (s)
        self.emergency_decel = rospy.get_param('~emergency_decel', 1.0)   # 비상 감속도 (m/s²)
        
        # === 로봇 연결 객체 ===
        self.robot: Optional[WooshRobot] = None
        self.is_connected = False
        
        # === 이동 상태 ===
        self.motion_state = MotionState.IDLE
        self.current_speed = 0.0           # 현재 속도 (부호 포함)
        self.target_distance = 0.0         # 목표 이동 거리
        self.estimated_distance = 0.0      # 추정 이동 거리
        self.is_moving = False
        self.emergency_stop_requested = False
        
        # === 스레드 통신 ===
        self.command_queue = Queue()
        self.result_queue = Queue()
        self.state_lock = Lock()
        
        # === ROS 서비스 ===
        if SERVICE_AVAILABLE:
            self.move_service = rospy.Service(
                '/mobile_lidar_control/move_distance',
                MoveDistance,
                self.handle_move_request
            )
        
        self.stop_service = rospy.Service(
            '/mobile_lidar_control/emergency_stop',
            EmptySrv,
            self.handle_emergency_stop
        )
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("Twist Motion Controller 초기화")
        rospy.loginfo(f"  Robot IP: {self.robot_ip}")
        rospy.loginfo(f"  Robot Port: {self.robot_port}")
        rospy.loginfo(f"  Max Velocity: {self.max_linear_vel} m/s")
        rospy.loginfo(f"  Acceleration: {self.linear_accel} m/s²")
        rospy.loginfo(f"  Deceleration: {self.linear_decel} m/s²")
        rospy.loginfo(f"  Control Rate: {self.control_rate} Hz")
        rospy.loginfo("=" * 60)
    
    # =========================================================================
    # 로봇 연결 관련
    # =========================================================================
    
    async def connect(self) -> bool:
        """로봇에 연결합니다."""
        try:
            rospy.loginfo(f"로봇 연결 시도: {self.robot_ip}:{self.robot_port}")
            
            settings = CommuSettings(
                addr=self.robot_ip,
                port=self.robot_port,
                identity=self.robot_identity
            )
            
            self.robot = WooshRobot(settings)
            
            if not await self.robot.run():
                rospy.logerr("로봇 연결 실패")
                return False
            
            # 로봇 정보 확인
            info, ok, msg = await self.robot.robot_info_req(RobotInfo(), NO_PRINT, NO_PRINT)
            if ok:
                rospy.loginfo(f"✅ 로봇 연결 성공! (배터리: {info.battery.power}%)")
            else:
                rospy.logwarn(f"로봇 정보 요청 실패: {msg}")
            
            self.is_connected = True
            return True
            
        except Exception as e:
            rospy.logerr(f"로봇 연결 중 예외 발생: {e}")
            self.is_connected = False
            return False
    
    # =========================================================================
    # 이동 제어 관련
    # =========================================================================
    
    async def move_distance(self, distance: float) -> Tuple[bool, str]:
        """
        지정된 거리만큼 이동합니다.
        
        Args:
            distance: 이동 거리 (m), 양수=전진, 음수=후진
            
        Returns:
            (성공 여부, 메시지)
        """
        if not self.is_connected or self.robot is None:
            return False, "로봇이 연결되지 않았습니다"
        
        # 너무 작은 거리는 무시
        if abs(distance) < 0.005:
            return True, f"거리가 너무 작음: {distance:.3f}m"
        
        # 상태 초기화
        with self.state_lock:
            self.target_distance = distance
            self.estimated_distance = 0.0
            self.current_speed = 0.0
            self.is_moving = True
            self.emergency_stop_requested = False
            self.motion_state = MotionState.ACCELERATING
        
        direction = np.sign(distance)  # +1 또는 -1
        period = 1.0 / self.control_rate
        start_time = asyncio.get_event_loop().time()
        last_time = start_time
        
        rospy.loginfo(f"이동 시작: {distance:+.3f}m ({'전진' if direction > 0 else '후진'})")
        
        try:
            while self.is_moving and not rospy.is_shutdown():
                now = asyncio.get_event_loop().time()
                dt = max(now - last_time, period)
                last_time = now
                
                # 타임아웃 체크
                if now - start_time > self.timeout:
                    rospy.logwarn("이동 타임아웃!")
                    await self._stop_robot()
                    return False, "타임아웃"
                
                # 비상 정지 체크
                if self.emergency_stop_requested:
                    await self._emergency_stop()
                    return False, "비상 정지"
                
                # 속도 계산 및 이동
                with self.state_lock:
                    remaining = self.target_distance - self.estimated_distance
                    abs_remaining = abs(remaining)
                    speed_abs = abs(self.current_speed)
                    
                    # 정지 조건
                    if abs_remaining < self.position_tolerance:
                        if speed_abs < self.min_linear_vel:
                            self.current_speed = 0.0
                            self.is_moving = False
                            self.motion_state = MotionState.IDLE
                        else:
                            # 강제 감속
                            self.current_speed *= 0.7
                            self.motion_state = MotionState.STOPPING
                    else:
                        # 감속 거리 계산
                        stop_dist = max((speed_abs ** 2) / (2 * self.linear_decel), 0.015)
                        
                        # 목표 속도 결정
                        if abs_remaining <= stop_dist + 0.01:
                            target_speed = 0.0
                            self.motion_state = MotionState.DECELERATING
                        else:
                            target_speed = self.max_linear_vel * direction
                            if speed_abs < self.max_linear_vel * 0.9:
                                self.motion_state = MotionState.ACCELERATING
                            else:
                                self.motion_state = MotionState.CRUISING
                        
                        # 가속/감속 적용
                        if target_speed > self.current_speed:
                            self.current_speed = min(
                                self.current_speed + self.linear_accel * dt, 
                                target_speed
                            )
                        else:
                            self.current_speed = max(
                                self.current_speed - self.linear_decel * dt, 
                                target_speed
                            )
                    
                    # Twist 명령 전송
                    await self.robot.twist_req(
                        Twist(linear=self.current_speed, angular=0.0),
                        NO_PRINT, NO_PRINT
                    )
                    
                    # 추정 거리 업데이트
                    self.estimated_distance += self.current_speed * dt
                
                await asyncio.sleep(period)
            
            # 최종 정지 명령
            await self._stop_robot()
            
            result_msg = f"완료: 목표 {distance:+.3f}m → 추정 {self.estimated_distance:+.3f}m"
            rospy.loginfo(result_msg)
            return True, result_msg
            
        except Exception as e:
            await self._stop_robot()
            rospy.logerr(f"이동 중 오류: {e}")
            return False, str(e)
    
    async def _stop_robot(self):
        """로봇을 정지시킵니다."""
        if self.robot:
            await self.robot.twist_req(
                Twist(linear=0.0, angular=0.0),
                NO_PRINT, NO_PRINT
            )
        with self.state_lock:
            self.current_speed = 0.0
            self.is_moving = False
            self.motion_state = MotionState.IDLE
    
    async def _emergency_stop(self):
        """비상 정지를 수행합니다."""
        rospy.logwarn("비상 정지 실행!")
        
        if self.robot:
            # 즉시 정지 명령
            await self.robot.twist_req(
                Twist(linear=0.0, angular=0.0),
                NO_PRINT, NO_PRINT
            )
        
        with self.state_lock:
            self.current_speed = 0.0
            self.is_moving = False
            self.motion_state = MotionState.EMERGENCY_STOP
            self.emergency_stop_requested = False
    
    # =========================================================================
    # ROS 서비스 핸들러
    # =========================================================================
    
    def handle_move_request(self, req):
        """이동 요청 서비스 핸들러"""
        if not self.is_connected:
            return MoveDistanceResponse(False, "로봇이 연결되지 않았습니다")
        
        if self.is_moving:
            return MoveDistanceResponse(False, "이미 이동 중입니다")
        
        rospy.loginfo(f"이동 요청 수신: {req.distance:+.3f}m")
        
        # 명령 큐에 추가
        self.command_queue.put(req.distance)
        
        # 결과 대기
        try:
            success, msg = self.result_queue.get(timeout=self.timeout + 5.0)
            return MoveDistanceResponse(success, msg)
        except Empty:
            return MoveDistanceResponse(False, "응답 타임아웃")
    
    def handle_emergency_stop(self, req):
        """비상 정지 서비스 핸들러"""
        rospy.logwarn("비상 정지 요청 수신!")
        with self.state_lock:
            self.emergency_stop_requested = True
        return EmptyResponse()
    
    # =========================================================================
    # 메인 루프
    # =========================================================================
    
    async def control_loop(self):
        """제어 루프: 명령 큐에서 명령을 가져와 실행"""
        while not rospy.is_shutdown():
            try:
                distance = self.command_queue.get_nowait()
            except Empty:
                await asyncio.sleep(0.01)
                continue
            
            success, msg = await self.move_distance(distance)
            self.result_queue.put((success, msg))
    
    async def run(self):
        """메인 실행 함수"""
        # 1. 로봇 연결
        if not await self.connect():
            rospy.logerr("로봇 연결 실패. 종료합니다.")
            return
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("Twist Motion Controller 실행 중")
        rospy.loginfo("서비스: /mobile_lidar_control/move_distance")
        rospy.loginfo("서비스: /mobile_lidar_control/move_distance")
        rospy.loginfo("비상정지: /mobile_lidar_control/emergency_stop")
        rospy.loginfo("=" * 60)
        
        # 2. 제어 루프 실행
        await self.control_loop()
    
    def get_status(self) -> dict:
        """현재 상태 반환"""
        with self.state_lock:
            return {
                'connected': self.is_connected,
                'motion_state': self.motion_state.name,
                'current_speed': self.current_speed,
                'target_distance': self.target_distance,
                'estimated_distance': self.estimated_distance,
                'is_moving': self.is_moving
            }


# =============================================================================
# Asyncio + ROS 통합
# =============================================================================

def run_asyncio_loop(controller: TwistMotionController):
    """별도 스레드에서 asyncio 이벤트 루프 실행"""
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    
    try:
        loop.run_until_complete(controller.run())
    except Exception as e:
        rospy.logerr(f"Asyncio 루프 오류: {e}")
    finally:
        loop.close()


def main():
    """메인 함수"""
    rospy.init_node('twist_motion_controller', anonymous=False)
    
    controller = TwistMotionController()
    
    # 별도 스레드에서 asyncio 루프 실행
    asyncio_thread = Thread(target=run_asyncio_loop, args=(controller,), daemon=True)
    asyncio_thread.start()
    
    rospy.loginfo("ROS spin 시작...")
    rospy.spin()
    
    rospy.loginfo("Twist Motion Controller 종료")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("사용자에 의해 종료됨")

