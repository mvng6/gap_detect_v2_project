#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LiDAR Corrected Mover Node (통합 노드)

LiDAR 센서값을 활용하여 로봇의 이동 거리를 실시간으로 보정하면서 
정밀하게 목표 거리만큼 이동시키는 통합 노드입니다.

기능:
    1. 초기 LiDAR 거리 측정
    2. Twist 명령으로 로봇 이동
    3. 실시간 LiDAR 거리 모니터링
    4. 목표 거리 도달 시 자동 정지
    5. 오차 발생 시 추가 보정 이동

로봇 LiDAR 배치:
    - 왼쪽 앞 LiDAR: 전진 시 전방 거리 측정
    - 오른쪽 뒤 LiDAR: 후진 시 후방 거리 측정

Author: KATECH Robotics Team
Date: 2025-12-05
"""

import rospy
import asyncio
import numpy as np
import sys
import os
from threading import Thread, Lock, Event
from typing import Optional, Tuple
from enum import Enum
from dataclasses import dataclass

# ROS 메시지 및 서비스
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Bool
from std_srvs.srv import Empty, EmptyResponse

# 커스텀 서비스
from mobile_lidar_control.srv import MoveDistance, MoveDistanceResponse

# === WooshRobot SDK 경로 추가 ===
script_dir = os.path.dirname(os.path.abspath(__file__))
woosh_sdk_dir = os.path.join(script_dir, '../../woosh_robot_py')
sys.path.insert(0, os.path.abspath(woosh_sdk_dir))

# WooshRobot SDK
try:
    from woosh_robot import WooshRobot
    from woosh_interface import CommuSettings, NO_PRINT, FULL_PRINT
    from woosh.proto.robot.robot_pack_pb2 import Twist
except ImportError as e:
    print(f"[ERROR] WooshRobot SDK를 찾을 수 없습니다: {e}")
    print(f"[INFO] woosh_robot_py 경로: {woosh_sdk_dir}")
    sys.exit(1)


class MoveState(Enum):
    """이동 상태"""
    IDLE = 0          # 대기
    ACCELERATING = 1  # 가속
    CRUISING = 2      # 정속
    DECELERATING = 3  # 감속
    CORRECTING = 4    # 보정
    COMPLETED = 5     # 완료
    ERROR = 6         # 오류
    EMERGENCY_STOP = 7  # 비상 정지


class MoveDirection(Enum):
    """이동 방향"""
    FORWARD = 1
    BACKWARD = -1
    STOPPED = 0


@dataclass
class MoveConfig:
    """이동 설정"""
    max_linear_vel: float = 0.12      # 최대 선속도 (m/s)
    linear_accel: float = 0.25        # 가속도 (m/s²)
    linear_decel: float = 0.50        # 감속도 (m/s²)
    control_rate: float = 50.0        # 제어 주기 (Hz)
    
    # LiDAR 보정 설정
    position_tolerance: float = 0.005  # 위치 허용 오차 (m) - 5mm
    correction_speed: float = 0.03     # 보정 속도 (m/s)
    max_corrections: int = 3           # 최대 보정 횟수
    
    # 타임아웃
    timeout: float = 30.0              # 전체 타임아웃 (s)


@dataclass
class LidarConfig:
    """LiDAR 설정"""
    front_angle_min: float = -0.26    # 전방 최소 각도 (rad)
    front_angle_max: float = 0.26     # 전방 최대 각도 (rad)
    rear_angle_min: float = 2.88      # 후방 최소 각도 (rad)
    rear_angle_max: float = 3.40      # 후방 최대 각도 (rad)
    min_valid_points: int = 3         # 최소 유효 포인트 수
    outlier_threshold: float = 0.5    # 이상치 임계값 (m)


class LidarCorrectedMover:
    """
    LiDAR 보정 통합 이동 컨트롤러
    
    로봇을 목표 거리만큼 이동시키면서 LiDAR로 실시간 보정합니다.
    """
    
    def __init__(self):
        """초기화"""
        # === ROS 파라미터 로드 ===
        self.robot_ip = rospy.get_param('~robot_ip', '169.254.128.2')
        self.robot_port = rospy.get_param('~robot_port', 5480)
        self.robot_identity = rospy.get_param('~robot_identity', 'lidar_corrected_mover')
        self.scan_topic = rospy.get_param('~scan_topic', '/mobile_lidar/scan')
        
        # 이동 설정
        self.move_config = MoveConfig(
            max_linear_vel=rospy.get_param('~max_linear_vel', 0.12),
            linear_accel=rospy.get_param('~linear_accel', 0.25),
            linear_decel=rospy.get_param('~linear_decel', 0.50),
            control_rate=rospy.get_param('~control_rate', 50.0),
            position_tolerance=rospy.get_param('~position_tolerance', 0.005),
            correction_speed=rospy.get_param('~correction_speed', 0.03),
            max_corrections=rospy.get_param('~max_corrections', 3),
            timeout=rospy.get_param('~timeout', 30.0)
        )
        
        # LiDAR 설정
        self.lidar_config = LidarConfig(
            front_angle_min=rospy.get_param('~front_angle_min', -0.26),
            front_angle_max=rospy.get_param('~front_angle_max', 0.26),
            rear_angle_min=rospy.get_param('~rear_angle_min', 2.88),
            rear_angle_max=rospy.get_param('~rear_angle_max', 3.40),
            min_valid_points=rospy.get_param('~min_valid_points', 3),
            outlier_threshold=rospy.get_param('~outlier_threshold', 0.5)
        )
        
        # === 상태 변수 ===
        self.robot: Optional[WooshRobot] = None
        self.is_connected = False
        self.is_moving = False
        self.emergency_stop_flag = Event()
        
        self.current_scan: Optional[LaserScan] = None
        self.move_state = MoveState.IDLE
        self.move_direction = MoveDirection.STOPPED
        
        # 이동 측정
        self.target_distance = 0.0
        self.initial_lidar_distance: Optional[float] = None
        self.current_lidar_distance: Optional[float] = None
        self.traveled_distance = 0.0
        
        self.lock = Lock()
        self.asyncio_loop: Optional[asyncio.AbstractEventLoop] = None
        
        # === ROS Subscriber ===
        self.scan_sub = rospy.Subscriber(
            self.scan_topic,
            LaserScan,
            self.scan_callback,
            queue_size=1
        )
        
        # === ROS Publisher ===
        self.traveled_pub = rospy.Publisher(
            '/mobile_lidar_control/traveled_distance',
            Float32,
            queue_size=10
        )
        self.remaining_pub = rospy.Publisher(
            '/mobile_lidar_control/remaining_distance',
            Float32,
            queue_size=10
        )
        self.is_moving_pub = rospy.Publisher(
            '/mobile_lidar_control/is_moving',
            Bool,
            queue_size=10
        )
        
        # === ROS 서비스 ===
        self.move_srv = rospy.Service(
            '/mobile_lidar_control/move_with_correction',
            MoveDistance,
            self.handle_move_request
        )
        self.stop_srv = rospy.Service(
            '/mobile_lidar_control/emergency_stop',
            Empty,
            self.handle_emergency_stop
        )
        
        # Asyncio 스레드 시작
        self._start_asyncio_thread()
        
        rospy.loginfo("=" * 60)
        rospy.loginfo("LiDAR Corrected Mover 초기화 완료")
        rospy.loginfo(f"  Robot: {self.robot_ip}:{self.robot_port}")
        rospy.loginfo(f"  Scan Topic: {self.scan_topic}")
        rospy.loginfo(f"  Position Tolerance: {self.move_config.position_tolerance*1000:.1f}mm")
        rospy.loginfo("=" * 60)
    
    # =========================================================================
    # Asyncio 관리
    # =========================================================================
    
    def _start_asyncio_thread(self):
        """Asyncio 스레드 시작"""
        def run_loop():
            self.asyncio_loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.asyncio_loop)
            self.asyncio_loop.run_forever()
        
        thread = Thread(target=run_loop, daemon=True)
        thread.start()
        
        # 루프 시작 대기
        while self.asyncio_loop is None:
            rospy.sleep(0.01)
    
    def _run_async(self, coro):
        """비동기 코루틴을 동기적으로 실행"""
        if self.asyncio_loop is None:
            return None
        future = asyncio.run_coroutine_threadsafe(coro, self.asyncio_loop)
        return future.result(timeout=self.move_config.timeout)
    
    # =========================================================================
    # 로봇 연결
    # =========================================================================
    
    async def connect(self) -> bool:
        """로봇에 연결"""
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
            
            self.is_connected = True
            rospy.loginfo("✅ 로봇 연결 성공!")
            return True
            
        except Exception as e:
            rospy.logerr(f"로봇 연결 오류: {e}")
            return False
    
    # =========================================================================
    # LiDAR 처리
    # =========================================================================
    
    def scan_callback(self, msg: LaserScan):
        """LiDAR 스캔 콜백"""
        with self.lock:
            self.current_scan = msg
            
            # 현재 방향에 맞는 거리 계산
            if self.move_direction == MoveDirection.FORWARD:
                self.current_lidar_distance = self._get_distance_in_range(
                    msg, 
                    self.lidar_config.front_angle_min, 
                    self.lidar_config.front_angle_max
                )
            elif self.move_direction == MoveDirection.BACKWARD:
                self.current_lidar_distance = self._get_distance_in_range(
                    msg,
                    self.lidar_config.rear_angle_min,
                    self.lidar_config.rear_angle_max
                )
            
            # 이동 중이면 이동 거리 계산 및 발행
            if self.is_moving and self.initial_lidar_distance is not None:
                if self.current_lidar_distance is not None:
                    if self.move_direction == MoveDirection.FORWARD:
                        self.traveled_distance = self.initial_lidar_distance - self.current_lidar_distance
                    else:
                        self.traveled_distance = -(self.initial_lidar_distance - self.current_lidar_distance)
                    
                    self.traveled_pub.publish(Float32(self.traveled_distance))
                    remaining = abs(self.target_distance) - abs(self.traveled_distance)
                    self.remaining_pub.publish(Float32(max(0, remaining)))
    
    def _get_distance_in_range(self, scan: LaserScan, 
                                angle_min: float, angle_max: float) -> Optional[float]:
        """지정된 각도 범위의 평균 거리 계산"""
        if scan is None or len(scan.ranges) == 0:
            return None
        
        idx_min = self._angle_to_index(scan, angle_min)
        idx_max = self._angle_to_index(scan, angle_max)
        
        if idx_min is None or idx_max is None:
            return None
        
        if idx_min > idx_max:
            idx_min, idx_max = idx_max, idx_min
        
        ranges_in_range = scan.ranges[idx_min:idx_max+1]
        
        valid_ranges = [
            r for r in ranges_in_range
            if (not np.isinf(r) and 
                not np.isnan(r) and 
                scan.range_min < r < scan.range_max)
        ]
        
        if len(valid_ranges) < self.lidar_config.min_valid_points:
            return None
        
        median = np.median(valid_ranges)
        filtered_ranges = [
            r for r in valid_ranges
            if abs(r - median) < self.lidar_config.outlier_threshold
        ]
        
        if len(filtered_ranges) < self.lidar_config.min_valid_points:
            return median
        
        return np.mean(filtered_ranges)
    
    def _angle_to_index(self, scan: LaserScan, angle: float) -> Optional[int]:
        """각도를 인덱스로 변환"""
        if scan.angle_increment == 0:
            return None
        
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        
        if angle < scan.angle_min or angle > scan.angle_max:
            if angle < scan.angle_min:
                return 0
            else:
                return len(scan.ranges) - 1
        
        idx = int((angle - scan.angle_min) / scan.angle_increment)
        return max(0, min(idx, len(scan.ranges) - 1))
    
    # =========================================================================
    # 이동 제어 (핵심 로직)
    # =========================================================================
    
    async def move_with_correction(self, target_distance: float) -> Tuple[bool, str]:
        """
        LiDAR 보정을 적용하여 목표 거리만큼 이동
        
        Args:
            target_distance: 목표 거리 (m), 음수는 후진
            
        Returns:
            (성공 여부, 메시지)
        """
        # 연결 확인
        if not self.is_connected:
            if not await self.connect():
                return False, "로봇 연결 실패"
        
        # LiDAR 데이터 확인
        if self.current_scan is None:
            return False, "LiDAR 데이터 없음 - lidar_subscriber.py 실행 필요"
        
        # 방향 설정
        if target_distance > 0:
            self.move_direction = MoveDirection.FORWARD
        elif target_distance < 0:
            self.move_direction = MoveDirection.BACKWARD
        else:
            return True, "이동 거리가 0입니다"
        
        self.target_distance = target_distance
        self.traveled_distance = 0.0
        self.emergency_stop_flag.clear()
        
        # 초기 LiDAR 거리 측정 (잠시 대기 후)
        await asyncio.sleep(0.1)
        
        with self.lock:
            if self.move_direction == MoveDirection.FORWARD:
                self.initial_lidar_distance = self._get_distance_in_range(
                    self.current_scan,
                    self.lidar_config.front_angle_min,
                    self.lidar_config.front_angle_max
                )
            else:
                self.initial_lidar_distance = self._get_distance_in_range(
                    self.current_scan,
                    self.lidar_config.rear_angle_min,
                    self.lidar_config.rear_angle_max
                )
        
        if self.initial_lidar_distance is None:
            return False, "초기 LiDAR 거리 측정 실패"
        
        rospy.loginfo(f"[이동 시작] 목표: {target_distance:.3f}m, 초기 거리: {self.initial_lidar_distance:.3f}m")
        
        self.is_moving = True
        self.is_moving_pub.publish(Bool(True))
        
        try:
            # 1차 이동: 사다리꼴 속도 프로파일
            success, msg = await self._execute_trapezoidal_move(target_distance)
            
            if not success:
                return False, msg
            
            # 보정 이동
            for correction_count in range(self.move_config.max_corrections):
                await asyncio.sleep(0.2)  # LiDAR 안정화 대기
                
                # 현재 오차 계산
                error = abs(target_distance) - abs(self.traveled_distance)
                
                rospy.loginfo(f"[보정 {correction_count+1}] 오차: {error*1000:.1f}mm")
                
                if abs(error) <= self.move_config.position_tolerance:
                    rospy.loginfo(f"✅ 목표 도달! 총 이동: {self.traveled_distance:.4f}m")
                    return True, f"성공: {self.traveled_distance:.4f}m 이동 (오차: {error*1000:.1f}mm)"
                
                # 보정 이동
                correction_distance = error if target_distance > 0 else -error
                rospy.loginfo(f"[보정 이동] {correction_distance*1000:.1f}mm")
                
                await self._execute_correction_move(correction_distance)
            
            # 최종 결과
            final_error = abs(target_distance) - abs(self.traveled_distance)
            return True, f"완료: {self.traveled_distance:.4f}m 이동 (최종 오차: {final_error*1000:.1f}mm)"
            
        except Exception as e:
            rospy.logerr(f"이동 중 오류: {e}")
            await self._stop_robot()
            return False, f"오류: {e}"
        
        finally:
            self.is_moving = False
            self.is_moving_pub.publish(Bool(False))
            self.move_direction = MoveDirection.STOPPED
    
    async def _execute_trapezoidal_move(self, target_distance: float) -> Tuple[bool, str]:
        """사다리꼴 속도 프로파일로 이동"""
        direction = 1 if target_distance > 0 else -1
        abs_target = abs(target_distance)
        
        current_vel = 0.0
        dt = 1.0 / self.move_config.control_rate
        
        max_vel = self.move_config.max_linear_vel
        accel = self.move_config.linear_accel
        decel = self.move_config.linear_decel
        
        # 감속 시작 거리 계산
        decel_distance = (max_vel ** 2) / (2 * decel)
        
        start_time = rospy.Time.now()
        
        while not rospy.is_shutdown():
            # 비상 정지 체크
            if self.emergency_stop_flag.is_set():
                await self._stop_robot()
                return False, "비상 정지"
            
            # 타임아웃 체크
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed > self.move_config.timeout:
                await self._stop_robot()
                return False, "타임아웃"
            
            # 현재 이동 거리 (LiDAR 기반)
            with self.lock:
                traveled = abs(self.traveled_distance)
            
            remaining = abs_target - traveled
            
            # 목표 도달 체크
            if remaining <= 0.002:  # 2mm 이내면 완료
                await self._stop_robot()
                return True, "목표 도달"
            
            # 속도 프로파일 계산
            if remaining <= decel_distance:
                # 감속 구간
                target_vel = np.sqrt(2 * decel * remaining)
                target_vel = max(0.01, min(target_vel, max_vel))
                self.move_state = MoveState.DECELERATING
            elif current_vel < max_vel:
                # 가속 구간
                target_vel = min(current_vel + accel * dt, max_vel)
                self.move_state = MoveState.ACCELERATING
            else:
                # 정속 구간
                target_vel = max_vel
                self.move_state = MoveState.CRUISING
            
            current_vel = target_vel
            
            # Twist 명령 전송
            twist_cmd = Twist(linear=current_vel * direction, angular=0.0)
            await self.robot.twist_req(twist_cmd, NO_PRINT, NO_PRINT)
            
            await asyncio.sleep(dt)
        
        return False, "ROS 종료"
    
    async def _execute_correction_move(self, correction_distance: float):
        """보정 이동 (저속)"""
        direction = 1 if correction_distance > 0 else -1
        abs_correction = abs(correction_distance)
        
        dt = 1.0 / self.move_config.control_rate
        correction_vel = self.move_config.correction_speed
        
        initial_traveled = abs(self.traveled_distance)
        
        self.move_state = MoveState.CORRECTING
        
        while not rospy.is_shutdown():
            if self.emergency_stop_flag.is_set():
                break
            
            with self.lock:
                current_traveled = abs(self.traveled_distance)
            
            correction_moved = current_traveled - initial_traveled
            
            if correction_moved >= abs_correction:
                break
            
            twist_cmd = Twist(linear=correction_vel * direction, angular=0.0)
            await self.robot.twist_req(twist_cmd, NO_PRINT, NO_PRINT)
            
            await asyncio.sleep(dt)
        
        await self._stop_robot()
    
    async def _stop_robot(self):
        """로봇 정지"""
        if self.robot is not None:
            try:
                twist_cmd = Twist(linear=0.0, angular=0.0)
                await self.robot.twist_req(twist_cmd, NO_PRINT, NO_PRINT)
                self.move_state = MoveState.IDLE
            except Exception as e:
                rospy.logerr(f"정지 명령 오류: {e}")
    
    # =========================================================================
    # ROS 서비스 핸들러
    # =========================================================================
    
    def handle_move_request(self, req) -> MoveDistanceResponse:
        """이동 서비스 핸들러"""
        if self.is_moving:
            return MoveDistanceResponse(False, "이미 이동 중입니다")
        
        rospy.loginfo(f"[서비스 호출] 목표 거리: {req.distance:.3f}m")
        
        try:
            success, msg = self._run_async(self.move_with_correction(req.distance))
            return MoveDistanceResponse(success, msg)
        except Exception as e:
            rospy.logerr(f"이동 서비스 오류: {e}")
            return MoveDistanceResponse(False, str(e))
    
    def handle_emergency_stop(self, req) -> EmptyResponse:
        """비상 정지 서비스 핸들러"""
        rospy.logwarn("🚨 비상 정지!")
        self.emergency_stop_flag.set()
        self._run_async(self._stop_robot())
        self.move_state = MoveState.EMERGENCY_STOP
        return EmptyResponse()


def main():
    """메인 함수"""
    rospy.init_node('lidar_corrected_mover', anonymous=False)
    
    mover = LidarCorrectedMover()
    
    rospy.loginfo("=" * 60)
    rospy.loginfo("LiDAR Corrected Mover 실행 중")
    rospy.loginfo("서비스:")
    rospy.loginfo("  - /mobile_lidar_control/move_with_correction")
    rospy.loginfo("  - /mobile_lidar_control/emergency_stop")
    rospy.loginfo("토픽:")
    rospy.loginfo("  - /mobile_lidar_control/traveled_distance")
    rospy.loginfo("  - /mobile_lidar_control/remaining_distance")
    rospy.loginfo("  - /mobile_lidar_control/is_moving")
    rospy.loginfo("=" * 60)
    
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("사용자에 의해 종료됨")

