#!/usr/bin/env python3
"""
Mobile Robot Control Node - 모바일 로봇 제어 노드

이 노드는 Woosh 모바일 로봇(TR200)을 제어하기 위한 확장 가능한 프레임워크입니다.
Odometry 기반 정밀 제어, 부드러운 가감속, ROS 통합을 제공합니다.

Author: KATECH Robotics Team
License: MIT
"""

import rospy
import asyncio
import argparse
import math
import logging
from typing import Optional, Tuple
from dataclasses import dataclass
from enum import Enum

from woosh_robot import WooshRobot
from woosh_interface import CommuSettings, NO_PRINT
from woosh.proto.robot.robot_pb2 import RobotInfo, PoseSpeed
from woosh.proto.robot.robot_pack_pb2 import Twist


# ==================== 데이터 클래스 ====================

@dataclass
class RobotConfig:
    """로봇 연결 및 제어 설정"""
    ip: str = '169.254.128.2'
    port: int = 5480
    identity: str = 'mobile_robot_controller'
    verbose: bool = False


@dataclass
class VelocityProfileConfig:
    """속도 프로파일 설정"""
    max_speed: float = 0.2  # 최대 속도 (m/s)
    min_speed: float = 0.03  # 최소 속도 (m/s)
    accel_distance: float = 0.15  # 가속 구간 거리 (m)
    decel_distance: float = 0.2  # 감속 구간 거리 (m)
    control_hz: float = 20.0  # 제어 주기 (Hz)


@dataclass
class MotionResult:
    """이동 결과"""
    success: bool
    traveled_distance: float
    target_distance: float
    error: float
    duration: float


class MotionPhase(Enum):
    """이동 단계"""
    ACCELERATION = "🚀 가속"
    CONSTANT = "⚡ 등속"
    DECELERATION = "🛑 감속"


# ==================== 속도 프로파일 계산기 ====================

class VelocityProfileCalculator:
    """사다리꼴 속도 프로파일 계산 클래스"""
    
    def __init__(self, config: VelocityProfileConfig):
        """
        초기화
        
        Args:
            config: 속도 프로파일 설정
        """
        self.config = config
        self.accel_distance = config.accel_distance
        self.decel_distance = config.decel_distance
    
    def adjust_for_distance(self, target_distance: float) -> None:
        """
        목표 거리에 맞춰 가감속 구간 자동 조정
        
        Args:
            target_distance: 목표 이동 거리 (m)
        """
        total_accel_decel = self.config.accel_distance + self.config.decel_distance
        
        if total_accel_decel > target_distance * 0.8:
            scale = (target_distance * 0.8) / total_accel_decel
            self.accel_distance = self.config.accel_distance * scale
            self.decel_distance = self.config.decel_distance * scale
            rospy.logwarn(
                f"⚠️ 가감속 구간 자동 조정: "
                f"가속 {self.accel_distance:.2f}m, 감속 {self.decel_distance:.2f}m"
            )
    
    def calculate_speed(self, traveled_distance: float, remaining_distance: float) -> Tuple[float, MotionPhase]:
        """
        현재 위치에서의 목표 속도 계산 (사다리꼴 프로파일)
        
        Args:
            traveled_distance: 이미 이동한 거리 (m)
            remaining_distance: 남은 거리 (m)
        
        Returns:
            (목표 속도, 이동 단계)
        """
        if traveled_distance < self.accel_distance:
            # 가속 구간: 선형 증가
            progress = traveled_distance / self.accel_distance
            target_speed = self.config.min_speed + (self.config.max_speed - self.config.min_speed) * progress
            return target_speed, MotionPhase.ACCELERATION
        
        elif remaining_distance < self.decel_distance:
            # 감속 구간: 선형 감소
            progress = remaining_distance / self.decel_distance
            target_speed = self.config.min_speed + (self.config.max_speed - self.config.min_speed) * progress
            return target_speed, MotionPhase.DECELERATION
        
        else:
            # 등속 구간: 최대 속도 유지
            return self.config.max_speed, MotionPhase.CONSTANT


# ==================== 메인 컨트롤러 클래스 ====================

class MobileRobotController:
    """모바일 로봇 제어 메인 클래스"""
    
    def __init__(self, config: RobotConfig):
        """
        초기화
        
        Args:
            config: 로봇 설정
        """
        self.config = config
        self.robot: Optional[WooshRobot] = None
        self.current_pose: Optional[object] = None
        
        # ROS 노드 초기화
        rospy.init_node('mobile_robot_control', anonymous=True, disable_signals=True)
        
        rospy.loginfo("🤖 Mobile Robot Controller 초기화 완료")
        rospy.loginfo(f"   연결 대상: {config.ip}:{config.port}")
    
    # ==================== 연결 관리 ====================
    
    async def connect(self) -> None:
        """로봇에 연결하고 초기화"""
        # SDK 로거 설정
        sdk_logger = self._setup_logger()
        
        # 연결 설정
        settings = CommuSettings(
            addr=self.config.ip,
            port=self.config.port,
            identity=self.config.identity,
            logger=sdk_logger
        )
        
        # 로봇 연결
        self.robot = WooshRobot(settings)
        await self.robot.run()
        
        # 연결 검증
        await self._verify_connection()
        
        # 위치 피드백 구독
        await self.robot.robot_pose_speed_sub(self._pose_callback, NO_PRINT)
        rospy.loginfo("📍 위치 피드백 구독 시작")
    
    def _setup_logger(self) -> logging.Logger:
        """SDK 로거 설정"""
        sdk_logger = logging.getLogger(self.config.identity)
        
        if self.config.verbose:
            sdk_logger.setLevel(logging.DEBUG)
        else:
            sdk_logger.setLevel(logging.WARNING)
        
        return sdk_logger
    
    async def _verify_connection(self) -> None:
        """연결 상태 검증"""
        info, ok, msg = await self.robot.robot_info_req(RobotInfo(), NO_PRINT, NO_PRINT)
        
        if not ok:
            rospy.logerr(f"❌ 로봇 연결 실패: {msg}")
            raise ConnectionError(f"Failed to connect: {msg}")
        
        rospy.loginfo("✅ 로봇 연결 성공!")
        rospy.loginfo(f"🔋 배터리 잔량: {info.battery.power}%")
    
    async def disconnect(self) -> None:
        """로봇 연결 종료"""
        if self.robot:
            rospy.loginfo("📋 로봇 연결 종료 중...")
            await self.robot.stop()
            rospy.loginfo("✅ 연결 종료 완료")
    
    # ==================== 위치 관리 ====================
    
    def _pose_callback(self, pose_speed: PoseSpeed) -> None:
        """
        위치 업데이트 콜백 (백그라운드에서 계속 실행)
        
        Args:
            pose_speed: 로봇의 위치 및 속도 정보
        """
        self.current_pose = pose_speed.pose
    
    async def get_current_pose(self):
        """
        현재 위치 조회 (요청 방식)
        
        Returns:
            현재 위치 (Pose 객체) 또는 None
        """
        pose_speed, ok, msg = await self.robot.robot_pose_speed_req(PoseSpeed(), NO_PRINT, NO_PRINT)
        
        if ok:
            self.current_pose = pose_speed.pose
            return pose_speed.pose
        else:
            rospy.logwarn(f"⚠️ 위치 조회 실패: {msg}")
            return None
    
    @staticmethod
    def calculate_distance(pose1, pose2) -> float:
        """
        두 위치 사이의 유클리드 거리 계산
        
        Args:
            pose1: 시작 위치
            pose2: 끝 위치
        
        Returns:
            거리 (m)
        """
        dx = pose2.x - pose1.x
        dy = pose2.y - pose1.y
        return math.sqrt(dx**2 + dy**2)
    
    # ==================== 이동 제어 ====================
    
    async def move_distance(
        self,
        target_distance: float,
        speed: float = 0.2,
        angle: float = 0.0,
        timeout: float = 60.0,
        tolerance: float = 0.02,
        velocity_config: Optional[VelocityProfileConfig] = None
    ) -> MotionResult:
        """
        지정된 거리만큼 로봇을 이동 (Odometry 기반 정밀 제어)
        
        Args:
            target_distance: 목표 이동 거리 (m)
            speed: 최대 이동 속도 (m/s), 음수면 후진
            angle: 회전 각속도 (rad/s), 기본 0.0 (직진)
            timeout: 최대 대기 시간 (초)
            tolerance: 허용 오차 (m)
            velocity_config: 속도 프로파일 설정 (None이면 기본값 사용)
        
        Returns:
            MotionResult: 이동 결과
        """
        # 기본 설정 사용
        if velocity_config is None:
            velocity_config = VelocityProfileConfig(max_speed=abs(speed))
        else:
            velocity_config.max_speed = abs(speed)
        
        # 속도 프로파일 계산기 생성
        profile_calc = VelocityProfileCalculator(velocity_config)
        profile_calc.adjust_for_distance(target_distance)
        
        # 헤더 출력
        self._print_motion_header(target_distance, speed, profile_calc, velocity_config)
        
        # 시작 위치 기록
        start_pose = await self.get_current_pose()
        if start_pose is None:
            rospy.logerr("❌ 시작 위치를 가져올 수 없습니다")
            return MotionResult(False, 0.0, target_distance, target_distance, 0.0)
        
        rospy.loginfo(f"📍 시작 위치: X={start_pose.x:.3f}, Y={start_pose.y:.3f}")
        rospy.loginfo("🚀 이동 시작!")
        
        # 이동 실행
        motion_result = await self._execute_motion(
            start_pose=start_pose,
            target_distance=target_distance,
            speed_direction=1 if speed > 0 else -1,
            angle=angle,
            timeout=timeout,
            tolerance=tolerance,
            profile_calc=profile_calc,
            velocity_config=velocity_config
        )
        
        # 결과 출력
        self._print_motion_result(motion_result)
        
        return motion_result
    
    async def _execute_motion(
        self,
        start_pose,
        target_distance: float,
        speed_direction: int,
        angle: float,
        timeout: float,
        tolerance: float,
        profile_calc: VelocityProfileCalculator,
        velocity_config: VelocityProfileConfig
    ) -> MotionResult:
        """
        이동 실행 (내부 메서드)
        
        Args:
            start_pose: 시작 위치
            target_distance: 목표 거리
            speed_direction: 속도 방향 (+1: 전진, -1: 후진)
            angle: 각속도
            timeout: 타임아웃
            tolerance: 허용 오차
            profile_calc: 속도 프로파일 계산기
            velocity_config: 속도 설정
        
        Returns:
            MotionResult: 이동 결과
        """
        # 제어 루프 초기화
        control_period = 1.0 / velocity_config.control_hz
        start_time = asyncio.get_event_loop().time()
        last_log_time = start_time
        last_control_time = start_time
        
        current_speed = velocity_config.min_speed * speed_direction
        
        # 제어 루프
        while True:
            current_time = asyncio.get_event_loop().time()
            elapsed = current_time - start_time
            
            # 타임아웃 체크
            if elapsed > timeout:
                rospy.logwarn(f"⚠️ 타임아웃 ({timeout}초 초과)")
                break
            
            # Twist 명령 주기적 전송
            if current_time - last_control_time >= control_period:
                await self._send_twist_command(current_speed, angle)
                last_control_time = current_time
            
            # 현재 위치 확인
            current_pose = await self.get_current_pose()
            if current_pose is None:
                await asyncio.sleep(control_period / 2)
                continue
            
            # 이동 거리 계산
            traveled_distance = self.calculate_distance(start_pose, current_pose)
            remaining = target_distance - traveled_distance
            
            # 속도 프로파일 계산
            target_speed, phase = profile_calc.calculate_speed(traveled_distance, remaining)
            current_speed = target_speed * speed_direction
            
            # 주기적 로그 출력
            if current_time - last_log_time >= 0.5:
                rospy.loginfo(
                    f"📊 {phase.value} | {traveled_distance:.3f}m / {target_distance:.3f}m | "
                    f"속도: {abs(current_speed):.2f}m/s | 남은: {remaining:.3f}m"
                )
                last_log_time = current_time
            
            # 목표 거리 도달 확인
            if traveled_distance >= target_distance - tolerance:
                rospy.loginfo(f"✅ 목표 거리 도달! (실제: {traveled_distance:.3f}m)")
                break
            
            await asyncio.sleep(control_period / 2)
        
        # 정지
        await self._stop_robot()
        
        # 최종 위치 확인
        await asyncio.sleep(0.5)
        final_pose = await self.get_current_pose()
        final_distance = self.calculate_distance(start_pose, final_pose)
        final_time = asyncio.get_event_loop().time() - start_time
        
        return MotionResult(
            success=True,
            traveled_distance=final_distance,
            target_distance=target_distance,
            error=abs(final_distance - target_distance),
            duration=final_time
        )
    
    async def _send_twist_command(self, linear_speed: float, angular_speed: float) -> None:
        """
        Twist 명령 전송
        
        Args:
            linear_speed: 선속도 (m/s)
            angular_speed: 각속도 (rad/s)
        """
        twist = Twist(linear=linear_speed, angular=angular_speed)
        await self.robot.twist_req(twist, NO_PRINT, NO_PRINT)
    
    async def _stop_robot(self) -> None:
        """로봇 정지 (여러 번 전송으로 확실한 정지 보장)"""
        rospy.loginfo("🛑 정지 중...")
        stop_twist = Twist(linear=0.0, angular=0.0)
        
        for _ in range(3):
            await self.robot.twist_req(stop_twist, NO_PRINT, NO_PRINT)
            await asyncio.sleep(0.05)
        
        rospy.loginfo("🛑 정지 완료")
    
    # ==================== 회전 제어 ====================
    
    async def rotate(
        self,
        angle_degrees: float,
        angular_speed: float = 0.5,
        control_hz: float = 20
    ) -> bool:
        """
        제자리 회전
        
        Args:
            angle_degrees: 회전 각도 (도, 양수=좌회전, 음수=우회전)
            angular_speed: 회전 속도 (rad/s)
            control_hz: 제어 주기 (Hz)
        
        Returns:
            성공 여부
        """
        angle_rad = math.radians(angle_degrees)
        duration = abs(angle_rad / angular_speed)
        direction = 1 if angle_degrees > 0 else -1
        control_period = 1.0 / control_hz
        
        rospy.loginfo(f"🔄 회전: {angle_degrees}도 (예상 시간: {duration:.1f}초)")
        
        # Twist 명령 주기적 전송
        rotate_twist = Twist(linear=0.0, angular=direction * angular_speed)
        start_time = asyncio.get_event_loop().time()
        
        while True:
            elapsed = asyncio.get_event_loop().time() - start_time
            if elapsed >= duration:
                break
            
            await self.robot.twist_req(rotate_twist, NO_PRINT, NO_PRINT)
            await asyncio.sleep(control_period)
        
        # 정지
        await self._stop_robot()
        rospy.loginfo("✅ 회전 완료")
        
        return True
    
    # ==================== 유틸리티 ====================
    
    @staticmethod
    def _print_motion_header(
        target_distance: float,
        speed: float,
        profile_calc: VelocityProfileCalculator,
        velocity_config: VelocityProfileConfig
    ) -> None:
        """이동 시작 헤더 출력"""
        rospy.loginfo("=" * 60)
        rospy.loginfo(f"🎯 목표: {target_distance:.3f}m 이동 (최대 속도: {abs(speed):.2f}m/s)")
        rospy.loginfo(
            f"   가속 구간: {profile_calc.accel_distance:.2f}m | "
            f"감속 구간: {profile_calc.decel_distance:.2f}m"
        )
        rospy.loginfo(f"   제어 주기: {velocity_config.control_hz}Hz ({1000/velocity_config.control_hz:.0f}ms마다 명령 전송)")
        rospy.loginfo("=" * 60)
    
    @staticmethod
    def _print_motion_result(result: MotionResult) -> None:
        """이동 결과 출력"""
        rospy.loginfo("=" * 60)
        rospy.loginfo("📊 최종 결과:")
        rospy.loginfo(f"   목표 거리: {result.target_distance:.3f}m")
        rospy.loginfo(f"   실제 거리: {result.traveled_distance:.3f}m")
        rospy.loginfo(
            f"   오차: {result.error:.3f}m "
            f"({result.error/result.target_distance*100:.1f}%)"
        )
        rospy.loginfo(f"   소요 시간: {result.duration:.1f}초")
        rospy.loginfo("=" * 60)


# ==================== CLI 인터페이스 ====================

async def main():
    """메인 함수 - CLI 인터페이스"""
    parser = argparse.ArgumentParser(
        description='모바일 로봇 제어 노드',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
사용 예시:
  # 기본: 전진 1m (부드러운 가감속)
  python3 mobile_robot_control_node.py --distance 1.0 --speed 0.2
  
  # 가감속 커스텀: 빠른 가속, 긴 감속
  python3 mobile_robot_control_node.py --distance 1.0 --speed 0.3 --accel 0.1 --decel 0.3
  
  # 후진 0.5m
  python3 mobile_robot_control_node.py --distance 0.5 --speed -0.2
  
  # 90도 좌회전
  python3 mobile_robot_control_node.py --rotate 90
        """
    )
    
    # 이동 관련 인자
    parser.add_argument('--distance', type=float, default=0.5,
                       help='이동 거리 (m), 기본값: 0.5')
    parser.add_argument('--speed', type=float, default=0.2,
                       help='이동 속도 (m/s), 기본값: 0.2 (음수: 후진)')
    parser.add_argument('--rotate', type=float, default=None,
                       help='회전 각도 (도), 예: --rotate 90')
    
    # 속도 프로파일 인자
    parser.add_argument('--accel', type=float, default=0.15,
                       help='가속 구간 거리 (m), 기본값: 0.15')
    parser.add_argument('--decel', type=float, default=0.2,
                       help='감속 구간 거리 (m), 기본값: 0.2')
    parser.add_argument('--control-hz', type=float, default=20,
                       help='제어 주기 (Hz), 기본값: 20')
    
    # 연결 관련 인자
    parser.add_argument('--ip', type=str, default='169.254.128.2',
                       help='로봇 IP 주소')
    parser.add_argument('--port', type=int, default=5480,
                       help='로봇 포트')
    parser.add_argument('--verbose', action='store_true',
                       help='상세 모드 (SDK 로그 포함)')
    
    args = parser.parse_args()
    
    # 설정 생성
    robot_config = RobotConfig(
        ip=args.ip,
        port=args.port,
        verbose=args.verbose
    )
    
    velocity_config = VelocityProfileConfig(
        max_speed=abs(args.speed),
        accel_distance=args.accel,
        decel_distance=args.decel,
        control_hz=args.control_hz
    )
    
    try:
        # 컨트롤러 생성 및 연결
        controller = MobileRobotController(robot_config)
        await controller.connect()
        
        # 초기 위치 출력
        await asyncio.sleep(1.0)
        initial_pose = await controller.get_current_pose()
        if initial_pose:
            rospy.loginfo(
                f"📍 초기 위치: X={initial_pose.x:.3f}, "
                f"Y={initial_pose.y:.3f}, Theta={initial_pose.theta:.3f}"
            )
        
        # 명령 실행
        if args.rotate is not None:
            # 회전 모드
            rospy.loginfo("\n🔄 회전 모드")
            await controller.rotate(args.rotate)
        else:
            # 이동 모드
            rospy.loginfo("\n🎯 정밀 이동 모드 (Odometry 피드백)")
            await controller.move_distance(
                target_distance=args.distance,
                speed=args.speed,
                velocity_config=velocity_config
            )
        
        # 종료 대기
        rospy.loginfo("\n✅ 작업 완료! Ctrl+C로 종료하세요.")
        await asyncio.Event().wait()
    
    except asyncio.CancelledError:
        rospy.loginfo("🛑 작업 취소됨")
    except Exception as e:
        rospy.logerr(f"💥 오류 발생: {e}")
        import traceback
        rospy.logerr(traceback.format_exc())
    finally:
        if 'controller' in locals():
            await controller.disconnect()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        rospy.loginfo("👋 프로그램 종료")

