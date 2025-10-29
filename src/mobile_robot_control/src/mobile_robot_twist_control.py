#!/usr/bin/env python3
"""
모바일 로봇 Twist 기반 거리 제어 노드

이 노드는 맵 없이 Twist 속도 제어와 Odometry 피드백을 사용하여
정확한 거리만큼 로봇을 이동시킵니다.

사용법:
    python3 mobile_robot_twist_control.py --distance 0.5 --speed 0.2
"""
import rospy
import asyncio
import argparse
import math

# from woosh_robot import WooshRobot, CommuSettings

from woosh_robot import WooshRobot
from woosh_interface import CommuSettings, NO_PRINT

from woosh.proto.robot.robot_pb2 import RobotInfo, PoseSpeed
from woosh.proto.robot.robot_pack_pb2 import Twist


class MobileRobotTwistController:
    """Twist 방식으로 모바일 로봇의 거리를 제어하는 클래스"""
    
    def __init__(self, verbose=False):
        """ROS 노드 및 파라미터 초기화"""
        rospy.init_node('mobile_robot_twist_control', anonymous=True, disable_signals=True)
        
        # 파라미터 로드
        self.robot_ip = rospy.get_param('~robot_ip', '169.254.128.2')
        self.robot_port = rospy.get_param('~robot_port', 5480)
        self.robot_identity = rospy.get_param('~robot_identity', 'twist_controller')
        self.verbose = verbose  # 상세 모드 플래그
        
        self.robot = None
        self.current_pose = None  # 현재 위치 저장
        
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
        
        # 위치 구독 시작
        await self.robot.robot_pose_speed_sub(self._pose_callback, NO_PRINT)
        rospy.loginfo("📍 위치 피드백 구독 시작")
    
    def _pose_callback(self, pose_speed: PoseSpeed):
        """위치 업데이트 콜백 (백그라운드에서 계속 실행)"""
        self.current_pose = pose_speed.pose
    
    async def get_current_pose(self):
        """현재 위치 조회 (요청 방식)"""
        pose_speed, ok, msg = await self.robot.robot_pose_speed_req(PoseSpeed(), NO_PRINT, NO_PRINT)
        if ok:
            self.current_pose = pose_speed.pose
            return pose_speed.pose
        else:
            rospy.logwarn(f"⚠️ 위치 조회 실패: {msg}")
            return None
    
    def calculate_distance(self, pose1, pose2):
        """두 위치 사이의 유클리드 거리 계산"""
        dx = pose2.x - pose1.x
        dy = pose2.y - pose1.y
        return math.sqrt(dx**2 + dy**2)
    
    async def move_distance(self, target_distance, speed=0.1, angle=0.0, 
                           timeout=60.0, tolerance=0.02, control_hz=20,
                           accel_distance=0.15, decel_distance=0.2):
        """
        지정된 거리만큼 로봇을 이동시킵니다 (부드러운 가감속 적용)
        
        Args:
            target_distance: 목표 이동 거리 (m)
            speed: 최대 이동 속도 (m/s), 기본 0.1m/s
            angle: 회전 각속도 (rad/s), 기본 0.0 (직진)
            timeout: 최대 대기 시간 (초), 기본 60초
            tolerance: 허용 오차 (m), 기본 2cm
            control_hz: Twist 명령 주기 (Hz), 기본 20Hz
            accel_distance: 가속 구간 거리 (m), 기본 0.15m
            decel_distance: 감속 구간 거리 (m), 기본 0.2m
        
        Returns:
            (bool, float): (성공 여부, 실제 이동 거리)
        """
        rospy.loginfo("=" * 60)
        rospy.loginfo(f"🎯 목표: {target_distance:.3f}m 이동 (최대 속도: {speed:.2f}m/s)")
        rospy.loginfo(f"   가속 구간: {accel_distance:.2f}m | 감속 구간: {decel_distance:.2f}m")
        rospy.loginfo(f"   제어 주기: {control_hz}Hz ({1000/control_hz:.0f}ms마다 명령 전송)")
        rospy.loginfo("=" * 60)
        
        # 1. 시작 위치 기록
        start_pose = await self.get_current_pose()
        if start_pose is None:
            rospy.logerr("❌ 시작 위치를 가져올 수 없습니다")
            return False, 0.0
        
        rospy.loginfo(f"📍 시작 위치: X={start_pose.x:.3f}, Y={start_pose.y:.3f}")
        rospy.loginfo("🚀 이동 시작!")
        
        # 2. 제어 루프 변수
        control_period = 1.0 / control_hz  # 제어 주기 (초)
        start_time = asyncio.get_event_loop().time()
        last_log_time = start_time
        last_control_time = start_time
        
        # 속도 프로파일 설정
        min_speed = 0.03  # 최소 속도 (m/s) - 너무 느리면 로봇이 멈춤
        max_speed = abs(speed)  # 최대 속도
        speed_direction = 1 if speed > 0 else -1  # 전진(+1) 또는 후진(-1)
        
        # 가감속 구간이 전체 거리보다 크면 자동 조정
        total_accel_decel = accel_distance + decel_distance
        if total_accel_decel > target_distance * 0.8:
            scale = (target_distance * 0.8) / total_accel_decel
            accel_distance *= scale
            decel_distance *= scale
            rospy.logwarn(f"⚠️ 가감속 구간 자동 조정: 가속 {accel_distance:.2f}m, 감속 {decel_distance:.2f}m")
        
        current_speed = min_speed * speed_direction  # 최소 속도로 시작
        
        # 3. Twist 명령을 주기적으로 전송하면서 거리 모니터링
        while True:
            # 현재 시간
            current_time = asyncio.get_event_loop().time()
            elapsed = current_time - start_time
            
            # 타임아웃 체크
            if elapsed > timeout:
                rospy.logwarn(f"⚠️ 타임아웃 ({timeout}초 초과)")
                break
            
            # ★ 핵심: Twist 명령을 주기적으로 재전송 (조이스틱처럼)
            if current_time - last_control_time >= control_period:
                move_twist = Twist(linear=current_speed, angular=angle)
                await self.robot.twist_req(move_twist, NO_PRINT, NO_PRINT)
                last_control_time = current_time
            
            # 현재 위치 확인
            current_pose = await self.get_current_pose()
            if current_pose is None:
                await asyncio.sleep(control_period / 2)
                continue
            
            # 이동 거리 계산
            traveled_distance = self.calculate_distance(start_pose, current_pose)
            remaining = target_distance - traveled_distance
            
            # ★ 핵심: 사다리꼴 속도 프로파일 계산
            if traveled_distance < accel_distance:
                # 🚀 가속 구간: 선형 증가
                progress = traveled_distance / accel_distance  # 0.0 ~ 1.0
                target_speed = min_speed + (max_speed - min_speed) * progress
                phase = "🚀 가속"
            elif remaining < decel_distance:
                # 🛑 감속 구간: 선형 감소
                progress = remaining / decel_distance  # 1.0 ~ 0.0
                target_speed = min_speed + (max_speed - min_speed) * progress
                phase = "🛑 감속"
            else:
                # ⚡ 등속 구간: 최대 속도 유지
                target_speed = max_speed
                phase = "⚡ 등속"
            
            # 방향 적용
            current_speed = target_speed * speed_direction
            
            # 주기적 로그 출력 (0.5초마다)
            if current_time - last_log_time >= 0.5:
                rospy.loginfo(
                    f"📊 {phase} | {traveled_distance:.3f}m / {target_distance:.3f}m | "
                    f"속도: {abs(current_speed):.2f}m/s | 남은: {remaining:.3f}m"
                )
                last_log_time = current_time
            
            # 목표 거리 도달 확인
            if traveled_distance >= target_distance - tolerance:
                rospy.loginfo(f"✅ 목표 거리 도달! (실제: {traveled_distance:.3f}m)")
                break
            
            # 제어 주기의 절반만큼 대기 (응답성 향상)
            await asyncio.sleep(control_period / 2)
        
        # 4. 정지 명령을 여러 번 전송 (확실한 정지)
        rospy.loginfo("🛑 정지 중...")
        stop_twist = Twist(linear=0.0, angular=0.0)
        for _ in range(3):  # 3번 전송
            await self.robot.twist_req(stop_twist, NO_PRINT, NO_PRINT)
            await asyncio.sleep(0.05)
        rospy.loginfo("🛑 정지 완료")
        
        # 5. 최종 위치 확인
        await asyncio.sleep(0.5)
        final_pose = await self.get_current_pose()
        final_distance = self.calculate_distance(start_pose, final_pose)
        
        rospy.loginfo("=" * 60)
        rospy.loginfo(f"📊 최종 결과:")
        rospy.loginfo(f"   목표 거리: {target_distance:.3f}m")
        rospy.loginfo(f"   실제 거리: {final_distance:.3f}m")
        rospy.loginfo(f"   오차: {abs(final_distance - target_distance):.3f}m ({abs(final_distance - target_distance)/target_distance*100:.1f}%)")
        rospy.loginfo(f"   최종 위치: X={final_pose.x:.3f}, Y={final_pose.y:.3f}")
        rospy.loginfo("=" * 60)
        
        return True, final_distance
    
    async def move_distance_simple(self, target_distance, speed=0.1, angle=0.0, control_hz=20):
        """
        간단한 시간 기반 이동 (Odometry 피드백 없음)
        
        Args:
            target_distance: 목표 이동 거리 (m)
            speed: 이동 속도 (m/s)
            angle: 회전 각속도 (rad/s)
            control_hz: Twist 명령 주기 (Hz)
        
        Returns:
            bool: 성공 여부
        """
        rospy.loginfo("=" * 60)
        rospy.loginfo(f"🎯 간단 이동: {target_distance:.3f}m (시간 기반)")
        rospy.loginfo("=" * 60)
        
        # 이동 시간 계산
        duration = target_distance / speed if speed > 0 else 0
        control_period = 1.0 / control_hz
        num_commands = int(duration / control_period)
        
        rospy.loginfo(f"⏱️  예상 시간: {duration:.1f}초")
        rospy.loginfo(f"   제어 주기: {control_hz}Hz ({num_commands}번 명령 전송)")
        rospy.loginfo("🚀 이동 시작!")
        
        # Twist 명령을 주기적으로 전송
        move_twist = Twist(linear=speed, angular=angle)
        start_time = asyncio.get_event_loop().time()
        
        while True:
            elapsed = asyncio.get_event_loop().time() - start_time
            if elapsed >= duration:
                break
            
            # Twist 명령 전송
            await self.robot.twist_req(move_twist)
            await asyncio.sleep(control_period)
        
        # 정지 (여러 번 전송)
        rospy.loginfo("🛑 정지 중...")
        stop_twist = Twist(linear=0.0, angular=0.0)
        for _ in range(3):
            await self.robot.twist_req(stop_twist)
            await asyncio.sleep(0.05)
        rospy.loginfo("🛑 정지 완료")
        
        return True
    
    async def rotate(self, angle_degrees, angular_speed=0.5, control_hz=20):
        """
        제자리 회전
        
        Args:
            angle_degrees: 회전 각도 (도, 양수=좌회전, 음수=우회전)
            angular_speed: 회전 속도 (rad/s)
            control_hz: Twist 명령 주기 (Hz)
        
        Returns:
            bool: 성공 여부
        """
        angle_rad = math.radians(angle_degrees)
        duration = abs(angle_rad / angular_speed)
        direction = 1 if angle_degrees > 0 else -1
        control_period = 1.0 / control_hz
        
        rospy.loginfo(f"🔄 회전: {angle_degrees}도 (예상 시간: {duration:.1f}초)")
        
        # Twist 명령을 주기적으로 전송
        rotate_twist = Twist(linear=0.0, angular=direction * angular_speed)
        start_time = asyncio.get_event_loop().time()
        
        while True:
            elapsed = asyncio.get_event_loop().time() - start_time
            if elapsed >= duration:
                break
            
            await self.robot.twist_req(rotate_twist)
            await asyncio.sleep(control_period)
        
        # 정지 (여러 번 전송)
        stop_twist = Twist(linear=0.0, angular=0.0)
        for _ in range(3):
            await self.robot.twist_req(stop_twist)
            await asyncio.sleep(0.05)
        rospy.loginfo("✅ 회전 완료")
        
        return True
    
    async def stop(self):
        """로봇 연결 종료"""
        if self.robot:
            rospy.loginfo("📋 로봇 연결 종료 중...")
            await self.robot.stop()
            rospy.loginfo("✅ 연결 종료 완료")


async def main():
    """메인 함수"""
    # 명령줄 인자 파싱
    parser = argparse.ArgumentParser(
        description='모바일 로봇 Twist 거리 제어',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
사용 예시:
  # 기본: 전진 1m (부드러운 가감속)
  python3 mobile_robot_twist_control.py --distance 1.0 --speed 0.2
  
  # 가감속 커스텀: 빠른 가속, 긴 감속
  python3 mobile_robot_twist_control.py --distance 1.0 --speed 0.3 --accel 0.1 --decel 0.3
  
  # 후진 0.5m (속도를 음수로)
  python3 mobile_robot_twist_control.py --distance 0.5 --speed -0.1
  
  # 후진 0.5m (--backward 옵션 사용)
  python3 mobile_robot_twist_control.py --distance 0.5 --backward
  
  # 90도 좌회전
  python3 mobile_robot_twist_control.py --rotate 90
        """
    )
    parser.add_argument('--distance', type=float, default=0.5, 
                       help='이동 거리 (m), 기본값: 0.5')
    parser.add_argument('--speed', type=float, default=0.1, 
                       help='이동 속도 (m/s), 기본값: 0.1 (음수: 후진)')
    parser.add_argument('--backward', action='store_true',
                       help='후진 모드 (speed를 음수로 변환)')
    parser.add_argument('--simple', action='store_true',
                       help='간단 모드 (Odometry 피드백 없음)')
    parser.add_argument('--rotate', type=float, default=None,
                       help='회전 각도 (도), 예: --rotate 90')
    parser.add_argument('--verbose', action='store_true',
                       help='상세 모드 (SDK 로그 포함)')
    parser.add_argument('--accel', type=float, default=0.15,
                       help='가속 구간 거리 (m), 기본값: 0.15')
    parser.add_argument('--decel', type=float, default=0.2,
                       help='감속 구간 거리 (m), 기본값: 0.2')
    args = parser.parse_args()
    
    # --backward 옵션 처리
    if args.backward and args.speed > 0:
        args.speed = -args.speed
        rospy.loginfo(f"🔙 후진 모드: 속도를 {-args.speed:.2f}m/s → {args.speed:.2f}m/s로 변경")
    
    # --verbose 옵션 처리
    if args.verbose:
        rospy.loginfo("📢 상세 모드: SDK 로그 활성화")
    
    try:
        # 컨트롤러 생성 및 연결
        controller = MobileRobotTwistController(verbose=args.verbose)
        await controller.connect()
        
        # 초기 위치 출력
        await asyncio.sleep(1.0)  # 구독 안정화 대기
        initial_pose = await controller.get_current_pose()
        if initial_pose:
            rospy.loginfo(f"📍 초기 위치: X={initial_pose.x:.3f}, Y={initial_pose.y:.3f}, Theta={initial_pose.theta:.3f}")
        
        # 회전 명령
        if args.rotate is not None:
            rospy.loginfo("\n🔄 회전 모드")
            await controller.rotate(args.rotate)
        # 이동 명령
        else:
            if args.simple:
                rospy.loginfo("\n🚀 간단 이동 모드 (시간 기반)")
                await controller.move_distance_simple(args.distance, args.speed)
            else:
                rospy.loginfo("\n🎯 정밀 이동 모드 (Odometry 피드백)")
                await controller.move_distance(
                    args.distance, 
                    args.speed,
                    accel_distance=args.accel,
                    decel_distance=args.decel
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
            await controller.stop()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except (rospy.ROSInterruptException, KeyboardInterrupt):
        rospy.loginfo("👋 프로그램 종료")

