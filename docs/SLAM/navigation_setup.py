#!/usr/bin/env python3
"""
TR-200 모바일로봇 SLAM 기반 네비게이션 초기화 스크립트

이 스크립트는 저장된 맵을 이용한 네비게이션을 수행하기 전에 
필요한 모든 초기화 절차를 자동으로 수행합니다.

사용법:
    python navigation_setup.py --ip <로봇IP> [--scene <장면명>] [--x <초기X>] [--y <초기Y>] [--theta <초기각도>]

예시:
    # 기본 실행 (첫 번째 맵 자동 로드, 원점에서 시작)
    python navigation_setup.py --ip 169.254.128.2
    
    # 특정 맵과 초기 위치 지정
    python navigation_setup.py --ip 169.254.128.2 --scene "my_map" --x 0.5 --y 0.3 --theta 0.0

작성자: KATECH Robotics Lab
작성일: 2025-12-02
"""

import sys
import asyncio
import argparse
from typing import Optional, Tuple, List

# Woosh Robot SDK 임포트
from woosh_robot import WooshRobot
from woosh_interface import CommuSettings, NO_PRINT, FULL_PRINT

# 로봇 상태 관련 Protobuf 메시지
from woosh.proto.robot.robot_pb2 import (
    PoseSpeed,
    OperationState,
    TaskProc,
    Scene,
    Mode,
)

# 맵 관련 Protobuf 메시지
from woosh.proto.map.map_pack_pb2 import SceneList

# 로봇 제어 관련 Protobuf 메시지
from woosh.proto.robot.robot_pack_pb2 import (
    InitRobot,
    SetRobotPose,
    SwitchMap,
    SwitchControlMode,
    ExecTask,
)

# 상수 정의
from woosh.proto.util.robot_pb2 import ControlMode
from woosh.proto.util.task_pb2 import (
    Type as TaskType,
    State as TaskState,
    Direction as TaskDirection,
)


class NavigationSetup:
    """네비게이션 초기화를 위한 헬퍼 클래스"""
    
    def __init__(self, robot: WooshRobot, verbose: bool = True):
        """
        Args:
            robot: WooshRobot 인스턴스
            verbose: 상세 로그 출력 여부
        """
        self.robot = robot
        self.verbose = verbose
        self.print_level = FULL_PRINT if verbose else NO_PRINT
    
    def _print(self, msg: str):
        """조건부 출력"""
        if self.verbose:
            print(msg)
    
    def _print_header(self, step_num: int, title: str):
        """단계 헤더 출력"""
        if self.verbose:
            print(f"\n{'='*60}")
            print(f"STEP {step_num}: {title}")
            print(f"{'='*60}")
    
    # ==================== STEP 1: 장면/맵 목록 확인 ====================
    async def get_available_scenes(self) -> List[str]:
        """
        로봇에 저장된 사용 가능한 장면(맵) 목록을 조회합니다.
        
        Returns:
            사용 가능한 장면 이름 리스트
        """
        self._print_header(1, "장면/맵 목록 확인")
        
        scene_list_req = SceneList()
        scene_list, ok, msg = await self.robot.scene_list_req(
            scene_list_req, NO_PRINT, self.print_level
        )
        
        available_scenes = []
        if ok and scene_list:
            self._print("✅ 사용 가능한 장면 목록:")
            for i, scene in enumerate(scene_list.scenes):
                maps_info = f", 포함된 맵: {list(scene.maps)}" if scene.maps else ""
                self._print(f"   {i+1}. {scene.name}{maps_info}")
                available_scenes.append(scene.name)
            
            if not available_scenes:
                self._print("⚠️  사용 가능한 장면이 없습니다!")
        else:
            self._print(f"❌ 장면 목록 요청 실패: {msg}")
        
        return available_scenes
    
    # ==================== STEP 2: 현재 상태 확인 ====================
    async def get_current_status(self) -> Tuple[Optional[PoseSpeed], int, bool]:
        """
        현재 로봇의 위치와 맵 상태를 확인합니다.
        
        Returns:
            (PoseSpeed 객체, map_id, 맵 로드 여부)
        """
        self._print_header(2, "현재 상태 확인")
        
        pose_speed, ok, msg = await self.robot.robot_pose_speed_req(
            PoseSpeed(), NO_PRINT, self.print_level
        )
        
        if ok:
            self._print(f"현재 위치: X={pose_speed.pose.x:.2f}, Y={pose_speed.pose.y:.2f}, Theta={pose_speed.pose.theta:.2f}")
            self._print(f"맵 ID: {pose_speed.map_id}")
            
            map_loaded = pose_speed.map_id != 0
            if map_loaded:
                self._print(f"✅ 맵이 로드되어 있습니다 (ID: {pose_speed.map_id})")
            else:
                self._print("⚠️  맵이 로드되지 않았습니다 (map_id == 0)")
            
            return pose_speed, pose_speed.map_id, map_loaded
        else:
            self._print(f"❌ 위치 정보 요청 실패: {msg}")
            return None, 0, False
    
    # ==================== STEP 3: 맵 로드 ====================
    async def load_map(self, scene_name: str, map_name: str = "") -> bool:
        """
        지정된 장면(맵)을 로드합니다.
        
        Args:
            scene_name: 로드할 장면 이름
            map_name: 특정 맵 이름 (선택사항)
        
        Returns:
            로드 성공 여부
        """
        self._print_header(3, "맵 로드")
        self._print(f"🗺️  장면 '{scene_name}' 로드 중...")
        
        switch_map = SwitchMap()
        switch_map.scene_name = scene_name
        if map_name:
            switch_map.map_name = map_name
        
        _, ok, msg = await self.robot.switch_map_req(
            switch_map, NO_PRINT, self.print_level
        )
        
        if ok:
            self._print(f"✅ 장면 '{scene_name}' 로드 요청 성공")
            self._print("⏳ 맵 로드 완료 대기 중 (3초)...")
            await asyncio.sleep(3)
            
            # 로드 확인
            pose_speed, _, map_loaded = await self.get_current_status()
            if map_loaded:
                self._print(f"✅ 맵 로드 확인 완료 (ID: {pose_speed.map_id})")
                return True
            else:
                self._print("⚠️  맵 로드 후에도 map_id가 0입니다. 추가 대기...")
                await asyncio.sleep(2)
                return True  # 일단 성공으로 처리
        else:
            self._print(f"❌ 맵 로드 실패: {msg}")
            return False
    
    # ==================== STEP 4: 로봇 위치 설정 (로컬라이제이션) ====================
    async def set_robot_pose(self, x: float, y: float, theta: float) -> bool:
        """
        로봇의 현재 위치를 맵 상에 설정합니다 (로컬라이제이션).
        
        중요: 이 좌표는 실제 로봇이 맵에서 있는 위치와 일치해야 합니다!
        
        Args:
            x: X 좌표 (미터)
            y: Y 좌표 (미터)
            theta: 방향 (라디안)
        
        Returns:
            설정 성공 여부
        """
        self._print_header(4, "로봇 위치 설정 (로컬라이제이션)")
        self._print(f"📍 위치 설정: X={x:.2f}, Y={y:.2f}, Theta={theta:.2f}")
        self._print("   ⚠️  주의: 이 좌표는 실제 로봇 위치와 일치해야 합니다!")
        
        set_pose = SetRobotPose()
        set_pose.pose.x = x
        set_pose.pose.y = y
        set_pose.pose.theta = theta
        
        _, ok, msg = await self.robot.set_robot_pose_req(
            set_pose, NO_PRINT, self.print_level
        )
        
        if ok:
            self._print("✅ 로봇 위치 설정 성공")
            await asyncio.sleep(2)
            return True
        else:
            self._print(f"❌ 로봇 위치 설정 실패: {msg}")
            return False
    
    # ==================== STEP 5: 로봇 초기화 ====================
    async def initialize_robot(self, x: float = 0.0, y: float = 0.0, theta: float = 0.0) -> bool:
        """
        로봇을 초기화합니다.
        
        Args:
            x, y, theta: 초기화 시 위치 (일반적으로 set_robot_pose와 동일)
        
        Returns:
            초기화 성공 여부
        """
        self._print_header(5, "로봇 초기화")
        
        init_robot = InitRobot()
        init_robot.is_record = False
        init_robot.pose.x = x
        init_robot.pose.y = y
        init_robot.pose.theta = theta
        
        _, ok, msg = await self.robot.init_robot_req(
            init_robot, NO_PRINT, self.print_level
        )
        
        if ok:
            self._print("✅ 로봇 초기화 성공")
            await asyncio.sleep(2)
            return True
        else:
            self._print(f"❌ 로봇 초기화 실패: {msg}")
            return False
    
    # ==================== STEP 6: 제어 모드 설정 ====================
    async def set_control_mode_auto(self) -> bool:
        """
        로봇의 제어 모드를 자동(kAuto)으로 설정합니다.
        
        Returns:
            설정 성공 여부
        """
        self._print_header(6, "제어 모드 설정 (자동)")
        
        switch_mode = SwitchControlMode()
        switch_mode.mode = ControlMode.kAuto
        
        _, ok, msg = await self.robot.switch_control_mode_req(
            switch_mode, NO_PRINT, self.print_level
        )
        
        if ok:
            self._print("✅ 자동 제어 모드 설정 성공")
            await asyncio.sleep(1)
            return True
        else:
            self._print(f"❌ 제어 모드 설정 실패: {msg}")
            return False
    
    # ==================== STEP 7: 최종 상태 확인 ====================
    async def verify_taskable(self) -> bool:
        """
        로봇이 작업을 받을 수 있는 상태(kTaskable)인지 확인합니다.
        
        Returns:
            작업 수행 가능 여부
        """
        self._print_header(7, "최종 상태 확인")
        
        state, ok, msg = await self.robot.robot_operation_state_req(
            OperationState(), NO_PRINT, self.print_level
        )
        
        if ok:
            self._print(f"robot 비트: {state.robot} (이진: {bin(state.robot)})")
            self._print(f"nav 비트: {state.nav} (이진: {bin(state.nav)})")
            
            # kTaskable 비트 확인
            is_taskable = bool(state.robot & OperationState.RobotBit.kTaskable)
            
            # 장애물 확인
            has_obstacle = bool(state.nav & OperationState.NavBit.kImpede)
            
            if is_taskable:
                self._print("✅ 로봇이 작업을 받을 수 있는 상태입니다 (kTaskable)")
            else:
                self._print("❌ 로봇이 작업을 받을 수 없는 상태입니다")
            
            if has_obstacle:
                self._print("⚠️  장애물이 감지되었습니다")
            else:
                self._print("✅ 네비게이션 경로가 깨끗합니다")
            
            return is_taskable
        else:
            self._print(f"❌ 운행 상태 확인 실패: {msg}")
            return False
    
    # ==================== 전체 초기화 프로세스 ====================
    async def run_full_setup(
        self,
        scene_name: Optional[str] = None,
        init_x: float = 0.0,
        init_y: float = 0.0,
        init_theta: float = 0.0
    ) -> bool:
        """
        네비게이션을 위한 전체 초기화 프로세스를 실행합니다.
        
        Args:
            scene_name: 로드할 장면 이름 (None이면 첫 번째 맵 자동 선택)
            init_x, init_y, init_theta: 초기 로봇 위치
        
        Returns:
            초기화 성공 여부
        """
        print("\n" + "="*60)
        print("🚀 네비게이션 초기화 시작")
        print("="*60)
        
        # STEP 1: 사용 가능한 장면 확인
        available_scenes = await self.get_available_scenes()
        if not available_scenes:
            self._print("\n❌ 사용 가능한 맵이 없습니다!")
            self._print("💡 해결 방법: 윈도우 프로그램에서 맵을 생성하고 로봇에 저장하세요.")
            return False
        
        # STEP 2: 현재 상태 확인
        pose_speed, map_id, map_loaded = await self.get_current_status()
        
        # STEP 3: 맵 로드 (필요한 경우)
        if not map_loaded:
            target_scene = scene_name if scene_name else available_scenes[0]
            
            if target_scene not in available_scenes:
                self._print(f"❌ 지정된 장면 '{target_scene}'이(가) 존재하지 않습니다!")
                self._print(f"   사용 가능한 장면: {available_scenes}")
                return False
            
            if not await self.load_map(target_scene):
                self._print("❌ 맵 로드 실패")
                return False
        
        # STEP 4: 로봇 위치 설정
        await self.set_robot_pose(init_x, init_y, init_theta)
        
        # STEP 5: 로봇 초기화
        await self.initialize_robot(init_x, init_y, init_theta)
        
        # STEP 6: 제어 모드 설정
        await self.set_control_mode_auto()
        
        # STEP 7: 최종 상태 확인
        is_ready = await self.verify_taskable()
        
        # 결과 출력
        print("\n" + "="*60)
        if is_ready:
            print("🎉 네비게이션 초기화 성공!")
            print("   로봇이 네비게이션 명령을 받을 준비가 되었습니다.")
        else:
            print("❌ 네비게이션 초기화 실패")
            print("\n💡 추가 해결 방법:")
            print("   1. 로봇을 맵의 시작점(원점) 근처로 수동 이동")
            print("   2. 윈도우 프로그램에서 로봇 위치를 수동 설정")
            print("   3. 초기 좌표(--x, --y, --theta)를 실제 위치에 맞게 조정")
            print("   4. 로봇 재시작 후 다시 시도")
        print("="*60)
        
        return is_ready


async def main():
    """메인 함수"""
    # 명령줄 인수 파싱
    parser = argparse.ArgumentParser(
        description="TR-200 모바일로봇 SLAM 네비게이션 초기화",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
예시:
    # 기본 실행 (첫 번째 맵 자동 로드)
    python navigation_setup.py --ip 169.254.128.2
    
    # 특정 장면과 초기 위치 지정
    python navigation_setup.py --ip 169.254.128.2 --scene "factory_map" --x 0.5 --y 0.3
    
    # 조용한 모드 (로그 최소화)
    python navigation_setup.py --ip 169.254.128.2 --quiet
        """
    )
    parser.add_argument("--ip", type=str, default="169.254.128.2",
                        help="로봇 IP 주소 (기본값: 169.254.128.2)")
    parser.add_argument("--port", type=int, default=5480,
                        help="로봇 포트 (기본값: 5480)")
    parser.add_argument("--scene", type=str, default=None,
                        help="로드할 장면(맵) 이름 (기본값: 첫 번째 맵)")
    parser.add_argument("--x", type=float, default=0.0,
                        help="초기 X 좌표 (기본값: 0.0)")
    parser.add_argument("--y", type=float, default=0.0,
                        help="초기 Y 좌표 (기본값: 0.0)")
    parser.add_argument("--theta", type=float, default=0.0,
                        help="초기 방향 각도 (라디안, 기본값: 0.0)")
    parser.add_argument("--quiet", "-q", action="store_true",
                        help="조용한 모드 (최소 로그)")
    parser.add_argument("--test-nav", action="store_true",
                        help="초기화 후 간단한 네비게이션 테스트 실행")
    parser.add_argument("--nav-x", type=float, default=1.0,
                        help="테스트 네비게이션 목표 X (기본값: 1.0)")
    parser.add_argument("--nav-y", type=float, default=0.5,
                        help="테스트 네비게이션 목표 Y (기본값: 0.5)")
    
    args = parser.parse_args()
    
    # 연결 설정
    settings = CommuSettings(
        addr=args.ip,
        port=args.port,
        identity="nav-setup-script"
    )
    
    robot = WooshRobot(settings)
    
    try:
        # 로봇 연결
        print(f"🔌 로봇에 연결 중... ({args.ip}:{args.port})")
        if not await robot.run():
            print("❌ 로봇 연결 실패")
            print("   - 로봇 IP 주소와 포트를 확인하세요")
            print("   - 로봇이 켜져 있고 네트워크에 연결되어 있는지 확인하세요")
            return 1
        print("✅ 로봇 연결 성공")
        
        # 네비게이션 초기화
        setup = NavigationSetup(robot, verbose=not args.quiet)
        
        is_ready = await setup.run_full_setup(
            scene_name=args.scene,
            init_x=args.x,
            init_y=args.y,
            init_theta=args.theta
        )
        
        if not is_ready:
            return 1
        
        # 테스트 네비게이션 (옵션)
        if args.test_nav:
            print("\n" + "="*60)
            print("🧪 테스트 네비게이션 실행")
            print("="*60)
            
            input(f"엔터를 눌러 ({args.nav_x}, {args.nav_y})로 이동을 시작하세요...")
            
            nav_task = ExecTask(
                task_id=99999,
                type=TaskType.kParking,
                direction=TaskDirection.kDirectionUndefined,
            )
            nav_task.pose.x = args.nav_x
            nav_task.pose.y = args.nav_y
            nav_task.pose.theta = 0.0
            
            _, ok, msg = await robot.exec_task_req(nav_task, FULL_PRINT, FULL_PRINT)
            
            if ok:
                print("✅ 네비게이션 작업 전송 성공!")
                print("⏳ 작업 완료를 기다립니다 (30초)...")
                
                # 작업 진행 상태 콜백
                def task_callback(info: TaskProc):
                    state_name = TaskState.Name(info.state)
                    print(f"📍 작업 상태: {state_name}")
                    if info.msg:
                        print(f"   메시지: {info.msg}")
                
                await robot.robot_task_process_sub(task_callback, NO_PRINT)
                await asyncio.sleep(30)
            else:
                print(f"❌ 네비게이션 실패: {msg}")
                return 1
        
        return 0
        
    except KeyboardInterrupt:
        print("\n⏹️  사용자에 의해 중단되었습니다")
        return 130
    except Exception as e:
        print(f"\n❌ 오류 발생: {e}")
        return 1
    finally:
        # 연결 종료
        print("\n🔌 로봇 연결 종료 중...")
        if robot.comm.is_connected():
            try:
                await robot.stop()
                print("✅ 연결이 안전하게 종료되었습니다")
            except Exception as e:
                print(f"⚠️  연결 종료 중 오류: {e}")


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))

