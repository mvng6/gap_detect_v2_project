
'''
TR-200 로봇을 지정된 목표 위치로 이동시키는 예제 스크립트.

이 스크립트는 Woosh Robot SDK를 사용하여 로봇에 무선으로 연결하고,
명령줄 인수로 제공된 X, Y, Theta 좌표로 이동 명령을 보냅니다.

사용법:
python tr200_move_to_goal.py --ip <robot_ip> --x <target_x> --y <target_y> --theta <target_theta>

예시:
python tr200_move_to_goal.py --ip 169.254.128.2 --x 1.5 --y 2.0 --theta 1.57
'''
import sys
import asyncio
import argparse

# Woosh Robot SDK 관련 모듈 임포트
from woosh_robot import WooshRobot
from woosh_interface import CommuSettings, NO_PRINT
from woosh.proto.robot.robot_pack_pb2 import ExecTask
from woosh.proto.util.task_pb2 import Type as TaskType, State as TaskState, Direction as TaskDirection
from woosh.proto.robot.robot_pb2 import TaskProc, PoseSpeed

# 작업 완료를 감지하고 스크립트를 종료하기 위한 비동기 이벤트 객체
task_finished_event = asyncio.Event()

def create_task_callback():
    """작업 진행 상황을 모니터링하기 위한 콜백 함수를 생성하는 팩토리 함수."""
    def task_proc_callback(info: TaskProc):
        """
        로봇으로부터 작업(Task) 상태 업데이트를 받을 때마다 호출되는 콜백 함수.
        이 함수는 작업의 현재 상태를 출력하고, 작업이 완료/실패/취소되었는지 확인합니다.
        """
        print(f"작업 업데이트 수신: ID={info.robot_task_id}, 상태={TaskState.Name(info.state)}, 메시지='{info.msg}'")
        
        # 작업이 종료 상태(완료, 실패, 취소) 중 하나에 도달했는지 확인
        if info.state in [TaskState.kCompleted, TaskState.kFailed, TaskState.kCanceled]:
            print(f"작업이 다음 상태로 종료됨: {TaskState.Name(info.state)}")
            # main 함수에서 대기 중인 task_finished_event에 신호를 보내 스크립트가 종료될 수 있도록 함
            task_finished_event.set()
            
    return task_proc_callback

def pose_speed_callback(info: PoseSpeed):
    """로봇의 현재 위치와 속도를 주기적으로 출력하는 콜백 함수."""
    print(f"위치 업데이트: X={info.pose.x:.2f}, Y={info.pose.y:.2f}, Theta={info.pose.theta:.2f}")

async def main():
    """로봇에 연결하고 내비게이션 목표를 전송하는 메인 비동기 함수."""
    # --- 명령줄 인수 파싱 ---
    parser = argparse.ArgumentParser(description="TR-200 로봇 내비게이션 예제")
    parser.add_argument("--ip", type=str, default="169.254.128.2", help="로봇의 IP 주소")
    parser.add_argument("--port", type=int, default=5480, help="로봇의 WebSocket 포트")
    parser.add_argument("--x", type=float, required=True, help="목표 지점의 X 좌표")
    parser.add_argument("--y", type=float, required=True, help="목표 지점의 Y 좌표")
    parser.add_argument("--theta", type=float, default=0.0, help="목표 지점에서의 로봇 방향 (라디안 단위)")
    args = parser.parse_args()

    print(f"{args.ip}:{args.port} 로봇에 연결 중...")

    # --- 1. SDK 초기화 및 로봇 연결 ---
    # CommuSettings: 로봇의 주소, 포트, SDK 식별자 등 연결에 필요한 설정을 정의
    settings = CommuSettings(addr=args.ip, port=args.port, identity="tr200_nav_script")
    # WooshRobot: SDK의 메인 클래스. 모든 로봇 제어 기능은 이 객체를 통해 이루어짐
    robot = WooshRobot(settings)
    
    # robot.run()을 호출하여 로봇과의 WebSocket 연결을 시작
    if not await robot.run():
        print("로봇 연결에 실패했습니다. 프로그램을 종료합니다.")
        return

    print("로봇에 성공적으로 연결되었습니다.")

    # --- 2. 작업 진행 및 위치 정보 구독 ---
    # 로봇의 작업 상태가 변경될 때마다 task_proc_callback 함수가 호출되도록 구독
    await robot.robot_task_process_sub(create_task_callback())
    # 로봇의 위치가 변경될 때마다 pose_speed_callback 함수가 호출되도록 구독
    await robot.robot_pose_speed_sub(pose_speed_callback)
    print("작업 진행 및 위치 업데이트를 구독했습니다.")

    # --- 2.5 로봇 상태 및 맵 확인 ---
    from woosh.proto.robot.robot_pb2 import PoseSpeed, OperationState
    from woosh.proto.map.map_pack_pb2 import SceneList
    from woosh.proto.robot.robot_pack_pb2 import SwitchMap
    print("\n로봇의 현재 상태를 확인합니다...")
    
    # 현재 위치 확인
    pose_speed, ok, msg = await robot.robot_pose_speed_req(PoseSpeed(), NO_PRINT, NO_PRINT)
    if ok:
        print(f"✅ 현재 위치: X={pose_speed.pose.x:.2f}, Y={pose_speed.pose.y:.2f}, Theta={pose_speed.pose.theta:.2f}")
        print(f"   맵 ID: {pose_speed.map_id}")
        
        # 맵 ID가 0이면 맵이 로드되지 않은 상태
        if pose_speed.map_id == 0:
            print("⚠️  맵 ID가 0입니다. 맵이 로드되지 않았을 수 있습니다.")
        else:
            print(f"✅ 맵이 로드되어 있습니다 (ID: {pose_speed.map_id})")
    else:
        print(f"❌ 위치 정보 요청 실패: {msg}")
    
    # 사용 가능한 맵/장면 목록 확인
    print("\n📍 사용 가능한 맵/장면 목록을 확인합니다...")
    scene_list_req = SceneList()
    scene_list, scene_ok, scene_msg = await robot.scene_list_req(scene_list_req, NO_PRINT, NO_PRINT)
    
    available_scenes = []
    if scene_ok and scene_list:
        print("✅ 사용 가능한 장면 목록:")
        for i, scene in enumerate(scene_list.scenes):
            maps_info = f", 맵: {list(scene.maps)}" if scene.maps else ""
            print(f"   {i+1}. {scene.name}{maps_info}")
            available_scenes.append(scene.name)
        
        if not available_scenes:
            print("❌ 사용 가능한 장면이 없습니다.")
    else:
        print(f"❌ 장면 목록 요청 실패: {scene_msg}")

    # 운행 상태 확인
    state, ok, msg = await robot.robot_operation_state_req(OperationState(), NO_PRINT, NO_PRINT)
    if ok:
        print(f"🔍 로봇 상태 상세 정보:")
        print(f"   - robot 비트: {state.robot} (이진: {bin(state.robot)})")
        print(f"   - nav 비트: {state.nav} (이진: {bin(state.nav)})")
        
        if state.robot & OperationState.RobotBit.kTaskable:
            print("✅ 로봇이 작업을 받을 수 있는 상태입니다.")
        else:
            print("⚠️  로봇이 작업을 받을 수 없는 상태입니다.")
            print("   해결 방법을 시도합니다...")
            
        if state.nav & OperationState.NavBit.kImpede:
            print("⚠️  장애물이 감지되었습니다.")
        else:
            print("✅ 내비게이션 경로가 깨끗합니다.")
    else:
        print(f"❌ 운행 상태 요청 실패: {msg}")

    # 맵 관련 문제 해결 시도
    if ok and pose_speed and pose_speed.map_id == 0:
        if available_scenes:
            print("\n🗺️ 맵이 로드되지 않아 첫 번째 사용 가능한 맵을 로드합니다...")
            first_scene = available_scenes[0]
            print(f"   로드할 맵: {first_scene}")
            
            switch_map = SwitchMap()
            switch_map.scene_name = first_scene
            _, map_ok, map_msg = await robot.switch_map_req(switch_map, NO_PRINT, NO_PRINT)
            if map_ok:
                print(f"   ✅ 맵 '{first_scene}' 로드 성공")
                await asyncio.sleep(3)  # 맵 로드 완료 대기
                
                # 맵 로드 후 위치 재확인
                pose_speed, ok, msg = await robot.robot_pose_speed_req(PoseSpeed(), NO_PRINT, NO_PRINT)
                if ok and pose_speed.map_id != 0:
                    print(f"   ✅ 맵 ID가 {pose_speed.map_id}로 업데이트되었습니다.")
                else:
                    print("   ⚠️  맵 로드 후에도 맵 ID가 0입니다.")
            else:
                print(f"   ❌ 맵 로드 실패: {map_msg}")
        else:
            print("\n🗺️ 사용 가능한 맵이 없습니다. 새로운 맵을 생성해보겠습니다...")
            
            # 새로운 맵 생성 시도
            from woosh.proto.robot.robot_pack_pb2 import BuildMap
            print("1️⃣ 새로운 맵 생성을 시도합니다...")
            
            new_map_name = "auto_generated_map"
            new_scene_name = "auto_scene"
            
            build_map = BuildMap()
            build_map.type = BuildMap.BuildType.kAdd  # 새로운 맵 추가
            build_map.scene_name = new_scene_name
            build_map.map_name = new_map_name
            
            print(f"   생성할 맵: 장면명='{new_scene_name}', 맵명='{new_map_name}'")
            
            _, build_ok, build_msg = await robot.build_map_req(build_map, NO_PRINT, NO_PRINT)
            if build_ok:
                print(f"   ✅ 새로운 맵 생성 요청 성공")
                print("   📍 맵 생성은 시간이 걸릴 수 있습니다. 로봇을 수동으로 움직여 맵을 구축해주세요.")
                print("   💡 맵 생성 완료 후 다시 스크립트를 실행해보세요.")
                
                # 맵 생성 데이터 구독 (선택사항)
                from woosh.proto.robot.robot_pack_pb2 import BuildMapData
                def build_map_callback(data: BuildMapData):
                    print(f"🗺️ 맵 생성 진행: 해상도={data.resolution}, 크기={data.width}x{data.height}")
                
                # 맵 생성 데이터 구독 시작
                await robot.build_map_data_sub(build_map_callback, NO_PRINT)
                print("   📊 맵 생성 데이터 구독을 시작했습니다.")
                
                # 잠시 대기하여 맵 생성 시작 확인
                await asyncio.sleep(5)
                
            else:
                print(f"   ❌ 맵 생성 요청 실패: {build_msg}")
                print("   💡 수동으로 맵을 생성하거나 기존 맵을 로드해주세요.")

    # 로봇이 작업을 받을 수 없는 상태인 경우 해결 시도
    if ok and not (state.robot & OperationState.RobotBit.kTaskable):
        print("\n🔧 로봇을 작업 가능 상태로 만들기 위한 시도...")
        
        # 1. 로봇 초기화 시도
        from woosh.proto.robot.robot_pack_pb2 import InitRobot
        print("1️⃣ 로봇 초기화 시도...")
        init_robot = InitRobot()
        init_robot.is_record = False
        init_robot.pose.x = pose_speed.pose.x if pose_speed else 0.0
        init_robot.pose.y = pose_speed.pose.y if pose_speed else 0.0
        init_robot.pose.theta = pose_speed.pose.theta if pose_speed else 0.0
        
        _, init_ok, init_msg = await robot.init_robot_req(init_robot, NO_PRINT, NO_PRINT)
        if init_ok:
            print("   ✅ 로봇 초기화 성공")
            await asyncio.sleep(2)  # 초기화 완료 대기
        else:
            print(f"   ❌ 로봇 초기화 실패: {init_msg}")
        
        # 2. 제어 모드 확인/설정
        from woosh.proto.robot.robot_pack_pb2 import SwitchControlMode
        from woosh.proto.util.robot_pb2 import ControlMode
        print("2️⃣ 제어 모드 자동 모드로 설정 시도...")
        
        switch_mode = SwitchControlMode()
        switch_mode.mode = ControlMode.kAuto
        _, mode_ok, mode_msg = await robot.switch_control_mode_req(switch_mode, NO_PRINT, NO_PRINT)
        if mode_ok:
            print("   ✅ 자동 제어 모드 설정 성공")
            await asyncio.sleep(1)
        else:
            print(f"   ❌ 제어 모드 설정 실패: {mode_msg}")
        
        # 3. 로봇 위치 수동 설정 시도 (로컬라이제이션 문제 해결)
        print("3️⃣ 로봇 위치를 맵에 수동으로 설정합니다...")
        from woosh.proto.robot.robot_pack_pb2 import SetRobotPose
        
        # 현재 로봇의 실제 위치를 맵 상의 위치로 설정
        set_pose = SetRobotPose()
        set_pose.pose.x = pose_speed.pose.x if pose_speed else 0.0
        set_pose.pose.y = pose_speed.pose.y if pose_speed else 0.0
        set_pose.pose.theta = pose_speed.pose.theta if pose_speed else 0.0
        
        print(f"   설정할 위치: X={set_pose.pose.x:.2f}, Y={set_pose.pose.y:.2f}, Theta={set_pose.pose.theta:.2f}")
        
        _, pose_ok, pose_msg = await robot.set_robot_pose_req(set_pose, NO_PRINT, NO_PRINT)
        if pose_ok:
            print("   ✅ 로봇 위치 설정 성공")
            await asyncio.sleep(2)  # 위치 설정 완료 대기
            
            # 위치 설정 후 맵 ID 재확인
            pose_speed_after, ok_after, _ = await robot.robot_pose_speed_req(PoseSpeed(), NO_PRINT, NO_PRINT)
            if ok_after and pose_speed_after.map_id != 0:
                print(f"   ✅ 맵 ID가 {pose_speed_after.map_id}로 업데이트되었습니다!")
                pose_speed = pose_speed_after  # 업데이트된 위치 정보 사용
            else:
                print("   ⚠️  위치 설정 후에도 맵 ID가 0입니다.")
        else:
            print(f"   ❌ 로봇 위치 설정 실패: {pose_msg}")
            
            # 대안: 맵의 원점(0,0) 근처로 위치 설정 시도
            print("   🔄 대안: 맵 원점 근처로 위치 설정을 시도합니다...")
            set_pose_origin = SetRobotPose()
            set_pose_origin.pose.x = 0.0
            set_pose_origin.pose.y = 0.0
            set_pose_origin.pose.theta = 0.0
            
            _, origin_ok, origin_msg = await robot.set_robot_pose_req(set_pose_origin, NO_PRINT, NO_PRINT)
            if origin_ok:
                print("   ✅ 원점 위치 설정 성공")
                await asyncio.sleep(2)
                
                # 원점 설정 후 맵 ID 재확인
                pose_speed_origin, ok_origin, _ = await robot.robot_pose_speed_req(PoseSpeed(), NO_PRINT, NO_PRINT)
                if ok_origin and pose_speed_origin.map_id != 0:
                    print(f"   ✅ 맵 ID가 {pose_speed_origin.map_id}로 업데이트되었습니다!")
                    pose_speed = pose_speed_origin
                else:
                    print("   ⚠️  원점 설정 후에도 맵 ID가 0입니다.")
            else:
                print(f"   ❌ 원점 위치 설정도 실패: {origin_msg}")
        
        # 4. 상태 재확인
        print("4️⃣ 로봇 상태 재확인...")
        state, ok, msg = await robot.robot_operation_state_req(OperationState(), NO_PRINT, NO_PRINT)
        if ok:
            print(f"   - robot 비트: {state.robot} (이진: {bin(state.robot)})")
            print(f"   - nav 비트: {state.nav} (이진: {bin(state.nav)})")
            
            if state.robot & OperationState.RobotBit.kTaskable:
                print("   ✅ 로봇이 이제 작업을 받을 수 있는 상태입니다!")
            else:
                print("   ⚠️  여전히 작업을 받을 수 없는 상태입니다.")
                
                # 추가 진단: 구체적인 문제 확인
                if not (state.robot & OperationState.RobotBit.kTaskable):
                    print("   🔍 kTaskable 비트가 설정되지 않았습니다.")
                if state.nav & OperationState.NavBit.kImpede:
                    print("   🔍 장애물이 감지되었습니다.")
                    
                print("   💡 해결 방법:")
                print("      - 로봇 주변의 장애물을 제거하세요")
                print("      - 로봇을 맵의 알려진 위치로 수동으로 이동하세요")
                print("      - TR-200 앱에서 수동으로 로봇 위치를 설정하세요")
        
        print()

    # --- 3. 내비게이션 작업 정의 및 전송 ---
    print(f"내비게이션 목표 전송: X={args.x}, Y={args.y}, Theta={args.theta}")
    
    # 작업 전송 전 최종 상태 확인
    final_state, final_ok, _ = await robot.robot_operation_state_req(OperationState(), NO_PRINT, NO_PRINT)
    if final_ok and not (final_state.robot & OperationState.RobotBit.kTaskable):
        print("⚠️  로봇이 여전히 작업을 받을 수 없는 상태입니다.")
        print("   네비게이션 요청이 실패할 가능성이 높습니다.")
        print("   그래도 시도해보겠습니다...")
    elif final_ok:
        print("✅ 로봇이 작업을 받을 수 있는 상태입니다. 네비게이션을 시작합니다!")
    
    # ExecTask 메시지를 생성하여 로봇에게 보낼 작업을 정의
    # kParking은 특정 지점으로 이동하는 일반적인 내비게이션 작업 유형으로 사용
    # CLI 코드를 참고하여 필수 필드들을 모두 설정
    nav_task = ExecTask(
        task_id=88888,  # 고유한 작업 ID 설정
        type=TaskType.kParking,
        direction=TaskDirection.kDirectionUndefined,  # 방향 미정의 (단순 이동)
    )
    # 메시지에 목표 위치(x, y)와 방향(theta)을 설정
    nav_task.pose.x = args.x
    nav_task.pose.y = args.y
    nav_task.pose.theta = args.theta
    
    print(f"작업 설정: ID={nav_task.task_id}, type={TaskType.Name(nav_task.type)}, direction={TaskDirection.Name(nav_task.direction)}")
    print(f"목표 좌표: X={nav_task.pose.x}, Y={nav_task.pose.y}, Theta={nav_task.pose.theta}")

    # exec_task_req를 통해 로봇에게 작업 메시지를 전송
    # 디버깅을 위해 요청과 응답을 모두 출력
    from woosh_interface import FULL_PRINT
    print("🚀 네비게이션 작업을 로봇에 전송합니다...")
    _, ok, msg = await robot.exec_task_req(nav_task, FULL_PRINT, FULL_PRINT)

    if not ok:
        print(f"내비게이션 작업 전송 실패: {msg}")
        await robot.stop()
        return

    print("내비게이션 작업이 성공적으로 전송되었습니다. 완료를 기다립니다...")

    # --- 4. 작업 완료 대기 ---
    try:
        # task_proc_callback에서 event.set()이 호출될 때까지 최대 5분간 대기
        await asyncio.wait_for(task_finished_event.wait(), timeout=300.0)
        print("내비게이션 목표에 도달했거나 작업이 종료되었습니다.")
    except asyncio.TimeoutError:
        print("작업 완료 대기 시간을 초과했습니다.")

    # --- 5. 연결 종료 ---
    print("로봇과의 연결을 종료합니다.")
    await robot.stop()

if __name__ == "__main__":
    try:
        # 메인 비동기 함수 실행
        asyncio.run(main())
    except KeyboardInterrupt:
        # Ctrl+C로 프로그램을 중단할 경우
        print("\n사용자에 의해 스크립트가 중단되었습니다.")
