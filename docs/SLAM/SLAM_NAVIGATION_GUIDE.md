# Woosh 모바일로봇 SLAM 기반 네비게이션 작업 지시서

## 📋 문서 정보
- **작성일**: 2025-12-02
- **대상 로봇**: TR-200 (Woosh Robot)
- **SDK 버전**: woosh_robot_py
- **목적**: 기존 맵을 이용한 SLAM 기반 네비게이션 구현

---

## 🔍 1. 문제 분석

### 1.1 현재 증상
- `demo_lite_ko.py` 실행 시 내비게이션 작업(5.1)이 동작하지 않음
- 스텝 제어(5.3)도 동작하지 않음
- **오직 속도 제어(Twist)를 이용한 회전(5.4)만 동작함**

### 1.2 근본 원인 분석
네비게이션이 실패하는 주요 원인은 다음과 같습니다:

| 원인 | 설명 | 진단 방법 |
|------|------|-----------|
| **맵 미로드** | 로봇에 맵이 로드되지 않은 상태 | `map_id == 0` 확인 |
| **로컬라이제이션 실패** | 로봇이 맵 상에서 자신의 위치를 모름 | `kTaskable` 비트 미설정 |
| **제어 모드 오류** | 자동 모드가 아닌 수동/유지보수 모드 | `ControlMode` 확인 |
| **로봇 미초기화** | 로봇이 작업 수행 가능 상태가 아님 | `kTaskable` 비트 미설정 |

### 1.3 왜 Twist(회전)만 동작하는가?
```
┌─────────────────────────────────────────────────────────────────┐
│  Twist (속도 제어)                                               │
│  ├── 맵 필요 없음 (Open-loop control)                           │
│  ├── 로컬라이제이션 필요 없음                                     │
│  └── 단순히 모터에 속도 명령 전달                                 │
│                                                                  │
│  Navigation / StepControl (위치 제어)                            │
│  ├── 맵 필수 (경로 계획을 위해)                                   │
│  ├── 로컬라이제이션 필수 (현재 위치 파악)                         │
│  └── kTaskable 상태 필수                                         │
└─────────────────────────────────────────────────────────────────┘
```

---

## 🔧 2. 필수 초기화 절차

네비게이션을 수행하기 전에 반드시 다음 절차를 순서대로 수행해야 합니다.

### 2.1 절차 흐름도

```
┌──────────────────┐
│  로봇 연결         │
│  (robot.run())   │
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│  장면/맵 목록      │  ← scene_list_req()
│  확인             │
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│  맵 ID 확인       │  ← robot_pose_speed_req()
│  (map_id != 0?)  │
└────────┬─────────┘
         │
    map_id == 0?
         │
    Yes  │  No
    ▼    └───────────────────────────────────────────┐
┌──────────────────┐                                 │
│  맵 전환/로드       │  ← switch_map_req()            │
└────────┬─────────┘                                 │
         │                                           │
         ▼                                           │
┌──────────────────┐                                 │
│  로봇 위치 설정     │  ← set_robot_pose_req()         │
│  (로컬라이제이션)    │                                 │
└────────┬─────────┘                                 │
         │                                           │
         ▼                                           │
┌──────────────────┐                                 │
│  로봇 초기화       │  ← init_robot_req()             │
└────────┬─────────┘                                 │
         │                                           │
         ▼                                           │
┌──────────────────┐                                 │
│  제어 모드 설정     │  ← switch_control_mode_req()    │
│  (kAuto)         │                                 │
└────────┬─────────┘                                 │
         │                   │                     
         ▼                   ▼
┌──────────────────────────────┐
│  운행 상태 확인                 │
│  (kTaskable 비트 확인)         │  ← robot_operation_state_req()
└────────┬─────────────────────┘
         │
    kTaskable?
         │
    Yes  │
         ▼
┌──────────────────┐
│  네비게이션 실행 │  ← exec_task_req()
└──────────────────┘
```

---

## 📝 3. 상세 구현 가이드

### 3.1 필수 Import 문

```python
import asyncio
from woosh_robot import WooshRobot
from woosh_interface import CommuSettings, NO_PRINT, FULL_PRINT

# 로봇 상태 관련
from woosh.proto.robot.robot_pb2 import (
    PoseSpeed,
    OperationState,
    TaskProc,
    Scene,
)

# 맵 관련
from woosh.proto.map.map_pack_pb2 import SceneList

# 로봇 제어 관련
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
```

### 3.2 STEP 1: 장면/맵 목록 확인

```python
async def get_available_maps(robot: WooshRobot) -> list:
    """로봇에 저장된 사용 가능한 맵/장면 목록을 조회합니다."""
    scene_list_req = SceneList()
    scene_list, ok, msg = await robot.scene_list_req(scene_list_req, NO_PRINT, FULL_PRINT)
    
    available_scenes = []
    if ok and scene_list:
        print("✅ 사용 가능한 장면 목록:")
        for i, scene in enumerate(scene_list.scenes):
            maps_info = f", 포함된 맵: {list(scene.maps)}" if scene.maps else ""
            print(f"   {i+1}. {scene.name}{maps_info}")
            available_scenes.append(scene.name)
    else:
        print(f"❌ 장면 목록 요청 실패: {msg}")
    
    return available_scenes
```

### 3.3 STEP 2: 현재 맵 상태 확인

```python
async def check_map_status(robot: WooshRobot) -> tuple:
    """현재 로봇의 맵 로드 상태를 확인합니다."""
    pose_speed, ok, msg = await robot.robot_pose_speed_req(PoseSpeed(), NO_PRINT, NO_PRINT)
    
    if ok:
        print(f"현재 위치: X={pose_speed.pose.x:.2f}, Y={pose_speed.pose.y:.2f}, Theta={pose_speed.pose.theta:.2f}")
        print(f"맵 ID: {pose_speed.map_id}")
        
        if pose_speed.map_id == 0:
            print("⚠️  맵 ID가 0입니다. 맵이 로드되지 않았습니다!")
            return pose_speed, False
        else:
            print(f"✅ 맵이 로드되어 있습니다 (ID: {pose_speed.map_id})")
            return pose_speed, True
    else:
        print(f"❌ 위치 정보 요청 실패: {msg}")
        return None, False
```

### 3.4 STEP 3: 맵 로드 (Switch Map)

```python
async def load_map(robot: WooshRobot, scene_name: str) -> bool:
    """지정된 장면(맵)을 로드합니다."""
    print(f"🗺️  맵 '{scene_name}'을(를) 로드합니다...")
    
    switch_map = SwitchMap()
    switch_map.scene_name = scene_name
    # 특정 맵을 지정해야 하는 경우 아래 추가:
    # switch_map.map_name = "map_name"
    
    _, ok, msg = await robot.switch_map_req(switch_map, NO_PRINT, FULL_PRINT)
    
    if ok:
        print(f"✅ 맵 '{scene_name}' 로드 요청 성공")
        # 맵 로드 완료를 위해 대기 (로봇 내부 처리 시간 필요)
        await asyncio.sleep(3)
        return True
    else:
        print(f"❌ 맵 로드 실패: {msg}")
        return False
```

### 3.5 STEP 4: 로봇 위치 설정 (로컬라이제이션)

```python
async def set_robot_localization(robot: WooshRobot, x: float, y: float, theta: float) -> bool:
    """
    로봇의 현재 위치를 맵 상에 설정합니다 (로컬라이제이션).
    
    주의: 이 좌표는 실제 로봇이 맵에서 있는 위치와 일치해야 합니다!
          - 윈도우 프로그램에서 맵 생성 시 시작점 좌표를 확인하세요.
          - 보통 맵 생성 시작점이 (0, 0, 0)입니다.
    """
    print(f"📍 로봇 위치를 설정합니다: X={x}, Y={y}, Theta={theta}")
    
    set_pose = SetRobotPose()
    set_pose.pose.x = x
    set_pose.pose.y = y
    set_pose.pose.theta = theta
    
    _, ok, msg = await robot.set_robot_pose_req(set_pose, NO_PRINT, FULL_PRINT)
    
    if ok:
        print("✅ 로봇 위치 설정 성공")
        await asyncio.sleep(2)  # 위치 설정 반영 대기
        return True
    else:
        print(f"❌ 로봇 위치 설정 실패: {msg}")
        return False
```

### 3.6 STEP 5: 로봇 초기화

```python
async def initialize_robot(robot: WooshRobot, x: float = 0.0, y: float = 0.0, theta: float = 0.0) -> bool:
    """로봇을 초기화합니다."""
    print("🔧 로봇 초기화를 수행합니다...")
    
    init_robot = InitRobot()
    init_robot.is_record = False  # 위치 기록 여부
    init_robot.pose.x = x
    init_robot.pose.y = y
    init_robot.pose.theta = theta
    
    _, ok, msg = await robot.init_robot_req(init_robot, NO_PRINT, FULL_PRINT)
    
    if ok:
        print("✅ 로봇 초기화 성공")
        await asyncio.sleep(2)
        return True
    else:
        print(f"❌ 로봇 초기화 실패: {msg}")
        return False
```

### 3.7 STEP 6: 제어 모드 설정

```python
async def set_control_mode_auto(robot: WooshRobot) -> bool:
    """로봇의 제어 모드를 자동(kAuto)으로 설정합니다."""
    print("⚙️  제어 모드를 자동(Auto)으로 설정합니다...")
    
    switch_mode = SwitchControlMode()
    switch_mode.mode = ControlMode.kAuto
    
    _, ok, msg = await robot.switch_control_mode_req(switch_mode, NO_PRINT, FULL_PRINT)
    
    if ok:
        print("✅ 자동 제어 모드 설정 성공")
        await asyncio.sleep(1)
        return True
    else:
        print(f"❌ 제어 모드 설정 실패: {msg}")
        return False
```

### 3.8 STEP 7: 운행 상태 확인 (kTaskable)

```python
async def check_taskable_status(robot: WooshRobot) -> bool:
    """로봇이 작업을 받을 수 있는 상태인지 확인합니다."""
    print("🔍 로봇 운행 상태를 확인합니다...")
    
    state, ok, msg = await robot.robot_operation_state_req(OperationState(), NO_PRINT, FULL_PRINT)
    
    if ok:
        print(f"   - robot 비트: {state.robot} (이진: {bin(state.robot)})")
        print(f"   - nav 비트: {state.nav} (이진: {bin(state.nav)})")
        
        # kTaskable 비트 확인 (가장 중요!)
        is_taskable = bool(state.robot & OperationState.RobotBit.kTaskable)
        
        if is_taskable:
            print("✅ 로봇이 작업을 받을 수 있는 상태입니다 (kTaskable)")
        else:
            print("❌ 로봇이 작업을 받을 수 없는 상태입니다")
            print("   가능한 원인:")
            print("   - 맵이 로드되지 않음")
            print("   - 로컬라이제이션 실패")
            print("   - 로봇 초기화 필요")
            print("   - 수동 모드 또는 유지보수 모드")
        
        # 장애물 감지 여부
        if state.nav & OperationState.NavBit.kImpede:
            print("⚠️  장애물이 감지되었습니다")
        else:
            print("✅ 네비게이션 경로가 깨끗합니다")
        
        return is_taskable
    else:
        print(f"❌ 운행 상태 요청 실패: {msg}")
        return False
```

### 3.9 STEP 8: 네비게이션 실행

```python
async def navigate_to_goal(robot: WooshRobot, x: float, y: float, theta: float, task_id: int = 77777) -> bool:
    """지정된 좌표로 네비게이션을 실행합니다."""
    print(f"🚀 네비게이션 시작: 목표 X={x}, Y={y}, Theta={theta}")
    
    # ExecTask 메시지 생성
    nav_task = ExecTask(
        task_id=task_id,
        type=TaskType.kParking,  # 단순 위치 이동용
        direction=TaskDirection.kDirectionUndefined,  # 방향 미정의
    )
    nav_task.pose.x = x
    nav_task.pose.y = y
    nav_task.pose.theta = theta
    
    print(f"   작업 설정: type={TaskType.Name(nav_task.type)}, task_id={task_id}")
    
    # 작업 전송
    _, ok, msg = await robot.exec_task_req(nav_task, FULL_PRINT, FULL_PRINT)
    
    if ok:
        print("✅ 네비게이션 작업 전송 성공")
        return True
    else:
        print(f"❌ 네비게이션 작업 전송 실패: {msg}")
        return False
```

---

## 🚀 4. 완전한 네비게이션 초기화 코드

아래 코드는 모든 필수 초기화 절차를 포함한 완전한 예제입니다.

```python
import asyncio
from woosh_robot import WooshRobot
from woosh_interface import CommuSettings, NO_PRINT, FULL_PRINT

from woosh.proto.robot.robot_pb2 import PoseSpeed, OperationState, TaskProc
from woosh.proto.map.map_pack_pb2 import SceneList
from woosh.proto.robot.robot_pack_pb2 import (
    InitRobot, SetRobotPose, SwitchMap, SwitchControlMode, ExecTask
)
from woosh.proto.util.robot_pb2 import ControlMode
from woosh.proto.util.task_pb2 import Type as TaskType, State as TaskState, Direction as TaskDirection


async def setup_navigation(robot: WooshRobot) -> bool:
    """네비게이션을 위한 모든 초기화 절차를 수행합니다."""
    
    # ========== STEP 1: 장면/맵 목록 확인 ==========
    print("\n" + "="*50)
    print("STEP 1: 장면/맵 목록 확인")
    print("="*50)
    
    scene_list_req = SceneList()
    scene_list, ok, msg = await robot.scene_list_req(scene_list_req, NO_PRINT, FULL_PRINT)
    
    available_scenes = []
    if ok and scene_list:
        print("✅ 사용 가능한 장면 목록:")
        for i, scene in enumerate(scene_list.scenes):
            print(f"   {i+1}. {scene.name}")
            available_scenes.append(scene.name)
        
        if not available_scenes:
            print("❌ 사용 가능한 맵이 없습니다!")
            print("💡 해결 방법: 윈도우 프로그램에서 맵을 생성하고 로봇에 저장하세요.")
            return False
    else:
        print(f"❌ 장면 목록 요청 실패: {msg}")
        return False
    
    # ========== STEP 2: 현재 맵 상태 확인 ==========
    print("\n" + "="*50)
    print("STEP 2: 현재 맵 상태 확인")
    print("="*50)
    
    pose_speed, ok, msg = await robot.robot_pose_speed_req(PoseSpeed(), NO_PRINT, FULL_PRINT)
    
    if ok:
        print(f"현재 위치: X={pose_speed.pose.x:.2f}, Y={pose_speed.pose.y:.2f}, Theta={pose_speed.pose.theta:.2f}")
        print(f"맵 ID: {pose_speed.map_id}")
        
        # 맵이 로드되지 않은 경우 로드 필요
        if pose_speed.map_id == 0:
            print("⚠️  맵이 로드되지 않았습니다!")
            
            # ========== STEP 3: 맵 로드 ==========
            print("\n" + "="*50)
            print("STEP 3: 맵 로드")
            print("="*50)
            
            # 첫 번째 사용 가능한 맵 로드
            target_scene = available_scenes[0]
            print(f"🗺️  맵 '{target_scene}'을(를) 로드합니다...")
            
            switch_map = SwitchMap()
            switch_map.scene_name = target_scene
            
            _, map_ok, map_msg = await robot.switch_map_req(switch_map, NO_PRINT, FULL_PRINT)
            
            if map_ok:
                print(f"✅ 맵 '{target_scene}' 로드 성공")
                await asyncio.sleep(3)
            else:
                print(f"❌ 맵 로드 실패: {map_msg}")
                return False
    else:
        print(f"❌ 위치 정보 요청 실패: {msg}")
        return False
    
    # ========== STEP 4: 로봇 위치 설정 (로컬라이제이션) ==========
    print("\n" + "="*50)
    print("STEP 4: 로봇 위치 설정 (로컬라이제이션)")
    print("="*50)
    
    # 중요: 실제 로봇이 맵에서 있는 위치를 입력해야 합니다!
    # 맵 생성 시작점이 보통 (0, 0, 0)입니다.
    init_x, init_y, init_theta = 0.0, 0.0, 0.0
    
    print(f"📍 로봇 위치 설정: X={init_x}, Y={init_y}, Theta={init_theta}")
    print("   ⚠️  주의: 이 좌표는 실제 로봇 위치와 일치해야 합니다!")
    
    set_pose = SetRobotPose()
    set_pose.pose.x = init_x
    set_pose.pose.y = init_y
    set_pose.pose.theta = init_theta
    
    _, pose_ok, pose_msg = await robot.set_robot_pose_req(set_pose, NO_PRINT, FULL_PRINT)
    
    if pose_ok:
        print("✅ 로봇 위치 설정 성공")
        await asyncio.sleep(2)
    else:
        print(f"❌ 로봇 위치 설정 실패: {pose_msg}")
        # 실패해도 계속 진행 (다른 단계에서 해결될 수 있음)
    
    # ========== STEP 5: 로봇 초기화 ==========
    print("\n" + "="*50)
    print("STEP 5: 로봇 초기화")
    print("="*50)
    
    init_robot = InitRobot()
    init_robot.is_record = False
    init_robot.pose.x = init_x
    init_robot.pose.y = init_y
    init_robot.pose.theta = init_theta
    
    _, init_ok, init_msg = await robot.init_robot_req(init_robot, NO_PRINT, FULL_PRINT)
    
    if init_ok:
        print("✅ 로봇 초기화 성공")
        await asyncio.sleep(2)
    else:
        print(f"❌ 로봇 초기화 실패: {init_msg}")
    
    # ========== STEP 6: 제어 모드 설정 ==========
    print("\n" + "="*50)
    print("STEP 6: 제어 모드 설정 (자동)")
    print("="*50)
    
    switch_mode = SwitchControlMode()
    switch_mode.mode = ControlMode.kAuto
    
    _, mode_ok, mode_msg = await robot.switch_control_mode_req(switch_mode, NO_PRINT, FULL_PRINT)
    
    if mode_ok:
        print("✅ 자동 제어 모드 설정 성공")
        await asyncio.sleep(1)
    else:
        print(f"❌ 제어 모드 설정 실패: {mode_msg}")
    
    # ========== STEP 7: 최종 상태 확인 ==========
    print("\n" + "="*50)
    print("STEP 7: 최종 상태 확인")
    print("="*50)
    
    # 맵 ID 재확인
    pose_speed, ok, _ = await robot.robot_pose_speed_req(PoseSpeed(), NO_PRINT, NO_PRINT)
    if ok:
        print(f"맵 ID: {pose_speed.map_id}")
        if pose_speed.map_id == 0:
            print("❌ 맵이 여전히 로드되지 않았습니다!")
    
    # kTaskable 상태 확인
    state, ok, _ = await robot.robot_operation_state_req(OperationState(), NO_PRINT, FULL_PRINT)
    
    if ok:
        print(f"robot 비트: {state.robot} (이진: {bin(state.robot)})")
        print(f"nav 비트: {state.nav} (이진: {bin(state.nav)})")
        
        is_taskable = bool(state.robot & OperationState.RobotBit.kTaskable)
        
        if is_taskable:
            print("✅ 로봇이 작업을 받을 수 있는 상태입니다!")
            print("🎉 네비게이션 준비 완료!")
            return True
        else:
            print("❌ 로봇이 여전히 작업을 받을 수 없는 상태입니다")
            print("\n💡 추가 해결 방법:")
            print("   1. 로봇을 맵의 시작점(원점) 근처로 수동 이동")
            print("   2. 윈도우 프로그램에서 로봇 위치를 수동 설정")
            print("   3. 로봇 재시작 후 다시 시도")
            return False
    else:
        print("❌ 운행 상태 확인 실패")
        return False


async def main():
    """메인 함수"""
    # 로봇 연결 설정
    settings = CommuSettings(
        addr="169.254.128.2",  # 로봇 IP 주소
        port=5480,             # 로봇 포트
        identity="nav-setup"
    )
    
    robot = WooshRobot(settings)
    
    try:
        # 로봇 연결
        print("🔌 로봇에 연결 중...")
        if not await robot.run():
            print("❌ 로봇 연결 실패")
            return
        print("✅ 로봇 연결 성공")
        
        # 네비게이션 초기화
        if await setup_navigation(robot):
            # 네비게이션 테스트
            print("\n" + "="*50)
            print("네비게이션 테스트")
            print("="*50)
            
            input("엔터를 눌러 네비게이션을 시작하세요...")
            
            # 목표 좌표 (맵 상의 유효한 좌표로 변경하세요)
            target_x, target_y, target_theta = 1.0, 0.5, 0.0
            
            nav_task = ExecTask(
                task_id=77777,
                type=TaskType.kParking,
                direction=TaskDirection.kDirectionUndefined,
            )
            nav_task.pose.x = target_x
            nav_task.pose.y = target_y
            nav_task.pose.theta = target_theta
            
            _, ok, msg = await robot.exec_task_req(nav_task, FULL_PRINT, FULL_PRINT)
            
            if ok:
                print("✅ 네비게이션 작업 전송 성공!")
                print("⏳ 작업 완료를 기다립니다...")
                
                # 작업 완료 대기 (실제로는 콜백으로 처리하는 것이 좋음)
                await asyncio.sleep(30)
            else:
                print(f"❌ 네비게이션 실패: {msg}")
        else:
            print("\n❌ 네비게이션 초기화 실패")
            print("위의 해결 방법을 확인하세요.")
    
    finally:
        # 연결 종료
        print("\n🔌 로봇 연결 종료...")
        if robot.comm.is_connected():
            await robot.stop()


if __name__ == "__main__":
    asyncio.run(main())
```

---

## ⚠️ 5. 주의사항 및 트러블슈팅

### 5.1 로컬라이제이션 좌표 설정 시 주의사항

```
┌─────────────────────────────────────────────────────────────────┐
│                     중요!                                       │
│                                                                  │
│  set_robot_pose_req()로 설정하는 좌표는 반드시 로봇의 실제       │
│  물리적 위치와 일치해야 합니다!                                  │
│                                                                  │
│  틀린 좌표를 설정하면:                                           │
│  - 네비게이션이 완전히 실패함                                    │
│  - 로봇이 이상한 방향으로 움직임                                 │
│  - 장애물 충돌 위험                                              │
│                                                                  │
│  올바른 좌표 확인 방법:                                          │
│  1. 윈도우 프로그램에서 맵 생성 시작점 확인                      │
│  2. 로봇을 맵 생성 시작점에 수동으로 배치                        │
│  3. 해당 위치를 (0, 0, 0)으로 설정                               │
└─────────────────────────────────────────────────────────────────┘
```

### 5.2 일반적인 오류 및 해결 방법

| 오류 증상 | 가능한 원인 | 해결 방법 |
|-----------|-------------|-----------|
| `map_id == 0` | 맵이 로드되지 않음 | `switch_map_req()` 실행 |
| `kTaskable` 미설정 | 로컬라이제이션 실패 | `set_robot_pose_req()` 로 위치 재설정 |
| 네비게이션 즉시 실패 | 목표점이 맵 외부 | 맵 내부의 유효한 좌표 사용 |
| 네비게이션 중 멈춤 | 장애물 감지 | 장애물 제거 또는 경로 재계획 |
| "Not connected" | 연결 끊김 | 네트워크 확인 및 재연결 |

### 5.3 제어 모드 종류

| 모드 | 설명 | 네비게이션 가능 |
|------|------|-----------------|
| `kAuto` | 자동 모드 | ✅ 가능 |
| `kManual` | 수동 모드 (조이스틱) | ❌ 불가 |
| `kMaintain` | 유지보수 모드 | ❌ 불가 |

---

## 📚 6. 참고 자료

### 6.1 SDK 클래스 구조

```
WooshRobot
├── RobotInterface (작업 실행)
│   ├── init_robot_req()       - 로봇 초기화
│   ├── set_robot_pose_req()   - 위치 설정 (로컬라이제이션)
│   ├── switch_map_req()       - 맵 전환
│   ├── switch_control_mode_req() - 제어 모드 변경
│   └── exec_task_req()        - 작업(네비게이션) 실행
│
├── RobotInfo (상태 조회)
│   ├── robot_pose_speed_req() - 현재 위치/속도 조회
│   ├── robot_operation_state_req() - 운행 상태 조회
│   └── robot_scene_req()      - 현재 장면 정보 조회
│
└── MapInfo (맵 관리)
    └── scene_list_req()       - 사용 가능한 맵 목록 조회
```

### 6.2 관련 예제 파일

| 파일 | 설명 |
|------|------|
| `tr200_move_to_goal.py` | 완전한 네비게이션 예제 (진단 코드 포함) |
| `demo_lite_ko.py` | 기본 SDK 사용 예제 |
| `cli/robot_request_menu.py` | CLI 기반 요청 예제 |
| `cli/map_info_menu.py` | 맵 관리 기능 예제 |

---

## 📞 7. 지원

문제가 지속되면 다음 정보와 함께 제조사에 문의하세요:

1. `robot_operation_state_req()` 응답 결과 (robot 비트, nav 비트)
2. `robot_pose_speed_req()` 응답 결과 (map_id)
3. `scene_list_req()` 응답 결과 (사용 가능한 맵 목록)
4. 윈도우 프로그램에서 생성한 맵 정보

---

**작성자**: AI Assistant  
**최종 수정**: 2025-12-02

