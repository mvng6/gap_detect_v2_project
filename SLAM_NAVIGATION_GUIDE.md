# Woosh Robot SDK를 이용한 SLAM 및 네비게이션 가이드

## 목차
1. [문제 분석](#문제-분석)
2. [Woosh Robot SDK 주요 API](#woosh-robot-sdk-주요-api)
3. [LiDAR 센서 데이터 가져오기](#lidar-센서-데이터-가져오기)
4. [맵 로드 및 로컬라이제이션 해결 방법](#맵-로드-및-로컬라이제이션-해결-방법)
5. [SLAM 맵 생성 방법](#slam-맵-생성-방법)
6. [네비게이션 작업 실행 방법](#네비게이션-작업-실행-방법)
7. [기존 코드 통합 방법](#기존-코드-통합-방법)
8. [완전한 예제 코드](#완전한-예제-코드)

---

## 문제 분석

### 발생한 문제
`demo_lite_ko.py` 실행 시 다음과 같은 문제가 발생합니다:
- **맵을 못 불러오는 문제**: `map_id`가 0으로 표시됨
- **맵상에서 현재 로봇 위치를 찾지 못하는 문제**: 로컬라이제이션 실패

### 원인 분석
1. **맵 미로드**: 로봇에 맵이 로드되지 않았거나 잘못된 맵이 로드됨
2. **로컬라이제이션 실패**: 로봇이 맵 상에서 자신의 위치를 인식하지 못함
3. **작업 불가 상태**: `OperationState.RobotBit.kTaskable` 비트가 설정되지 않음

---

## Woosh Robot SDK 주요 API

### 맵 관련 API

#### 1. 맵/장면 목록 조회
```python
from woosh.proto.map.map_pack_pb2 import SceneList

scene_list_req = SceneList()
scene_list, ok, msg = await robot.scene_list_req(scene_list_req, NO_PRINT, NO_PRINT)

if ok and scene_list:
    for scene in scene_list.scenes:
        print(f"장면명: {scene.name}, 맵: {list(scene.maps)}")
```

#### 2. 맵 전환 (맵 로드)
```python
from woosh.proto.robot.robot_pack_pb2 import SwitchMap

switch_map = SwitchMap()
switch_map.scene_name = "your_scene_name"  # 장면 이름
result, ok, msg = await robot.switch_map_req(switch_map, NO_PRINT, NO_PRINT)
```

#### 3. SLAM 맵 생성
```python
from woosh.proto.robot.robot_pack_pb2 import BuildMap, BuildMapData

# 맵 생성 요청
build_map = BuildMap()
build_map.type = BuildMap.BuildType.kAdd  # 새 맵 추가
build_map.scene_name = "new_scene"         # 장면 이름
build_map.map_name = "new_map"             # 맵 이름
result, ok, msg = await robot.build_map_req(build_map, NO_PRINT, NO_PRINT)

# 맵 생성 진행 상황 구독 (선택사항)
def build_map_callback(data: BuildMapData):
    print(f"맵 생성 진행: 해상도={data.resolution}, 크기={data.width}x{data.height}")

await robot.build_map_data_sub(build_map_callback, NO_PRINT)
```

### 로봇 위치 관련 API

#### 1. 현재 위치 조회
```python
from woosh.proto.robot.robot_pb2 import PoseSpeed

pose_speed, ok, msg = await robot.robot_pose_speed_req(PoseSpeed(), NO_PRINT, NO_PRINT)
if ok:
    print(f"위치: X={pose_speed.pose.x:.2f}, Y={pose_speed.pose.y:.2f}, Theta={pose_speed.pose.theta:.2f}")
    print(f"맵 ID: {pose_speed.map_id}")  # 0이면 맵 미로드
```

#### 2. 로봇 위치 설정 (로컬라이제이션)
```python
from woosh.proto.robot.robot_pack_pb2 import SetRobotPose

set_pose = SetRobotPose()
set_pose.pose.x = 0.0      # 맵 상의 X 좌표
set_pose.pose.y = 0.0      # 맵 상의 Y 좌표
set_pose.pose.theta = 0.0  # 맵 상의 방향 (라디안)
result, ok, msg = await robot.set_robot_pose_req(set_pose, NO_PRINT, NO_PRINT)
```

#### 3. 위치 업데이트 구독
```python
def pose_speed_callback(info: PoseSpeed):
    print(f"위치 업데이트: X={info.pose.x:.2f}, Y={info.pose.y:.2f}, Theta={info.pose.theta:.2f}")
    print(f"맵 ID: {info.map_id}")

await robot.robot_pose_speed_sub(pose_speed_callback, NO_PRINT)
```

### 네비게이션 관련 API

#### 1. 네비게이션 작업 실행
```python
from woosh.proto.robot.robot_pack_pb2 import ExecTask
from woosh.proto.util.task_pb2 import Type as TaskType, Direction as TaskDirection

nav_task = ExecTask(
    task_id=77777,  # 고유한 작업 ID
    type=TaskType.kParking,  # 작업 유형
    direction=TaskDirection.kDirectionUndefined,  # 방향
)
nav_task.pose.x = 1.5      # 목표 X 좌표
nav_task.pose.y = 0.5      # 목표 Y 좌표
nav_task.pose.theta = 1.57 # 목표 방향 (라디안)

result, ok, msg = await robot.exec_task_req(nav_task, NO_PRINT, NO_PRINT)
```

#### 2. 작업 진행 상황 구독
```python
from woosh.proto.robot.robot_pb2 import TaskProc
from woosh.proto.util.task_pb2 import State as TaskState

def task_proc_callback(info: TaskProc):
    print(f"작업 ID: {info.robot_task_id}, 상태: {TaskState.Name(info.state)}")
    if info.state == TaskState.kCompleted:
        print("작업 완료!")
    elif info.state == TaskState.kFailed:
        print(f"작업 실패: {info.msg}")

await robot.robot_task_process_sub(task_proc_callback, NO_PRINT)
```

#### 3. 로봇 상태 확인
```python
from woosh.proto.robot.robot_pb2 import OperationState

state, ok, msg = await robot.robot_operation_state_req(OperationState(), NO_PRINT, NO_PRINT)
if ok:
    # 작업 가능 여부 확인
    if state.robot & OperationState.RobotBit.kTaskable:
        print("로봇이 작업을 받을 수 있는 상태입니다.")
    else:
        print("로봇이 작업을 받을 수 없는 상태입니다.")
    
    # 장애물 감지 확인
    if state.nav & OperationState.NavBit.kImpede:
        print("장애물이 감지되었습니다.")
```

---

## LiDAR 센서 데이터 가져오기

### LiDAR 데이터 API

Woosh Robot SDK는 LiDAR(레이저 스캐너) 센서 데이터를 가져오는 두 가지 방법을 제공합니다:

1. **구독 방식** (`scanner_data_sub`): 지속적으로 LiDAR 데이터를 수신
2. **요청 방식** (`scanner_data_req`): 한 번만 LiDAR 데이터를 요청

### ScannerData 구조

`ScannerData` 메시지는 다음 필드를 포함합니다:

- `robot_id`: 로봇 ID
- `angle_min`: 최소 스캔 각도 (라디안)
- `angle_max`: 최대 스캔 각도 (라디안)
- `angle_increment`: 각도 증가량 (라디안)
- `range_min`: 최소 거리 (미터)
- `range_max`: 최대 거리 (미터)
- `ranges`: 거리 데이터 배열 (각도별 거리 값, 미터)
- `scan_time`: 스캔 시간
- `time_increment`: 시간 증가량
- `pose`: 로봇의 현재 위치 (Pose2D)
- `offset`: 센서 오프셋 (Vector3)

### 방법 1: 구독 방식 (지속적 데이터 수신)

#### 기본 사용법
```python
from woosh.proto.robot.robot_pb2 import ScannerData
from woosh_interface import NO_PRINT

def scanner_data_callback(data: ScannerData):
    """LiDAR 데이터를 수신할 때마다 호출되는 콜백 함수"""
    print(f"LiDAR 데이터 업데이트:")
    print(f"  - 각도 범위: {data.angle_min:.3f} ~ {data.angle_max:.3f} rad")
    print(f"  - 거리 범위: {data.range_min:.3f} ~ {data.range_max:.3f} m")
    print(f"  - 스캔 포인트 수: {len(data.ranges)}")
    print(f"  - 로봇 위치: ({data.pose.x:.2f}, {data.pose.y:.2f}, {data.pose.theta:.2f})")
    
    # 거리 데이터 처리 예제
    for i, distance in enumerate(data.ranges):
        angle = data.angle_min + i * data.angle_increment
        if distance < data.range_max:  # 유효한 거리 데이터만 처리
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)
            # 점 cloud 데이터로 활용 가능

# LiDAR 데이터 구독 시작
await robot.scanner_data_sub(scanner_data_callback, NO_PRINT)
```

#### 상세 예제: 점 cloud 데이터 변환
```python
import math
import numpy as np
from woosh.proto.robot.robot_pb2 import ScannerData

def lidar_to_pointcloud(data: ScannerData):
    """LiDAR 데이터를 점 cloud (x, y) 좌표로 변환"""
    points = []
    
    for i, distance in enumerate(data.ranges):
        # 유효한 거리 데이터만 처리
        if data.range_min <= distance <= data.range_max:
            angle = data.angle_min + i * data.angle_increment
            
            # 로봇 좌표계에서의 점 좌표
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)
            
            points.append((x, y))
    
    return np.array(points)

def scanner_callback_with_pointcloud(data: ScannerData):
    """점 cloud로 변환하는 콜백"""
    pointcloud = lidar_to_pointcloud(data)
    print(f"점 cloud 포인트 수: {len(pointcloud)}")
    
    # 장애물 감지 예제: 가까운 거리의 점 확인
    if len(pointcloud) > 0:
        distances = np.sqrt(pointcloud[:, 0]**2 + pointcloud[:, 1]**2)
        min_distance = np.min(distances)
        if min_distance < 0.5:  # 0.5m 이내 장애물 감지
            print(f"⚠️ 가까운 장애물 감지: {min_distance:.2f}m")

await robot.scanner_data_sub(scanner_callback_with_pointcloud, NO_PRINT)
```

### 방법 2: 요청 방식 (한 번만 데이터 요청)

```python
from woosh.proto.robot.robot_pb2 import ScannerData
from woosh_interface import NO_PRINT

# LiDAR 데이터 요청
scanner_req = ScannerData()
scanner_data, ok, msg = await robot.scanner_data_req(scanner_req, NO_PRINT, NO_PRINT)

if ok and scanner_data:
    print(f"LiDAR 데이터 수신 성공:")
    print(f"  - 스캔 포인트 수: {len(scanner_data.ranges)}")
    print(f"  - 각도 범위: {scanner_data.angle_min:.3f} ~ {scanner_data.angle_max:.3f} rad")
    print(f"  - 거리 범위: {scanner_data.range_min:.3f} ~ {scanner_data.range_max:.3f} m")
    
    # 거리 데이터 처리
    for i, distance in enumerate(scanner_data.ranges[:10]):  # 처음 10개만 출력
        angle = scanner_data.angle_min + i * scanner_data.angle_increment
        print(f"  포인트 {i}: 각도={angle:.3f} rad, 거리={distance:.3f} m")
else:
    print(f"LiDAR 데이터 요청 실패: {msg}")
```

### ROS 통합 예제

기존 ROS 코드에 LiDAR 데이터를 통합하는 방법:

```python
import rospy
from sensor_msgs.msg import LaserScan
import asyncio
from threading import Thread
from woosh_robot import WooshRobot
from woosh_interface import CommuSettings, NO_PRINT
from woosh.proto.robot.robot_pb2 import ScannerData

class WooshLidarPublisher:
    """Woosh Robot LiDAR 데이터를 ROS LaserScan 메시지로 발행"""
    
    def __init__(self, robot_ip: str = "169.254.128.2", robot_port: int = 5480):
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        self.robot = None
        self.lidar_pub = rospy.Publisher('/scan', LaserScan, queue_size=10)
        self.robot_loop = None
        
    async def connect(self):
        """로봇 연결"""
        settings = CommuSettings(
            addr=self.robot_ip,
            port=self.robot_port,
            identity="lidar_publisher"
        )
        self.robot = WooshRobot(settings)
        
        if not await self.robot.run():
            raise RuntimeError("로봇 연결 실패")
        
        rospy.loginfo("로봇 연결 성공")
    
    def scanner_to_laserscan(self, data: ScannerData) -> LaserScan:
        """ScannerData를 ROS LaserScan 메시지로 변환"""
        scan = LaserScan()
        
        scan.header.stamp = rospy.Time.now()
        scan.header.frame_id = "laser_frame"  # 프레임 ID 설정
        
        scan.angle_min = data.angle_min
        scan.angle_max = data.angle_max
        scan.angle_increment = data.angle_increment
        scan.time_increment = data.time_increment
        scan.scan_time = data.scan_time
        scan.range_min = data.range_min
        scan.range_max = data.range_max
        
        scan.ranges = list(data.ranges)
        
        return scan
    
    def scanner_callback(self, data: ScannerData):
        """LiDAR 데이터 콜백"""
        try:
            laserscan_msg = self.scanner_to_laserscan(data)
            self.lidar_pub.publish(laserscan_msg)
        except Exception as e:
            rospy.logerr(f"LiDAR 데이터 처리 오류: {e}")
    
    async def start_subscription(self):
        """LiDAR 데이터 구독 시작"""
        await self.robot.scanner_data_sub(self.scanner_callback, NO_PRINT)
        rospy.loginfo("LiDAR 데이터 구독 시작")
    
    def run(self):
        """비동기 루프 실행"""
        def run_asyncio():
            self.robot_loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.robot_loop)
            
            async def main():
                await self.connect()
                await self.start_subscription()
                # 계속 실행
                while not rospy.is_shutdown():
                    await asyncio.sleep(0.1)
            
            try:
                self.robot_loop.run_until_complete(main())
            except KeyboardInterrupt:
                pass
        
        thread = Thread(target=run_asyncio, daemon=True)
        thread.start()

# 사용 예제
if __name__ == '__main__':
    rospy.init_node('woosh_lidar_publisher', anonymous=True)
    
    lidar_pub = WooshLidarPublisher()
    lidar_pub.run()
    
    rospy.spin()
```

### 실전 활용 예제: 장애물 감지

```python
import math
from woosh.proto.robot.robot_pb2 import ScannerData

class ObstacleDetector:
    """LiDAR 데이터를 이용한 장애물 감지 클래스"""
    
    def __init__(self, safe_distance: float = 0.5):
        self.safe_distance = safe_distance
        self.obstacle_detected = False
    
    def detect_obstacle(self, data: ScannerData) -> tuple:
        """
        장애물 감지
        
        Returns:
            (bool, float, float): (장애물 감지 여부, 최소 거리, 장애물 각도)
        """
        min_distance = data.range_max
        min_angle = 0.0
        
        for i, distance in enumerate(data.ranges):
            if data.range_min <= distance <= data.range_max:
                if distance < min_distance:
                    min_distance = distance
                    min_angle = data.angle_min + i * data.angle_increment
        
        obstacle_detected = min_distance < self.safe_distance
        
        return obstacle_detected, min_distance, min_angle
    
    def scanner_callback(self, data: ScannerData):
        """LiDAR 콜백에서 장애물 감지"""
        detected, distance, angle = self.detect_obstacle(data)
        
        if detected:
            angle_deg = math.degrees(angle)
            print(f"⚠️ 장애물 감지!")
            print(f"   최소 거리: {distance:.2f}m")
            print(f"   장애물 방향: {angle_deg:.1f}도")
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

# 사용 예제
detector = ObstacleDetector(safe_distance=0.5)
await robot.scanner_data_sub(detector.scanner_callback, NO_PRINT)
```

### 주의사항

1. **구독 방식**: 지속적으로 데이터를 수신하므로 콜백 함수가 빠르게 실행되어야 합니다. 무거운 처리는 별도 스레드에서 수행하세요.

2. **데이터 유효성**: `ranges` 배열의 값이 `range_min`과 `range_max` 사이에 있는지 확인하세요.

3. **좌표계**: LiDAR 데이터는 로봇 좌표계 기준입니다. 맵 좌표계로 변환하려면 로봇의 현재 위치(`pose`)를 사용하세요.

4. **성능**: 많은 스캔 포인트가 있을 수 있으므로, 필요한 데이터만 처리하도록 필터링하세요.

---

## 맵 로드 및 로컬라이제이션 해결 방법

### 단계별 해결 절차

#### 1단계: 현재 상태 확인
```python
# 현재 위치 및 맵 상태 확인
pose_speed, ok, msg = await robot.robot_pose_speed_req(PoseSpeed(), NO_PRINT, NO_PRINT)
if ok:
    if pose_speed.map_id == 0:
        print("⚠️ 맵이 로드되지 않았습니다.")
    else:
        print(f"✅ 맵이 로드되어 있습니다 (ID: {pose_speed.map_id})")
```

#### 2단계: 사용 가능한 맵 목록 확인
```python
from woosh.proto.map.map_pack_pb2 import SceneList

scene_list_req = SceneList()
scene_list, scene_ok, scene_msg = await robot.scene_list_req(scene_list_req, NO_PRINT, NO_PRINT)

available_scenes = []
if scene_ok and scene_list:
    for scene in scene_list.scenes:
        available_scenes.append(scene.name)
        print(f"사용 가능한 장면: {scene.name}")
```

#### 3단계: 맵 로드
```python
if available_scenes:
    # 첫 번째 사용 가능한 맵 로드
    switch_map = SwitchMap()
    switch_map.scene_name = available_scenes[0]
    _, map_ok, map_msg = await robot.switch_map_req(switch_map, NO_PRINT, NO_PRINT)
    
    if map_ok:
        print(f"✅ 맵 '{available_scenes[0]}' 로드 성공")
        await asyncio.sleep(3)  # 맵 로드 완료 대기
        
        # 맵 로드 후 위치 재확인
        pose_speed, ok, msg = await robot.robot_pose_speed_req(PoseSpeed(), NO_PRINT, NO_PRINT)
        if ok and pose_speed.map_id != 0:
            print(f"✅ 맵 ID가 {pose_speed.map_id}로 업데이트되었습니다.")
```

#### 4단계: 로봇 위치 설정 (로컬라이제이션)
```python
# 방법 1: 현재 위치를 맵 상의 위치로 설정
set_pose = SetRobotPose()
set_pose.pose.x = pose_speed.pose.x      # 현재 X 좌표
set_pose.pose.y = pose_speed.pose.y      # 현재 Y 좌표
set_pose.pose.theta = pose_speed.pose.theta  # 현재 방향

result, ok, msg = await robot.set_robot_pose_req(set_pose, NO_PRINT, NO_PRINT)

# 방법 2: 맵의 원점(0,0)으로 설정
set_pose_origin = SetRobotPose()
set_pose_origin.pose.x = 0.0
set_pose_origin.pose.y = 0.0
set_pose_origin.pose.theta = 0.0

result, ok, msg = await robot.set_robot_pose_req(set_pose_origin, NO_PRINT, NO_PRINT)
```

#### 5단계: 로봇 초기화 (필요시)
```python
from woosh.proto.robot.robot_pack_pb2 import InitRobot

init_robot = InitRobot()
init_robot.is_record = False
init_robot.pose.x = pose_speed.pose.x if pose_speed else 0.0
init_robot.pose.y = pose_speed.pose.y if pose_speed else 0.0
init_robot.pose.theta = pose_speed.pose.theta if pose_speed else 0.0

result, ok, msg = await robot.init_robot_req(init_robot, NO_PRINT, NO_PRINT)
```

---

## SLAM 맵 생성 방법

### SLAM 맵 생성 절차

#### 1단계: 맵 생성 요청
```python
from woosh.proto.robot.robot_pack_pb2 import BuildMap

build_map = BuildMap()
build_map.type = BuildMap.BuildType.kAdd  # 새 맵 추가
build_map.scene_name = "my_scene"          # 장면 이름
build_map.map_name = "my_map"              # 맵 이름

result, ok, msg = await robot.build_map_req(build_map, NO_PRINT, NO_PRINT)
if ok:
    print("✅ 맵 생성 요청 성공")
    print("📍 로봇을 수동으로 움직여 맵을 구축해주세요.")
else:
    print(f"❌ 맵 생성 요청 실패: {msg}")
```

#### 2단계: 맵 생성 진행 상황 모니터링
```python
from woosh.proto.robot.robot_pack_pb2 import BuildMapData

def build_map_callback(data: BuildMapData):
    print(f"🗺️ 맵 생성 진행:")
    print(f"   - 해상도: {data.resolution}")
    print(f"   - 크기: {data.width} x {data.height}")
    print(f"   - 원점: ({data.origin.x}, {data.origin.y})")

await robot.build_map_data_sub(build_map_callback, NO_PRINT)
```

#### 3단계: 맵 생성 완료 후 저장
맵 생성이 완료되면 로봇이 자동으로 맵을 저장합니다. 이후 `scene_list_req`로 확인할 수 있습니다.

---

## 네비게이션 작업 실행 방법

### 네비게이션 작업 절차

#### 1단계: 로봇 상태 확인
```python
state, ok, msg = await robot.robot_operation_state_req(OperationState(), NO_PRINT, NO_PRINT)
if ok and not (state.robot & OperationState.RobotBit.kTaskable):
    print("⚠️ 로봇이 작업을 받을 수 없는 상태입니다.")
    # 로봇 초기화 및 위치 설정 필요
```

#### 2단계: 네비게이션 작업 생성 및 전송
```python
from woosh.proto.robot.robot_pack_pb2 import ExecTask
from woosh.proto.util.task_pb2 import Type as TaskType, Direction as TaskDirection

nav_task = ExecTask(
    task_id=77777,  # 고유한 작업 ID
    type=TaskType.kParking,  # 작업 유형 (kParking = 위치 이동)
    direction=TaskDirection.kDirectionUndefined,  # 방향 미정의
)
nav_task.pose.x = 1.5      # 목표 X 좌표 (미터)
nav_task.pose.y = 0.5      # 목표 Y 좌표 (미터)
nav_task.pose.theta = 1.57 # 목표 방향 (라디안, 약 90도)

result, ok, msg = await robot.exec_task_req(nav_task, NO_PRINT, NO_PRINT)
if ok:
    print("✅ 네비게이션 작업 전송 성공")
else:
    print(f"❌ 네비게이션 작업 전송 실패: {msg}")
```

#### 3단계: 작업 완료 대기
```python
import asyncio
from woosh.proto.util.task_pb2 import State as TaskState

# 작업 완료를 감지하기 위한 이벤트
navigation_completed = asyncio.Event()

def navigation_task_callback(info: TaskProc):
    print(f"작업 업데이트: ID={info.robot_task_id}, 상태={TaskState.Name(info.state)}")
    
    if info.state == TaskState.kCompleted:
        print("✅ 네비게이션 작업 완료!")
        navigation_completed.set()
    elif info.state == TaskState.kFailed:
        print(f"❌ 네비게이션 작업 실패: {info.msg}")
        navigation_completed.set()
    elif info.state == TaskState.kCanceled:
        print("⏹️ 네비게이션 작업 취소됨")
        navigation_completed.set()

# 작업 진행 상황 구독
await robot.robot_task_process_sub(navigation_task_callback, NO_PRINT)

# 작업 완료 대기 (최대 60초)
try:
    await asyncio.wait_for(navigation_completed.wait(), timeout=60.0)
    print("🎯 네비게이션 작업 처리가 완료되었습니다.")
except asyncio.TimeoutError:
    print("⏰ 네비게이션 작업 대기 시간이 초과되었습니다.")
```

---

## 기존 코드 통합 방법

### `mobile_posiotion_server_twist.py` 통합

`mobile_posiotion_server_twist.py`는 이미 `WooshRobot`을 사용하고 있으므로, 네비게이션 기능을 추가할 수 있습니다.

#### 통합 예제
```python
# mobile_posiotion_server_twist.py에 추가할 네비게이션 메서드

async def navigate_to_goal(self, x: float, y: float, theta: float = 0.0):
    """
    네비게이션 목표로 이동
    
    Args:
        x: 목표 X 좌표 (미터)
        y: 목표 Y 좌표 (미터)
        theta: 목표 방향 (라디안, 기본값 0.0)
    
    Returns:
        bool: 성공 여부
    """
    from woosh.proto.robot.robot_pack_pb2 import ExecTask
    from woosh.proto.util.task_pb2 import Type as TaskType, Direction as TaskDirection
    
    # 맵 및 위치 확인
    from woosh.proto.robot.robot_pb2 import PoseSpeed, OperationState
    
    pose_speed, ok, _ = await self.robot.robot_pose_speed_req(PoseSpeed(), NO_PRINT, NO_PRINT)
    if not ok or pose_speed.map_id == 0:
        rospy.logerr("맵이 로드되지 않았습니다. 먼저 맵을 로드하세요.")
        return False
    
    state, ok, _ = await self.robot.robot_operation_state_req(OperationState(), NO_PRINT, NO_PRINT)
    if not ok or not (state.robot & OperationState.RobotBit.kTaskable):
        rospy.logerr("로봇이 작업을 받을 수 없는 상태입니다.")
        return False
    
    # 네비게이션 작업 생성
    nav_task = ExecTask(
        task_id=99999,
        type=TaskType.kParking,
        direction=TaskDirection.kDirectionUndefined,
    )
    nav_task.pose.x = x
    nav_task.pose.y = y
    nav_task.pose.theta = theta
    
    # 작업 전송
    result, ok, msg = await self.robot.exec_task_req(nav_task, NO_PRINT, NO_PRINT)
    if ok:
        rospy.loginfo(f"네비게이션 작업 전송 성공: ({x}, {y}, {theta})")
        return True
    else:
        rospy.logerr(f"네비게이션 작업 전송 실패: {msg}")
        return False
```

### `testbed_operation_client_all_with_camera.py` 통합

이 파일은 ROS 서비스를 사용하므로, 새로운 ROS 서비스를 추가하여 네비게이션 기능을 제공할 수 있습니다.

#### 새로운 ROS 서비스 정의
```python
# testbed_operation/srv/NavigateToGoal.srv
float64 x
float64 y
float64 theta
---
bool success
string message
```

#### 서비스 핸들러 추가
```python
# testbed_operation_client_all_with_camera.py에 추가

import asyncio
from threading import Thread
from woosh_robot import WooshRobot
from woosh_interface import CommuSettings, NO_PRINT
from woosh.proto.robot.robot_pack_pb2 import ExecTask
from woosh.proto.util.task_pb2 import Type as TaskType, Direction as TaskDirection
from woosh.proto.robot.robot_pb2 import PoseSpeed, OperationState, TaskProc
from woosh.proto.util.task_pb2 import State as TaskState

# 전역 로봇 인스턴스
woosh_robot = None
robot_loop = None

def init_woosh_robot():
    """Woosh Robot 초기화"""
    global woosh_robot, robot_loop
    
    robot_ip = rospy.get_param('~robot_ip', '169.254.128.2')
    robot_port = rospy.get_param('~robot_port', 5480)
    
    settings = CommuSettings(
        addr=robot_ip,
        port=robot_port,
        identity="testbed_operation"
    )
    
    woosh_robot = WooshRobot(settings)
    
    def run_asyncio():
        global robot_loop
        robot_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(robot_loop)
        
        async def main():
            await woosh_robot.run()
            rospy.loginfo("Woosh Robot 연결 성공")
        
        try:
            robot_loop.run_until_complete(main())
            robot_loop.run_forever()
        except KeyboardInterrupt:
            pass
    
    thread = Thread(target=run_asyncio, daemon=True)
    thread.start()
    
    # 연결 대기
    rospy.sleep(2)

def navigate_to_goal_service_handler(req):
    """네비게이션 서비스 핸들러"""
    global woosh_robot, robot_loop
    
    if woosh_robot is None:
        return NavigateToGoalResponse(False, "Woosh Robot이 초기화되지 않았습니다.")
    
    async def navigate():
        # 맵 및 상태 확인
        pose_speed, ok, _ = await woosh_robot.robot_pose_speed_req(PoseSpeed(), NO_PRINT, NO_PRINT)
        if not ok or pose_speed.map_id == 0:
            return False, "맵이 로드되지 않았습니다."
        
        state, ok, _ = await woosh_robot.robot_operation_state_req(OperationState(), NO_PRINT, NO_PRINT)
        if not ok or not (state.robot & OperationState.RobotBit.kTaskable):
            return False, "로봇이 작업을 받을 수 없는 상태입니다."
        
        # 네비게이션 작업 생성
        nav_task = ExecTask(
            task_id=int(time.time() * 1000) % 100000,  # 타임스탬프 기반 ID
            type=TaskType.kParking,
            direction=TaskDirection.kDirectionUndefined,
        )
        nav_task.pose.x = req.x
        nav_task.pose.y = req.y
        nav_task.pose.theta = req.theta
        
        # 작업 전송
        result, ok, msg = await woosh_robot.exec_task_req(nav_task, NO_PRINT, NO_PRINT)
        if ok:
            return True, "네비게이션 작업 전송 성공"
        else:
            return False, f"네비게이션 작업 전송 실패: {msg}"
    
    # 비동기 함수 실행
    future = asyncio.run_coroutine_threadsafe(navigate(), robot_loop)
    try:
        success, message = future.result(timeout=10.0)
        return NavigateToGoalResponse(success, message)
    except Exception as e:
        return NavigateToGoalResponse(False, f"오류 발생: {str(e)}")

# main() 함수에 추가
def main():
    rospy.init_node('integrated_robot_client', anonymous=True)
    
    # Woosh Robot 초기화
    init_woosh_robot()
    
    # 네비게이션 서비스 등록
    from testbed_operation.srv import NavigateToGoal, NavigateToGoalResponse
    rospy.Service('navigate_to_goal', NavigateToGoal, navigate_to_goal_service_handler)
    
    # 기존 코드...
```

---

## 완전한 예제 코드

### 맵 로드 및 네비게이션 통합 예제

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import asyncio
import rospy
from woosh_robot import WooshRobot
from woosh_interface import CommuSettings, NO_PRINT
from woosh.proto.robot.robot_pb2 import PoseSpeed, OperationState, TaskProc
from woosh.proto.robot.robot_pack_pb2 import (
    ExecTask, SwitchMap, SetRobotPose, InitRobot, BuildMap, BuildMapData
)
from woosh.proto.map.map_pack_pb2 import SceneList
from woosh.proto.util.task_pb2 import Type as TaskType, Direction as TaskDirection, State as TaskState


class WooshNavigationManager:
    """Woosh Robot 네비게이션 관리 클래스"""
    
    def __init__(self, robot_ip: str = "169.254.128.2", robot_port: int = 5480):
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        self.robot = None
        
    async def connect(self):
        """로봇 연결"""
        settings = CommuSettings(
            addr=self.robot_ip,
            port=self.robot_port,
            identity="navigation_manager"
        )
        self.robot = WooshRobot(settings)
        
        if not await self.robot.run():
            raise RuntimeError("로봇 연결 실패")
        
        rospy.loginfo("로봇 연결 성공")
    
    async def check_map_status(self):
        """맵 상태 확인"""
        pose_speed, ok, msg = await self.robot.robot_pose_speed_req(PoseSpeed(), NO_PRINT, NO_PRINT)
        if not ok:
            return False, None, f"위치 정보 요청 실패: {msg}"
        
        if pose_speed.map_id == 0:
            return False, pose_speed, "맵이 로드되지 않았습니다."
        
        return True, pose_speed, f"맵이 로드되어 있습니다 (ID: {pose_speed.map_id})"
    
    async def load_map(self, scene_name: str = None):
        """맵 로드"""
        # 사용 가능한 맵 목록 확인
        scene_list_req = SceneList()
        scene_list, ok, msg = await self.robot.scene_list_req(scene_list_req, NO_PRINT, NO_PRINT)
        
        if not ok:
            return False, f"맵 목록 요청 실패: {msg}"
        
        if not scene_list or not scene_list.scenes:
            return False, "사용 가능한 맵이 없습니다."
        
        # 맵 선택
        target_scene = scene_name if scene_name else scene_list.scenes[0].name
        
        # 맵 로드
        switch_map = SwitchMap()
        switch_map.scene_name = target_scene
        result, ok, msg = await self.robot.switch_map_req(switch_map, NO_PRINT, NO_PRINT)
        
        if not ok:
            return False, f"맵 로드 실패: {msg}"
        
        rospy.loginfo(f"맵 '{target_scene}' 로드 성공")
        await asyncio.sleep(3)  # 맵 로드 완료 대기
        
        return True, f"맵 '{target_scene}' 로드 성공"
    
    async def set_robot_pose(self, x: float = None, y: float = None, theta: float = None):
        """로봇 위치 설정"""
        pose_speed, ok, _ = await self.robot.robot_pose_speed_req(PoseSpeed(), NO_PRINT, NO_PRINT)
        if not ok:
            return False, "현재 위치를 가져올 수 없습니다."
        
        set_pose = SetRobotPose()
        set_pose.pose.x = x if x is not None else pose_speed.pose.x
        set_pose.pose.y = y if y is not None else pose_speed.pose.y
        set_pose.pose.theta = theta if theta is not None else pose_speed.pose.theta
        
        result, ok, msg = await self.robot.set_robot_pose_req(set_pose, NO_PRINT, NO_PRINT)
        if not ok:
            return False, f"위치 설정 실패: {msg}"
        
        rospy.loginfo(f"로봇 위치 설정 성공: ({set_pose.pose.x}, {set_pose.pose.y}, {set_pose.pose.theta})")
        await asyncio.sleep(2)
        
        return True, "위치 설정 성공"
    
    async def ensure_robot_ready(self):
        """로봇이 작업 가능한 상태인지 확인 및 설정"""
        state, ok, msg = await self.robot.robot_operation_state_req(OperationState(), NO_PRINT, NO_PRINT)
        if not ok:
            return False, f"상태 확인 실패: {msg}"
        
        if state.robot & OperationState.RobotBit.kTaskable:
            return True, "로봇이 작업 가능한 상태입니다."
        
        # 로봇 초기화 시도
        pose_speed, ok, _ = await self.robot.robot_pose_speed_req(PoseSpeed(), NO_PRINT, NO_PRINT)
        if ok:
            init_robot = InitRobot()
            init_robot.is_record = False
            init_robot.pose.x = pose_speed.pose.x
            init_robot.pose.y = pose_speed.pose.y
            init_robot.pose.theta = pose_speed.pose.theta
            
            result, ok, msg = await self.robot.init_robot_req(init_robot, NO_PRINT, NO_PRINT)
            if ok:
                await asyncio.sleep(2)
                return True, "로봇 초기화 완료"
        
        return False, "로봇을 작업 가능 상태로 만들 수 없습니다."
    
    async def navigate_to_goal(self, x: float, y: float, theta: float = 0.0, timeout: float = 60.0):
        """네비게이션 목표로 이동"""
        # 맵 상태 확인
        map_ok, pose_speed, map_msg = await self.check_map_status()
        if not map_ok:
            rospy.logwarn(f"맵 상태 문제: {map_msg}")
            # 맵 자동 로드 시도
            load_ok, load_msg = await self.load_map()
            if not load_ok:
                return False, f"맵 로드 실패: {load_msg}"
        
        # 로봇 준비 상태 확인
        ready_ok, ready_msg = await self.ensure_robot_ready()
        if not ready_ok:
            return False, f"로봇 준비 실패: {ready_msg}"
        
        # 네비게이션 작업 생성
        nav_task = ExecTask(
            task_id=int(asyncio.get_event_loop().time() * 1000) % 100000,
            type=TaskType.kParking,
            direction=TaskDirection.kDirectionUndefined,
        )
        nav_task.pose.x = x
        nav_task.pose.y = y
        nav_task.pose.theta = theta
        
        # 작업 완료 이벤트
        navigation_completed = asyncio.Event()
        task_id = nav_task.task_id
        
        def task_callback(info: TaskProc):
            if info.robot_task_id == task_id:
                if info.state == TaskState.kCompleted:
                    navigation_completed.set()
                elif info.state in [TaskState.kFailed, TaskState.kCanceled]:
                    navigation_completed.set()
        
        await self.robot.robot_task_process_sub(task_callback, NO_PRINT)
        
        # 작업 전송
        result, ok, msg = await self.robot.exec_task_req(nav_task, NO_PRINT, NO_PRINT)
        if not ok:
            return False, f"네비게이션 작업 전송 실패: {msg}"
        
        rospy.loginfo(f"네비게이션 작업 전송 성공: ({x}, {y}, {theta})")
        
        # 작업 완료 대기
        try:
            await asyncio.wait_for(navigation_completed.wait(), timeout=timeout)
            return True, "네비게이션 완료"
        except asyncio.TimeoutError:
            return False, "네비게이션 타임아웃"
    
    async def create_map(self, scene_name: str, map_name: str):
        """SLAM 맵 생성"""
        build_map = BuildMap()
        build_map.type = BuildMap.BuildType.kAdd
        build_map.scene_name = scene_name
        build_map.map_name = map_name
        
        result, ok, msg = await self.robot.build_map_req(build_map, NO_PRINT, NO_PRINT)
        if not ok:
            return False, f"맵 생성 요청 실패: {msg}"
        
        rospy.loginfo(f"맵 생성 요청 성공: {scene_name}/{map_name}")
        return True, "맵 생성 요청 성공"


# 사용 예제
async def main():
    rospy.init_node('woosh_navigation_example', anonymous=True)
    
    nav_manager = WooshNavigationManager()
    
    try:
        # 연결
        await nav_manager.connect()
        
        # 맵 로드
        ok, msg = await nav_manager.load_map()
        rospy.loginfo(f"맵 로드: {msg}")
        
        # 로봇 위치 설정
        ok, msg = await nav_manager.set_robot_pose(0.0, 0.0, 0.0)
        rospy.loginfo(f"위치 설정: {msg}")
        
        # 네비게이션 실행
        ok, msg = await nav_manager.navigate_to_goal(1.5, 0.5, 1.57)
        rospy.loginfo(f"네비게이션: {msg}")
        
    except Exception as e:
        rospy.logerr(f"오류 발생: {e}")
    finally:
        if nav_manager.robot:
            await nav_manager.robot.stop()


if __name__ == '__main__':
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        rospy.loginfo("프로그램 종료")
```

---

## 요약 및 권장사항

### 네비게이션 작업 전 필수 확인 사항

1. **맵 로드 확인**: `map_id`가 0이 아닌지 확인
2. **로봇 위치 설정**: `set_robot_pose_req`로 로컬라이제이션 수행
3. **작업 가능 상태 확인**: `OperationState.RobotBit.kTaskable` 비트 확인
4. **장애물 확인**: `OperationState.NavBit.kImpede` 비트 확인

### 문제 해결 순서

1. 맵 목록 조회 → 맵 로드 → 위치 설정 → 로봇 초기화 → 네비게이션 실행
2. 각 단계마다 상태를 확인하고 실패 시 다음 단계로 진행하지 않음
3. 로그를 충분히 남겨 디버깅 용이하게 함

### 주의사항

- 맵 생성 중에는 네비게이션을 실행할 수 없음
- 로봇이 작업을 받을 수 없는 상태에서는 네비게이션 실패
- 맵 ID가 0이면 반드시 맵을 로드해야 함
- 위치 설정은 맵 상의 실제 위치와 일치해야 함

