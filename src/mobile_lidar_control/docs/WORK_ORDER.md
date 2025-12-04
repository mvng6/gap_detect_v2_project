# 🚀 Mobile LiDAR Control - 작업 지시서

## 📋 프로젝트 개요

**목적**: 모바일 로봇의 LiDAR 센서를 활용한 정밀 위치 제어 시스템 구현

**주요 기능**:
1. Twist 명령을 통한 로봇 구동
2. LiDAR 센서값 획득 및 처리
3. LiDAR 기반 이동 거리/위치 보정
4. Rviz 시각화 연동

---

## 🔍 시스템 분석 결과

### 1. 기존 서버 코드 분석 (`ldj_mobile_posiotion_server_twist.py`)

**아키텍처**:
- ROS1 (rospy) 기반 서비스 서버
- `WooshRobot` SDK를 통한 비동기 로봇 통신
- 별도 스레드에서 asyncio 이벤트 루프 실행

**주요 기능**:
- `/mobile_positiontwist` 서비스 제공
- Twist 명령으로 직선 이동 (전진/후진)
- 사다리꼴 속도 프로파일 (가속-정속-감속)
- 시간 기반 거리 추정 (현재 방식의 한계점)

**코드 구조**:
```
SmoothTwistController
├── connect()              # 로봇 연결
├── _setup_map()           # 맵 로드 및 로컬라이제이션
├── _move_exact_distance() # 거리 기반 이동 제어
└── _control_loop()        # 명령 처리 루프
```

### 2. WooshRobot SDK - LiDAR API 분석

**✅ LiDAR 데이터 획득 가능 확인**

| API 메서드 | 설명 | 사용 방식 |
|-----------|------|----------|
| `scanner_data_req()` | LiDAR 데이터 1회 요청 | Request-Response |
| `scanner_data_sub()` | LiDAR 데이터 스트리밍 구독 | Publish-Subscribe |

**ScannerData 메시지 구조** (sensor_msgs/LaserScan과 유사):
```python
class ScannerData:
    robot_id: int              # 로봇 ID
    angle_min: float           # 최소 스캔 각도 (rad)
    angle_max: float           # 최대 스캔 각도 (rad)
    angle_increment: float     # 각도 증분 (rad)
    time_increment: float      # 시간 증분 (s)
    scan_time: float           # 스캔 시간 (s)
    range_min: float           # 최소 거리 (m)
    range_max: float           # 최대 거리 (m)
    ranges: List[float]        # 거리 측정값 배열 (m)
    pose: Pose2D               # 현재 위치 (x, y, theta)
    offset: Vector3            # 센서 오프셋
```

### 3. Twist 명령 분석

**사용 방법**:
```python
from woosh.proto.robot.robot_pack_pb2 import Twist

# 속도 명령 전송
await robot.twist_req(Twist(linear=0.1, angular=0.0), NO_PRINT, NO_PRINT)
```

**현재 구현된 제어 파라미터**:
- `max_speed`: 0.12 m/s
- `accel`: 0.25 m/s²
- `decel`: 0.50 m/s²
- `control_hz`: 50 Hz

---

## 📊 전체 프로세스 플로우

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        Mobile LiDAR Control System                       │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ┌──────────────┐     ┌──────────────┐     ┌──────────────┐            │
│  │   ROS Node   │────▶│ WooshRobot   │────▶│  Mobile      │            │
│  │  (PC측)      │◀────│    SDK       │◀────│  Robot       │            │
│  └──────────────┘     └──────────────┘     └──────────────┘            │
│         │                    │                    │                     │
│         │                    │                    │                     │
│         ▼                    ▼                    ▼                     │
│  ┌──────────────┐     ┌──────────────┐     ┌──────────────┐            │
│  │  Rviz        │     │ Twist Cmd    │     │  LiDAR       │            │
│  │  시각화       │◀────│ 속도 제어     │     │  Sensor      │            │
│  └──────────────┘     └──────────────┘     └──────────────┘            │
│                              │                    │                     │
│                              ▼                    ▼                     │
│                       ┌─────────────────────────────────┐              │
│                       │      Position Correction        │              │
│                       │      (LiDAR 기반 거리 보정)      │              │
│                       └─────────────────────────────────┘              │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

---

## 📝 작업 단계별 상세 지시

### Phase 1: LiDAR 센서값 획득 노드 구현

**파일**: `scripts/lidar_subscriber.py`

**목표**: WooshRobot SDK를 통해 LiDAR 데이터를 수신하고 ROS 토픽으로 발행

**구현 항목**:
- [x] WooshRobot 연결 및 초기화 ✅ (2025-12-04)
- [x] `scanner_data_sub()` 콜백 구현 ✅ (2025-12-04)
- [x] `ScannerData` → `sensor_msgs/LaserScan` 변환 ✅ (2025-12-04)
- [x] ROS 토픽 `/mobile_lidar/scan` 발행 ✅ (2025-12-04)
- [ ] 연결 상태 모니터링 및 재연결 로직 (추후 구현)

**핵심 코드 구조**:
```python
class LidarSubscriber:
    def __init__(self):
        self.robot = None
        self.scan_pub = rospy.Publisher('/mobile_lidar/scan', LaserScan, queue_size=10)
    
    async def scanner_callback(self, data: ScannerData):
        """LiDAR 데이터 수신 콜백"""
        scan_msg = self._convert_to_ros_laserscan(data)
        self.scan_pub.publish(scan_msg)
    
    def _convert_to_ros_laserscan(self, data: ScannerData) -> LaserScan:
        """ScannerData → LaserScan 변환"""
        msg = LaserScan()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "laser_frame"
        msg.angle_min = data.angle_min
        msg.angle_max = data.angle_max
        msg.angle_increment = data.angle_increment
        msg.time_increment = data.time_increment
        msg.scan_time = data.scan_time
        msg.range_min = data.range_min
        msg.range_max = data.range_max
        msg.ranges = list(data.ranges)
        return msg
```

---

### Phase 2: Twist 명령 기반 로봇 구동 노드 구현

**파일**: `scripts/twist_motion_controller.py`

**목표**: ROS 서비스를 통해 Twist 명령으로 로봇 이동 제어

**구현 항목**:
- [x] 자체 `MoveDistance.srv` 서비스 정의 ✅ (2025-12-04)
- [x] 사다리꼴 속도 프로파일 적용 ✅ (2025-12-04)
- [x] 비상 정지 기능 구현 ✅ (2025-12-04)
- [x] 속도/가속도 제한 설정 ✅ (2025-12-04)

**서비스 정의** (`srv/MoveDistance.srv`):
```
# request
float32 distance    # 이동할 거리 (m), 음수는 후진
---
# response
bool success
string message
```

**주요 기능**:
- `/mobile_lidar_control/move_distance` - 거리 이동 서비스
- `/mobile_lidar_control/emergency_stop` - 비상 정지 서비스
- 사다리꼴 속도 프로파일 (가속 → 정속 → 감속)
- 50Hz 제어 루프
- ROS 파라미터로 속도/가속도 설정 가능

---

### Phase 3: LiDAR 기반 이동 거리 보정 구현

**파일**: `scripts/lidar_motion_corrector.py`

**목표**: LiDAR 센서값을 활용하여 실제 이동 거리를 측정하고 보정

**보정 알고리즘 선택지**:

| 방식 | 장점 | 단점 | 적용 환경 |
|------|------|------|----------|
| **Wall Following** | 구현 간단, 안정적 | 벽 필요 | 복도, 실내 |
| **Scan Matching (ICP)** | 정확도 높음 | 계산량 많음 | 범용 |
| **Feature-based** | 특징점 기반 정밀도 | 특징점 필요 | 구조화된 환경 |
| **단방향 거리 모니터링** | 가장 간단 | 정확도 제한 | 직선 이동 |

**권장 구현 (Phase 3-1): 단방향 거리 모니터링**
```python
class LidarMotionCorrector:
    """전방/후방 LiDAR 거리를 모니터링하여 이동 거리 측정"""
    
    def __init__(self):
        self.front_distance_initial = None
        self.front_distance_current = None
        
    def start_measurement(self, scan_data: LaserScan):
        """이동 시작 시 기준 거리 저장"""
        self.front_distance_initial = self._get_front_distance(scan_data)
        
    def get_traveled_distance(self, scan_data: LaserScan) -> float:
        """현재까지 이동한 거리 계산"""
        self.front_distance_current = self._get_front_distance(scan_data)
        return self.front_distance_initial - self.front_distance_current
        
    def _get_front_distance(self, scan_data: LaserScan) -> float:
        """전방 중앙 거리 추출 (필터링 적용)"""
        center_idx = len(scan_data.ranges) // 2
        front_ranges = scan_data.ranges[center_idx-5:center_idx+5]
        # inf, nan 필터링 후 중앙값 반환
        valid_ranges = [r for r in front_ranges 
                       if scan_data.range_min < r < scan_data.range_max]
        return np.median(valid_ranges) if valid_ranges else float('inf')
```

**보정 로직 통합**:
```
이동 시작
    │
    ├─▶ 초기 LiDAR 거리 측정 (d₀)
    │
    ├─▶ Twist 명령으로 이동
    │       │
    │       ├─▶ 시간 기반 추정 거리: d_est = v × t
    │       │
    │       └─▶ LiDAR 기반 실측 거리: d_real = d₀ - d_current
    │
    ├─▶ 오차 계산: error = d_target - d_real
    │
    └─▶ 보정 이동 또는 완료
```

---

### Phase 4: Rviz 시각화 연동

**파일**: `scripts/rviz_publisher.py`

**목표**: 로봇 상태, LiDAR 데이터, 이동 경로를 Rviz에서 시각화

**구현 항목**:
- [ ] `/mobile_lidar/scan` 토픽 Rviz 연동
- [ ] TF 브로드캐스터 구현 (`base_link` ↔ `laser_frame`)
- [ ] 목표 위치 마커 발행
- [ ] 이동 경로 마커 발행
- [ ] 현재 로봇 위치 마커 발행

**TF 트리 구조**:
```
odom
 └── base_link
      └── laser_frame
```

**마커 토픽 목록**:
| 토픽 | 타입 | 설명 |
|------|------|------|
| `/mobile_lidar/target_marker` | Marker | 목표 위치 표시 |
| `/mobile_lidar/path_marker` | Marker | 이동 경로 표시 |
| `/mobile_lidar/robot_marker` | Marker | 로봇 현재 위치 |

---

## 📁 최종 패키지 구조

```
mobile_lidar_control/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/
│   └── params.yaml                    # 파라미터 설정
├── docs/
│   └── WORK_ORDER.md                  # 작업 지시서 (본 문서)
├── scripts/
│   ├── lidar_subscriber.py            # Phase 1: LiDAR 데이터 획득
│   ├── twist_motion_controller.py     # Phase 2: Twist 기반 이동 제어
│   ├── lidar_motion_corrector.py      # Phase 3: LiDAR 기반 보정
│   └── rviz_publisher.py              # Phase 4: Rviz 시각화
├── srv/
│   └── MobilePositionTwist.srv        # 이동 서비스 정의
├── msg/
│   └── (필요시 커스텀 메시지)
└── rviz/
    └── mobile_lidar.rviz              # Rviz 설정 파일
```

---

## ⚙️ 의존성 및 환경 설정

### 필수 의존성

```yaml
# package.xml에 추가 필요
rospy
std_msgs
geometry_msgs
sensor_msgs
nav_msgs
visualization_msgs
tf2_ros
```

### Python 의존성

```
# woosh_robot_py SDK (이미 설치됨)
numpy
```

### 로봇 연결 정보

```yaml
robot_ip: "169.254.128.2"
robot_port: 5480
robot_identity: "mobile_lidar_ctrl"
```

---

## 🔧 실행 방법

### 1. 서버 실행 (기존)
```bash
# 터미널 1: 로봇 서버 실행
rosrun mobile_robot_server ldj_mobile_posiotion_server_twist.py
```

### 2. LiDAR 노드 실행
```bash
# 터미널 2: LiDAR 구독 노드
rosrun mobile_lidar_control lidar_subscriber.py
```

### 3. Rviz 시각화
```bash
# 터미널 3: Rviz 실행
rviz -d $(rospack find mobile_lidar_control)/rviz/mobile_lidar.rviz
```

### 4. 이동 명령 테스트
```bash
# 터미널 4: 이동 서비스 호출
rosservice call /mobile_positiontwist "{distance: 0.5}"
```

---

## 📈 성능 목표

| 항목 | 현재 (시간 기반) | 목표 (LiDAR 보정) |
|------|-----------------|-------------------|
| 위치 정확도 | ±5cm | ±1cm |
| 반복 정밀도 | ±3cm | ±0.5cm |
| 제어 주기 | 50Hz | 50Hz |
| 최대 속도 | 0.12 m/s | 0.15 m/s |

---

## ⚠️ 주의사항 및 안전

1. **비상 정지**: 모든 이동 명령에 타임아웃 및 비상 정지 기능 포함
2. **속도 제한**: 하드웨어 안전 속도 이하로 제한
3. **장애물 감지**: LiDAR 데이터로 전방 장애물 감지 시 자동 정지
4. **연결 끊김 처리**: 로봇 연결 끊김 시 즉시 정지

---

## 📅 개발 일정 (예상)

| Phase | 작업 내용 | 예상 소요 |
|-------|----------|----------|
| 1 | LiDAR 센서값 획득 노드 | 1일 |
| 2 | Twist 이동 제어 노드 | 1일 |
| 3 | LiDAR 기반 보정 구현 | 2일 |
| 4 | Rviz 시각화 연동 | 1일 |
| - | 통합 테스트 및 튜닝 | 2일 |
| **Total** | | **7일** |

---

## 📚 참고 자료

- [WooshRobot SDK 문서](/home/katech/robot_ws/src/woosh_robot_py/README.md)
- [기존 서버 코드](/home/katech/robot_ws/src/mobile_robot_server/scripts/ldj_mobile_posiotion_server_twist.py)
- [testbed_operation 클라이언트](/home/katech/robot_ws/src/testbed_operation/scripts/testbed_operation_client_all.py)
- [ROS LaserScan 메시지](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html)

---

---

## 📌 진행 현황 (Progress Log)

### 2025-12-04: Phase 1 구현 및 테스트 완료 ✅

**구현 완료 항목:**
- ✅ `scripts/lidar_subscriber.py` 생성
  - WooshRobot SDK 연결 및 초기화
  - `scanner_data_sub()` 콜백 구현
  - `ScannerData` → `sensor_msgs/LaserScan` 변환
  - `/mobile_lidar/scan` 토픽 발행
  - asyncio + ROS 통합 구조 (별도 스레드)
  - ROS 파라미터 지원 (`~robot_ip`, `~robot_port`, `~frame_id`, `~verbose`)

**테스트 결과 (2025-12-04):**
```bash
# 토픽 발행 주파수
$ rostopic hz /mobile_lidar/scan
average rate: 5.036
    min: 0.000s max: 0.779s std dev: 0.15634s window: 32

# 토픽 정보
$ rostopic info /mobile_lidar/scan
Type: sensor_msgs/LaserScan
Publishers: 
 * /lidar_subscriber (http://katech:42615/)

# 메시지 타입
$ rostopic type /mobile_lidar/scan
sensor_msgs/LaserScan
```

**유닛 테스트 결과:**
```
Ran 9 tests in 0.081s - OK
  - 실행: 9, 성공: 9, 실패: 0, 에러: 0, 스킵: 0
```

**미구현 항목:**
- 연결 끊김 시 자동 재연결 로직

---

### 2025-12-04: Phase 2 구현 완료 ✅

**구현 완료 항목:**
- ✅ `scripts/twist_motion_controller.py` 생성
  - 사다리꼴 속도 프로파일 (가속-정속-감속)
  - ROS 서비스 인터페이스 (`/mobile_lidar_control/move_distance`)
  - 비상 정지 서비스 (`/mobile_lidar_control/emergency_stop`)
  - 50Hz 제어 루프
  - 속도/가속도 제한 파라미터화

- ✅ `srv/MoveDistance.srv` 서비스 정의 생성

**테스트 방법:**
```bash
# 1. 빌드 (서비스 생성)
cd ~/catkin_ws && catkin_make --force-cmake

# 2. 노드 실행
rosrun mobile_lidar_control twist_motion_controller.py

# 3. 이동 서비스 호출 (다른 터미널)
rosservice call /mobile_lidar_control/move_distance "{distance: 0.5}"
rosservice call /mobile_lidar_control/move_distance "{distance: -0.3}"

# 4. 비상 정지
rosservice call /mobile_lidar_control/emergency_stop
```

**ROS 파라미터:**
| 파라미터 | 기본값 | 설명 |
|----------|--------|------|
| `~robot_ip` | 169.254.128.2 | 로봇 IP |
| `~robot_port` | 5480 | 로봇 포트 |
| `~max_linear_vel` | 0.12 | 최대 선속도 (m/s) |
| `~linear_accel` | 0.25 | 가속도 (m/s²) |
| `~linear_decel` | 0.50 | 감속도 (m/s²) |
| `~control_rate` | 50.0 | 제어 주기 (Hz) |
| `~timeout` | 30.0 | 타임아웃 (s) |

---

*작성일: 2025-12-04*
*작성자: AI Assistant*
*검토: User (KATECH Robotics Team)*
*최종 수정: 2025-12-04*

