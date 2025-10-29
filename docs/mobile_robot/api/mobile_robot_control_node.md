# Mobile Robot Control Node API Reference

## 개요

`mobile_robot_control_node.py`는 Woosh 모바일 로봇(TR200)을 제어하기 위한 확장 가능한 ROS 노드입니다. Odometry 기반 정밀 제어, 부드러운 사다리꼴 속도 프로파일, 비동기 통신을 지원합니다.

**작성자**: KATECH Robotics Team  
**라이센스**: MIT  
**ROS 버전**: ROS 1 Noetic  
**Python 버전**: 3.8+

---

## 목차

1. [데이터 클래스](#데이터-클래스)
2. [메인 클래스](#메인-클래스)
3. [유틸리티 클래스](#유틸리티-클래스)
4. [CLI 인터페이스](#cli-인터페이스)

---

## 데이터 클래스

### RobotConfig

로봇 연결 및 제어 설정을 담는 데이터 클래스입니다.

```python
@dataclass
class RobotConfig:
    ip: str = '169.254.128.2'
    port: int = 5480
    identity: str = 'mobile_robot_controller'
    verbose: bool = False
```

#### 속성

| 속성 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `ip` | `str` | `'169.254.128.2'` | 로봇 IP 주소 |
| `port` | `int` | `5480` | 로봇 WebSocket 포트 |
| `identity` | `str` | `'mobile_robot_controller'` | SDK 연결 식별자 |
| `verbose` | `bool` | `False` | 상세 로그 출력 여부 |

#### 사용 예시

```python
config = RobotConfig(
    ip='192.168.1.100',
    port=5480,
    verbose=True
)
```

---

### VelocityProfileConfig

속도 프로파일 설정을 담는 데이터 클래스입니다.

```python
@dataclass
class VelocityProfileConfig:
    max_speed: float = 0.2
    min_speed: float = 0.03
    accel_distance: float = 0.15
    decel_distance: float = 0.2
    control_hz: float = 20.0
```

#### 속성

| 속성 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `max_speed` | `float` | `0.2` | 최대 속도 (m/s) |
| `min_speed` | `float` | `0.03` | 최소 속도 (m/s) - 로봇이 멈추지 않는 최저 속도 |
| `accel_distance` | `float` | `0.15` | 가속 구간 거리 (m) |
| `decel_distance` | `float` | `0.2` | 감속 구간 거리 (m) |
| `control_hz` | `float` | `20.0` | Twist 명령 전송 주기 (Hz) |

#### 사용 예시

```python
velocity_config = VelocityProfileConfig(
    max_speed=0.3,
    accel_distance=0.1,
    decel_distance=0.25,
    control_hz=30.0
)
```

---

### MotionResult

이동 결과를 담는 데이터 클래스입니다.

```python
@dataclass
class MotionResult:
    success: bool
    traveled_distance: float
    target_distance: float
    error: float
    duration: float
```

#### 속성

| 속성 | 타입 | 설명 |
|------|------|------|
| `success` | `bool` | 이동 성공 여부 |
| `traveled_distance` | `float` | 실제 이동 거리 (m) |
| `target_distance` | `float` | 목표 이동 거리 (m) |
| `error` | `float` | 오차 (m) |
| `duration` | `float` | 소요 시간 (초) |

---

### MotionPhase (Enum)

이동 단계를 나타내는 열거형입니다.

```python
class MotionPhase(Enum):
    ACCELERATION = "🚀 가속"
    CONSTANT = "⚡ 등속"
    DECELERATION = "🛑 감속"
```

---

## 유틸리티 클래스

### VelocityProfileCalculator

사다리꼴 속도 프로파일을 계산하는 클래스입니다.

#### 생성자

```python
def __init__(self, config: VelocityProfileConfig)
```

**인자:**
- `config` (`VelocityProfileConfig`): 속도 프로파일 설정

#### 메서드

##### adjust_for_distance()

목표 거리에 맞춰 가감속 구간을 자동 조정합니다.

```python
def adjust_for_distance(self, target_distance: float) -> None
```

**인자:**
- `target_distance` (`float`): 목표 이동 거리 (m)

**동작:**
- 가감속 구간의 합이 목표 거리의 80%를 초과하면 자동으로 축소
- 조정 시 경고 로그 출력

**예시:**
```python
profile_calc = VelocityProfileCalculator(velocity_config)
profile_calc.adjust_for_distance(0.5)  # 0.5m 이동에 맞춰 조정
```

---

##### calculate_speed()

현재 위치에서의 목표 속도를 계산합니다 (사다리꼴 프로파일).

```python
def calculate_speed(
    self,
    traveled_distance: float,
    remaining_distance: float
) -> Tuple[float, MotionPhase]
```

**인자:**
- `traveled_distance` (`float`): 이미 이동한 거리 (m)
- `remaining_distance` (`float`): 남은 거리 (m)

**반환:**
- `Tuple[float, MotionPhase]`: (목표 속도, 이동 단계)

**로직:**
- **가속 구간**: `traveled_distance < accel_distance`
  - 선형 증가: `min_speed → max_speed`
- **등속 구간**: 가속 완료 후 감속 전까지
  - `max_speed` 유지
- **감속 구간**: `remaining_distance < decel_distance`
  - 선형 감소: `max_speed → min_speed`

**예시:**
```python
target_speed, phase = profile_calc.calculate_speed(0.1, 0.4)
print(f"속도: {target_speed:.2f}m/s, 단계: {phase.value}")
# 출력: 속도: 0.15m/s, 단계: 🚀 가속
```

---

## 메인 클래스

### MobileRobotController

모바일 로봇 제어 메인 클래스입니다.

#### 생성자

```python
def __init__(self, config: RobotConfig)
```

**인자:**
- `config` (`RobotConfig`): 로봇 설정

**동작:**
- ROS 노드 초기화 (`mobile_robot_control`)
- 설정 저장 및 초기 상태 설정

---

### 연결 관리 메서드

#### connect()

로봇에 연결하고 초기화합니다.

```python
async def connect(self) -> None
```

**동작:**
1. SDK 로거 설정
2. WebSocket 연결 수립
3. 연결 상태 검증 (`robot_info_req`)
4. 위치 피드백 구독 시작

**예외:**
- `ConnectionError`: 연결 실패 시

**예시:**
```python
controller = MobileRobotController(config)
await controller.connect()
```

---

#### disconnect()

로봇 연결을 종료합니다.

```python
async def disconnect(self) -> None
```

**동작:**
- SDK 연결 종료
- 리소스 정리

---

### 위치 관리 메서드

#### get_current_pose()

현재 위치를 조회합니다 (요청 방식).

```python
async def get_current_pose(self)
```

**반환:**
- `Pose` 객체 또는 `None` (실패 시)

**예시:**
```python
pose = await controller.get_current_pose()
if pose:
    print(f"X: {pose.x:.3f}, Y: {pose.y:.3f}, Theta: {pose.theta:.3f}")
```

---

#### calculate_distance() (정적 메서드)

두 위치 사이의 유클리드 거리를 계산합니다.

```python
@staticmethod
def calculate_distance(pose1, pose2) -> float
```

**인자:**
- `pose1`: 시작 위치 (Pose 객체)
- `pose2`: 끝 위치 (Pose 객체)

**반환:**
- `float`: 거리 (m)

**수식:**
```
distance = √((x₂ - x₁)² + (y₂ - y₁)²)
```

---

### 이동 제어 메서드

#### move_distance()

지정된 거리만큼 로봇을 이동합니다 (Odometry 기반 정밀 제어).

```python
async def move_distance(
    self,
    target_distance: float,
    speed: float = 0.2,
    angle: float = 0.0,
    timeout: float = 60.0,
    tolerance: float = 0.02,
    velocity_config: Optional[VelocityProfileConfig] = None
) -> MotionResult
```

**인자:**

| 인자 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `target_distance` | `float` | - | 목표 이동 거리 (m) |
| `speed` | `float` | `0.2` | 최대 이동 속도 (m/s), 음수면 후진 |
| `angle` | `float` | `0.0` | 회전 각속도 (rad/s), 0이면 직진 |
| `timeout` | `float` | `60.0` | 최대 대기 시간 (초) |
| `tolerance` | `float` | `0.02` | 허용 오차 (m) |
| `velocity_config` | `Optional[VelocityProfileConfig]` | `None` | 속도 프로파일 설정 (None이면 기본값) |

**반환:**
- `MotionResult`: 이동 결과

**동작 흐름:**
1. 속도 프로파일 설정 및 조정
2. 시작 위치 기록
3. 사다리꼴 속도 프로파일로 이동
4. 목표 거리 도달 시 정지
5. 최종 위치 확인 및 결과 반환

**예시:**

```python
# 기본 사용: 전진 0.5m
result = await controller.move_distance(0.5, speed=0.2)

# 후진 0.3m (음수 속도)
result = await controller.move_distance(0.3, speed=-0.15)

# 커스텀 속도 프로파일
velocity_config = VelocityProfileConfig(
    max_speed=0.3,
    accel_distance=0.1,
    decel_distance=0.25
)
result = await controller.move_distance(
    target_distance=1.0,
    speed=0.3,
    velocity_config=velocity_config
)

print(f"성공: {result.success}")
print(f"오차: {result.error:.3f}m ({result.error/result.target_distance*100:.1f}%)")
```

---

#### rotate()

제자리 회전합니다.

```python
async def rotate(
    self,
    angle_degrees: float,
    angular_speed: float = 0.5,
    control_hz: float = 20
) -> bool
```

**인자:**

| 인자 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `angle_degrees` | `float` | - | 회전 각도 (도), 양수=좌회전, 음수=우회전 |
| `angular_speed` | `float` | `0.5` | 회전 속도 (rad/s) |
| `control_hz` | `float` | `20` | 제어 주기 (Hz) |

**반환:**
- `bool`: 성공 여부

**예시:**

```python
# 90도 좌회전
await controller.rotate(90)

# 180도 우회전 (빠른 속도)
await controller.rotate(-180, angular_speed=1.0)
```

---

## CLI 인터페이스

### 명령줄 인자

#### 이동 관련

| 인자 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `--distance` | `float` | `0.5` | 이동 거리 (m) |
| `--speed` | `float` | `0.2` | 이동 속도 (m/s), 음수면 후진 |
| `--rotate` | `float` | `None` | 회전 각도 (도), 지정 시 회전 모드 |

#### 속도 프로파일

| 인자 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `--accel` | `float` | `0.15` | 가속 구간 거리 (m) |
| `--decel` | `float` | `0.2` | 감속 구간 거리 (m) |
| `--control-hz` | `float` | `20` | 제어 주기 (Hz) |

#### 연결 관련

| 인자 | 타입 | 기본값 | 설명 |
|------|------|--------|------|
| `--ip` | `str` | `'169.254.128.2'` | 로봇 IP 주소 |
| `--port` | `int` | `5480` | 로봇 포트 |
| `--verbose` | `flag` | `False` | 상세 모드 (SDK 로그 포함) |

### 사용 예시

```bash
# 기본: 전진 1m (부드러운 가감속)
python3 mobile_robot_control_node.py --distance 1.0 --speed 0.2

# 가감속 커스텀: 빠른 가속, 긴 감속
python3 mobile_robot_control_node.py --distance 1.0 --speed 0.3 --accel 0.1 --decel 0.3

# 후진 0.5m
python3 mobile_robot_control_node.py --distance 0.5 --speed -0.2

# 90도 좌회전
python3 mobile_robot_control_node.py --rotate 90

# 다른 IP의 로봇 제어
python3 mobile_robot_control_node.py --ip 192.168.1.100 --distance 0.8

# 상세 모드 (디버깅용)
python3 mobile_robot_control_node.py --distance 0.5 --verbose
```

---

## 내부 메서드 (고급)

### _execute_motion()

이동 실행 내부 로직입니다.

```python
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
) -> MotionResult
```

**제어 루프 로직:**
1. 주기적으로 Twist 명령 전송 (20Hz)
2. Odometry로 현재 위치 확인
3. 속도 프로파일 계산 및 적용
4. 목표 도달 시 정지

---

### _send_twist_command()

Twist 명령을 전송합니다.

```python
async def _send_twist_command(self, linear_speed: float, angular_speed: float) -> None
```

---

### _stop_robot()

로봇을 정지합니다 (3번 전송으로 확실한 정지 보장).

```python
async def _stop_robot(self) -> None
```

---

## 의존성

### 필수 패키지

```bash
# ROS 1 Noetic
sudo apt-get install ros-noetic-ros-base

# Python 패키지
pip install asyncio
```

### SDK 의존성

```python
from woosh_robot import WooshRobot
from woosh_interface import CommuSettings, NO_PRINT
from woosh.proto.robot.robot_pb2 import RobotInfo, PoseSpeed
from woosh.proto.robot.robot_pack_pb2 import Twist
```

---

## 성능 특성

### 제어 정밀도

- **위치 오차**: 일반적으로 ±2cm (기본 `tolerance=0.02`)
- **속도 제어**: 20Hz 주기 (50ms마다 업데이트)
- **응답 시간**: 약 10~25ms (Odometry 업데이트 포함)

### 속도 프로파일

**사다리꼴 프로파일 예시 (1.0m 이동, 속도 0.2m/s):**

```
속도 (m/s)
0.20 │     ╱▔▔▔▔▔▔▔╲
0.15 │    ╱         ╲
0.10 │   ╱           ╲
0.05 │  ╱             ╲
0.03 │_╱               ╲_
     └───────────────────→ 거리 (m)
     0  0.15  0.65  0.8 1.0
        가속   등속   감속
```

---

## 주의사항

1. **Odometry 정확도**: 바닥 미끄러짐이나 센서 오차로 인해 실제 거리와 차이가 발생할 수 있습니다.
2. **Twist 명령 특성**: 연속적으로 전송해야 하며 (조이스틱처럼), 단발성 명령은 무시됩니다.
3. **최소 속도**: `min_speed=0.03m/s` 이하로 설정 시 로봇이 멈출 수 있습니다.
4. **가감속 조정**: 짧은 거리 이동 시 가감속 구간이 자동으로 축소됩니다.

---

## 라이센스

MIT License - KATECH Robotics Team

