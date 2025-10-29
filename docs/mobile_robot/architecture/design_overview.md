# Mobile Robot Control Node - 아키텍처 개요

## 📚 목차

1. [시스템 개요](#시스템-개요)
2. [코드 구조](#코드-구조)
3. [설계 원칙](#설계-원칙)
4. [주요 컴포넌트](#주요-컴포넌트)
5. [데이터 흐름](#데이터-흐름)
6. [확장 가이드](#확장-가이드)

---

## 시스템 개요

`mobile_robot_control_node.py`는 Woosh 모바일 로봇(TR200)을 제어하기 위한 **모듈화되고 확장 가능한** ROS 노드입니다.

### 설계 목표

1. **정밀성**: Odometry 피드백으로 ±2cm 정밀도
2. **부드러움**: 사다리꼴 속도 프로파일로 자연스러운 이동
3. **확장성**: 새로운 기능 추가가 용이한 구조
4. **안정성**: 에러 처리 및 타임아웃 메커니즘
5. **가독성**: 명확한 클래스 분리 및 문서화

---

## 코드 구조

### 전체 아키텍처

```
mobile_robot_control_node.py
│
├── 📦 데이터 클래스
│   ├── RobotConfig              # 로봇 연결 설정
│   ├── VelocityProfileConfig    # 속도 프로파일 설정
│   ├── MotionResult             # 이동 결과
│   └── MotionPhase (Enum)       # 이동 단계 (가속/등속/감속)
│
├── 🧮 유틸리티 클래스
│   └── VelocityProfileCalculator # 속도 프로파일 계산
│
├── 🤖 메인 컨트롤러
│   └── MobileRobotController     # 로봇 제어 메인 클래스
│       ├── connect()             # 연결 관리
│       ├── move_distance()       # 이동 제어
│       ├── rotate()              # 회전 제어
│       └── get_current_pose()    # 위치 관리
│
└── 🖥️ CLI 인터페이스
    └── main()                    # 명령줄 실행
```

### 클래스 다이어그램

```
┌─────────────────────┐
│   RobotConfig       │
├─────────────────────┤
│ + ip: str           │
│ + port: int         │
│ + identity: str     │
│ + verbose: bool     │
└─────────────────────┘
          │
          │ 사용됨
          ▼
┌─────────────────────────────────┐
│ MobileRobotController           │
├─────────────────────────────────┤
│ - config: RobotConfig           │
│ - robot: WooshRobot             │
│ - current_pose: Pose            │
├─────────────────────────────────┤
│ + connect()                     │
│ + disconnect()                  │
│ + move_distance()               │
│ + rotate()                      │
│ + get_current_pose()            │
│ - _execute_motion()             │
│ - _send_twist_command()         │
│ - _stop_robot()                 │
└─────────────────────────────────┘
          │
          │ 사용함
          ▼
┌─────────────────────────────────┐
│ VelocityProfileCalculator       │
├─────────────────────────────────┤
│ - config: VelocityProfileConfig │
│ - accel_distance: float         │
│ - decel_distance: float         │
├─────────────────────────────────┤
│ + adjust_for_distance()         │
│ + calculate_speed()             │
└─────────────────────────────────┘
```

---

## 설계 원칙

### 1. 관심사의 분리 (Separation of Concerns)

각 클래스는 명확한 단일 책임을 갖습니다:

- **RobotConfig**: 설정 관리
- **VelocityProfileCalculator**: 속도 계산
- **MobileRobotController**: 로봇 제어

### 2. 의존성 주입 (Dependency Injection)

```python
# ❌ 나쁜 예: 하드코딩
class MobileRobotController:
    def __init__(self):
        self.robot_ip = '169.254.128.2'  # 변경 어려움

# ✅ 좋은 예: 설정 주입
class MobileRobotController:
    def __init__(self, config: RobotConfig):
        self.config = config  # 외부에서 설정 제공
```

### 3. 데이터 클래스 활용

타입 힌팅과 데이터 검증을 위해 `@dataclass` 사용:

```python
@dataclass
class VelocityProfileConfig:
    max_speed: float = 0.2
    min_speed: float = 0.03
    # ... 명확한 타입과 기본값
```

### 4. 비동기 프로그래밍

WebSocket 통신의 특성상 `async/await` 사용:

```python
async def move_distance(self, target_distance: float) -> MotionResult:
    # 비블로킹 I/O로 효율적인 제어
    await self.robot.twist_req(twist)
```

---

## 주요 컴포넌트

### 1. 연결 관리 (Connection Management)

#### connect()

```python
async def connect(self) -> None:
    """
    1. SDK 로거 설정
    2. WebSocket 연결
    3. 연결 검증
    4. 위치 피드백 구독
    """
```

**시퀀스 다이어그램:**

```
Controller         SDK            Robot
    │               │               │
    │──setup_logger→│               │
    │               │               │
    │───────────────run()──────────→│
    │               │               │
    │←───────────connected──────────│
    │               │               │
    │─robot_info_req()─────────────→│
    │               │               │
    │←─────────battery:80%──────────│
    │               │               │
    │─pose_speed_sub()─────────────→│
    │               │               │
    │←──────streaming pose──────────│
```

---

### 2. 속도 프로파일 계산 (Velocity Profile)

#### VelocityProfileCalculator

**사다리꼴 프로파일 로직:**

```python
def calculate_speed(self, traveled, remaining):
    if traveled < accel_distance:
        # 가속 구간: 선형 증가
        progress = traveled / accel_distance  # 0.0 → 1.0
        speed = min_speed + (max_speed - min_speed) * progress
        
    elif remaining < decel_distance:
        # 감속 구간: 선형 감소
        progress = remaining / decel_distance  # 1.0 → 0.0
        speed = min_speed + (max_speed - min_speed) * progress
        
    else:
        # 등속 구간
        speed = max_speed
    
    return speed, phase
```

**그래프:**

```
속도 (m/s)
     │
0.20 │       B────────C
     │      ╱          ╲
0.15 │     ╱            ╲
     │    ╱              ╲
0.10 │   ╱                ╲
     │  ╱                  ╲
0.03 │_A                    D___
     └──────────────────────────→ 거리 (m)
     0   0.15        0.80  1.0
         ↑            ↑
      accel_distance decel_distance
```

- **A → B**: 가속 (0.03 → 0.20 m/s)
- **B → C**: 등속 (0.20 m/s 유지)
- **C → D**: 감속 (0.20 → 0.03 m/s)

---

### 3. 이동 제어 (Motion Control)

#### move_distance() 실행 흐름

```
┌─────────────────────────────┐
│ 1. 파라미터 검증 및 설정     │
│    - 속도 프로파일 생성      │
│    - 가감속 구간 조정        │
└──────────┬──────────────────┘
           │
           ▼
┌─────────────────────────────┐
│ 2. 시작 위치 기록           │
│    start_pose = get_pose()  │
└──────────┬──────────────────┘
           │
           ▼
┌─────────────────────────────┐
│ 3. 제어 루프                │
│  ┌─────────────────────┐   │
│  │ while not_reached:  │   │
│  │  - Send Twist       │   │
│  │  - Get current pose │   │
│  │  - Calculate speed  │   │
│  │  - Check target     │   │
│  └─────────────────────┘   │
└──────────┬──────────────────┘
           │
           ▼
┌─────────────────────────────┐
│ 4. 정지                     │
│    - Stop Twist × 3         │
│    - Verify stopped         │
└──────────┬──────────────────┘
           │
           ▼
┌─────────────────────────────┐
│ 5. 결과 반환                │
│    - Final distance         │
│    - Error                  │
│    - Duration               │
└─────────────────────────────┘
```

---

### 4. 제어 루프 (Control Loop)

#### 주기적 Twist 전송

```python
control_period = 1.0 / control_hz  # 20Hz → 0.05초

while True:
    current_time = loop.time()
    
    # 주기적으로 Twist 명령 전송 (조이스틱처럼)
    if current_time - last_control_time >= control_period:
        await send_twist(speed)
        last_control_time = current_time
    
    # Odometry 확인 및 속도 조정
    pose = await get_pose()
    speed = calculate_speed(traveled, remaining)
    
    # 목표 도달 확인
    if reached_target():
        break
    
    await sleep(control_period / 2)  # 응답성 향상
```

**타이밍 다이어그램:**

```
시간 (ms)   0    25    50    75   100   125
            │     │     │     │     │     │
Twist 전송  ●─────●─────●─────●─────●─────●  (20Hz)
            │     │     │     │     │     │
Pose 확인   ●──●──●──●──●──●──●──●──●──●──●  (40Hz)
            │     │     │     │     │     │
            └─────┴─────┴─────┴─────┴─────┘
             50ms  50ms  50ms  50ms  50ms
```

---

## 데이터 흐름

### 전체 데이터 흐름도

```
┌──────────────┐
│  User Input  │
│ (CLI or API) │
└──────┬───────┘
       │
       ▼
┌─────────────────────────┐
│ MobileRobotController   │
│                         │
│  move_distance(1.0m)    │
└──────┬──────────────────┘
       │
       ├─────────────────────────────────┐
       │                                 │
       ▼                                 ▼
┌───────────────────┐          ┌──────────────────┐
│ VelocityProfile   │          │   WooshRobot     │
│    Calculator     │          │      SDK         │
│                   │          │                  │
│ calculate_speed() │          │  twist_req()     │
│       │           │          │       │          │
│       │           │          │       │          │
│   speed=0.15m/s   │          │   Twist(0.15)    │
│       │           │          │       │          │
└───────┼───────────┘          └───────┼──────────┘
        │                              │
        └────────────┬─────────────────┘
                     │
                     ▼
              ┌─────────────┐
              │   Robot     │
              │   Hardware  │
              │             │
              │  Move!      │
              └──────┬──────┘
                     │
                     │ Odometry
                     │ Feedback
                     ▼
              ┌─────────────┐
              │ PoseSpeed   │
              │  Callback   │
              │             │
              │ current_pose│
              └──────┬──────┘
                     │
                     ▼
              ┌──────────────┐
              │  Distance    │
              │  Calculation │
              │              │
              │ traveled=0.5m│
              └──────────────┘
```

---

## 확장 가이드

### 1. 새로운 속도 프로파일 추가

**예: S-커브 프로파일**

```python
class SCurveProfileCalculator(VelocityProfileCalculator):
    """S-커브 속도 프로파일"""
    
    def calculate_speed(self, traveled, remaining):
        if traveled < self.accel_distance:
            progress = traveled / self.accel_distance
            # Smoothstep: 3x² - 2x³
            smooth = 3 * progress**2 - 2 * progress**3
            speed = self.config.min_speed + \
                   (self.config.max_speed - self.config.min_speed) * smooth
            return speed, MotionPhase.ACCELERATION
        # ... (등속, 감속 구간 생략)
```

**사용:**

```python
profile = SCurveProfileCalculator(velocity_config)
# move_distance()에서 profile_calc 파라미터로 전달
```

---

### 2. 새로운 이동 패턴 추가

**예: 원형 경로 이동**

```python
class MobileRobotController:
    # ... (기존 코드)
    
    async def move_circle(
        self,
        radius: float,
        speed: float = 0.2,
        clockwise: bool = True
    ) -> bool:
        """
        원형 경로 이동
        
        Args:
            radius: 반지름 (m)
            speed: 이동 속도 (m/s)
            clockwise: 시계방향 여부
        """
        # 원주 계산
        circumference = 2 * math.pi * radius
        
        # 각속도 계산 (v = ωr → ω = v/r)
        angular_speed = speed / radius
        if clockwise:
            angular_speed = -angular_speed
        
        # 이동 시간 계산
        duration = circumference / speed
        
        # Twist 명령 전송
        twist = Twist(linear=speed, angular=angular_speed)
        
        start_time = asyncio.get_event_loop().time()
        while asyncio.get_event_loop().time() - start_time < duration:
            await self.robot.twist_req(twist, NO_PRINT, NO_PRINT)
            await asyncio.sleep(0.05)
        
        # 정지
        await self._stop_robot()
        return True
```

---

### 3. ROS Action Server 통합

**예: 이동 액션 서버**

```python
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback, MoveBaseResult

class MobileRobotActionServer:
    """ROS Action Server for mobile robot control"""
    
    def __init__(self, controller: MobileRobotController):
        self.controller = controller
        self.server = actionlib.SimpleActionServer(
            'mobile_robot/move',
            MoveBaseAction,
            execute_cb=self.execute_callback,
            auto_start=False
        )
        self.server.start()
    
    async def execute_callback(self, goal: MoveBaseGoal):
        """액션 실행 콜백"""
        # 목표 거리 추출
        target_distance = goal.target_pose.pose.position.x
        
        # Feedback 전송 루프
        start_pose = await self.controller.get_current_pose()
        
        # 이동 태스크 생성
        move_task = asyncio.create_task(
            self.controller.move_distance(target_distance, speed=0.2)
        )
        
        # Feedback 전송
        while not move_task.done():
            current_pose = await self.controller.get_current_pose()
            traveled = self.controller.calculate_distance(start_pose, current_pose)
            
            feedback = MoveBaseFeedback()
            feedback.base_position.pose.position.x = traveled
            self.server.publish_feedback(feedback)
            
            await asyncio.sleep(0.5)
        
        # Result 전송
        result_data = await move_task
        result = MoveBaseResult()
        result.success = result_data.success
        self.server.set_succeeded(result)
```

---

### 4. 센서 통합

**예: 라이다 기반 장애물 회피**

```python
class ObstacleAvoidanceController(MobileRobotController):
    """장애물 회피 기능이 추가된 컨트롤러"""
    
    def __init__(self, config: RobotConfig):
        super().__init__(config)
        
        # 라이다 구독
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        self.min_obstacle_distance = float('inf')
    
    def lidar_callback(self, msg: LaserScan):
        """라이다 데이터 수신"""
        # 전방 ±30도 범위의 최소 거리
        front_ranges = msg.ranges[330:] + msg.ranges[:30]
        self.min_obstacle_distance = min(front_ranges)
    
    async def move_distance_safe(self, target_distance: float, speed: float = 0.2):
        """
        장애물 회피하며 이동
        """
        traveled = 0.0
        
        while traveled < target_distance:
            # 장애물 확인
            if self.min_obstacle_distance < 0.5:
                rospy.logwarn("장애물 감지! 회피 중...")
                
                # 정지
                await self._stop_robot()
                
                # 우회전
                await self.rotate(-90)
                
                # 옆으로 이동
                await self.move_distance(0.5, speed=0.15)
                
                # 좌회전
                await self.rotate(90)
            
            else:
                # 안전하면 전진
                result = await self.move_distance(0.3, speed=speed)
                traveled += result.traveled_distance
```

---

## 성능 최적화

### 1. 제어 주기 조정

```python
# 더 부드러운 제어 (높은 CPU 사용)
velocity_config = VelocityProfileConfig(control_hz=30)

# 표준 제어 (권장)
velocity_config = VelocityProfileConfig(control_hz=20)

# 저사양 시스템 (낮은 CPU 사용)
velocity_config = VelocityProfileConfig(control_hz=10)
```

### 2. 로그 레벨 조정

```python
# 프로덕션: WARNING 이상만
config = RobotConfig(verbose=False)

# 개발/디버깅: 모든 로그
config = RobotConfig(verbose=True)
```

---

## 테스트 전략

### 단위 테스트

```python
import unittest
from mobile_robot_control_node import VelocityProfileCalculator, VelocityProfileConfig

class TestVelocityProfile(unittest.TestCase):
    def setUp(self):
        config = VelocityProfileConfig(
            max_speed=0.2,
            min_speed=0.03,
            accel_distance=0.15,
            decel_distance=0.2
        )
        self.calc = VelocityProfileCalculator(config)
    
    def test_acceleration_phase(self):
        """가속 구간 테스트"""
        speed, phase = self.calc.calculate_speed(
            traveled_distance=0.075,  # 가속 구간 중간
            remaining_distance=0.925
        )
        
        # 중간 속도여야 함
        self.assertGreater(speed, 0.03)
        self.assertLess(speed, 0.2)
        self.assertEqual(phase, MotionPhase.ACCELERATION)
```

---

## 보안 및 안전

### 안전 메커니즘

1. **타임아웃**: 무한 대기 방지
2. **정지 확인**: 3번 전송으로 확실한 정지
3. **오차 검증**: 허용 오차 초과 시 경고
4. **연결 검증**: 매 명령 전 연결 상태 확인

```python
# 타임아웃 설정
await controller.move_distance(1.0, timeout=30.0)

# 허용 오차 설정
await controller.move_distance(1.0, tolerance=0.03)
```

---

## 라이센스

MIT License - KATECH Robotics Team

---

**작성일**: 2025-10-29  
**버전**: 1.0.0

