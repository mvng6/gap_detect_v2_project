# Mobile Robot Control - 예제 모음

## 📚 목차

1. [기본 이동](#기본-이동)
2. [패턴 주행](#패턴-주행)
3. [센서 연동](#센서-연동)
4. [ROS 통합](#ros-통합)
5. [고급 제어](#고급-제어)

---

## 기본 이동

### 예제 1: 간단한 전진/후진

```python
#!/usr/bin/env python3
"""간단한 전진/후진 예제"""
import asyncio
from mobile_robot_control_node import MobileRobotController, RobotConfig

async def simple_move():
    """전진 후 후진"""
    config = RobotConfig(ip='169.254.128.2')
    controller = MobileRobotController(config)
    
    await controller.connect()
    print("로봇 연결 완료!")
    
    # 1m 전진
    print("\n[1/2] 1m 전진...")
    result = await controller.move_distance(1.0, speed=0.2)
    print(f"✅ 전진 완료 (오차: {result.error:.3f}m)")
    
    await asyncio.sleep(2.0)  # 2초 대기
    
    # 1m 후진
    print("\n[2/2] 1m 후진...")
    result = await controller.move_distance(1.0, speed=-0.2)
    print(f"✅ 후진 완료 (오차: {result.error:.3f}m)")
    
    await controller.disconnect()

if __name__ == "__main__":
    asyncio.run(simple_move())
```

**실행:**
```bash
python3 simple_move.py
```

---

### 예제 2: 회전 테스트

```python
#!/usr/bin/env python3
"""회전 각도 테스트"""
import asyncio
from mobile_robot_control_node import MobileRobotController, RobotConfig

async def rotation_test():
    """여러 각도로 회전 테스트"""
    config = RobotConfig()
    controller = MobileRobotController(config)
    await controller.connect()
    
    angles = [90, 180, -90, -180]  # 좌회전 90, 180, 우회전 90, 180
    
    for angle in angles:
        direction = "좌회전" if angle > 0 else "우회전"
        print(f"\n{direction} {abs(angle)}도...")
        
        await controller.rotate(angle, angular_speed=0.5)
        print(f"✅ 회전 완료")
        
        await asyncio.sleep(2.0)
    
    await controller.disconnect()

if __name__ == "__main__":
    asyncio.run(rotation_test())
```

---

## 패턴 주행

### 예제 3: 사각형 주행

```python
#!/usr/bin/env python3
"""사각형 경로 주행"""
import asyncio
from mobile_robot_control_node import (
    MobileRobotController,
    RobotConfig,
    VelocityProfileConfig
)

async def drive_square(side_length=0.5, speed=0.2):
    """
    사각형 경로 주행
    
    Args:
        side_length: 한 변의 길이 (m)
        speed: 이동 속도 (m/s)
    """
    config = RobotConfig()
    controller = MobileRobotController(config)
    await controller.connect()
    
    # 속도 프로파일 설정 (부드러운 이동)
    velocity_config = VelocityProfileConfig(
        max_speed=speed,
        accel_distance=0.1,
        decel_distance=0.15
    )
    
    print(f"사각형 주행 시작! (한 변: {side_length}m)")
    
    for i in range(4):
        # 직진
        print(f"\n[변 {i+1}/4] {side_length}m 직진...")
        result = await controller.move_distance(
            target_distance=side_length,
            speed=speed,
            velocity_config=velocity_config
        )
        print(f"  ✅ 완료 (오차: {result.error:.3f}m)")
        
        await asyncio.sleep(1.0)
        
        # 90도 좌회전 (마지막 변에서는 회전 생략)
        if i < 3:
            print(f"  🔄 90도 좌회전...")
            await controller.rotate(90, angular_speed=0.5)
            await asyncio.sleep(1.0)
    
    await controller.disconnect()
    print("\n✅ 사각형 주행 완료!")

if __name__ == "__main__":
    asyncio.run(drive_square(side_length=0.8, speed=0.25))
```

---

### 예제 4: 지그재그 패턴

```python
#!/usr/bin/env python3
"""지그재그 패턴 주행"""
import asyncio
from mobile_robot_control_node import MobileRobotController, RobotConfig

async def drive_zigzag(segment_length=0.5, num_segments=4):
    """
    지그재그 패턴 주행
    
    패턴:
        ───╮
           │
        ╭──╯
        │
        ╰──╮
    """
    config = RobotConfig()
    controller = MobileRobotController(config)
    await controller.connect()
    
    print(f"지그재그 주행 시작! (구간: {num_segments}개)")
    
    for i in range(num_segments):
        # 직진
        print(f"\n[구간 {i+1}/{num_segments}] {segment_length}m 직진...")
        await controller.move_distance(segment_length, speed=0.2)
        
        await asyncio.sleep(0.5)
        
        # 번갈아가며 좌/우 회전
        angle = 90 if i % 2 == 0 else -90
        direction = "좌회전" if angle > 0 else "우회전"
        print(f"  🔄 {direction} 90도...")
        await controller.rotate(angle)
        
        await asyncio.sleep(0.5)
    
    await controller.disconnect()
    print("\n✅ 지그재그 주행 완료!")

if __name__ == "__main__":
    asyncio.run(drive_zigzag(segment_length=0.6, num_segments=6))
```

---

## 센서 연동

### 예제 5: 거리 센서 기반 장애물 회피

```python
#!/usr/bin/env python3
"""거리 센서 기반 장애물 회피 (의사 코드)"""
import asyncio
from mobile_robot_control_node import MobileRobotController, RobotConfig

async def obstacle_avoidance():
    """
    전방 장애물 감지 시 회피
    
    주의: 실제로는 거리 센서 SDK가 필요합니다.
    """
    config = RobotConfig()
    controller = MobileRobotController(config)
    await controller.connect()
    
    print("장애물 회피 모드 시작!")
    
    for _ in range(10):  # 10번 반복
        # 가상의 거리 센서 값 (실제로는 센서 SDK 사용)
        # distance_to_obstacle = await read_distance_sensor()
        distance_to_obstacle = 0.8  # 예시: 0.8m
        
        if distance_to_obstacle < 0.5:
            # 장애물이 0.5m 이내에 있으면 회피
            print(f"\n⚠️ 장애물 감지! (거리: {distance_to_obstacle:.2f}m)")
            print("  🔄 우회전 90도...")
            await controller.rotate(-90)
            
            print("  ➡️  옆으로 0.5m 이동...")
            await controller.move_distance(0.5, speed=0.15)
            
            print("  🔄 좌회전 90도...")
            await controller.rotate(90)
        else:
            # 장애물이 없으면 전진
            print(f"\n✅ 전진 가능 (거리: {distance_to_obstacle:.2f}m)")
            await controller.move_distance(0.3, speed=0.2)
        
        await asyncio.sleep(1.0)
    
    await controller.disconnect()
    print("\n✅ 장애물 회피 완료!")

if __name__ == "__main__":
    asyncio.run(obstacle_avoidance())
```

---

## ROS 통합

### 예제 6: ROS Topic 기반 제어

```python
#!/usr/bin/env python3
"""ROS Topic을 통한 이동 명령 수신"""
import rospy
import asyncio
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist as ROSTwist
from mobile_robot_control_node import MobileRobotController, RobotConfig

class ROSMobileRobotBridge:
    """ROS와 모바일 로봇 컨트롤러를 연결하는 브릿지"""
    
    def __init__(self):
        rospy.init_node('mobile_robot_bridge', anonymous=True)
        
        # 컨트롤러 초기화
        config = RobotConfig()
        self.controller = MobileRobotController(config)
        
        # ROS Subscriber
        self.distance_sub = rospy.Subscriber(
            '/mobile_robot/move_distance',
            Float32,
            self.distance_callback
        )
        
        rospy.loginfo("ROS-모바일 로봇 브릿지 시작!")
    
    async def connect(self):
        """로봇 연결"""
        await self.controller.connect()
    
    def distance_callback(self, msg):
        """이동 거리 명령 수신"""
        distance = msg.data
        rospy.loginfo(f"이동 명령 수신: {distance:.2f}m")
        
        # 비동기 함수를 동기 방식으로 실행
        asyncio.run(self._move_async(distance))
    
    async def _move_async(self, distance):
        """비동기 이동"""
        speed = 0.2 if distance > 0 else -0.2
        result = await self.controller.move_distance(abs(distance), speed=speed)
        rospy.loginfo(f"이동 완료! 오차: {result.error:.3f}m")
    
    async def spin(self):
        """ROS 스핀"""
        rospy.loginfo("명령 대기 중... (Ctrl+C로 종료)")
        rospy.spin()
        await self.controller.disconnect()

async def main():
    bridge = ROSMobileRobotBridge()
    await bridge.connect()
    await bridge.spin()

if __name__ == "__main__":
    asyncio.run(main())
```

**테스트:**
```bash
# 터미널 1: 브릿지 실행
python3 ros_bridge.py

# 터미널 2: 이동 명령 전송
rostopic pub /mobile_robot/move_distance std_msgs/Float32 "data: 0.5"
```

---

## 고급 제어

### 예제 7: 커스텀 속도 프로파일

```python
#!/usr/bin/env python3
"""S-커브 속도 프로파일 (개념 예제)"""
import asyncio
import math
from mobile_robot_control_node import (
    VelocityProfileCalculator,
    VelocityProfileConfig,
    MotionPhase
)

class SCurveProfileCalculator(VelocityProfileCalculator):
    """S-커브 속도 프로파일 (부드러운 가감속)"""
    
    def calculate_speed(self, traveled_distance, remaining_distance):
        """
        S-커브 프로파일로 속도 계산
        
        사다리꼴 대신 S자 곡선을 사용하여 더 부드러운 가감속
        """
        if traveled_distance < self.accel_distance:
            # 가속 구간: S-커브 (0 → 1)
            progress = traveled_distance / self.accel_distance
            # Smoothstep 함수: 3x² - 2x³
            smooth_progress = 3 * progress**2 - 2 * progress**3
            target_speed = self.config.min_speed + \
                          (self.config.max_speed - self.config.min_speed) * smooth_progress
            return target_speed, MotionPhase.ACCELERATION
        
        elif remaining_distance < self.decel_distance:
            # 감속 구간: S-커브 (1 → 0)
            progress = remaining_distance / self.decel_distance
            smooth_progress = 3 * progress**2 - 2 * progress**3
            target_speed = self.config.min_speed + \
                          (self.config.max_speed - self.config.min_speed) * smooth_progress
            return target_speed, MotionPhase.DECELERATION
        
        else:
            # 등속 구간
            return self.config.max_speed, MotionPhase.CONSTANT

# 사용법:
# controller.move_distance()에서 profile_calc를 교체하여 사용
```

---

### 예제 8: 다중 로봇 제어

```python
#!/usr/bin/env python3
"""여러 로봇을 동시에 제어"""
import asyncio
from mobile_robot_control_node import MobileRobotController, RobotConfig

async def control_robot(robot_id, ip_address, distance):
    """단일 로봇 제어"""
    print(f"[로봇 {robot_id}] 연결 중...")
    config = RobotConfig(ip=ip_address, identity=f'robot_{robot_id}')
    controller = MobileRobotController(config)
    
    await controller.connect()
    print(f"[로봇 {robot_id}] 연결 완료!")
    
    # 이동
    print(f"[로봇 {robot_id}] {distance}m 이동 중...")
    result = await controller.move_distance(distance, speed=0.2)
    print(f"[로봇 {robot_id}] 이동 완료 (오차: {result.error:.3f}m)")
    
    await controller.disconnect()

async def multi_robot_control():
    """여러 로봇 동시 제어"""
    robots = [
        (1, '169.254.128.2', 1.0),  # 로봇1: 1m
        (2, '169.254.128.3', 0.8),  # 로봇2: 0.8m
        (3, '169.254.128.4', 1.2),  # 로봇3: 1.2m
    ]
    
    # 모든 로봇을 병렬로 제어
    tasks = [
        control_robot(robot_id, ip, distance)
        for robot_id, ip, distance in robots
    ]
    
    await asyncio.gather(*tasks)
    print("\n✅ 모든 로봇 제어 완료!")

if __name__ == "__main__":
    asyncio.run(multi_robot_control())
```

---

### 예제 9: 이동 중 중단

```python
#!/usr/bin/env python3
"""이동 중 조건부 중단"""
import asyncio
from mobile_robot_control_node import MobileRobotController, RobotConfig

async def interruptible_move():
    """
    특정 조건에서 이동 중단
    (예: 외부 신호, 센서 값 등)
    """
    config = RobotConfig()
    controller = MobileRobotController(config)
    await controller.connect()
    
    # 이동 태스크 생성
    move_task = asyncio.create_task(
        controller.move_distance(5.0, speed=0.2, timeout=30.0)
    )
    
    # 중단 조건 체크 (예: 5초 후 중단)
    try:
        await asyncio.wait_for(move_task, timeout=5.0)
    except asyncio.TimeoutError:
        print("\n⚠️ 이동 중단!")
        move_task.cancel()
        
        # 정지 명령
        await controller._stop_robot()
    
    await controller.disconnect()

if __name__ == "__main__":
    asyncio.run(interruptible_move())
```

---

### 예제 10: 성능 벤치마크

```python
#!/usr/bin/env python3
"""이동 정밀도 벤치마크"""
import asyncio
from mobile_robot_control_node import MobileRobotController, RobotConfig

async def benchmark():
    """여러 거리에서 정밀도 테스트"""
    config = RobotConfig()
    controller = MobileRobotController(config)
    await controller.connect()
    
    test_distances = [0.3, 0.5, 0.8, 1.0, 1.5, 2.0]
    results = []
    
    print("이동 정밀도 벤치마크 시작!\n")
    print("목표 거리 | 실제 거리 | 오차    | 오차율")
    print("-" * 50)
    
    for distance in test_distances:
        result = await controller.move_distance(distance, speed=0.2)
        
        error_percent = (result.error / result.target_distance) * 100
        results.append((distance, result.traveled_distance, result.error, error_percent))
        
        print(f"{distance:8.2f}m | {result.traveled_distance:8.3f}m | "
              f"{result.error:6.3f}m | {error_percent:5.1f}%")
        
        await asyncio.sleep(2.0)
    
    # 통계
    avg_error = sum(r[2] for r in results) / len(results)
    avg_error_percent = sum(r[3] for r in results) / len(results)
    
    print("-" * 50)
    print(f"평균 오차: {avg_error:.3f}m ({avg_error_percent:.1f}%)")
    
    await controller.disconnect()

if __name__ == "__main__":
    asyncio.run(benchmark())
```

**출력 예시:**
```
이동 정밀도 벤치마크 시작!

목표 거리 | 실제 거리 | 오차    | 오차율
--------------------------------------------------
    0.30m |    0.302m |  0.002m |   0.7%
    0.50m |    0.498m |  0.002m |   0.4%
    0.80m |    0.815m |  0.015m |   1.9%
    1.00m |    1.008m |  0.008m |   0.8%
    1.50m |    1.485m |  0.015m |   1.0%
    2.00m |    2.025m |  0.025m |   1.3%
--------------------------------------------------
평균 오차: 0.011m (1.0%)
```

---

## 실행 가능한 스크립트 모음

모든 예제를 실행할 수 있는 스크립트:

```bash
#!/bin/bash
# run_all_examples.sh

echo "====== Mobile Robot Control 예제 실행 ======"

examples=(
    "simple_move.py:간단한 전진/후진"
    "rotation_test.py:회전 테스트"
    "drive_square.py:사각형 주행"
    "drive_zigzag.py:지그재그 주행"
)

for example in "${examples[@]}"; do
    IFS=':' read -r file description <<< "$example"
    
    echo ""
    echo "===== $description ====="
    read -p "실행하시겠습니까? (y/n): " response
    
    if [[ "$response" == "y" ]]; then
        python3 "$file"
        echo "완료!"
    else
        echo "건너뜀"
    fi
done

echo ""
echo "====== 모든 예제 완료 ======"
```

---

## 다음 단계

- [API 레퍼런스](../api/mobile_robot_control_node.md): 상세한 API 문서
- [시작 가이드](../guides/getting_started.md): 설치 및 기본 사용법
- [아키텍처 문서](../architecture/): 코드 구조 및 설계

---

**작성자**: KATECH Robotics Team  
**라이센스**: MIT

