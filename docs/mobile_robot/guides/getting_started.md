# Mobile Robot Control Node - 시작 가이드

## 📚 목차

1. [소개](#소개)
2. [설치](#설치)
3. [빠른 시작](#빠른-시작)
4. [기본 사용법](#기본-사용법)
5. [고급 기능](#고급-기능)
6. [문제 해결](#문제-해결)

---

## 소개

`mobile_robot_control_node.py`는 Woosh 모바일 로봇(TR200)을 ROS 환경에서 제어하기 위한 Python 노드입니다.

### 주요 기능

- ✅ **Odometry 기반 정밀 제어**: 실제 이동 거리를 피드백으로 사용
- ✅ **부드러운 가감속**: 사다리꼴 속도 프로파일 자동 적용
- ✅ **맵 불필요**: 로컬라이제이션 없이 즉시 사용 가능
- ✅ **ROS 통합**: ROS 1 Noetic와 완벽 호환
- ✅ **확장 가능**: 모듈화된 구조로 커스터마이징 용이

### 시스템 요구사항

- **OS**: Ubuntu 20.04 / 22.04
- **ROS**: ROS 1 Noetic
- **Python**: 3.8+
- **네트워크**: 로봇과 동일 네트워크 (169.254.128.x)

---

## 설치

### 1. ROS 워크스페이스 준비

```bash
cd /home/katech/robot_ws
```

### 2. 의존성 설치

```bash
# Woosh Robot SDK가 이미 설치되어 있다고 가정
cd src/woosh_robot_py
pip3 install -e .
```

### 3. 파일 확인

```bash
ls src/mobile_robot_control/src/
# 출력: mobile_robot_control_node.py
```

### 4. 실행 권한 부여

```bash
chmod +x src/mobile_robot_control/src/mobile_robot_control_node.py
```

---

## 빠른 시작

### Step 1: 로봇 전원 켜기

1. TR200 로봇의 전원 버튼을 누릅니다
2. 초록색 LED가 켜질 때까지 대기 (약 30초)

### Step 2: 네트워크 연결 확인

```bash
# 로봇 IP 주소 확인 (기본: 169.254.128.2)
ping 169.254.128.2
```

**출력 예시:**
```
64 bytes from 169.254.128.2: icmp_seq=1 ttl=64 time=2.45 ms
```

### Step 3: 첫 이동 테스트

```bash
cd /home/katech/robot_ws/src/mobile_robot_control/src

# 전진 0.5m
python3 mobile_robot_control_node.py --distance 0.5 --speed 0.2
```

**예상 출력:**
```
[INFO] 🤖 Mobile Robot Controller 초기화 완료
[INFO]    연결 대상: 169.254.128.2:5480
[INFO] ✅ 로봇 연결 성공!
[INFO] 🔋 배터리 잔량: 80%
[INFO] 📍 위치 피드백 구독 시작
[INFO] 📍 초기 위치: X=0.000, Y=0.000, Theta=0.000
[INFO] 
🎯 정밀 이동 모드 (Odometry 피드백)
============================================================
[INFO] 🎯 목표: 0.500m 이동 (최대 속도: 0.20m/s)
[INFO]    가속 구간: 0.15m | 감속 구간: 0.20m
[INFO]    제어 주기: 20Hz (50ms마다 명령 전송)
============================================================
[INFO] 📍 시작 위치: X=0.000, Y=0.000
[INFO] 🚀 이동 시작!
[INFO] 📊 🚀 가속 | 0.075m / 0.500m | 속도: 0.13m/s | 남은: 0.425m
[INFO] 📊 ⚡ 등속 | 0.250m / 0.500m | 속도: 0.20m/s | 남은: 0.250m
[INFO] 📊 🛑 감속 | 0.420m / 0.500m | 속도: 0.11m/s | 남은: 0.080m
[INFO] ✅ 목표 거리 도달! (실제: 0.498m)
[INFO] 🛑 정지 중...
[INFO] 🛑 정지 완료
============================================================
[INFO] 📊 최종 결과:
[INFO]    목표 거리: 0.500m
[INFO]    실제 거리: 0.502m
[INFO]    오차: 0.002m (0.4%)
[INFO]    소요 시간: 3.2초
============================================================
[INFO] 
✅ 작업 완료! Ctrl+C로 종료하세요.
```

### Step 4: 종료

`Ctrl+C`를 눌러 프로그램을 종료합니다.

---

## 기본 사용법

### 1. 전진/후진

#### 전진 1m (속도 0.2m/s)
```bash
python3 mobile_robot_control_node.py --distance 1.0 --speed 0.2
```

#### 후진 0.5m (속도 0.15m/s)
```bash
python3 mobile_robot_control_node.py --distance 0.5 --speed -0.15
```

**💡 Tip**: 속도를 음수로 설정하면 후진합니다.

---

### 2. 회전

#### 좌회전 90도
```bash
python3 mobile_robot_control_node.py --rotate 90
```

#### 우회전 180도
```bash
python3 mobile_robot_control_node.py --rotate -180
```

**💡 Tip**: 양수는 좌회전(반시계), 음수는 우회전(시계)입니다.

---

### 3. 가감속 조정

#### 빠른 가속 + 긴 감속 (안정적)
```bash
python3 mobile_robot_control_node.py \
    --distance 1.0 \
    --speed 0.3 \
    --accel 0.1 \
    --decel 0.3
```

#### 부드러운 가속 + 짧은 감속 (스포티)
```bash
python3 mobile_robot_control_node.py \
    --distance 0.8 \
    --speed 0.25 \
    --accel 0.25 \
    --decel 0.1
```

**속도 프로파일 비교:**

```
안정적 (accel=0.1, decel=0.3):
속도 │    ╱▔▔▔▔▔▔▔▔▔▔╲
     │  ╱            ╲___
     └─────────────────────→ 거리
       빠른 가속    긴 감속

스포티 (accel=0.25, decel=0.1):
속도 │       ╱▔▔▔▔▔▔╲
     │ ____╱        ╲
     └─────────────────────→ 거리
      부드러운 가속  빠른 감속
```

---

### 4. 다른 IP의 로봇 제어

로봇이 다른 IP 주소를 사용하는 경우:

```bash
python3 mobile_robot_control_node.py \
    --ip 192.168.1.100 \
    --port 5480 \
    --distance 0.5
```

---

## 고급 기능

### 1. 상세 모드 (디버깅)

SDK의 모든 로그를 출력합니다:

```bash
python3 mobile_robot_control_node.py --distance 0.5 --verbose
```

**출력 예시:**
```
2025-10-29 09:07:18.357 [INFO] mobile_robot_controller - RobotCommunication initialized
2025-10-29 09:07:18.357 [INFO] mobile_robot_controller - Starting communication...
2025-10-29 09:07:18.386 [INFO] mobile_robot_controller - Connected to ws://169.254.128.2:5480
[INFO] ✅ 로봇 연결 성공!
```

**사용 시나리오:**
- 연결 문제 디버깅
- WebSocket 통신 상태 확인
- SDK 내부 동작 분석

---

### 2. 제어 주기 조정

더 부드러운 제어를 위해 주기를 높입니다:

```bash
# 30Hz로 제어 (기본: 20Hz)
python3 mobile_robot_control_node.py \
    --distance 1.0 \
    --speed 0.3 \
    --control-hz 30
```

**주의**: 너무 높은 주기(50Hz 이상)는 네트워크 부하를 증가시킵니다.

---

### 3. Python 코드에서 사용

#### 기본 예제

```python
import asyncio
from mobile_robot_control_node import (
    MobileRobotController,
    RobotConfig,
    VelocityProfileConfig
)

async def main():
    # 설정 생성
    config = RobotConfig(ip='169.254.128.2', verbose=False)
    
    # 컨트롤러 생성 및 연결
    controller = MobileRobotController(config)
    await controller.connect()
    
    # 1m 전진
    result = await controller.move_distance(
        target_distance=1.0,
        speed=0.2
    )
    
    print(f"이동 완료! 오차: {result.error:.3f}m")
    
    # 연결 종료
    await controller.disconnect()

if __name__ == "__main__":
    asyncio.run(main())
```

#### 고급 예제: 사각형 주행

```python
import asyncio
from mobile_robot_control_node import MobileRobotController, RobotConfig

async def drive_square(side_length=0.5):
    """사각형 경로 주행"""
    config = RobotConfig()
    controller = MobileRobotController(config)
    await controller.connect()
    
    for i in range(4):
        print(f"변 {i+1}/4 주행 중...")
        
        # 직진
        await controller.move_distance(side_length, speed=0.2)
        await asyncio.sleep(1.0)
        
        # 90도 좌회전
        await controller.rotate(90, angular_speed=0.5)
        await asyncio.sleep(1.0)
    
    await controller.disconnect()
    print("사각형 주행 완료!")

asyncio.run(drive_square(side_length=0.8))
```

---

## 문제 해결

### 1. 연결 실패: "Failed to connect"

**증상:**
```
[ERROR] ❌ 로봇 연결 실패: Connection refused
```

**해결 방법:**
1. 로봇 전원 확인
2. 네트워크 연결 확인: `ping 169.254.128.2`
3. IP 주소 확인: 로봇 설정 앱에서 실제 IP 확인
4. 방화벽 확인: `sudo ufw status`

---

### 2. 로봇이 움직이지 않음

**증상:**
- 프로그램은 정상 실행되지만 로봇이 멈춰있음

**해결 방법:**
1. **비상 정지 확인**: 로봇의 E-stop 버튼이 눌려있지 않은지 확인
2. **배터리 확인**: 배터리가 20% 이상인지 확인
3. **속도 확인**: 속도가 너무 낮으면 (< 0.05m/s) 로봇이 멈출 수 있음

---

### 3. 이동 거리 오차가 큼 (> 5%)

**증상:**
```
[INFO]    오차: 0.08m (8.0%)
```

**원인:**
- Odometry 오차 (바닥 미끄러짐)
- 센서 캘리브레이션 불량

**해결 방법:**
1. **바닥 확인**: 미끄러운 바닥에서는 오차가 증가
2. **속도 조정**: 속도를 낮춰 (0.1~0.15m/s) 미끄러짐 감소
3. **허용 오차 조정**: `--tolerance` 옵션 사용 (기본: 0.02m)

```bash
python3 mobile_robot_control_node.py \
    --distance 1.0 \
    --speed 0.15 \
    --tolerance 0.03
```

---

### 4. "타임아웃" 메시지

**증상:**
```
[WARN] ⚠️ 타임아웃 (60초 초과)
```

**원인:**
- 목표 거리에 도달하지 못함
- 로봇이 장애물에 막힘

**해결 방법:**
1. **장애물 제거**: 로봇 경로에 장애물이 없는지 확인
2. **타임아웃 증가**:
```bash
# Python 코드에서
await controller.move_distance(
    target_distance=2.0,
    speed=0.15,
    timeout=120.0  # 2분
)
```

---

### 5. "위치 조회 실패" 반복

**증상:**
```
[WARN] ⚠️ 위치 조회 실패: Request timeout
```

**원인:**
- 네트워크 지연
- 로봇 펌웨어 문제

**해결 방법:**
1. 네트워크 케이블 연결 확인 (유선 연결 권장)
2. 로봇 재부팅
3. 제어 주기 낮추기: `--control-hz 10`

---

## 다음 단계

### 학습 리소스

- [API 레퍼런스](../api/mobile_robot_control_node.md): 모든 클래스와 메서드 상세 설명
- [예제 모음](../examples/): 다양한 사용 사례
- [아키텍처 문서](../architecture/): 내부 구조 및 설계 철학

### 확장 기능 개발

리팩토링된 코드는 확장이 용이합니다:

1. **새로운 이동 패턴 추가**
   - `MobileRobotController` 클래스에 메서드 추가
   - 예: `move_arc()`, `follow_path()`

2. **커스텀 속도 프로파일**
   - `VelocityProfileCalculator` 상속
   - S-커브, 지수 함수 등 다양한 프로파일 구현

3. **ROS Action Server 통합**
   - `actionlib`을 사용하여 Goal-Feedback-Result 패턴 구현
   - 진행 상황 실시간 모니터링

---

## 지원

### 문의

- **이메일**: robotics@katech.re.kr
- **GitHub Issues**: [robot_ws/issues](https://github.com/katech/robot_ws/issues)

### 라이센스

MIT License - KATECH Robotics Team

---

**Happy Coding! 🚀**

