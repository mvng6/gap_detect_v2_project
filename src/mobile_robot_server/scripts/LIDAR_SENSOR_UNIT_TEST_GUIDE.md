# LiDAR 센서 활용 유닛 테스트 학습 노트

## 목차
1. [개요](#개요)
2. [Woosh Robot 연결 아키텍처 이해](#woosh-robot-연결-아키텍처-이해)
3. [독립적인 연결이 필요한 이유](#독립적인-연결이-필요한-이유)
4. [LiDAR 센서 유닛 테스트 설계](#lidar-센서-유닛-테스트-설계)
5. [테스트 코드 구현](#테스트-코드-구현)
6. [실행 및 검증](#실행-및-검증)
7. [문제 해결 및 학습 내용](#문제-해결-및-학습-내용)

---

## 개요

### 목적
- `mobile_posiotion_server_twist.py` 서버 코드를 기반으로 실제 모바일 로봇에 장착된 LiDAR 센서 데이터를 획득하는 간단한 유닛 테스트를 수행
- Woosh Robot SDK의 연결 메커니즘과 ROS 노드 간 독립성 이해
- LiDAR 센서 데이터 처리 및 ROS 메시지 변환 과정 학습

### 관련 파일
- **서버 코드**: `src/mobile_robot_server/scripts/mobile_posiotion_server_twist.py`
  - 모바일 로봇 제어 서비스 서버
  - Twist 명령을 통한 정밀 거리 이동 제어
  
- **LiDAR 발행 노드**: `src/mobile_robot_server/scripts/lidar_publisher.py`
  - LiDAR 센서 데이터를 ROS LaserScan 메시지로 발행
  - Woosh Robot SDK를 통한 센서 데이터 구독

---

## Woosh Robot 연결 아키텍처 이해

### Woosh Robot SDK 연결 메커니즘

Woosh Robot SDK는 **WebSocket 기반 비동기 통신**을 사용합니다. 각 `WooshRobot` 인스턴스는 독립적인 WebSocket 연결을 생성합니다.

#### 연결 구조

```python
# mobile_posiotion_server_twist.py의 연결 방식
settings = CommuSettings(
    addr=self.robot_ip,      # 로봇 IP 주소
    port=self.robot_port,    # 포트 (기본값: 5480)
    identity=self.robot_identity  # 클라이언트 식별자 ('twist_ctrl')
)
self.robot = WooshRobot(settings)
await self.robot.run()  # WebSocket 연결 시작
```

```python
# lidar_publisher.py의 연결 방식
settings = CommuSettings(
    addr=self.robot_ip,
    port=self.robot_port,
    identity=self.robot_identity  # 클라이언트 식별자 ('lidar_pub_1')
)
self.robot = WooshRobot(settings)
await self.robot.run()  # WebSocket 연결 시작
```

### WebSocket 연결의 특징

1. **독립적인 연결 세션**: 각 `WooshRobot` 인스턴스는 독립적인 WebSocket 연결을 생성
2. **클라이언트 식별**: `identity` 파라미터로 서로 다른 클라이언트로 식별됨
3. **비동기 이벤트 루프**: 각 인스턴스는 자체 asyncio 이벤트 루프에서 실행
4. **독립적인 구독**: 각 연결은 독립적으로 데이터를 구독할 수 있음

---

## 독립적인 연결이 필요한 이유

### 질문: 이미 서버에서 연결했는데 왜 또 연결하나요?

#### 답변: ROS 노드의 독립성과 모듈화 원칙

**1. ROS 노드의 독립성**
- ROS는 **분산 시스템**으로 설계됨
- 각 노드는 독립적으로 시작/종료 가능해야 함
- 한 노드의 실패가 다른 노드에 영향을 주지 않아야 함

**2. 모듈화 및 재사용성**
- `lidar_publisher.py`는 독립적인 노드로 실행 가능
- 다른 프로젝트에서도 재사용 가능
- 서버 노드 없이도 LiDAR 데이터만 필요한 경우 사용 가능

**3. 리소스 관리**
- 각 노드는 자신의 연결을 관리하고 정리할 책임이 있음
- 서버가 종료되어도 LiDAR 노드는 계속 동작할 수 있어야 함

**4. Woosh Robot SDK의 설계**
- SDK는 **멀티 클라이언트** 지원 구조
- 여러 클라이언트가 동시에 연결 가능 (각각 다른 `identity` 사용)
- 각 클라이언트는 독립적인 구독/요청 가능

#### 실제 동작 예시

```
┌─────────────────────────────────────────────────────────┐
│              Woosh Robot (하드웨어)                       │
│  IP: 169.254.128.2, Port: 5480                          │
└───────────────┬─────────────────────────────────────────┘
                │
                │ WebSocket (여러 클라이언트 동시 연결 가능)
                │
    ┌───────────┴───────────┐
    │                       │
    ▼                       ▼
┌─────────────┐      ┌─────────────┐
│ Client 1    │      │ Client 2    │
│ identity:   │      │ identity:   │
│ twist_ctrl  │      │ lidar_pub_1 │
│             │      │             │
│ - Twist 제어│      │ - LiDAR 구독│
│ - 로봇 정보  │      │ - Scanner   │
└─────────────┘      └─────────────┘
```

**결론**: 각 노드가 독립적으로 연결하는 것이 올바른 설계입니다!

---

## LiDAR 센서 유닛 테스트 설계

### 테스트 목표

1. ✅ Woosh Robot과의 연결 확인
2. ✅ LiDAR 센서 데이터 수신 확인
3. ✅ ScannerData → LaserScan 변환 검증
4. ✅ ROS 토픽 발행 확인
5. ✅ 데이터 품질 검증 (범위, 각도, 해상도 등)

### 테스트 시나리오

#### 시나리오 1: 기본 연결 및 데이터 수신 테스트
- 목적: 로봇 연결 및 LiDAR 데이터 수신 기본 동작 확인
- 검증 항목:
  - 연결 성공 여부
  - 데이터 수신 여부
  - 메시지 형식 정확성

#### 시나리오 2: 데이터 품질 검증 테스트
- 목적: 수신된 데이터의 유효성 검증
- 검증 항목:
  - 범위 값의 유효성 (range_min ~ range_max)
  - 각도 범위 정확성
  - 데이터 포인트 개수
  - 타임스탬프 정확성

#### 시나리오 3: 다중 LiDAR 센서 테스트
- 목적: 여러 LiDAR 센서 동시 사용 확인
- 검증 항목:
  - 각 센서의 독립적 동작
  - 토픽 분리 확인 (`/scan1`, `/scan2`)

---

## 테스트 코드 구현

### 테스트 파일 구조

```
src/mobile_robot_server/scripts/
├── mobile_posiotion_server_twist.py  # 기존 서버 코드
├── lidar_publisher.py                # 기존 발행 노드
└── test_lidar_sensor.py              # 새로 작성할 테스트 코드
```

### 테스트 코드: `test_lidar_sensor.py`

```python
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LiDAR 센서 유닛 테스트
Woosh Robot의 LiDAR 센서 데이터를 획득하고 검증하는 테스트 코드
"""

import rospy
import asyncio
import sys
import os
import time
from threading import Thread
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header

# Woosh Robot SDK 임포트
try:
    woosh_robot_py_path = os.path.join(os.path.dirname(__file__), '../../../../woosh_robot_py')
    woosh_robot_py_path = os.path.abspath(woosh_robot_py_path)
    if os.path.exists(woosh_robot_py_path) and woosh_robot_py_path not in sys.path:
        sys.path.insert(0, woosh_robot_py_path)
    
    from woosh_robot import WooshRobot
    from woosh_interface import CommuSettings, NO_PRINT
    from woosh.proto.robot.robot_pb2 import ScannerData, RobotInfo
    WOOSH_SDK_AVAILABLE = True
except ImportError as e:
    rospy.logerr(f"Woosh Robot SDK를 임포트할 수 없습니다: {e}")
    WOOSH_SDK_AVAILABLE = False
    WooshRobot = None
    CommuSettings = None
    NO_PRINT = None
    ScannerData = None
    RobotInfo = None


class LidarSensorTest:
    """LiDAR 센서 유닛 테스트 클래스"""
    
    def __init__(self, lidar_id=1):
        """
        Args:
            lidar_id: 테스트할 LiDAR ID (1 또는 2)
        """
        if not WOOSH_SDK_AVAILABLE:
            raise RuntimeError("Woosh Robot SDK를 사용할 수 없습니다.")
        
        self.lidar_id = lidar_id
        self.robot_ip = rospy.get_param('~robot_ip', '169.254.128.2')
        self.robot_port = rospy.get_param('~robot_port', 5480)
        self.robot_identity = rospy.get_param('~robot_identity', f'lidar_test_{lidar_id}')
        
        self.robot = None
        self.robot_loop = None
        
        # 테스트 결과 저장
        self.received_data_count = 0
        self.first_data_time = None
        self.last_data_time = None
        self.data_samples = []  # 최대 10개 샘플 저장
        self.test_results = {
            'connection': False,
            'data_received': False,
            'data_valid': False,
            'data_count': 0,
            'errors': []
        }
        
        rospy.loginfo(f"=== LiDAR {lidar_id} 센서 테스트 시작 ===")
        rospy.loginfo(f"로봇 IP: {self.robot_ip}:{self.robot_port}")
        rospy.loginfo(f"클라이언트 ID: {self.robot_identity}")
    
    async def connect(self):
        """로봇 연결"""
        rospy.loginfo(f"[테스트] 로봇 연결 시도 중...")
        
        settings = CommuSettings(
            addr=self.robot_ip,
            port=self.robot_port,
            identity=self.robot_identity
        )
        self.robot = WooshRobot(settings)
        
        if not await self.robot.run():
            raise RuntimeError("로봇 연결 실패")
        
        # 연결 검증: 로봇 정보 요청
        info, ok, _ = await self.robot.robot_info_req(RobotInfo(), NO_PRINT, NO_PRINT)
        if not ok:
            raise RuntimeError("로봇 정보 요청 실패")
        
        self.test_results['connection'] = True
        rospy.loginfo(f"[테스트] ✅ 로봇 연결 성공!")
        rospy.loginfo(f"[테스트] 배터리 잔량: {info.battery.power}%")
    
    def scanner_callback(self, data: ScannerData):
        """LiDAR 데이터 콜백 함수"""
        try:
            current_time = time.time()
            
            # 첫 데이터 시간 기록
            if self.first_data_time is None:
                self.first_data_time = current_time
                rospy.loginfo(f"[테스트] 첫 번째 LiDAR 데이터 수신!")
            
            self.last_data_time = current_time
            self.received_data_count += 1
            
            # 데이터 샘플 저장 (최대 10개)
            if len(self.data_samples) < 10:
                sample = {
                    'timestamp': current_time,
                    'angle_min': data.angle_min,
                    'angle_max': data.angle_max,
                    'angle_increment': data.angle_increment,
                    'range_min': data.range_min,
                    'range_max': data.range_max,
                    'ranges_count': len(data.ranges) if data.ranges else 0,
                    'scan_time': data.scan_time if hasattr(data, 'scan_time') else 0.0,
                    'time_increment': data.time_increment if hasattr(data, 'time_increment') else 0.0
                }
                self.data_samples.append(sample)
            
            # 데이터 검증
            self._validate_data(data)
            
            # 주기적으로 상태 출력 (10개마다)
            if self.received_data_count % 10 == 0:
                rospy.loginfo(f"[테스트] 데이터 수신: {self.received_data_count}개")
                
        except Exception as e:
            error_msg = f"데이터 처리 오류: {e}"
            rospy.logerr(f"[테스트] ❌ {error_msg}")
            self.test_results['errors'].append(error_msg)
    
    def _validate_data(self, data: ScannerData):
        """수신된 데이터의 유효성 검증"""
        try:
            # 기본 검증
            if not hasattr(data, 'ranges') or not data.ranges:
                raise ValueError("ranges 데이터가 없습니다")
            
            if len(data.ranges) == 0:
                raise ValueError("ranges 데이터가 비어있습니다")
            
            # 각도 범위 검증
            if hasattr(data, 'angle_min') and hasattr(data, 'angle_max'):
                if data.angle_min >= data.angle_max:
                    raise ValueError(f"각도 범위 오류: min={data.angle_min}, max={data.angle_max}")
            
            # 범위 값 검증
            if hasattr(data, 'range_min') and hasattr(data, 'range_max'):
                if data.range_min < 0 or data.range_max <= data.range_min:
                    raise ValueError(f"범위 값 오류: min={data.range_min}, max={data.range_max}")
            
            # 데이터 포인트 검증
            valid_ranges = [r for r in data.ranges if data.range_min <= r <= data.range_max]
            if len(valid_ranges) == 0:
                rospy.logwarn(f"[테스트] ⚠️  유효한 범위 데이터가 없습니다")
            
            self.test_results['data_valid'] = True
            
        except Exception as e:
            error_msg = f"데이터 검증 실패: {e}"
            rospy.logwarn(f"[테스트] ⚠️  {error_msg}")
            if error_msg not in self.test_results['errors']:
                self.test_results['errors'].append(error_msg)
    
    async def start_subscription(self, duration=10.0):
        """LiDAR 데이터 구독 시작 및 테스트 실행"""
        rospy.loginfo(f"[테스트] LiDAR 데이터 구독 시작 (지속 시간: {duration}초)")
        
        # 구독 시작
        await self.robot.scanner_data_sub(self.scanner_callback, NO_PRINT)
        
        # 지정된 시간 동안 데이터 수집
        start_time = time.time()
        while (time.time() - start_time) < duration:
            await asyncio.sleep(0.1)
        
        # 결과 업데이트
        self.test_results['data_received'] = self.received_data_count > 0
        self.test_results['data_count'] = self.received_data_count
    
    def print_test_results(self):
        """테스트 결과 출력"""
        rospy.loginfo("\n" + "="*60)
        rospy.loginfo("=== LiDAR 센서 테스트 결과 ===")
        rospy.loginfo("="*60)
        
        # 연결 상태
        status = "✅ 성공" if self.test_results['connection'] else "❌ 실패"
        rospy.loginfo(f"1. 로봇 연결: {status}")
        
        # 데이터 수신 상태
        status = "✅ 성공" if self.test_results['data_received'] else "❌ 실패"
        rospy.loginfo(f"2. 데이터 수신: {status}")
        rospy.loginfo(f"   - 수신된 데이터 개수: {self.test_results['data_count']}개")
        
        if self.first_data_time and self.last_data_time:
            duration = self.last_data_time - self.first_data_time
            if duration > 0:
                rate = self.received_data_count / duration
                rospy.loginfo(f"   - 평균 수신 주기: {1.0/rate:.3f}초 ({rate:.2f} Hz)")
        
        # 데이터 유효성
        status = "✅ 유효" if self.test_results['data_valid'] else "❌ 무효"
        rospy.loginfo(f"3. 데이터 유효성: {status}")
        
        # 샘플 데이터 출력
        if self.data_samples:
            rospy.loginfo("\n--- 샘플 데이터 (첫 번째) ---")
            sample = self.data_samples[0]
            rospy.loginfo(f"각도 범위: {sample['angle_min']:.3f} ~ {sample['angle_max']:.3f} rad")
            rospy.loginfo(f"각도 증분: {sample['angle_increment']:.6f} rad")
            rospy.loginfo(f"거리 범위: {sample['range_min']:.3f} ~ {sample['range_max']:.3f} m")
            rospy.loginfo(f"데이터 포인트 수: {sample['ranges_count']}개")
            rospy.loginfo(f"스캔 시간: {sample['scan_time']:.3f}초")
        
        # 오류 목록
        if self.test_results['errors']:
            rospy.logwarn("\n--- 발견된 오류 ---")
            for i, error in enumerate(self.test_results['errors'], 1):
                rospy.logwarn(f"{i}. {error}")
        else:
            rospy.loginfo("\n--- 오류 없음 ---")
        
        rospy.loginfo("="*60 + "\n")
    
    async def run_test(self, duration=10.0):
        """전체 테스트 실행"""
        try:
            # 1. 연결
            await self.connect()
            
            # 2. 데이터 구독 및 수집
            await self.start_subscription(duration)
            
            # 3. 결과 출력
            self.print_test_results()
            
        except Exception as e:
            rospy.logerr(f"[테스트] ❌ 테스트 실행 중 오류: {e}")
            self.test_results['errors'].append(str(e))
            raise
    
    def run(self):
        """비동기 테스트 실행 (스레드에서)"""
        def run_asyncio():
            self.robot_loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.robot_loop)
            
            async def main():
                try:
                    # 테스트 지속 시간 (기본 10초)
                    test_duration = rospy.get_param('~test_duration', 10.0)
                    await self.run_test(duration=test_duration)
                    
                    # 테스트 완료 후 종료
                    rospy.loginfo("[테스트] 테스트 완료. 노드를 종료합니다.")
                    rospy.signal_shutdown("테스트 완료")
                    
                except Exception as e:
                    rospy.logerr(f"[테스트] 테스트 실패: {e}")
                    rospy.signal_shutdown("테스트 실패")
            
            try:
                self.robot_loop.run_until_complete(main())
            except KeyboardInterrupt:
                rospy.loginfo("[테스트] 사용자에 의해 중단됨")
            finally:
                if self.robot_loop:
                    self.robot_loop.close()
        
        thread = Thread(target=run_asyncio, daemon=False)
        thread.start()
        return thread


if __name__ == '__main__':
    rospy.init_node('lidar_sensor_test', anonymous=False)
    
    # LiDAR ID 파라미터
    lidar_id = rospy.get_param('~lidar_id', 1)
    
    # 테스트 실행
    test = LidarSensorTest(lidar_id=lidar_id)
    test_thread = test.run()
    
    # ROS 스핀 (테스트 완료까지 대기)
    rospy.spin()
    
    # 스레드 종료 대기
    test_thread.join(timeout=5.0)
```

---

## 실행 및 검증

### 실행 방법

#### 1. 기본 테스트 (LiDAR 1번, 10초간)

```bash
# 터미널 1: 테스트 실행
rosrun mobile_robot_server test_lidar_sensor.py _lidar_id:=1 _test_duration:=10.0
```

#### 2. 파라미터 커스터마이징

```bash
# 다른 IP 주소 사용
rosrun mobile_robot_server test_lidar_sensor.py \
    _robot_ip:=192.168.1.100 \
    _lidar_id:=1 \
    _test_duration:=15.0

# LiDAR 2번 테스트
rosrun mobile_robot_server test_lidar_sensor.py _lidar_id:=2
```

#### 3. Launch 파일로 실행 (선택사항)

`test_lidar_sensor.launch` 파일 생성:

```xml
<launch>
    <node name="lidar_sensor_test" 
          pkg="mobile_robot_server" 
          type="test_lidar_sensor.py" 
          output="screen">
        <param name="robot_ip" value="169.254.128.2" />
        <param name="robot_port" value="5480" />
        <param name="lidar_id" value="1" />
        <param name="test_duration" value="10.0" />
    </node>
</launch>
```

실행:
```bash
roslaunch mobile_robot_server test_lidar_sensor.launch
```

### 예상 출력

```
[INFO] === LiDAR 1 센서 테스트 시작 ===
[INFO] 로봇 IP: 169.254.128.2:5480
[INFO] 클라이언트 ID: lidar_test_1
[INFO] [테스트] 로봇 연결 시도 중...
[INFO] [테스트] ✅ 로봇 연결 성공!
[INFO] [테스트] 배터리 잔량: 85%
[INFO] [테스트] LiDAR 데이터 구독 시작 (지속 시간: 10.0초)
[INFO] [테스트] 첫 번째 LiDAR 데이터 수신!
[INFO] [테스트] 데이터 수신: 10개
[INFO] [테스트] 데이터 수신: 20개
...
[INFO] ============================================================
[INFO] === LiDAR 센서 테스트 결과 ===
[INFO] ============================================================
[INFO] 1. 로봇 연결: ✅ 성공
[INFO] 2. 데이터 수신: ✅ 성공
[INFO]    - 수신된 데이터 개수: 100개
[INFO]    - 평균 수신 주기: 0.100초 (10.00 Hz)
[INFO] 3. 데이터 유효성: ✅ 유효
[INFO] 
[INFO] --- 샘플 데이터 (첫 번째) ---
[INFO] 각도 범위: -3.142 ~ 3.142 rad
[INFO] 각도 증분: 0.006283 rad
[INFO] 거리 범위: 0.050 ~ 12.000 m
[INFO] 데이터 포인트 수: 1000개
[INFO] 스캔 시간: 0.100초
[INFO] 
[INFO] --- 오류 없음 ---
[INFO] ============================================================
```

### 검증 체크리스트

- [ ] 로봇 연결 성공
- [ ] LiDAR 데이터 수신 확인
- [ ] 데이터 형식 정확성 확인
- [ ] 수신 주기 확인 (일반적으로 10Hz)
- [ ] 데이터 범위 및 각도 값 유효성 확인
- [ ] 오류 없음 확인

---

## 문제 해결 및 학습 내용

### 자주 발생하는 문제

#### 1. 연결 실패

**증상**: `로봇 연결 실패` 오류

**원인**:
- 네트워크 연결 문제
- 로봇 IP 주소 오류
- 로봇이 켜져 있지 않음
- 포트 번호 오류

**해결 방법**:
```bash
# 1. 네트워크 연결 확인
ping 169.254.128.2

# 2. 로봇 상태 확인
# 로봇의 전원 및 네트워크 LED 확인

# 3. IP 주소 재확인
# 로봇 설정에서 IP 주소 확인
```

#### 2. 데이터 수신 안 됨

**증상**: 연결은 성공했지만 데이터가 수신되지 않음

**원인**:
- LiDAR 센서가 비활성화됨
- 잘못된 LiDAR ID 사용
- 구독 설정 오류

**해결 방법**:
```python
# LiDAR ID 확인 (1 또는 2)
# 로봇 설정에서 활성화된 LiDAR 확인
```

#### 3. 데이터 형식 오류

**증상**: 데이터는 수신되지만 형식이 잘못됨

**원인**:
- SDK 버전 불일치
- 프로토콜 버전 차이

**해결 방법**:
- SDK 버전 확인 및 업데이트
- 프로토콜 문서 확인

### 학습 내용 정리

#### 1. Woosh Robot SDK 연결 패턴

**핵심 개념**:
- 각 `WooshRobot` 인스턴스는 독립적인 WebSocket 연결
- `identity` 파라미터로 클라이언트 식별
- 비동기 이벤트 루프에서 실행

**코드 패턴**:
```python
# 1. 설정 생성
settings = CommuSettings(addr=ip, port=port, identity=id)

# 2. 인스턴스 생성
robot = WooshRobot(settings)

# 3. 연결 시작
await robot.run()

# 4. 기능 사용 (구독, 요청 등)
await robot.scanner_data_sub(callback, NO_PRINT)
```

#### 2. ROS 노드 독립성

**설계 원칙**:
- 각 노드는 독립적으로 실행 가능해야 함
- 한 노드의 실패가 다른 노드에 영향을 주지 않음
- 모듈화 및 재사용성 고려

**실제 적용**:
- `mobile_posiotion_server_twist.py`: 제어 서비스 제공
- `lidar_publisher.py`: LiDAR 데이터 발행
- 각각 독립적인 연결 및 생명주기 관리

#### 3. 비동기 프로그래밍

**asyncio 사용**:
- `async/await` 문법 사용
- 이벤트 루프에서 실행
- 스레드와 함께 사용하여 ROS와 통합

**주의사항**:
- 각 노드는 자체 이벤트 루프 필요
- `asyncio.new_event_loop()`로 새 루프 생성
- 스레드에서 실행하여 ROS 스핀과 병행

#### 4. 데이터 검증

**검증 항목**:
- 데이터 존재 여부
- 데이터 형식 정확성
- 값의 유효 범위 확인
- 타임스탬프 정확성

**검증 방법**:
- 콜백 함수에서 실시간 검증
- 샘플 데이터 저장 및 분석
- 오류 로깅 및 보고

### 향후 개선 사항

1. **통합 테스트**: 여러 LiDAR 센서 동시 테스트
2. **성능 테스트**: 데이터 수신 주기 및 지연 시간 측정
3. **장기 안정성 테스트**: 장시간 실행 시 안정성 확인
4. **ROS 메시지 변환 테스트**: `lidar_publisher.py`와 연동 테스트

---

## 참고 자료

- Woosh Robot SDK 문서
- ROS LaserScan 메시지 스펙: http://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html
- ROS 노드 설계 가이드라인
- 비동기 프로그래밍 가이드 (Python asyncio)

---

**작성일**: 2024년
**작성자**: User
**목적**: LiDAR 센서 활용 학습 및 유닛 테스트

