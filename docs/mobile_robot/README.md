# Mobile Robot Control Documentation

모바일 로봇(TR200) 제어를 위한 종합 문서입니다.

## 📚 문서 구조

```
docs/mobile_robot/
├── README.md                    # 이 파일
├── api/                         # API 레퍼런스
│   └── mobile_robot_control_node.md
├── guides/                      # 사용 가이드
│   └── getting_started.md
├── examples/                    # 예제 코드
│   └── basic_examples.md
└── architecture/                # 아키텍처 문서
    └── design_overview.md
```

---

## 🚀 빠른 시작

### 1. 설치 확인

```bash
cd /home/katech/robot_ws/src/mobile_robot_control/src
ls mobile_robot_control_node.py
```

### 2. 첫 이동 테스트

```bash
python3 mobile_robot_control_node.py --distance 0.5 --speed 0.2
```

### 3. 도움말 보기

```bash
python3 mobile_robot_control_node.py --help
```

---

## 📖 문서 가이드

### 초보자

1. **[시작 가이드](guides/getting_started.md)** ← **여기서 시작!**
   - 설치 방법
   - 기본 사용법
   - 문제 해결

2. **[예제 모음](examples/basic_examples.md)**
   - 10가지 실용 예제
   - 복사해서 바로 사용 가능

### 개발자

3. **[API 레퍼런스](api/mobile_robot_control_node.md)**
   - 모든 클래스와 메서드 설명
   - 파라미터 및 반환값
   - 사용 예시

4. **[아키텍처 문서](architecture/design_overview.md)**
   - 코드 구조
   - 설계 철학
   - 확장 방법

---

## 🎯 주요 기능

### ✅ Odometry 기반 정밀 제어
```python
result = await controller.move_distance(1.0, speed=0.2)
print(f"오차: {result.error:.3f}m")  # 일반적으로 ±2cm
```

### ✅ 부드러운 가감속 (사다리꼴 프로파일)
```
속도 (m/s)
0.20 │     ╱▔▔▔▔▔╲
     │    ╱       ╲
     │___╱         ╲___
     └────────────────→ 거리 (m)
        가속 등속 감속
```

### ✅ 모듈화된 구조
```python
# 커스텀 속도 프로파일 쉽게 추가
velocity_config = VelocityProfileConfig(
    max_speed=0.3,
    accel_distance=0.1,
    decel_distance=0.25
)
```

### ✅ ROS 통합
```bash
# ROS Topic으로 제어
rostopic pub /mobile_robot/move_distance std_msgs/Float32 "data: 0.5"
```

---

## 📊 성능 특성

| 항목 | 값 |
|------|-----|
| **위치 정밀도** | ±2cm (일반적) |
| **제어 주기** | 20Hz (50ms) |
| **최대 속도** | 0.5m/s (설정 가능) |
| **최소 속도** | 0.03m/s |
| **응답 시간** | 10~25ms |

---

## 🛠️ 시스템 요구사항

- **OS**: Ubuntu 20.04
- **ROS**: ROS 1 Noetic
- **Python**: 3.8+
- **로봇**: Woosh TR200
- **네트워크**: 169.254.128.x (로봇과 동일)

---

## 📝 사용 예시

### CLI (명령줄)

```bash
# 전진 1m
python3 mobile_robot_control_node.py --distance 1.0 --speed 0.2

# 후진 0.5m
python3 mobile_robot_control_node.py --distance 0.5 --speed -0.15

# 90도 좌회전
python3 mobile_robot_control_node.py --rotate 90

# 커스텀 가감속
python3 mobile_robot_control_node.py \
    --distance 1.0 \
    --speed 0.3 \
    --accel 0.1 \
    --decel 0.3
```

### Python 코드

```python
import asyncio
from mobile_robot_control_node import MobileRobotController, RobotConfig

async def main():
    config = RobotConfig(ip='169.254.128.2')
    controller = MobileRobotController(config)
    
    await controller.connect()
    
    # 1m 전진
    result = await controller.move_distance(1.0, speed=0.2)
    print(f"완료! 오차: {result.error:.3f}m")
    
    await controller.disconnect()

asyncio.run(main())
```

---

## 🔗 관련 리소스

### 내부 문서
- [프로젝트 루트 README](../../../README.md)
- [프로젝트 개요](../../../Project.md)
- [Doosan 로봇 문서](../../../src/doosan-robot/)

### 외부 리소스
- [ROS Noetic 문서](http://wiki.ros.org/noetic)
- [Woosh Robot SDK](../../../src/woosh_robot_py/)
- [Python asyncio](https://docs.python.org/3/library/asyncio.html)

---

## 🐛 문제 해결

### 자주 발생하는 문제

1. **연결 실패**
   ```bash
   ping 169.254.128.2  # 네트워크 확인
   ```

2. **로봇이 움직이지 않음**
   - E-stop 버튼 확인
   - 배터리 잔량 확인 (> 20%)

3. **오차가 큼 (> 5%)**
   - 속도를 낮춤 (0.1~0.15m/s)
   - 바닥 상태 확인 (미끄럽지 않은지)

자세한 내용: [문제 해결 가이드](guides/getting_started.md#문제-해결)

---

## 📞 지원

### 문의
- **이메일**: djlee2@katech.re.kr
- **GitHub**: [robot_ws/issues](https://github.com/katech/robot_ws/issues)

### 기여
Pull Request를 환영합니다!

1. Fork the repository
2. Create your feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

---

## 📄 라이센스

MIT License - KATECH Robotics Team

---

## 📅 변경 이력

### v1.0.0 (2025-10-29)
- ✨ 초기 릴리스
- ✅ Odometry 기반 정밀 제어
- ✅ 사다리꼴 속도 프로파일
- ✅ 모듈화된 구조
- ✅ 종합 문서

---

**Happy Coding! 🚀**

