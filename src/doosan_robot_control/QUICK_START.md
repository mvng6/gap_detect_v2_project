# Doosan Robot Control - 빠른 시작 가이드 🚀

두산로봇 제어를 5분 안에 시작하세요!

## 📝 전제 조건

- ✅ ROS Noetic이 설치된 Ubuntu 20.04 (또는 Docker 컨테이너)
- ✅ 두산로봇 패키지(`doosan-robot`)가 이미 빌드됨
- ✅ 로봇이 켜져 있고 네트워크에 연결됨

## ⚡ 3단계로 시작하기

### 1️⃣ 빌드

```bash
cd ~/robot_ws

# 빌드 (Docker 환경이면 컨테이너 내에서)
catkin_make

# 환경변수 설정
source devel/setup.bash
```

### 2️⃣ IP 주소 확인

로봇 티치펜던트에서:
- **설정** → **네트워크** → **IP 주소 확인**

예: `192.168.137.100`

### 3️⃣ 실행!

#### 방법 A: 통합 실행 (권장)

한 번에 모든 것을 시작:

```bash
roslaunch doosan_robot_control doosan_robot_with_dsr.launch \
  model:=a0912 \
  host:=192.168.137.100 \
  mode:=real
```

#### 방법 B: 단계별 실행

**터미널 1** - 두산로봇 드라이버:
```bash
roslaunch dsr_launcher dsr_moveit.launch \
  model:=a0912 \
  mode:=real \
  host:=192.168.137.100
```

**터미널 2** - 커스텀 제어 노드:
```bash
roslaunch doosan_robot_control doosan_robot_control.launch \
  model:=a0912 \
  host:=192.168.137.100
```

## ✅ 성공 확인

콘솔에 다음과 같은 메시지가 보이면 성공:

```
========================================
✅ 로봇 연결 성공!
========================================
🤖 로봇 상태 정보
========================================
연결 상태: ✅ 연결됨
```

## 🧪 테스트

### 현재 상태 확인

```bash
# 연결 상태
rostopic echo /katech/doosan_connected

# 로봇 상태
rostopic echo /katech/doosan_status

# 관절 상태
rostopic echo /dsr01a0912/joint_states
```

## 🎯 다음 단계

### 1. 데모 모드 실행

```bash
roslaunch doosan_robot_control doosan_robot_control.launch \
  model:=a0912 \
  host:=192.168.137.100 \
  run_demo:=true
```

### 2. 설정 파일 커스터마이징

`config/robot_config.yaml` 파일을 편집하여 속도, 가속도 등 조정

### 3. 자신만의 코드 작성

`src/doosan_robot_controller.cpp`를 기반으로 기능 추가

## 🔧 자주 하는 실수

### ❌ "로봇 연결 실패"

**원인:** `dsr_control` 노드가 실행되지 않음

**해결:**
```bash
# 노드 확인
rosnode list | grep dsr01

# 토픽 확인
rostopic list | grep dsr01
```

### ❌ 잘못된 네임스페이스

**문제:** `/dsr01a0912/joint_states`를 찾을 수 없음

**해결:** 
- Launch 파일의 `robot_id`와 `model`이 dsr_launcher와 일치하는지 확인
- 기본값: `robot_id=dsr01`, `model=a0912` → 네임스페이스: `/dsr01a0912`

### ❌ 컴파일 에러

**문제:** `dsr_msgs`를 찾을 수 없음

**해결:**
```bash
# dsr_msgs 먼저 빌드
cd ~/robot_ws
catkin_make --pkg dsr_msgs

# 전체 빌드
catkin_make

# 환경변수 재설정
source devel/setup.bash
```

## 🐳 Docker 환경

Docker 컨테이너 사용 시:

```bash
# 컨테이너 진입
docker exec -it my_noetic_ws bash

# 워크스페이스 이동
cd /root/robot_ws  # 또는 해당 경로

# 환경변수 설정
source devel/setup.bash

# 실행
roslaunch doosan_robot_control doosan_robot_with_dsr.launch \
  model:=a0912 \
  host:=192.168.137.100
```

## 📚 더 알아보기

- 📖 [전체 README](README.md) - 상세 문서
- 🏗️ [아키텍처 설명](README.md#아키텍처)
- 💡 [예제 코드](README.md#예제)
- 🔧 [문제 해결 가이드](README.md#문제-해결)

## 💬 도움이 필요하신가요?

- 📧 이메일: robotics@katech.re.kr
- 📝 이슈: GitHub Issues

---

**즐거운 로봇 프로그래밍 되세요! 🎉**

