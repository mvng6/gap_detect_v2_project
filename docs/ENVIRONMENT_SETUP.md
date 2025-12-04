# 환경 구축 가이드

**Docker 기반 ROS Noetic 개발 환경 구축**

---

## 📋 문서 정보

**작성자**: LDJ (Dongjun Lee)  
**이메일**: djlee2@katech.re.kr  
**소속**: KATECH 스마트제조기술연구센터  
**최종 수정일**: 2025-11-03  
**버전**: 1.0.0

---

## 📝 목차

1. [시스템 요구사항](#1-시스템-요구사항)
2. [Host OS 설정](#2-host-os-설정)
3. [Docker 설치 및 설정](#3-docker-설치-및-설정)
4. [ROS Noetic 컨테이너 생성](#4-ros-noetic-컨테이너-생성)
5. [워크스페이스 설정](#5-워크스페이스-설정)
6. [네트워크 설정](#6-네트워크-설정)
7. [문제 해결](#7-문제-해결)

---

## 1. 시스템 요구사항

### 1.1 하드웨어

- **PC**: Intel NUC 15 Pro 또는 동급 이상
- **RAM**: 최소 8GB (권장 16GB)
- **저장공간**: 최소 50GB
- **네트워크**: 
  - LAN 포트 (두산 로봇 연결용)
  - Wi-Fi (모바일 로봇 연결용)

### 1.2 소프트웨어

- **Host OS**: Ubuntu 24.04 LTS
- **Docker**: 최신 버전 (Docker CE)
- **ROS**: Noetic Ninjemys (Docker 컨테이너 내부)

---

## 2. Host OS 설정

### 2.1 Ubuntu 24.04 설치

```bash
# 시스템 업데이트
sudo apt update && sudo apt upgrade -y

# 필수 도구 설치
sudo apt install -y \
    git \
    vim \
    curl \
    wget \
    net-tools \
    build-essential
```

### 2.2 사용자 설정

```bash
# 현재 사용자를 docker 그룹에 추가 (나중에 Docker 설치 후 필요)
# 주의: Docker 설치 후 실행해야 함
sudo usermod -aG docker $USER
```

---

## 3. Docker 설치 및 설정

### 3.1 Docker Engine 설치

```bash
# Docker 공식 GPG 키 추가
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Docker 저장소 추가
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Docker 설치
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

### 3.2 Docker 권한 설정

```bash
# 현재 사용자를 docker 그룹에 추가
sudo usermod -aG docker $USER

# 변경 사항 적용 (재로그인 또는 다음 명령 실행)
newgrp docker

# 설치 확인
docker --version
docker run hello-world
```

---

## 4. ROS Noetic 컨테이너 생성

### 4.1 워크스페이스 디렉터리 생성

```bash
# Host OS에 영구 워크스페이스 생성
mkdir -p ~/robot_ws/src
cd ~/robot_ws
```

### 4.2 Docker 컨테이너 생성 및 실행

```bash
docker run -itd \
    --name my_noetic_ws \
    --network="host" \
    --privileged \
    --volume="$HOME/robot_ws:/root/catkin_ws" \
    --volume="/var/run/docker.sock:/var/run/docker.sock" \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    ros:noetic-robot \
    /bin/bash
```

**주요 옵션 설명**:
- `--name my_noetic_ws`: 컨테이너 이름 지정
- `--network="host"`: Host OS의 네트워크를 공유 (로봇 연결용)
- `--volume="$HOME/robot_ws:/root/catkin_ws"`: 소스 코드 마운트 (데이터 영속성)
- `--volume="/var/run/docker.sock:/var/run/docker.sock"`: Docker-in-Docker 지원

### 4.3 컨테이너 접속

```bash
# 컨테이너 내부로 접속
docker exec -it my_noetic_ws bash

# 이제부터 모든 명령은 컨테이너 내부에서 실행됩니다.
```

---

## 5. 워크스페이스 설정

### 5.1 컨테이너 내부 초기 설정

```bash
# (컨테이너 내부)

# 시스템 업데이트
apt-get update && apt-get upgrade -y

# ROS 의존성 설치
apt-get install -y \
    ros-noetic-moveit \
    ros-noetic-rqt \
    ros-noetic-rqt-common-plugins \
    ros-noetic-industrial-core \
    ros-noetic-controller-manager \
    ros-noetic-joint-state-controller \
    python3-pip \
    vim

# Docker CLI 설치 (Docker-in-Docker용)
apt-get install -y docker-ce-cli

# rosdep 초기화
rosdep update
```

### 5.2 두산 로봇 패키지 다운로드

```bash
# (컨테이너 내부)
cd /root/catkin_ws/src

# 두산 로봇 패키지 클론
git clone https://github.com/doosan-robotics/doosan-robot.git
git clone https://github.com/doosan-robotics/serial.git

# 의존성 설치
cd /root/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 5.3 에뮬레이터 설치 (선택사항)

```bash
# (컨테이너 내부)
cd /root/catkin_ws/src/doosan-robot/common/bin
./install_emulator.sh
```

### 5.4 빌드 및 환경 설정

```bash
# (컨테이너 내부)
cd /root/catkin_ws
catkin_make

# 환경 변수 설정을 .bashrc에 추가
echo "source /root/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 5.5 파일 권한 설정 (Host OS)

컨테이너 내부에서 생성된 파일은 root 소유권을 가지므로, Host OS에서 수정하려면 권한 변경이 필요합니다.

```bash
# (Host OS 터미널에서 실행)
sudo chown -R $USER:$USER ~/robot_ws
```

---

## 6. 네트워크 설정

### 6.1 두산 로봇 연결 (LAN)

```bash
# Host OS 네트워크 설정
# Settings → Network → Wired (LAN) → IPv4 Settings

# 설정 값:
- Method: Manual
- Address: 192.168.137.101
- Netmask: 255.255.255.0
- Gateway: (비워둠)
```

### 6.2 연결 확인

```bash
# (컨테이너 내부)
# 두산 로봇 Ping 테스트
ping 192.168.137.100

# 모바일 로봇 Ping 테스트
ping 169.254.128.2
```

---

## 7. 문제 해결

### 7.1 Docker 권한 오류

**문제**:
```
Got permission denied while trying to connect to the Docker daemon socket
```

**해결**:
```bash
# (Host OS)
sudo usermod -aG docker $USER
newgrp docker
```

### 7.2 컨테이너에서 Docker 명령 실행 불가

**문제**:
```bash
bash: docker: command not found
```

**해결**:
```bash
# (컨테이너 내부)
apt-get update
apt-get install -y docker-ce-cli
```

### 7.3 ROS 환경 변수 인식 안 됨

**문제**:
```bash
rosrun: command not found
```

**해결**:
```bash
# (컨테이너 내부)
source /root/catkin_ws/devel/setup.bash
```

### 7.4 Host에서 파일 수정 불가 (권한 오류)

**문제**:
```
Permission denied
```

**해결**:
```bash
# (Host OS)
sudo chown -R $USER:$USER ~/robot_ws
```

### 7.5 빌드 오류 (패키지 누락)

**문제**:
```
Could not find a package configuration file provided by "xxx"
```

**해결**:
```bash
# (컨테이너 내부)
cd /root/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
catkin_make clean
catkin_make
```

---

## 8. 컨테이너 관리

### 8.1 기본 명령어

```bash
# 컨테이너 시작
docker start my_noetic_ws

# 컨테이너 정지
docker stop my_noetic_ws

# 컨테이너 재시작
docker restart my_noetic_ws

# 컨테이너 접속
docker exec -it my_noetic_ws bash

# 컨테이너 로그 확인
docker logs my_noetic_ws

# 컨테이너 상태 확인
docker ps -a
```

### 8.2 새 터미널 추가

```bash
# Host OS에서 새 터미널을 열고
docker exec -it my_noetic_ws bash

# 각 터미널마다 환경 설정 필요
source /root/catkin_ws/devel/setup.bash
```

---

## 9. 다음 단계

환경 구축이 완료되었습니다! 이제 다음 문서를 참고하여 시스템을 개발하세요:

- [통합 개발 가이드](INTEGRATION_GUIDE.md) - 중앙 관제 시스템 개발
- [메인 README](../README.md) - 프로젝트 개요 및 실행 방법

---

## 📝 라이선스 및 저작권

**Copyright © 2025 KATECH (Korea Automotive Technology Institute)**  
**Smart Manufacturing Technology Research Center**

**Author**: LDJ (Dongjun Lee)  
**Email**: djlee2@katech.re.kr

---

**Environment Setup Guide for Mobile-Cobot Integrated Control System**  
**Built by KATECH Smart Manufacturing Technology Research Center**

