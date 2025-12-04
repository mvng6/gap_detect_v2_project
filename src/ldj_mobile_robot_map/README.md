# ldj_mobile_robot_map

woosh 모바일 로봇(TR-200)을 ROS 환경에서 맵 기반으로 제어하기 위한 패키지입니다.

## 개요

이 패키지는 두 가지 제어 방식을 지원합니다:

- **방안 1**: Twist 명령을 통한 직접 속도 제어
- **방안 2**: ROS Navigation Stack 연동 (추후 구현)

## 패키지 구조

```
ldj_mobile_robot_map/
├── CMakeLists.txt
├── package.xml
├── README.md
├── scripts/
│   ├── ldj_load_map.py          # 맵 로드 및 기본 제어
│   ├── woosh_ros_bridge.py      # woosh → ROS 브릿지 (예정)
│   └── twist_controller.py      # Twist 제어 노드 (예정)
├── launch/
│   ├── load_map.launch          # 맵 로드 런치 파일
│   ├── twist_control.launch     # Twist 제어 런치 파일
│   └── navigation.launch        # Navigation Stack 런치 파일 (예정)
├── config/
│   ├── robot_params.yaml        # 로봇 파라미터
│   ├── costmap_common.yaml      # costmap 공통 설정 (예정)
│   └── move_base_params.yaml    # move_base 설정 (예정)
└── docs/
    └── mobile_robot_control_methods.md  # 제어 방안 비교 문서
```

## 의존성

### 필수
- ROS (Noetic 권장)
- woosh_robot_py (woosh SDK)

### 방안 2 (Navigation Stack)
```bash
sudo apt-get install ros-noetic-navigation
sudo apt-get install ros-noetic-amcl
sudo apt-get install ros-noetic-move-base
sudo apt-get install ros-noetic-map-server
sudo apt-get install ros-noetic-dwa-local-planner
sudo apt-get install ros-noetic-global-planner
```

## 빌드

```bash
cd ~/robot_ws
catkin_make
source devel/setup.bash
```

## 사용법

### 맵 로드 및 초기화
```bash
rosrun ldj_mobile_robot_map ldj_load_map.py
```

### 런치 파일 사용
```bash
roslaunch ldj_mobile_robot_map load_map.launch
```

### 파라미터 설정
```bash
roslaunch ldj_mobile_robot_map load_map.launch robot_ip:=169.254.128.2 robot_port:=5480
```

## 개발 로드맵

### 1단계: 방안 1 (Twist 직접 제어) ✅ 진행 중
- [x] 맵 로드
- [x] 로컬라이제이션
- [x] 로봇 초기화
- [ ] woosh → ROS 브릿지 노드
- [ ] Twist 제어 테스트
- [ ] RViz 시각화

### 2단계: 방안 2 (Navigation Stack 연동) 📋 예정
- [ ] TF 설정
- [ ] Navigation Stack 설정
- [ ] move_base 파라미터 튜닝
- [ ] 경로 계획 테스트

## 참고 문서

- [제어 방안 비교](docs/mobile_robot_control_methods.md)
- [ROS Navigation Stack Wiki](http://wiki.ros.org/navigation)

## 작성자

- LDJ (djlee2@katech.re.kr)
- KATECH 로봇 자동화 연구팀

