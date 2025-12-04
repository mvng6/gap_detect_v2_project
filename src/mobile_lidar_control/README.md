# Mobile LiDAR Control Package

모바일 로봇의 LiDAR 기반 제어 및 위치 보정을 위한 ROS 패키지입니다.

## 📋 개요

이 패키지는 다음 기능을 제공합니다:
- **LiDAR 데이터 수신**: 모바일 로봇의 LiDAR 센서 데이터 획득 및 처리
- **위치 이동 보정**: LiDAR 센서값을 활용한 Twist 명령 기반 정밀 위치 제어
- **Rviz 연동**: 시각화 및 디버깅을 위한 Rviz 통합

## 📁 패키지 구조

```
mobile_lidar_control/
├── CMakeLists.txt          # CMake 빌드 설정
├── package.xml             # 패키지 메타데이터 및 의존성
├── README.md               # 패키지 문서
├── config/                 # 설정 파일
│   └── params.yaml         # ROS 파라미터 설정
├── scripts/                # Python 실행 스크립트
│   └── (노드 스크립트들)
├── src/                    # C++ 소스 코드
│   └── mobile_lidar_control/
├── include/                # C++ 헤더 파일
│   └── mobile_lidar_control/
├── msg/                    # 커스텀 메시지 정의
└── srv/                    # 커스텀 서비스 정의
```

## 🚀 빌드 방법

```bash
# 워크스페이스로 이동
cd ~/robot_ws

# 빌드
catkin_make

# 환경 설정
source devel/setup.bash
```

## 🎮 실행 방법

각 노드는 개별적으로 실행합니다:

```bash
# 예시: LiDAR 프로세서 노드 실행
rosrun mobile_lidar_control lidar_processor.py

# 예시: 모션 보정 노드 실행
rosrun mobile_lidar_control motion_corrector.py
```

## 📡 Topics

### Subscribed Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/scan` | `sensor_msgs/LaserScan` | LiDAR 스캔 데이터 |
| `/odom` | `nav_msgs/Odometry` | 오도메트리 데이터 |

### Published Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/cmd_vel` | `geometry_msgs/Twist` | 속도 명령 |
| `/visualization_marker` | `visualization_msgs/Marker` | Rviz 마커 |

## ⚙️ Parameters

`config/params.yaml` 파일에서 설정 가능:

```yaml
mobile_lidar_control:
  # LiDAR 설정
  lidar_topic: "/scan"
  lidar_frame: "laser_frame"
  
  # 제어 설정
  max_linear_vel: 0.5
  max_angular_vel: 1.0
  
  # 보정 설정
  position_tolerance: 0.01
  angle_tolerance: 0.02
```

## 📝 개발 노트

- 완전한 코드 구현 전까지 launch 파일은 생성하지 않습니다.
- 개별 노드 단위로 테스트 및 개발을 진행합니다.
- 새로운 기능 추가 시 `scripts/` 디렉토리에 노드를 추가합니다.

## 🔗 의존성

- ROS Noetic (또는 Melodic)
- rospy, roscpp
- std_msgs, geometry_msgs, sensor_msgs
- nav_msgs, visualization_msgs

## 👤 Author

KATECH Robotics Team

