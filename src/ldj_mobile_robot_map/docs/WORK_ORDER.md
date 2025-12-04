# 📋 작업 지시서: woosh 모바일 로봇 맵 기반 제어 시스템 개발

반드시 모든 작업은 한 단계씩 진행한 뒤 각 단계마다 간단한 유닛 테스트를 진행하여 각 단계가 잘 진행되는지 체크하기

## 📌 프로젝트 정보

| 항목 | 내용 |
|------|------|
| **프로젝트명** | ldj_mobile_robot_map |
| **작성일** | 2025-12-04 |
| **작성자** | LDJ (스마트제조기술연구센터) |
| **대상 로봇** | woosh TR-200 모바일 로봇 |
| **ROS 버전** | ROS Noetic (Ubuntu 20.04) |

---

## 🎯 프로젝트 목표

woosh 모바일 로봇(TR-200)을 ROS 환경에서 맵 기반으로 제어하는 시스템 개발

### 최종 목표
1. **방안 1**: Twist 명령을 통한 직접 속도 제어 + RViz 시각화
2. **방안 2**: ROS Navigation Stack 연동을 통한 자율 주행

---

## 📊 전체 진행 상태

```
전체 진행률: ██████████░░░░░░░░░░ 50%

방안 1 진행률: ████████████░░░░░░░░ 60%
방안 2 진행률: ░░░░░░░░░░░░░░░░░░░░  0%
```

| 단계 | 상태 | 진행률 |
|------|------|--------|
| 패키지 구조 생성 | ✅ 완료 | 100% |
| 기본 연결 및 맵 로드 | ✅ 완료 | 100% |
| 방안 1: woosh → ROS 브릿지 | 🔄 진행중 | 50% |
| 방안 1: Twist 제어 테스트 | 📋 대기 | 0% |
| 방안 1: RViz 시각화 | 📋 대기 | 0% |
| 방안 2: Navigation Stack 설정 | 📋 대기 | 0% |
| 방안 2: 자율 주행 테스트 | 📋 대기 | 0% |

**범례**: ✅ 완료 | 🔄 진행중 | 📋 대기 | ❌ 실패/보류

---

## 📁 파일 구조 및 상태

```
ldj_mobile_robot_map/
├── CMakeLists.txt                    ✅ 완료
├── package.xml                       ✅ 완료
├── README.md                         ✅ 완료
│
├── config/
│   ├── robot_params.yaml             ✅ 완료
│   ├── costmap_common.yaml           📋 방안 2에서 작성
│   ├── local_costmap.yaml            📋 방안 2에서 작성
│   ├── global_costmap.yaml           📋 방안 2에서 작성
│   └── move_base_params.yaml         📋 방안 2에서 작성
│
├── docs/
│   ├── WORK_ORDER.md                 ✅ 현재 파일
│   └── mobile_robot_control_methods.md ✅ 완료
│
├── launch/
│   ├── load_map.launch               ✅ 완료
│   ├── woosh_ros_bridge.launch       📋 방안 1에서 작성
│   ├── twist_control.launch          📋 방안 1에서 작성
│   ├── rviz.launch                   📋 방안 1에서 작성
│   └── navigation.launch             📋 방안 2에서 작성
│
├── rviz/
│   └── woosh_robot.rviz              📋 방안 1에서 작성
│
├── scripts/
│   ├── ldj_load_map.py               ✅ 완료
│   ├── woosh_ros_bridge.py           🔄 진행중 (LiDAR, Odom 완료)
│   └── twist_controller.py           📋 방안 1에서 작성
│
└── src/                              📋 C++ 노드 (필요시)
```

---

## 🔧 방안 1: Twist 직접 제어

### 1.1 개요
woosh SDK의 `twist_req()` API를 사용하여 로봇에 직접 속도 명령을 전송하고, 
woosh 데이터를 ROS 토픽으로 변환하여 RViz에서 시각화합니다.

### 1.2 작업 항목 체크리스트

#### Phase 1: 기본 인프라 (완료)
- [x] 패키지 구조 생성
- [x] CMakeLists.txt 작성
- [x] package.xml 작성
- [x] 로봇 연결 코드 (`ldj_load_map.py`)
- [x] 맵 로드 기능
- [x] 로컬라이제이션 기능
- [x] 로봇 초기화 기능

#### Phase 2: woosh → ROS 브릿지 노드
- [x] `woosh_ros_bridge.py` 스크립트 생성
- [x] LiDAR 데이터 변환 (`ScannerData` → `/scan`) - **~5Hz**
- [x] Odometry 데이터 변환 (`PoseSpeed` → `/odom`) - **~20Hz (polling)**
- [ ] 맵 데이터 변환 (`BuildMapData` → `/map`)
- [ ] TF 브로드캐스트 (`map` → `odom` → `base_link` → `laser`)
- [ ] 브릿지 노드 런치 파일 작성
- [x] 테스트: 토픽 발행 확인 (`rostopic echo`, `rostopic hz`)

#### Phase 3: RViz 시각화
- [ ] RViz 설정 파일 생성 (`woosh_robot.rviz`)
- [ ] 맵 표시 설정
- [ ] LiDAR 포인트 표시 설정
- [ ] 로봇 위치 마커 표시 설정
- [ ] TF 프레임 표시 설정
- [ ] RViz 런치 파일 작성
- [ ] 테스트: RViz에서 실시간 데이터 확인

#### Phase 4: Twist 제어 노드
- [ ] `twist_controller.py` 스크립트 생성
- [ ] `/cmd_vel` 토픽 구독 기능
- [ ] ROS Twist → woosh `twist_req()` 변환
- [ ] 키보드/조이스틱 제어 연동 (선택)
- [ ] 제어 런치 파일 작성
- [ ] 테스트: 전진/후진/회전 명령 확인

### 1.3 구현 상세

#### woosh_ros_bridge.py 구현 사항

```
┌─────────────────────────────────────────────────────────────┐
│                    woosh_ros_bridge.py                      │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  [구독 - woosh SDK]                [발행 - ROS 토픽]           │
│  ┌─────────────────┐               ┌─────────────────┐      │
│  │ scanner_data_sub│ ─────────────▶│ /scan           │      │
│  │ (ScannerData)   │               │ (LaserScan)     │      │
│  └─────────────────┘               └─────────────────┘      │
│                                                             │
│  ┌─────────────────┐               ┌─────────────────┐      │
│  │ pose_speed_sub  │ ─────────────▶│ /odom           │      │
│  │ (PoseSpeed)     │               │ (Odometry)      │      │
│  └─────────────────┘               └─────────────────┘      │
│                                                             │
│  ┌─────────────────┐               ┌─────────────────┐      │
│  │ build_map_sub   │ ─────────────▶│ /map            │      │
│  │ (BuildMapData)  │               │ (OccupancyGrid) │      │
│  └─────────────────┘               └─────────────────┘      │
│                                                             │
│  [TF 브로드캐스트]                                             │
│  map → odom → base_link → laser                             │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

#### 실제 ROS 토픽 구조 (측정값)

| 토픽 이름 | 메시지 타입 | 주기 | 상태 | 비고 |
|-----------|-------------|------|------|------|
| `/scan` | `sensor_msgs/LaserScan` | ~5Hz | ✅ 완료 | 1081개 포인트, 360° |
| `/odom` | `nav_msgs/Odometry` | ~20Hz | ✅ 완료 | polling 방식 |
| `/map` | `nav_msgs/OccupancyGrid` | 1Hz (latched) | 📋 대기 | - |
| `/tf` | `tf2_msgs/TFMessage` | - | 📋 대기 | - |
| `/cmd_vel` | `geometry_msgs/Twist` | - | 📋 대기 | 속도 명령 (입력) |

### 1.4 테스트 체크리스트

| 테스트 항목 | 상태 | 날짜 | 비고 |
|-------------|------|------|------|
| 로봇 연결 | ✅ | 2025-12-04 | 배터리 56% |
| 맵 로드 | ✅ | 2025-12-04 | 251127_Lobby_v2 |
| 로컬라이제이션 | ✅ | 2025-12-04 | - |
| LiDAR 토픽 발행 | ✅ | 2025-12-04 | ~5Hz, 1081포인트 |
| Odom 토픽 발행 | ✅ | 2025-12-04 | ~20Hz, polling 방식 |
| Map 토픽 발행 | 📋 | - | - |
| TF 브로드캐스트 | 📋 | - | - |
| RViz 맵 시각화 | 📋 | - | - |
| RViz LiDAR 시각화 | 📋 | - | - |
| Twist 전진 명령 | 📋 | - | - |
| Twist 후진 명령 | 📋 | - | - |
| Twist 회전 명령 | 📋 | - | - |

---

## 🚀 방안 2: ROS Navigation Stack 연동

### 2.1 개요
방안 1에서 구현한 브릿지 노드를 기반으로 ROS Navigation Stack을 연동하여
자율 주행 기능을 구현합니다.

### 2.2 사전 조건
- [ ] 방안 1 완료
- [ ] Navigation Stack 패키지 설치

### 2.3 작업 항목 체크리스트

#### Phase 1: Navigation Stack 설치
- [ ] `ros-noetic-navigation` 설치
- [ ] `ros-noetic-amcl` 설치
- [ ] `ros-noetic-move-base` 설치
- [ ] `ros-noetic-map-server` 설치
- [ ] `ros-noetic-dwa-local-planner` 설치
- [ ] `ros-noetic-global-planner` 설치

#### Phase 2: TF 설정
- [ ] TF 트리 구조 설계
- [ ] `map` → `odom` 변환 (amcl 또는 브릿지)
- [ ] `odom` → `base_link` 변환
- [ ] `base_link` → `laser` 변환 (static_transform_publisher)
- [ ] TF 트리 검증 (`rosrun tf view_frames`)

#### Phase 3: Costmap 설정
- [ ] `costmap_common.yaml` 작성
- [ ] `local_costmap.yaml` 작성
- [ ] `global_costmap.yaml` 작성
- [ ] 로봇 footprint 설정
- [ ] 장애물 감지 파라미터 튜닝

#### Phase 4: move_base 설정
- [ ] `move_base_params.yaml` 작성
- [ ] Global Planner 설정 (A* 또는 Dijkstra)
- [ ] Local Planner 설정 (DWA)
- [ ] 속도/가속도 제한 설정
- [ ] Recovery behavior 설정

#### Phase 5: 런치 파일 및 테스트
- [ ] `navigation.launch` 작성
- [ ] RViz에서 2D Nav Goal 테스트
- [ ] 파라미터 튜닝
- [ ] 장애물 회피 테스트

### 2.4 TF 트리 구조

```
map
 └── odom
      └── base_link
           ├── laser
           └── (기타 센서 프레임)
```

### 2.5 Navigation Stack 아키텍처

```
┌─────────────────────────────────────────────────────────────┐
│                       RViz                                   │
│  - 2D Nav Goal 설정                                          │
│  - 경로 시각화                                               │
│  - costmap 시각화                                            │
└──────────────────────────┬──────────────────────────────────┘
                           │ /move_base_simple/goal
                           ▼
┌─────────────────────────────────────────────────────────────┐
│                      move_base                               │
│  ┌──────────────┐    ┌──────────────┐    ┌──────────────┐   │
│  │Global Planner│───▶│Local Planner │───▶│   /cmd_vel   │   │
│  │   (A*)       │    │   (DWA)      │    │              │   │
│  └──────────────┘    └──────────────┘    └──────────────┘   │
│         ▲                   ▲                   │            │
│         │                   │                   │            │
│  ┌──────────────┐    ┌──────────────┐           │            │
│  │Global Costmap│    │Local Costmap │           │            │
│  └──────────────┘    └──────────────┘           │            │
└─────────────────────────────────────────────────┼────────────┘
                           ▲                      │
                           │                      ▼
┌──────────────────────────┴──────────────────────────────────┐
│                   woosh_ros_bridge.py                        │
│  /scan, /odom, /map, /tf                    twist_req()      │
└─────────────────────────────────────────────────────────────┘
                           ▲                      │
                           │                      ▼
┌─────────────────────────────────────────────────────────────┐
│                     woosh TR-200 로봇                        │
└─────────────────────────────────────────────────────────────┘
```

---

## 📝 개발 일지

### 2025-12-04

#### 오전 - 기본 인프라 구축
- [x] `ldj_mobile_robot_map` 패키지 생성
- [x] 기본 파일 구조 설정 (CMakeLists.txt, package.xml)
- [x] `ldj_load_map.py` 이전 및 수정
- [x] `mobile_robot_control_methods.md` 이전
- [x] 런치 파일 생성 (`load_map.launch`)
- [x] 설정 파일 생성 (`robot_params.yaml`)
- [x] README.md 작성
- [x] 작업 지시서 작성 (WORK_ORDER.md)

#### 오후 - woosh → ROS 브릿지 노드 구현
- [x] `woosh_ros_bridge.py` 스크립트 생성
- [x] 로봇 연결 및 맵 로드 기능 구현
- [x] LiDAR 데이터 변환 구현 (`ScannerData` → `/scan`)
  - subscribe 방식 사용
  - ~5Hz 발행, 1081개 포인트, 360° 범위
- [x] Odometry 데이터 변환 구현 (`PoseSpeed` → `/odom`)
  - polling 방식으로 변경 (subscribe는 정지 시 데이터 미발행)
  - ~20Hz 발행
  - frame_id: `odom`, child_frame_id: `base_link`
- [x] 유닛 테스트 완료 (`rostopic echo`, `rostopic hz`)

#### 이슈 및 해결
1. **ExecTask 실패 문제**: `kTaskable` 상태가 0으로 유지되어 Navigation Task 실행 불가
   - **원인**: woosh 내부 네비게이션 영역 미설정 또는 맵 설정 문제
   - **해결 방안**: Twist 직접 제어(방안 1)로 우선 진행

2. **PoseSpeed subscribe 미작동**: 정지 상태에서 콜백이 호출되지 않음
   - **원인**: woosh SDK의 subscribe는 데이터 변경 시에만 콜백 호출
   - **해결 방안**: polling 방식으로 변경 (20Hz로 주기적 요청)

3. **CMake 버전 호환성**: cmake_minimum_required 3.0.2 → 3.5 오류
   - **해결**: CMakeLists.txt 버전 업데이트, `-DCMAKE_POLICY_VERSION_MINIMUM=3.5` 플래그 추가

#### 다음 작업
- [ ] 맵 데이터 변환 구현 (`BuildMapData` → `/map`)
- [ ] TF 브로드캐스트 구현 (`map` → `odom` → `base_link` → `laser`)
- [ ] 브릿지 노드 런치 파일 작성

---

## ⚠️ 알려진 이슈

| ID | 이슈 | 상태 | 우선순위 | 해결방안 |
|----|------|------|----------|----------|
| #001 | ExecTask "No size matching task size" 오류 | 🔄 우회 | 높음 | 방안 1로 우회 |
| #002 | kTaskable 상태가 0으로 유지 | 🔄 우회 | 높음 | Twist 직접 제어 사용 |
| #003 | PoseSpeed subscribe 정지 시 미작동 | ✅ 해결 | 중간 | polling 방식으로 변경 |
| #004 | CMake 3.0.2 호환성 오류 | ✅ 해결 | 낮음 | 버전 3.5로 업데이트 |

---

## 📚 참고 자료

### woosh SDK
- `woosh_robot_py` 패키지 문서
- Protobuf 메시지 정의 (`woosh.proto.*`)

### ROS Navigation
- [ROS Navigation Stack Wiki](http://wiki.ros.org/navigation)
- [move_base 설정 가이드](http://wiki.ros.org/move_base)
- [costmap_2d 설정](http://wiki.ros.org/costmap_2d)
- [amcl 설정](http://wiki.ros.org/amcl)
- [DWA Local Planner](http://wiki.ros.org/dwa_local_planner)

### 관련 문서
- [제어 방안 비교](mobile_robot_control_methods.md)

---

## 📞 연락처

- **담당자**: LDJ
- **이메일**: djlee2@katech.re.kr
- **소속**: KATECH 스마트제조기술연구센터

---

*마지막 업데이트: 2025-12-04 (Phase 2: LiDAR, Odom 완료)*

