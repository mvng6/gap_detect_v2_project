# 코드베이스 학습 가이드

다음 학습 순서는 `integrated_system_code_overview.md`에 정리된 아키텍처를 실제 코드와 연결해 이해하도록 구성되었습니다. 각 단계는 **이해 목표 → 확인할 파일 → 실습 과제** 흐름으로 정리되어 있으니 순차적으로 따라가며 코드를 학습하세요.

## ✅ 진행 현황 체크리스트
- [x] 1단계: 시스템 전반 구조 파악 (`docs/integrated_system_code_overview.md`)
- [x] 2단계: 런치 파일 관점 학습 (`Study/launch_file_tutorial.md`)
- [x] 3단계: 중앙 관제 노드 분석 (`Study/central_coordinator_tutorial.md`)
- [ ] 4단계: 두산 로봇 명령 노드 분석 (`Study/doosan_helper_tutorial.md` 예정)
- [ ] 5단계: 두산 드라이버/메시지 인터페이스 이해
- [ ] 6단계: 모바일 로봇 ROS 노드 분석
- [ ] 7단계: 모바일 로봇 SDK 래퍼 분석
- [ ] 8단계: 보조 도구 및 테스트 노드 탐색
- [ ] 9단계: 문서/README 연계 검토
- [ ] 10단계: 통합 이해 정리

---

## 1. 시스템 전반 구조 파악
- **이해 목표**: 주요 패키지와 통합 실행 흐름 이해
- **필수 문서**: `docs/integrated_system_code_overview.md`, `README.md`
- **실습 과제**: 런치 파이프라인 다이어그램을 참고해 각 노드/토픽 간 관계를 손으로 그려보면서 암기하기

## 2. 런치 파일 관점에서 전체 시퀀스 읽기
- **이해 목표**: 통합 실행 시 어떤 노드가 어떤 순서로 기동되는지 파악
- **확인할 파일**: `src/central_coordinator/launch/integrated_system.launch`
- **실습 과제**: 런치 인자/파라미터를 정리한 표 작성, `roslaunch` 시 전달 가능한 커스텀 값을 스스로 정리

## 3. 중앙 관제 노드 (Python) 상세 분석
- **이해 목표**: 전체 협업 시나리오 로직과 안전 로직 이해
- **확인할 파일**: `src/central_coordinator/src/coordinator_node.py`
- **실습 과제**:
  1. 클래스 구조(초기화, 초기화 단계, 메인 루프, 헬퍼 함수)를 다이어그램으로 정리
  2. `wait_for_doosan_ready`, `wait_for_status` 함수의 상태 전이를 시퀀스 다이어그램으로 표현
  3. 테스트용 가짜 토픽 발행 스크립트를 작성해 단계별로 로직을 따라가며 로그 관찰

## 4. 두산 로봇 명령 노드 (C++) 이해
- **이해 목표**: `/dsr_robot/robot_cmd` 명령이 실제 MoveJoint 서비스 호출로 이어지는 흐름 파악
- **확인할 파일**: `src/doosan_helper/src/move_robot_node.cpp`
- **실습 과제**:
  1. 명령 ID ↔ 관절 각도 맵핑 정리
  2. 서비스 요청 필드(`vel`, `acc`, `mode`)가 의미하는 바 조사
  3. `rosservice call`을 활용해 수동으로 MoveJoint 요청 보내보기

## 5. 두산 드라이버 런치 및 메시지 인터페이스 확인
- **이해 목표**: 상위 노드들이 기대하는 서비스/토픽이 어디서 오는지 이해
- **확인할 파일**: `src/doosan-robot/dsr_launcher/launch/single_robot.launch`, `src/doosan-robot/dsr_msgs/srv/MoveJoint.srv`
- **실습 과제**:
  1. 런치 파일에서 include된 다른 런치들이 어떤 노드를 기동하는지 추적
  2. `MoveJoint.srv` 필드 정의를 토대로 C++ 측 요청 구조 재확인
  3. 실제 장비 연결 없이 `rosservice list` 결과를 비교해 가상/실 장비 차이 정리

## 6. 모바일 로봇 ROS 노드 (Python) 살펴보기
- **이해 목표**: `/mobile/cmd` 명령이 Woosh SDK 제어로 이어지는 흐름 이해
- **확인할 파일**: `src/mobile_robot_control/src/move_mobile_robot_node.py`
- **실습 과제**:
  1. 명령 수신 후 스레드/asyncio 구조를 플로우차트로 그리기
  2. 상태 토픽 발행 타이밍(`IDLE`, `MOVING`, `COMPLETED`, `ERROR`) 정리
  3. 가짜 명령을 `rostopic pub`으로 보내고 로그 변화를 추적

## 7. 모바일 로봇 SDK 래퍼 분석
- **이해 목표**: 실제 거리 제어가 어떻게 구현됐는지 이해
- **확인할 파일**: `src/mobile_robot_control/src/mobile_robot_twist_control.py`
- **실습 과제**:
  1. `move_distance` 함수의 가감속 프로파일 계산 부분을 수식으로 정리
  2. `get_current_pose`, `calculate_distance` 등 헬퍼 함수 흐름 이해
  3. 소규모 단위 테스트(예: distance=0.2, speed=0.1) 시뮬레이션 코드 작성

## 8. 보조 도구 및 테스트 노드 탐색
- **이해 목표**: 유지보수 시 활용 가능한 보조 스크립트 파악
- **확인할 파일**: `src/mobile_robot_control/src/battery_check.py`, `src/mobile_robot_control/src/test_connect_node.py`, `src/doosan_helper/src/trigger_*_node.cpp`
- **실습 과제**: 각 유틸리티의 사용법 및 출력 형태 기록, 필요 시 향후 개선 아이디어 메모

## 9. 문서/README 연계 확인
- **이해 목표**: 코드와 문서의 일관성 검증
- **필수 문서**: 각 패키지 README (central_coordinator, doosan_helper, mobile_robot_control)
- **실습 과제**: README의 설명과 실제 코드가 다르면 TODO 리스트로 남기고, 수정 필요 여부 판단

## 10. 통합 이해 확인 (정리 활동)
- **이해 목표**: 전체 협업 시나리오를 스스로 설명 가능하도록 만들기
- **실습 과제**:
  1. 전체 사이클을 1분 이내로 설명하는 스크립트 작성
  2. `/mobile/status`, `/doosan/status`, `/dsr01a0912/state` 세 토픽을 동시에 모니터링하는 명령 세트를 만들어 기록
  3. 향후 확장 아이디어(예: 상태 시각화, 장애물 회피 로직 추가)를 정리해 팀과 공유할 초안 작성

---

## 추가 팁
- 학습 중 궁금한 토픽/서비스: `rosnode info`, `rostopic echo`, `rosservice type`을 적극 활용
- 실제 장비 테스트 전: 모의 환경에서 토픽에 직접 메시지를 주입하여 로직을 검증하고 로그 패턴을 익히세요
- 변경 예정인 토픽 이름/파라미터는 즉시 문서화하여 전체 팀과 공유하면 추후 혼선을 줄일 수 있습니다


