# Mobile Robot Control - 문서 인덱스

> **버전**: 1.0.0  
> **작성일**: 2025-10-29  
> **작성자**: KATECH Robotics Team

---

## 📋 전체 문서 구조

```
docs/mobile_robot/
├── INDEX.md                          # 이 파일 (문서 인덱스)
├── README.md                         # 메인 README
│
├── api/                              # API 레퍼런스
│   └── mobile_robot_control_node.md  # 전체 API 문서
│
├── guides/                           # 사용 가이드
│   └── getting_started.md            # 시작 가이드
│
├── examples/                         # 예제 코드
│   └── basic_examples.md             # 10가지 기본 예제
│
└── architecture/                     # 아키텍처 문서
    └── design_overview.md            # 설계 개요
```

---

## 🎯 문서별 용도

### 1. [README.md](README.md)
- **대상**: 모든 사용자
- **내용**: 
  - 프로젝트 개요
  - 빠른 시작
  - 문서 가이드
  - 주요 기능 소개
- **읽는 시간**: 5분

---

### 2. [guides/getting_started.md](guides/getting_started.md) ⭐ **여기서 시작!**
- **대상**: 초보자
- **내용**:
  - 설치 및 설정
  - 첫 이동 테스트
  - 기본 사용법 (전진/후진/회전)
  - 문제 해결 가이드
- **읽는 시간**: 15분
- **실습 시간**: 30분

**포함된 주제:**
- ✅ 네트워크 연결 확인
- ✅ CLI 사용법
- ✅ 가감속 조정
- ✅ Python 코드 예제
- ✅ 문제 해결 (5가지 자주 발생하는 문제)

---

### 3. [examples/basic_examples.md](examples/basic_examples.md)
- **대상**: 실용 예제가 필요한 개발자
- **내용**: 10가지 복사-붙여넣기 가능한 예제
  1. 간단한 전진/후진
  2. 회전 테스트
  3. 사각형 주행
  4. 지그재그 패턴
  5. 거리 센서 기반 장애물 회피
  6. ROS Topic 기반 제어
  7. 커스텀 속도 프로파일
  8. 다중 로봇 제어
  9. 이동 중 중단
  10. 성능 벤치마크
- **읽는 시간**: 20분
- **실습 시간**: 1~2시간

**예제 카테고리:**
- 🔰 기본 이동 (예제 1~2)
- 🔄 패턴 주행 (예제 3~4)
- 🤖 센서 연동 (예제 5)
- 📡 ROS 통합 (예제 6)
- ⚙️ 고급 제어 (예제 7~10)

---

### 4. [api/mobile_robot_control_node.md](api/mobile_robot_control_node.md)
- **대상**: 고급 사용자, 기여자
- **내용**: 전체 API 레퍼런스
  - 데이터 클래스 (RobotConfig, VelocityProfileConfig, MotionResult)
  - 유틸리티 클래스 (VelocityProfileCalculator)
  - 메인 클래스 (MobileRobotController)
  - CLI 인터페이스
  - 내부 메서드 (고급)
- **읽는 시간**: 30분

**주요 섹션:**
- 📦 데이터 클래스: 설정 및 결과 구조
- 🧮 속도 프로파일 계산: 사다리꼴 프로파일 로직
- 🤖 로봇 제어: 이동/회전 메서드
- 🖥️ CLI: 명령줄 인터페이스
- ⚙️ 성능 특성: 정밀도, 속도, 응답 시간

---

### 5. [architecture/design_overview.md](architecture/design_overview.md)
- **대상**: 확장/커스터마이징을 원하는 개발자
- **내용**:
  - 시스템 개요
  - 코드 구조 (클래스 다이어그램)
  - 설계 원칙
  - 주요 컴포넌트 (연결, 속도 프로파일, 이동 제어)
  - 데이터 흐름
  - 확장 가이드 (4가지 확장 예제)
- **읽는 시간**: 40분

**확장 가이드:**
- 🔧 새로운 속도 프로파일 추가 (S-커브)
- 🔄 새로운 이동 패턴 (원형 경로)
- 📡 ROS Action Server 통합
- 🤖 센서 통합 (라이다 기반 장애물 회피)

---

## 🗺️ 학습 경로

### 초보자 경로

```
1. README.md (5분)
   ↓
2. getting_started.md (45분)
   ├─ 설치 및 첫 이동
   ├─ 기본 사용법
   └─ 문제 해결
   ↓
3. basic_examples.md (예제 1~4) (1시간)
   └─ 실제 코드 작성 및 테스트
```

**예상 시간**: 총 2시간

---

### 개발자 경로

```
1. README.md (5분)
   ↓
2. getting_started.md (빠르게 훑어보기) (15분)
   ↓
3. api/mobile_robot_control_node.md (30분)
   ├─ API 구조 파악
   └─ 주요 메서드 이해
   ↓
4. basic_examples.md (예제 5~10) (1시간)
   └─ 고급 예제 실습
   ↓
5. architecture/design_overview.md (40분)
   └─ 내부 구조 이해
```

**예상 시간**: 총 2.5시간

---

### 기여자 경로

```
1. README.md (5분)
   ↓
2. architecture/design_overview.md (40분)
   ├─ 전체 아키텍처 파악
   ├─ 설계 원칙 이해
   └─ 확장 가이드 숙지
   ↓
3. api/mobile_robot_control_node.md (30분)
   └─ API 세부사항 확인
   ↓
4. 코드 리뷰
   └─ mobile_robot_control_node.py (소스 코드)
```

**예상 시간**: 총 2시간 + 코드 리뷰

---

## 📚 문서별 빠른 참조

### 자주 찾는 내용

| 주제 | 문서 | 섹션 |
|------|------|------|
| **설치 방법** | getting_started.md | [설치](guides/getting_started.md#설치) |
| **첫 실행** | getting_started.md | [빠른 시작](guides/getting_started.md#빠른-시작) |
| **CLI 옵션** | api/mobile_robot_control_node.md | [CLI 인터페이스](api/mobile_robot_control_node.md#cli-인터페이스) |
| **Python 예제** | basic_examples.md | [기본 이동](examples/basic_examples.md#기본-이동) |
| **가감속 조정** | getting_started.md | [고급 기능](guides/getting_started.md#고급-기능) |
| **문제 해결** | getting_started.md | [문제 해결](guides/getting_started.md#문제-해결) |
| **API 전체** | api/mobile_robot_control_node.md | [전체](api/mobile_robot_control_node.md) |
| **아키텍처** | design_overview.md | [코드 구조](architecture/design_overview.md#코드-구조) |
| **확장 방법** | design_overview.md | [확장 가이드](architecture/design_overview.md#확장-가이드) |
| **ROS 통합** | basic_examples.md | [ROS 통합](examples/basic_examples.md#ros-통합) |

---

## 🔍 키워드별 문서 찾기

### A-C

- **API 레퍼런스** → [api/mobile_robot_control_node.md](api/mobile_robot_control_node.md)
- **가감속** → [getting_started.md - 가감속 조정](guides/getting_started.md#3-가감속-조정)
- **CLI** → [api/mobile_robot_control_node.md - CLI](api/mobile_robot_control_node.md#cli-인터페이스)

### D-M

- **데이터 클래스** → [api/mobile_robot_control_node.md - 데이터 클래스](api/mobile_robot_control_node.md#데이터-클래스)
- **문제 해결** → [getting_started.md - 문제 해결](guides/getting_started.md#문제-해결)

### N-S

- **ROS 통합** → [basic_examples.md - ROS 통합](examples/basic_examples.md#ros-통합)
- **사각형 주행** → [basic_examples.md - 예제 3](examples/basic_examples.md#예제-3-사각형-주행)
- **설치** → [getting_started.md - 설치](guides/getting_started.md#설치)
- **속도 프로파일** → [design_overview.md - 속도 프로파일](architecture/design_overview.md#2-속도-프로파일-계산-velocity-profile)

### T-Z

- **확장** → [design_overview.md - 확장 가이드](architecture/design_overview.md#확장-가이드)
- **회전** → [getting_started.md - 회전](guides/getting_started.md#2-회전)

---

## 📝 문서 작성 규칙

### 스타일 가이드

1. **제목**: 명확하고 계층적
   - H1: 문서 제목
   - H2: 주요 섹션
   - H3: 하위 섹션

2. **코드 예제**:
   - Python: 실행 가능한 완전한 코드
   - Bash: 명확한 주석 포함

3. **다이어그램**:
   - ASCII 아트 사용 (유니코드 박스 그리기)
   - 시퀀스, 플로우차트, 클래스 다이어그램

4. **링크**:
   - 상대 경로 사용
   - 섹션 앵커 적극 활용

---

## 🔄 문서 업데이트 히스토리

### v1.0.0 (2025-10-29)
- ✅ 초기 문서 세트 작성
- ✅ API 레퍼런스 완성
- ✅ 시작 가이드 작성
- ✅ 10가지 예제 추가
- ✅ 아키텍처 문서 작성

---

## 📞 피드백

문서 개선 사항이나 오류를 발견하시면:

- **이메일**: robotics@katech.re.kr
- **GitHub Issues**: [robot_ws/issues](https://github.com/katech/robot_ws/issues)

---

## 📄 라이센스

MIT License - KATECH Robotics Team

---

**마지막 업데이트**: 2025-10-29  
**작성자**: KATECH Robotics Team

