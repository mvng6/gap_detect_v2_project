# [NUC 15 Pro] Ubuntu 24.04 Host 기반 Docker-ROS 1 Noetic 환경 구축 및 두산 로봇(A0912) 연동 요약

🚀 **요약 (Summary)**

본 문서는 최신 하드웨어(NUC 15 Pro)와 특정 ROS 1 버전(Noetic) 간의 호환성 문제를 해결하기 위한 아키텍처 구축 과정을 요약합니다.

NUC 15 Pro에 Ubuntu 24.04를 호스트 OS로 설치하여 하드웨어 드라이버 호환성을 확보했습니다. 그 위에 Docker를 설치하고, ROS 1 Noetic 컨테이너를 생성하여 소프트웨어 의존성을 해결했습니다.

핵심적으로 Host OS의 소스 코드 폴더를 컨테이너 내부로 `volume` 마운트하여 데이터 영속성을 확보했으며, `network="host"` 모드를 통해 NUC의 LAN 포트로 두산 로봇(A0912)과 성공적으로 연결했습니다. 최종적으로 `rosservice call` 명령어를 통해 원하는 관절 각도로 로봇을 구동시키는 데 성공했습니다.

---

✅ **수행한 업무 내용 (Work Accomplished)**

1.  **Host OS (Ubuntu 24.04) 환경 설정**
    * - [x] `docker-ce` (Docker 엔진) 설치 및 현 사용자(`katech`) `docker` 그룹 추가
    * - [x] Host 홈 디렉터리에 영구 워크스페이스(`~/robot_ws/src`) 생성
    * - [x] Host 워크스페이스에 두산 로봇 패키지(`doosan-robot`, `serial`) `git clone`

2.  **Docker 컨테이너 생성 및 진입**
    * - [x] `ros:noetic-robot` Docker 이미지 다운로드
    * - [x] **[핵심]** `docker run` 명령어를 통해 `my_noetic_ws` 컨테이너 생성 및 실행
        * `--network="host"`: NUC의 LAN, Wi-Fi 네트워크를 컨테이너가 직접 공유
        * `--volume="$HOME/robot_ws:/root/catkin_ws"`: Host의 코드를 컨테이너의 `catkin_ws`로 연결 (마법 포탈)
        * `-v /var/run/docker.sock:/var/run/docker.sock`: Host의 Docker 엔진을 컨테이너가 사용할 수 있도록 연결 (직통 전화선)
    * - [x] `docker exec -it my_noetic_ws bash` 명령어로 "ROS 작업실"(컨테이너) 내부 진입

3.  **Docker 컨테이너 내부 (ROS Noetic) 환경 설정**
    * - [x] `docker-ce-cli` 설치 (에뮬레이터 설치 스크립트 실행용 "전화기" 설치)
    * - [x] `ros-noetic-moveit*`, `ros-noetic-rqt*` 등 `README.md`의 의존성 패키지(`apt`) 설치
    * - [x] `rosdep update` 및 `rosdep install ...` 실행
    * - [x] `install_emulator.sh` 스크립트 실행 (Docker-in-Docker)
    * - [x] `catkin_make`로 `/root/catkin_ws` 빌드 및 `~/.bashrc`에 환경 설정 추가

4.  **두산 로봇(A0912) 연결 및 구동**
    * - [x] Host OS(24.04)의 네트워크 설정에서 LAN 포트(`enp87s0`) 고정 IP (`192.168.137.101`) 설정 (게이트웨이 없음)
    * - [x] **[터미널 1]** `docker exec`로 컨테이너 진입 후, `roslaunch`로 로봇 연결 (모델: `a0912`, 모드: `real`, 호스트: `192.168.137.100`)
    * - [x] **[터미널 2]** `docker exec`로 컨테이너 **새로** 진입
    * - [x] **[터미널 2]** `source /root/catkin_ws/devel/setup.bash`로 환경 활성화
    * - [x] **[터미널 2]** `rosservice call` 명령어로 `move_joint` 서비스 호출, `[90.0, 0.0, 90.0, 0.0, 90.0, -90.0]` (Degree) 각도로 로봇 구동 성공

---

💥 **주요 문제 및 해결 과정 (Issues & Solutions)**

1.  **문제:** NUC 15 Pro의 최신 Wi-Fi 카드가 Ubuntu 20.04에서 인식되지 않음.
    * **근본 원인:** 20.04의 커널(5.x)이 NUC 15 Pro의 최신 하드웨어를 지원하지 않음.
    * **해결:** Host OS를 Ubuntu 24.04로 설치하여 하드웨어 호환성을 확보하고, ROS Noetic은 Docker 컨테이너로 격리하여 실행함.

2.  **문제:** `install_emulator.sh` 실행 시 `docker: not found` 오류 발생.
    * **근본 원인:** "ROS 작업실"(컨테이너 1) 내부에서 `docker pull` 명령을 실행하려 했으나, 컨테이너 내부에 Docker 엔진/CLI가 없음 (Docker-in-Docker 문제).
    * **해결:**
        1.  컨테이너 생성(`run`) 시 `-v /var/run/docker.sock:/var/run/docker.sock` 옵션을 추가하여 "직통 전화선" 연결.
        2.  컨테이너 내부(`exec`)에서 `apt install docker-ce-cli`를 실행하여 "특수 전화기" 설치.

3.  **문제:** `rosservice call` 실행 시 `ERROR: Unable to load type [dsr_msgs/MoveJoint]`.
    * **근본 원인:** `docker exec`로 접속한 **두 번째 터미널**이 `~/catkin_ws`의 빌드 정보(커스텀 메시지)를 인식하지 못함.
    * **해결:** `rosservice call` 실행 전, 해당 터미널에서 `source /root/catkin_ws/devel/setup.bash`를 먼저 실행하여 환경을 활성화함.

4.  **문제:** `rosservice call` 시 `pos: [90.0 0.0 ...]` (쉼표 없음) 전송 시 `Incorrect number of elements: 1 != 6` 오류 발생.
    * **근본 원인:** YAML 구문 오류. 쉼표 없는 리스트는 6개의 숫자가 아닌 1개의 긴 문자열로 인식됨.
    * **해결:** `pos: [90.0, 0.0, 90.0, 0.0, 90.0, -90.0]`와 같이 쉼표(`,`)를 포함한 올바른 리스트 형식으로 전송함.

---

💡 **결론 및 교훈 (Conclusion & Lessons Learned)**

* **Host-Container 분리:** 최신 하드웨어(NUC 15 Pro)와 구형 ROS 버전(Noetic)을 함께 사용하는 가장 안정적인 방법은, Host OS(24.04)가 하드웨어 드라이버를 전담하고 ROS 환경은 Docker 컨테이너로 완벽히 격리하는 것입니다.
* **데이터 영속성:** 소스 코드는 반드시 Host OS(`~/robot_ws`)에 `git clone`하고, 이를 `--volume` 옵션으로 컨테이너에 연결해야 합니다. 컨테이너 내부에 코드를 직접 저장하면 컨테이너 삭제 시 코드가 유실됩니다.
* **네트워크:** `network="host"` 모드는 컨테이너가 Host의 모든 물리적 네트워크(LAN, Wi-Fi)를 공유하게 해, 로봇(LAN)과 모바일 로봇(Wi-Fi)에 동시 접속하는 환경에 필수적입니다.
* **컨테이너 환경:** Docker 컨테이너에 `exec`로 접속하는 새 터미널은 항상 독립된 환경입니다. 커스텀 패키지를 사용하려면 `source ~/catkin_ws/devel/setup.bash`를 매번 실행해야 합니다.