# [NUC 15 Pro] Ubuntu 24.04 Host 기반 Docker-ROS 1 Noetic 환경 구축 및 두산 로봇(A0912) 연동 요약

🚀 **요약 (Summary)**

본 문서는 최신 하드웨어(NUC 15 Pro)와 특정 ROS 1 버전(Noetic) 간의 호환성 문제를 해결하기 위한 아키텍처 구축 과정을 요약합니다.

NUC 15 Pro에 Ubuntu 24.04를 호스트 OS로 설치하여 하드웨어 드라이버 호환성을 확보했습니다. 그 위에 Docker를 설치하고, ROS 1 Noetic 컨테이너를 생성하여 소프트웨어 의존성을 해결했습니다.

핵심적으로 Host OS의 소스 코드 폴더를 컨테이너 내부로 `volume` 마운트하여 데이터 영속성을 확보했으며, `network="host"` 모드를 통해 NUC의 LAN 포트로 두산 로봇(A0912)과 성공적으로 연결했습니다. `/katech/robot_command` 토픽 메시지(`std_msgs/Int32`) 값에 따라 지정된 관절 자세로 로봇을 이동시키는 커스텀 C++ 노드를 개발 및 테스트 완료했습니다.

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
    * - [x] `rosdep update` 및 `rosdep install --from-paths src ...` 실행
    * - [x] `install_emulator.sh` 스크립트 실행 (Docker-in-Docker)
    * - [x] `/root/catkin_ws` 빌드 (`catkin_make`) 및 `~/.bashrc`에 환경 설정 추가

4.  **커스텀 제어 노드 (`doosan_helper` 패키지) 개발**
    * - [x] **[패키지 생성]** 컨테이너 내 `/root/catkin_ws/src`에서 `catkin_create_pkg doosan_helper roscpp std_msgs dsr_msgs` 실행
    * - [x] **[코드 작성 1]** Host VS Code를 사용하여 `~/robot_ws/src/doosan_helper/src/move_robot_node.cpp` 작성 (토픽 값 `1` 또는 `0` 수신 시 `move_joint` 서비스 호출)
    * - [x] **[코드 작성 2]** Host VS Code를 사용하여 `~/robot_ws/src/doosan_helper/src/trigger_zero_node.cpp` 작성 (토픽 값 `0` 발행 노드)
    * - [x] **[빌드 설정]** Host VS Code를 사용하여 `~/robot_ws/src/doosan_helper/CMakeLists.txt` 수정 (두 노드의 `add_executable` 및 `target_link_libraries` 추가)
    * - [x] **[재빌드]** 컨테이너 내 `/root/catkin_ws`에서 `catkin_make` 재실행 및 `source devel/setup.bash`

5.  **두산 로봇(A0912) 연결 및 구동 테스트**
    * - [x] Host OS(24.04)의 네트워크 설정에서 LAN 포트(`enp87s0`) 고정 IP (`192.168.137.101`) 설정 (게이트웨이 없음)
    * - [x] **[터미널 1]** `docker exec`로 컨테이너 진입 후, `roslaunch`로 로봇 연결 (모델: `a0912`, 모드: `real`, 호스트: `192.168.137.100`)
    * - [x] **[터미널 2]** `docker exec`로 컨테이너 **새로** 진입 후 `source devel/setup.bash`, `rosrun doosan_helper move_robot_node` 실행
    * - [x] **[터미널 3]** `docker exec`로 컨테이너 **새로** 진입 후 `source devel/setup.bash`
    * - [x] **[테스트 1]** 터미널 3에서 `rostopic pub /katech/robot_command std_msgs/Int32 "data: 1" -1` 실행 $\rightarrow$ 로봇이 `[90, 0, 90, 0, 90, -90]` 자세로 이동 확인
    * - [x] **[테스트 2]** 터미널 3에서 `rosrun doosan_helper trigger_zero_node` 실행 $\rightarrow$ 로봇이 `[-90, 0, 90, 0, 90, -90]` 자세로 이동 확인

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

5.  **문제:** Host VS Code에서 컨테이너가 생성한 `doosan_helper` 폴더에 파일 저장 시 `permission denied` 오류 발생.
    * **근본 원인:** 컨테이너 내부의 `root` 사용자가 생성한 폴더/파일의 소유권이 Host에서도 `root`로 설정됨. Host의 `katech` 사용자는 쓰기 권한이 없음.
    * **해결:** Host 터미널에서 `sudo chown -R $USER:$USER ~/robot_ws` 명령어를 실행하여 `robot_ws` 폴더 전체의 소유권을 `katech` 사용자에게 부여함.

---

💡 **결론 및 교훈 (Conclusion & Lessons Learned)**

* **Host-Container 분리:** 최신 하드웨어(NUC 15 Pro)와 구형 ROS 버전(Noetic)을 함께 사용하는 가장 안정적인 방법은, Host OS(24.04)가 하드웨어 드라이버를 전담하고 ROS 환경은 Docker 컨테이너로 완벽히 격리하는 것입니다.
* **데이터 영속성:** 소스 코드는 반드시 Host OS(`~/robot_ws`)에 `git clone` 및 작성하고, 이를 `--volume` 옵션으로 컨테이너에 연결해야 합니다. 컨테이너 내부에 코드를 직접 저장하면 컨테이너 삭제 시 코드가 유실됩니다.
* **네트워크:** `network="host"` 모드는 컨테이너가 Host의 모든 물리적 네트워크(LAN, Wi-Fi)를 공유하게 해, 로봇(LAN)과 모바일 로봇(Wi-Fi)에 동시 접속하는 환경에 필수적입니다.
* **Docker-in-Docker:** 컨테이너 내부에서 `docker` 명령(예: 에뮬레이터 설치)을 사용해야 할 경우, 컨테이너 생성 시 `-v /var/run/docker.sock:/var/run/docker.sock` 옵션으로 Host의 Docker 소켓을 마운트하고, 컨테이너 내부에 `docker-ce-cli`를 설치해야 합니다.
* **컨테이너 환경:** Docker 컨테이너에 `exec`로 접속하는 새 터미널은 항상 독립된 환경입니다. 커스텀 패키지(`doosan_helper`)의 노드나 메시지를 사용하려면 해당 터미널에서 `source /root/catkin_ws/devel/setup.bash`를 매번 실행하여 환경을 활성화해야 합니다.
* **파일 권한:** 컨테이너 내부(`root`)에서 생성된 파일/폴더는 Host에서도 `root` 소유가 됩니다. Host 사용자(`katech`)가 VS Code 등으로 해당 파일을 수정하려면 Host에서 `sudo chown -R $USER:$USER [폴더 경로]` 명령어로 소유권을 변경해야 합니다.

---

## 📝 **예제: 토픽 메시지로 로봇 제어하기**

아래는 `/katech/robot_command` 토픽에 `1` 또는 `0` 메시지를 보내 두 가지 다른 자세로 로봇을 움직이는 예제입니다. **모든 명령어는 "ROS 작업실"(Docker 컨테이너) 내부에서 실행해야 합니다.**

**준비:** 총 3개의 터미널 창이 필요합니다. 각 창에서 NUC(Host) 터미널을 열고 아래 명령어로 "ROS 작업실"에 접속합니다.

```bash
docker exec -it my_noetic_ws bash
```

**터미널 1: 두산 로봇 연결**
(로봇 제어 드라이버 및 MoveIt 실행)

```bash
# (작업실 터미널 1)
roslaunch dsr_launcher single_robot.launch model:=a0912 mode:=real host:=192.168.137.100
```
**터미널 2: 명령 수신 노드 실행 (토픽 메시지를 받아 로봇 제어 서비스를 호출하는 노드)**
```bash
# (작업실 터미널 2)
# 먼저 환경 활성화
source /root/catkin_ws/devel/setup.bash

# 노드 실행 (1 또는 0 메시지 대기)
rosrun doosan_helper move_robot_node
```
**터미널 3: 명령 송신 (테스트) (토픽 메시지를 발행하여 로봇을 움직이는 테스트)**

- 자세 1 ([90, 0, 90, 0, 90, -90]) 실행:
```bash
# (작업실 터미널 3)
# 먼저 환경 활성화
source /root/catkin_ws/devel/setup.bash

# 토픽 발행 (메시지: 1)
rosrun doosan_helper trigger_zero_node
```
- 자세 2 ([-90, 0, 90, 0, 90, -90]) 실행:
```bash
# (작업실 터미널 3)
# 먼저 환경 활성화
source /root/catkin_ws/devel/setup.bash

# 토픽 발행 노드 실행 (메시지: 0)
rosrun doosan_helper trigger_one_node
```

- 자세 3 ([0, 0, 0, 0, 0, 0]) 실행:
```bash
# (작업실 터미널 3)
# 먼저 환경 활성화
source /root/catkin_ws/devel/setup.bash

# 토픽 발행 노드 실행 (메시지: 0)
rosrun doosan_helper trigger_home_node
```

결과: 터미널 3에서 명령을 실행할 때마다, 터미널 2에 해당 로그가 출력되고 실제 로봇이 지정된 자세로 이동합니다.

## 💡 커스텀 노드 설정 프로세스 (요약)**

위 "예제" 섹션에서 사용된 move_robot_node 및 trigger_zero_node는 다음과 같은 과정으로 설정되었습니다.

**1. 패키지 생성 (컨테이너 내부):**

```bash
# /root/catkin_ws/src 로 이동 후 실행
catkin_create_pkg doosan_helper roscpp std_msgs dsr_msgs
```
**2. 소스 코드 작성 (Host VS Code):**

- ~/robot_ws/src/doosan_helper/src/move_robot_node.cpp: /katech/robot_command 토픽을 구독(subscribe)하고, 수신된 값(1 또는 0)에 따라 /dsr01a0912/motion/move_joint 서비스를 호출하여 로봇 자세를 변경합니다.

- ~/robot_ws/src/doosan_helper/src/trigger_zero_node.cpp: /katech/robot_command 토픽에 0 값을 발행(publish)하는 간단한 노드입니다.

**3. 빌드 설정 (Host VS Code):**

- ~/robot_ws/src/doosan_helper/CMakeLists.txt 파일 하단에 add_executable(...) 및 target_link_libraries(...) 구문을 추가하여 두 개의 C++ 노드를 빌드 시스템에 등록합니다.

**4. 권한 변경 (Host 터미널): 컨테이너:** 
- (root)가 생성한 파일/폴더에 Host 사용자(katech)가 접근할 수 있도록 소유권을 변경합니다.

```bash
sudo chown -R $USER:$USER ~/robot_ws
```
**5. 빌드 (컨테이너 내부):**

```bash
# /root/catkin_ws 로 이동 후 실행
catkin_make
source devel/setup.bash
```
