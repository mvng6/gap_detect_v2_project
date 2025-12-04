# 1. 베이스 이미지 선택 (우리가 pull 했던 것)
FROM ros:noetic-robot

# 2. 작업 디렉토리 설정 (선택사항)
WORKDIR /root/catkin_ws

# 3. 필요한 apt 패키지 설치 (우리가 수동으로 했던 것들)
#    - Docker-in-Docker용 CLI
#    - 두산 로봇 의존성
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates curl gnupg lsb-release \
    ros-noetic-rqt* \
    ros-noetic-moveit* \
    ros-noetic-gazebo-ros-control \
    ros-noetic-joint-state-controller \
    ros-noetic-effort-controllers \
    ros-noetic-position-controllers \
    ros-noetic-ros-controllers \
    ros-noetic-ros-control \
    ros-noetic-joint-state-publisher-gui \
    ros-noetic-joint-state-publisher \
    python3-catkin-tools \
    && rm -rf /var/lib/apt/lists/* # 이미지 용량 줄이기

# 4. Docker CLI 설치 (Docker-in-Docker용 "전화기")
RUN mkdir -p /etc/apt/keyrings \
    && curl -fsSL https://download.docker.com/linux/ubuntu/gpg | gpg --dearmor -o /etc/apt/keyrings/docker.gpg \
    && echo \
      "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
      focal stable" | tee /etc/apt/sources.list.d/docker.list > /dev/null \
    && apt-get update && apt-get install -y --no-install-recommends docker-ce-cli \
    && rm -rf /var/lib/apt/lists/*

# 5. rosdep 초기화 및 업데이트 (빌드 시 필요)
RUN rosdep init || echo "rosdep init already done." \
    && rosdep update

# 6. .bashrc에 기본 환경 설정 추가
RUN echo "source /opt/ros/noetic/setup.bash" >> /root/.bashrc
RUN echo "source /root/catkin_ws/devel/setup.bash" >> /root/.bashrc

# (주의!) 소스 코드는 여기에 복사하지 않습니다. Volume으로 연결할 것이기 때문입니다.
# (주의!) catkin_make도 여기서 실행하지 않습니다. 코드가 없기 때문입니다.

# 7. 컨테이너 시작 시 기본 명령어 설정 (선택사항, bash 실행)
CMD ["bash"]