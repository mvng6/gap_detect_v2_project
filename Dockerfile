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

##############################################################
##############################################################
# Zivid SDK (Ubuntu 20.04)
###############################
###############################
# OpenCL (CPU, pocl)
###############################
RUN apt-get update && apt-get install -y --no-install-recommends \
    ocl-icd-libopencl1 \
    pocl-opencl-icd \
    clinfo \
 && rm -rf /var/lib/apt/lists/*
###############################
# Zivid CPU-only 설정
###############################
RUN mkdir -p /root/.config/Zivid/API && \
    printf "__version__:\n    serializer: 1\n    data: 19\nConfiguration:\n    ComputeDevice:\n        Type: CPU\n        AllowUnsupported: yes\n" \
    > /root/.config/Zivid/API/Config.yml

ARG ZIVID_VERSION=2.17.1+7516d437-1

RUN set -eux; \
    mkdir -p /tmp/Zivid && cd /tmp/Zivid && \
    wget -q \
      https://downloads.zivid.com/sdk/releases/${ZIVID_VERSION}/u20/amd64/zivid_${ZIVID_VERSION}_amd64.deb \
      https://downloads.zivid.com/sdk/releases/${ZIVID_VERSION}/u20/amd64/zivid-studio_${ZIVID_VERSION}_amd64.deb \
      https://downloads.zivid.com/sdk/releases/${ZIVID_VERSION}/u20/amd64/zivid-tools_${ZIVID_VERSION}_amd64.deb \
      https://downloads.zivid.com/sdk/releases/${ZIVID_VERSION}/u20/amd64/zivid-genicam_${ZIVID_VERSION}_amd64.deb \
    && apt-get update \
    && apt-get install -y ./zivid_${ZIVID_VERSION}_amd64.deb \
                          ./zivid-studio_${ZIVID_VERSION}_amd64.deb \
                          ./zivid-tools_${ZIVID_VERSION}_amd64.deb \
                          ./zivid-genicam_${ZIVID_VERSION}_amd64.deb \
    && rm -rf /var/lib/apt/lists/* /tmp/Zivid
# 4.x. Python3.8용 pip, venv 설치 + 기본 패키지 설치
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-venv \
    python3-dev \
    && rm -rf /var/lib/apt/lists/*

# 4.y. Python 패키지 설치 (zivid, opencv, numpy)
#     (주의: 여기서는 시스템 python3.8에 설치합니다)
RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install --no-cache-dir \
        zivid \
        opencv-python \
        numpy
######################################################################
##############################################################



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
