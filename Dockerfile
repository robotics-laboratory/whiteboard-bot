ARG ARCH=arm64v8

FROM ${ARCH}/ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive
ENV SHELL /bin/bash
SHELL ["/bin/bash", "-c"]

ENV ROS_VERSION=2
ENV ROS_DISTRO=galactic
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO}
ENV ROS_PYTHON_VERSION=3

# INSTALL COMMON

RUN apt-get update -q && \
    apt-get install -yq --no-install-recommends \
        apt-transport-https \
        apt-utils \
        ca-certificates \
        git \
        libpython3-dev \
        lsb-release \
        locales \
        software-properties-common \
        python3 \
        python3-dev \
        python3-distutils \
        python3-pip \
        python3-setuptools \
        tar \
        wget \
    && rm -rf /var/lib/apt/lists/* && apt-get clean

### INSTALL OPENCV

RUN apt-get update -q && \
    apt-get install -yq --no-install-recommends \
        libopencv-dev \
        libopencv-contrib-dev \
    && rm -rf /var/lib/apt/lists/* && apt-get clean

### INSTALL ROS2

RUN wget -q https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

RUN apt-get update -q \
    && apt-get install -yq --no-install-recommends \
        libjansson-dev \
        libasio-dev \
        libboost-dev \
        libtinyxml-dev \
        python3-bson \
        python3-colcon-common-extensions \
        python3-flake8 \
        python3-numpy \
        python3-pytest-cov \
        python3-rosdep \
        python3-vcstool \
        python3-rosinstall-generator \
        libtinyxml2-dev \
        libcunit1-dev \
    && rm -rf /var/lib/apt/lists/* && apt-get clean

RUN apt-get update -q \
    && apt install -yq --no-install-recommends \
        ros-${ROS_DISTRO}-ament-lint \
        ros-${ROS_DISTRO}-cv-bridge \
        ros-${ROS_DISTRO}-foxglove-msgs \
        ros-${ROS_DISTRO}-image-common \
        ros-${ROS_DISTRO}-image-geometry \
        ros-${ROS_DISTRO}-image-pipeline \
        ros-${ROS_DISTRO}-joy \
        ros-${ROS_DISTRO}-ros-base \
        ros-${ROS_DISTRO}-rosbridge-suite \
        ros-${ROS_DISTRO}-v4l2-camera \
        ros-${ROS_DISTRO}-vision-opencv \
        ros-${ROS_DISTRO}-visualization-msgs \
    && rm -rf /var/lib/apt/lists/* && apt-get clean

RUN printf "export ROS_ROOT=${ROS_ROOT}\n" >> /root/.bashrc \
    && printf "export ROS_DISTRO=${ROS_DISTRO}\n" >> /root/.bashrc \
    && printf "source ${ROS_ROOT}/setup.bash\n" >> /root/.bashrc

### INSTALL DEV PKGS

ENV CLANG_VERSION=12

ENV CC=/usr/bin/clang-${CLANG_VERSION}
ENV CXX=/usr/bin/clang++-${CLANG_VERSION}

RUN apt-get update -q \
    && apt-get install -yq --no-install-recommends \
        clang-${CLANG_VERSION} \
        clang-format-${CLANG_VERSION} \
        clang-tidy-${CLANG_VERSION} \
        cmake \
        htop \
        less \
        libboost-dev \
        lldb-${CLANG_VERSION} \
        make \
        tmux \
        ssh \
        vim \
    && rm -rf /var/lib/apt/lists/* && apt-get clean

RUN printf "export CC=/usr/bin/clang-${CLANG_VERSION}\n" >> /root/.bashrc \
    && printf "export CXX=/usr/bin/clang++-${CLANG_VERSION}\n" >> /root/.bashrc \
    && printf "PermitRootLogin yes\nPort 2222" >> /etc/ssh/sshd_config \
    && echo 'root:root' | chpasswd

### SETUP ENTRYPOINT

COPY /entrypoint.bash /entrypoint.bash
ENTRYPOINT ["/entrypoint.bash"]
WORKDIR /wbb