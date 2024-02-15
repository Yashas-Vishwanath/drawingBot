ARG ROS_DISTRO=noetic

FROM ros:$ROS_DISTRO-ros-base as base

ARG ROS_DISTRO

ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV DEBIAN_FRONTEND=noninteractive
ENV TERM xterm-256color

RUN apt-get update && apt-get install -y --no-install-recommends\
    ssh \
    git \
    curl \
    wget \
    nano \
    build-essential \
    cmake \
    python3-pip \
    python3-flake8 \
    python-is-python3 \
    tmux \
    terminator \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir --upgrade pip\
    argcomplete \
    flake8-blind-except \
    flake8-builtins \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-return \
    flake8-length \
    flake8-todo \
    flake8-quotes \
    black \
    mypy \
    prospector \
    pylint \
    numpy \
    opencv-python \
    opencv-contrib-python \
    pyquaternion \
    python-dateutil \
    matplotlib \
    pyyaml \
    pytransform3d \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends\
    pkg-config \
    python3-catkin-tools \
    python3-rosdep \
    python3-rosinstall-generator \
    python3-vcstool \
    python3-pykdl \
    ros-$ROS_DISTRO-rqt \
    ros-$ROS_DISTRO-rqt-common-plugins \
    ros-$ROS_DISTRO-rqt-robot-plugins \
    ros-$ROS_DISTRO-rqt-joint-trajectory-controller \   
    ros-$ROS_DISTRO-roslint \
    ros-$ROS_DISTRO-rqt-gui \
    ros-$ROS_DISTRO-rqt-gui-py \
    ros-$ROS_DISTRO-rqt-py-common \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

WORKDIR /
RUN mkdir -p dev_ws/src


## for Astra
FROM base as astra
WORKDIR /
RUN apt-get update && apt-get install -y --no-install-recommends\
    libgoogle-glog-dev \
    libusb-1.0-0-dev \
    libeigen3-dev \   
    libgflags-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends\
    libgflags-dev \
    ros-$ROS_DISTRO-image-geometry \
    ros-$ROS_DISTRO-camera-info-manager \
    ros-$ROS_DISTRO-image-transport \
    ros-$ROS_DISTRO-image-publisher \
    ros-$ROS_DISTRO-compressed-image-transport \
    ros-$ROS_DISTRO-compressed-depth-image-transport \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-rgbd-launch \
    ros-$ROS_DISTRO-joint-state-publisher \
    ros-$ROS_DISTRO-robot-state-publisher \
    libgoogle-glog-dev \
    libusb-1.0-0-dev \
    libeigen3-dev \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN ["/bin/bash", "-c", "git clone https://github.com/libuvc/libuvc.git &&\
    cd libuvc &&\
    mkdir build && cd build &&\
    cmake .. && make -j4 &&\
    sudo make install &&\
    sudo ldconfig"]

WORKDIR /dev_ws/src
RUN ["/bin/bash", "-c", "git clone https://github.com/orbbec/ros_astra_camera.git"]



## for TurtleBot and lidar
FROM astra as tbot
WORKDIR /
RUN apt-get update && apt-get install -y --no-install-recommends\
    ros-$ROS_DISTRO-rviz \
    ros-$ROS_DISTRO-rosbash \
    ros-$ROS_DISTRO-rqt-action \
    ros-$ROS_DISTRO-rqt-console \
    ros-$ROS_DISTRO-rqt-graph \
    ros-$ROS_DISTRO-rqt-topic \
    ros-$ROS_DISTRO-rqt-tf-tree \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends\
    ros-$ROS_DISTRO-joy \
    ros-$ROS_DISTRO-openni2-launch \
    ros-$ROS_DISTRO-gmapping \
    ros-$ROS_DISTRO-slam-gmapping \
    ros-$ROS_DISTRO-move-base \
    ros-$ROS_DISTRO-amcl \
    ros-$ROS_DISTRO-navigation \
    ros-$ROS_DISTRO-map-server \
    ros-$ROS_DISTRO-kobuki-driver \
    ros-$ROS_DISTRO-kobuki-ftdi \
    ros-$ROS_DISTRO-common-msgs \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# from vince docker file for lidar
RUN apt-get update && apt-get install -y --no-install-recommends\
    ros-$ROS_DISTRO-ecl-threads \
    ros-$ROS_DISTRO-ecl-geometry \
    ros-$ROS_DISTRO-ecl-streams \
    ros-$ROS_DISTRO-kdl-conversions \
    ros-$ROS_DISTRO-usb-cam \
    ros-$ROS_DISTRO-rplidar-ros \
    ros-$ROS_DISTRO-diagnostics \
    ros-$ROS_DISTRO-kobuki-core \
    ros-$ROS_DISTRO-kobuki-dock-drive \
    ros-$ROS_DISTRO-image-transport-plugins \
    ros-$ROS_DISTRO-theora-image-transport \
    ros-$ROS_DISTRO-rosbridge-suite \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# WORKDIR /dev_ws/src
# git repos/packages installed in the workspace outside the docker


# ## for Lidar
# git repos/packages installed in the workspace outside the docker


# FROM tbot as open3d

# RUN pip3 install --no-cache-dir --upgrade open3d

FROM tbot as build

COPY . /dev_ws/src/

WORKDIR /dev_ws

# RUN rosdep init
RUN apt-get update
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src -r -y
RUN apt-get clean && rm -rf /var/lib/apt/lists/*


RUN ["/bin/bash", "-c", "source /opt/ros/$ROS_DISTRO/setup.bash &&\
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release &&\
    catkin build \
    "]

FROM build as bash

RUN ["/bin/bash", "-c", "cp /dev_ws/src/.docker/setup.bash /dev_ws/setup.bash && chmod 777 /dev_ws/setup.bash"]
RUN ["/bin/bash", "-c", "cp /dev_ws/src/.docker/entrypoint.bash / && chmod 777 /entrypoint.bash"]
ENTRYPOINT ["bash", "/entrypoint.bash" ]

CMD ["bash"]
