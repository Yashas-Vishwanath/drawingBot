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
    build-essential \
    cmake \
    python3-pip \
    python3-flake8 \
    python-is-python3 \
    terminator \
    tmux \
    nano \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends\
    pkg-config \
    python3-catkin-tools \
    python3-rosdep \
    python3-rosinstall-generator \
    ros-$ROS_DISTRO-rqt \
    ros-$ROS_DISTRO-rqt-common-plugins \
    ros-$ROS_DISTRO-rqt-robot-plugins \
    ros-$ROS_DISTRO-roslint \
    ros-$ROS_DISTRO-rqt-gui \
    ros-$ROS_DISTRO-rqt-gui-py \
    ros-$ROS_DISTRO-rqt-py-common \
    ros-$ROS_DISTRO-usb-cam \
    ros-$ROS_DISTRO-rplidar-ros \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-ecl-threads \
    ros-$ROS_DISTRO-ecl-geometry \
    ros-$ROS_DISTRO-ecl-streams \
    ros-$ROS_DISTRO-kdl-conversions \
    ros-$ROS_DISTRO-image-geometry \
    ros-$ROS_DISTRO-diagnostics \
    ros-$ROS_DISTRO-robot-state-publisher \
    ros-$ROS_DISTRO-kobuki-driver \
    ros-$ROS_DISTRO-kobuki-ftdi \
    ros-$ROS_DISTRO-kobuki-core \
    ros-$ROS_DISTRO-kobuki-dock-drive \
    ros-$ROS_DISTRO-joy \
    ros-$ROS_DISTRO-gmapping \
    ros-$ROS_DISTRO-slam-gmapping \
    ros-$ROS_DISTRO-move-base \
    ros-$ROS_DISTRO-amcl \
    ros-$ROS_DISTRO-navigation \
    ros-$ROS_DISTRO-map-server \
    ros-$ROS_DISTRO-image-transport \
    ros-$ROS_DISTRO-image-transport-plugins \
    ros-$ROS_DISTRO-compressed-image-transport \
    ros-$ROS_DISTRO-theora-image-transport \
    ros-$ROS_DISTRO-rosbridge-suite \
    ros-$ROS_DISTRO-rosbridge-server \
    ros-$ROS_DISTRO-tf2-web-republisher \
    ros-$ROS_DISTRO-rosserial \
    ros-$ROS_DISTRO-rosserial-arduino \
    ros-$ROS_DISTRO-rosserial-python \
    ros-$ROS_DISTRO-tf2 \
    ros-$ROS_DISTRO-tf2-ros \
    ros-$ROS_DISTRO-robot-localization \
    && apt-get clean && rm -rf /var/lib/apt/lists/*

# Create a new user (replace "dockeruser" with your actual user)
#RUN useradd -m -s /bin/bash iaac
#RUN echo 'iaac:iaac' | chpasswd
#RUN adduser iaac sudo

# Allow the user to run chmod without a password
#RUN echo "iaac ALL=(ALL) NOPASSWD: /bin/chmod 666 /dev/ttyACM0" >> /etc/sudoers

# Switch to the new user
#USER iaac

# Copy the rest of your setup files and scripts here
WORKDIR /
RUN mkdir -p dev_ws/src

#RUN git clone -b $ROS_DISTRO-devel https://github.com/cra-ros-pkg/robot_localization.git /dev_ws/src/robot_localization


COPY . /dev_ws/src/

#WORKDIR /dev_ws/src
#RUN mkdir robot_localization
#RUN git clone -b $ROS_DISTRO-devel https://github.com/cra-ros-pkg/robot_localization.git

# Ensure the script is executable
RUN chmod +x /dev_ws/src/turtlebot/turtlebot_bringup/scripts/republish_transform.py
RUN chmod +x /dev_ws/src/turtlebot/turtlebot_bringup/scripts/path_recorder.py
RUN chmod +x /dev_ws/src/turtlebot/turtlebot_bringup/scripts/path_recorder2.py
#RUN chmod +x /dev_ws/src/turtlebot/turtlebot_bringup/scripts/setup_permissions.sh

WORKDIR /dev_ws
RUN ["/bin/bash", "-c", "source /opt/ros/$ROS_DISTRO/setup.bash &&\
    catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release &&\
    catkin build \
    "]

RUN ["/bin/bash", "-c", "cp /dev_ws/src/.docker/entrypoint.sh /entrypoint.sh && chmod 777 /entrypoint.sh"]
RUN ["/bin/bash", "-c", "cp /dev_ws/src/.docker/setup.bash /dev_ws/setup.bash && chmod 777 /dev_ws/setup.bash"]
ENTRYPOINT ["bash", "/entrypoint.sh" ]

# RUN chmod 666 /dev/ttyACM0

CMD ["bash"]

# # Set the working directory
# WORKDIR /dev_ws

# # Source ROS and build the workspace
# RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && \
#     catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release && \
#     catkin build"

# # Copy the entrypoint script and set permissions
# COPY /dev_ws/src/.docker/entrypoint.sh /entrypoint.sh
# RUN chmod +x /entrypoint.sh

# # Copy the setup script and set permissions
# COPY /dev_ws/src/.docker/setup.bash /dev_ws/setup.bash
# RUN chmod +x /dev_ws/setup.bash

# # Set the entrypoint
# ENTRYPOINT ["/entrypoint.sh"]

# # Default command
# CMD ["bash"]
