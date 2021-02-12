##############################################################################
##                                 Base Image                               ##
##############################################################################
ARG BASE_IMAGE=ubuntu:18.04
FROM ${BASE_IMAGE}
ARG DEBIAN_FRONTEND=noninteractive

##############################################################################
##                                 ROS2 Install                             ##
##############################################################################
# Set locate
RUN apt update && apt install -y \
    language-pack-ja-base \
    language-pack-ja \
    && apt clean \
    && rm -rf /var/lib/apt/lists/*
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8

# Add the ROS 2 apt repository
RUN apt update && apt install -y \
    curl \
    gnupg2 \
    lsb-release
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add -
RUN sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu `lsb_release -cs` main" > /etc/apt/sources.list.d/ros2-latest.list'

# ROS2 distro
ARG ROS_DISTRO=dashing
ENV ROS_DISTRO ${ROS_DISTRO}


# Install ROS2 packages
RUN apt update && apt install -y \
    ros-${ROS_DISTRO}-desktop \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-argcomplete \
    && apt clean \
    && rm -rf /var/lib/apt/lists/*
RUN rosdep init && rosdep update

# set entrypoint (source /opt/ros/${ROS_DISTRO}/setup.bash)
COPY ros_entrypoint.sh /
RUN chmod +x ros_entrypoint.sh
ENTRYPOINT ["/ros/ros_entrypoint.sh"]
CMD ["bash"]

##############################################################################
##                           Make ROS Workspace                             ##
##############################################################################
RUN mkdir -p /ros2_ws/src

WORKDIR /ros2_ws/src
COPY . ./robot_interface_eki

RUN git clone -b dev cpp_core

WORKDIR /ros2_ws
RUN	/bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash; colcon build --symlink-install; source /ros2_ws/install/setup.bash"

ENV LD_LIBRARY_PATH=/opt/ros/${ROS_DISTRO}/lib:/ros2_ws/build/robot_interface_eki/lib

WORKDIR /ros2_ws/src/robot_interface_eki/ros
RUN chmod -R +x scripts

WORKDIR /ros2_ws