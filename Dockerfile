##############################################################################
##                                 Base Image                               ##
##############################################################################
ARG ROS_DISTRO=dashing
FROM ros:$ROS_DISTRO-ros-base
ENV TZ=Europe/Berlin
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone
RUN rosdep update --rosdistro $ROS_DISTRO

##############################################################################
##                                 Global Dependecies                       ##
##############################################################################
RUN apt update && apt install -y \
    ros-$ROS_DISTRO-ament-* \
    python3-colcon-common-extensions \
    python3-argcomplete \
    net-tools \
    iputils-ping \
    && apt clean && rm -rf /var/lib/apt/lists/*

##############################################################################
##                                 Create User                              ##
##############################################################################
ARG USER=docker
ARG PASSWORD=docker
ARG UID=1000
ARG GID=1000
ENV UID=$UID
ENV GID=$GID
ENV USER=$USER
RUN groupadd -g "$GID" "$USER"  && \
    useradd -m -u "$UID" -g "$GID" --shell $(which bash) "$USER" -G sudo && \
    echo "$USER:$PASSWORD" | chpasswd && \
    echo "%sudo ALL=(ALL) NOPASSWD: ALL" > /etc/sudoers.d/sudogrp && \
    chmod 0440 /etc/sudoers.d/sudogrp && \
    chown ${UID}:${GID} -R /home/${USER}
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /etc/bash.bashrc

USER $USER 
RUN mkdir -p /home/$USER/ros2_ws/src

##############################################################################
##                                 User Dependecies                         ##
##############################################################################
WORKDIR /home/$USER/ros2_ws/src
RUN git config --global advice.detachedHead false

COPY . ./robot_interface_eki

# ENV LD_LIBRARY_PATH=/opt/ros/${ROS_DISTRO}/lib:/home/$USER/ros2_ws/build/robot_interface_eki/lib

##############################################################################
##                                 Build ROS and run                        ##
##############################################################################
WORKDIR /home/$USER/ros2_ws
RUN rosdep update --rosdistro $ROS_DISTRO
RUN rosdep install --from-paths src --ignore-src -y
RUN . /opt/ros/$ROS_DISTRO/setup.sh && colcon build --symlink-install
RUN echo "source /home/$USER/ros2_ws/install/setup.bash" >> /home/$USER/.bashrc

RUN sudo sed --in-place --expression \
    '$isource "/home/$USER/ros2_ws/install/setup.bash"' \
    /ros_entrypoint.sh

CMD /bin/bash

# RUN chmod -R +x scripts