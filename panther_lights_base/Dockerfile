FROM ros:noetic-ros-core

ENV  ROS_WS husarion_ws
ENV PANTHER_LIGHTS_CONTROLLER NONE

# Use bash instead of sh
SHELL ["/bin/bash", "-c"]

# Update Ubuntu Software repository
RUN apt update && \
    apt install -y

RUN apt install -y git \
    python3-dev \
    python3-pip \
    python3-rospkg

# Python 3 dependencies
RUN pip3 install \
        rosdep \
        rospkg \
        numpy \
        imageio

# Create and initialise ROS workspace
RUN mkdir -p /$ROS_WS/src
COPY ./panther_lights /$ROS_WS/src/panther_lights
RUN chmod +x /$ROS_WS/src/panther_lights/scripts/lights_node.py
WORKDIR /$ROS_WS
RUN mkdir build && \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    rosdep init && \
    rosdep update && \
    catkin_make install

WORKDIR /

# Clear 
RUN apt clean && \
    rm -rf /var/lib/apt/lists/* 
