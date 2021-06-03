# FROM ros:noetic-ros-core
FROM osrf/ros:noetic-desktop-full

ENV  ROS_WS husarion_ws

# Use bash instead of sh
SHELL ["/bin/bash", "-c"]

# Update Ubuntu Software repository
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - && \
    rm -rf /etc/ros/rosdep/sources.list.d/20-default.list && \
    apt update && \
    apt install -y -qq apt-utils

RUN apt install -y git \
    python3-dev \
    python3-pip \
    python3-rospkg \
    python3-tk

# Python 3 dependencies
RUN pip3 install \
        rosdep \
        rospkg \
        numpy \
        matplotlib

# Create and initialise ROS workspace
RUN mkdir -p /$ROS_WS/src
COPY ./panther_lights /$ROS_WS/src/panther_lights
RUN chmod +x /$ROS_WS/src/panther_lights/scripts/lights_node.py
WORKDIR /$ROS_WS
RUN mkdir build && \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    rosdep init && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y && \
    catkin_make install

WORKDIR /

# Clear 
RUN apt clean && \
    rm -rf /var/lib/apt/lists/* 

RUN echo "source \"/$ROS_WS/devel/setup.bash\"" >> ~/.bashrc
