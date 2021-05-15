FROM ros:noetic-ros-core

ENV  ROS_WS husarion_ws

# Use bash instead of sh
RUN rm /bin/sh && ln -s /bin/bash /bin/sh

# Update Ubuntu Software repository
RUN apt-get update && \
    apt-get install -y -qq apt-utils

RUN apt-get install -y git \
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
RUN chmod +x /$ROS_WS/src/panther_lights/scripts/lights_node
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
