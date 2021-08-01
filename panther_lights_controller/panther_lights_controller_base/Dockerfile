FROM ros:noetic-ros-core

ENV PANTHER_LIGHTS_CONTROLLER NONE

# Use bash instead of sh
SHELL ["/bin/bash", "-c"]

# Update Ubuntu Software repository
RUN apt update && \
    apt install -y

RUN apt install -y git \
    python3-dev \
    python3-pip \
    python3-rospkg \
    python3-pil

# Python 3 dependencies
RUN pip3 install \
        rosdep \
        rospkg \
        numpy \
        imageio

# Create and initialise ROS workspace
RUN mkdir -p /ros_ws/src
COPY ./panther_lights_controller /ros_ws/src/panther_lights_controller
RUN chmod +x /ros_ws/src/panther_lights_controller/src/panther_lights_controller/controller_node.py
WORKDIR /ros_ws
RUN git clone https://github.com/husarion/husarion_msgs.git --branch dev src/husarion_msgs && \
    mkdir build && \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    rosdep init && \
    rosdep update && \
    catkin_make install

# Run tests
RUN python3 /ros_ws/src/panther_lights_controller/tests/config_import_test.py

WORKDIR /

# Clear 
RUN apt clean && \
    rm -rf /var/lib/apt/lists/* 

COPY ./ros_entrypoint.sh /
