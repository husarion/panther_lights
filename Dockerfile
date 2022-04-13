FROM ros:noetic-ros-core

ENV PANTHER_LIGHTS_CONTROLLER APA102

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
RUN pip3 install --upgrade \
        rosdep \
        rospkg \
        numpy \
        imageio \
        RPi.GPIO

# Create and initialise ROS workspace
RUN mkdir -p /ros_ws/src
RUN git clone https://github.com/byq77/apa102-pi.git && \
    cd apa102-pi && sudo python3 setup.py install

# PROJECT SPECIFIC DEPENDENCIES
RUN apt install -y ros-noetic-actionlib \
    ros-noetic-actionlib-msgs   

COPY ./panther_lights /ros_ws/src/panther_lights

# RUN chmod +x /ros_ws/src/panther_lights_controller/src/panther_lights_controller/controller_node.py
WORKDIR /ros_ws
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    rosdep init && \
    rosdep update && \
    catkin_make install

# OLD
# RUN mkdir build && \
#     source /opt/ros/$ROS_DISTRO/setup.bash && \
#     rosdep init && \
#     rosdep update && \
#     catkin_make install

# Clear 
RUN apt clean && \
    rm -rf /var/lib/apt/lists/* 

COPY ./ros_entrypoint.sh / 
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
