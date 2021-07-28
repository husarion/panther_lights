FROM panther_lights_controller:base

ENV PANTHER_LIGHTS_CONTROLLER PLT_GUI

RUN apt update && \
    apt install -y  python3-tk && \
    pip3 install matplotlib

WORKDIR /$ROS_WS
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    catkin_make

# Clear 
RUN apt clean && \
    rm -rf /var/lib/apt/lists/* 
    
WORKDIR /
