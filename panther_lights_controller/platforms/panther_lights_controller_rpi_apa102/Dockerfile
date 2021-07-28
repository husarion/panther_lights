FROM husarion/panther-lights-controller:base

ENV PANTHER_LIGHTS_CONTROLLER APA102

RUN pip3 install RPi.GPIO

WORKDIR /$ROS_WS/src
RUN git clone https://github.com/byq77/apa102-pi.git && \
    cd apa102-pi && sudo python3 setup.py install

WORKDIR /$ROS_WS
RUN source /opt/ros/$ROS_DISTRO/setup.bash && \
    catkin_make

# Clear 
RUN apt clean && \
    rm -rf /var/lib/apt/lists/* 

WORKDIR /
