FROM ros:humble

SHELL ["/bin/bash", "-c"]

# dependencies
RUN apt-get update --fix-missing && \
    apt-get install -y git \
                       nano \
                       vim \
                       python3-pip \
                       libeigen3-dev \
                       tmux \
                       ros-humble-rviz2 \
                       ros-humble-xacro \
                       ros-humble-ackermann-msgs \
                       ros-humble-joy

RUN apt-get -y dist-upgrade
RUN pip3 install transforms3d requests paho-mqtt
RUN pip3 install setuptools==58.2.0

RUN mkdir -p racecar_ws/src/racecar
COPY ./src/racecar /racecar_ws/src/racecar
COPY ./src/agent /racecar_ws/src/agent
COPY ./src/core /racecar_ws/src/core
COPY ./src/composer /racecar_ws/src/composer
COPY ./src/messages /racecar_ws/src/messages
COPY ./src/reactive_gap_follower /racecar_ws/src/reactive_gap_follower
COPY ./launch_racecar.sh /racecar_ws/src/launch_racecar.sh

RUN chmod +x /racecar_ws/src/launch_racecar.sh
RUN source /opt/ros/humble/setup.bash && \
    cd racecar_ws/ && \
    apt-get update --fix-missing && \
    rosdep install -i --from-path src --rosdistro humble -y && \
    colcon build

WORKDIR '/racecar_ws'