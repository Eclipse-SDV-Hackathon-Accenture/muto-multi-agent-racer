FROM ros:humble

SHELL ["/bin/bash", "-c"]

RUN apt-get update
RUN apt-get install -y git \
    python3-pip \
    ros-humble-rviz2 \
    ros-humble-xacro \
    ros-humble-ackermann-msgs \
    ros-humble-joy \
    ros-humble-rosbridge-server \
    ros-humble-demo-nodes-cpp

RUN pip3 install requests paho-mqtt transforms3d setuptools==58.2.0
RUN apt-get auto-clean

