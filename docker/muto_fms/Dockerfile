FROM  ghcr.io/eclipse-muto/ros2:humble

SHELL ["/bin/bash", "-c"]

RUN pip3 install kuksa-client==0.4.1
RUN mkdir -p /home/muto
WORKDIR /home/muto
COPY src/kuksa_ros_provider ./src/kuksa_ros_provider

RUN source /opt/ros/humble/setup.bash && \
    rosdep install -i --from-path src --rosdistro humble -y && \
    colcon build

CMD /bin/bash -c "source install/setup.bash && ros2 launch kuksa_ros_provider kuksa.launch.py"