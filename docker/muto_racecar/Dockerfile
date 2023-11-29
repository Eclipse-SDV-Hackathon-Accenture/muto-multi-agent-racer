FROM ghcr.io/eclipse-muto/muto-barebones:humble

WORKDIR /home/muto/
COPY /racecar_ws/src /home/muto/src
RUN source /opt/ros/humble/setup.bash && \
    rosdep install -i --from-path src --rosdistro humble -y && \
    colcon build
CMD /bin/bash -c "source install/setup.bash && ros2 launch ./launch/muto.launch.py"
