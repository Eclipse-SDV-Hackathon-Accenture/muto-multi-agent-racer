FROM ghcr.io/eclipse-muto/muto-barebones:humble

WORKDIR /home/muto/

RUN git clone https://github.com/Eclipse-SDV-Hackathon-Accenture/muto-multi-agent-racer.git
RUN cp -r muto-multi-agent-racer/src/racecar ./src
RUN rm -rf muto-multi-agent-racer


RUN source /opt/ros/humble/setup.bash && \
    rosdep install -i --from-path src --rosdistro humble -y && \
    colcon build
CMD /bin/bash -c "source install/setup.bash && ros2 launch ./launch/muto.launch.py"
