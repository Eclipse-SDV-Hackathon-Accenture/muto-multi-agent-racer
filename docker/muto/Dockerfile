FROM  ghcr.io/eclipse-muto/ros2:humble

SHELL ["/bin/bash", "-c"]
RUN mkdir -p /home/muto/src
COPY launch /home/muto/launch
WORKDIR /home/muto
# TODO: REPLACE WITH MUTO GITHUB
RUN git clone https://github.com/eclipse-muto/agent.git src/agent
RUN git clone https://github.com/eclipse-muto/composer.git src/composer
RUN git clone https://github.com/eclipse-muto/core.git src/core
RUN git clone https://github.com/eclipse-muto/messages.git src/messages

RUN rosdep install -r -y --from-paths src --ignore-src
RUN source /opt/ros/humble/setup.bash && colcon build


