FROM  ghcr.io/eclipse-muto/ros2:humble

SHELL ["/bin/bash", "-c"]

# f1tenth gym
RUN mkdir -p /home/muto
WORKDIR /home/muto
RUN git clone https://github.com/f1tenth/f1tenth_gym
RUN cd f1tenth_gym && \
    pip3 install -e .

RUN git clone https://github.com/Eclipse-SDV-Hackathon-Accenture/muto-multi-agent-racer.git
RUN cp -r muto-multi-agent-racer/src/muto-multiagent-simulation ./sim_ws
RUN rm -rf muto-multi-agent-racer

WORKDIR /home/muto/sim_ws
RUN source /opt/ros/humble/setup.bash && \
    apt-get update --fix-missing && \
    rosdep install -i --from-path src --rosdistro humble -y && \
    colcon build

COPY /launch /home/muto/sim_ws/launch
RUN chmod +x launch/scripts/launch_sim.sh
EXPOSE 9090
CMD /bin/bash -c "/home/muto/sim_ws/launch/scripts/launch_sim.sh"