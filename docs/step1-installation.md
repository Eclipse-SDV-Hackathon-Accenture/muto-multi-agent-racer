# Hack Challenge "ROS Racers"

# F1TENTH gym environment ROS2 communication bridge
This is a ROS communication bridge inspired from [f1tenth_gym_ros](https://github.com/f1tenth/f1tenth_gym_ros) with multiagent support (up to 3) for the F1TENTH gym environment that turns it into a ROS2 simulation. The project is primarily intended for ease of use with [Eclipse Muto](https://projects.eclipse.org/projects/automotive.muto) which is an adaptive framework and a runtime platform for dynamically composable model-driven software stacks for ROS

# Installation Methods

### [Docker](#docker-1)
Supported almost on all platforms and you do not need to compile anything so it's the fastest method to get started.
#### Prerequisites
- [Docker](https://docs.docker.com/engine/install/)
- [Docker Compose](https://docs.docker.com/compose/install/)

### [Native on Ubuntu](#native-on-ubuntu-1)
You can use this method if you're using Ubuntu and don't want to use Docker for some reason
- Tested on:
  + Ubuntu 20.04 with ROS2 Foxy
  + Ubuntu 22.04 with ROS2 Humble

# Docker

### ROS2 Image
- Base image `ros:humble`
- Include deps to run muto safely:
    - ros-humble-rviz2
    - ros-humble-xacro
    - ros-humble-ackermann-msgs
    - ros-humble-joy
    - ros-humble-rosbridge-server 
- Tagged as `ghcr.io/eclipse-muto/ros2:humble`

### Step-1: Muto Multi Agent Simulation
```sh
docker run --rm -ti ghcr.io/eclipse-muto/multi-agent:humble
```
- Base image `ghcr.io/eclipse-muto/ros2:humble`
- Includes simulation ingredients, inherits following components from [F1tenth](https://f1tenth.org):
    - `f1tenth_gym_ros`: Simulation frontend
    - `f1tenth_gym`: Simulation backend
- Tagged as `ghcr.io/eclipse-muto/multi-agent:humble`.


### Step-2: Muto Racecar
- Base image `ghcr.io/eclipse-muto/muto-barebones:humble`
- Includes (on top of Muto orchestration) following ROS2 packages:
    - `racecar`
    - `reactive_gap_follower`
- Tagged as `ghcr.io/eclipse-muto/racecar:humble`
- Run with default [Eclipse Muto config](../src/racecar/racecar/config/muto.yaml):

```sh
docker run --rm -ti ghcr.io/eclipse-muto:racecar:humble
```
- To override the default configuration, adjust the contents of the [Eclipse Muto config](../src/racecar/racecar/config/muto.yaml) and run the below command:
```sh
docker run --rm -it \
    -v $(pwd)/docker/muto/launch/config/muto.yaml:/home/muto/launch/config/muto.yaml \
    ghcr.io/eclipse-muto/racecar:humble \
    /bin/bash -c "source install/setup.bash && ros2 launch ./launch/muto.launch.py"
```


## Native on Ubuntu

**Clone this Repo**
```bash
git clone --recurse-submodules https://github.com/Eclipse-SDV-Hackathon-Accenture/muto-multi-agent-racer.git
```

**Install the following dependencies:**
- **ROS 2** Follow the instructions [here](https://docs.ros.org/en/foxy/Installation.html) to install ROS 2 Foxy.
- **ROS 2** Follow the instructions [here](https://docs.ros.org/en/humble/Installation.html) to install ROS 2 Humble.
- **F1TENTH Gym**
  ```bash
  cd $HOME
  git clone https://github.com/f1tenth/f1tenth_gym
  cd f1tenth_gym && pip3 install -e .
  ```

**Installing the simulation:**

- The simulation and an example race algorithm lives under this repository. To clone Muto to this workspace's src folder easily, you can use [vcstool](https://github.com/dirk-thomas/vcstool): 
```bash
cd ~/path_to_ws
vcs import src < muto.repos
```

or if you don't have vcstool installed on your system, 
```bash
cd ~/path_to_ws/src
git clone https://github.com/eclipse-muto/composer
git clone https://github.com/eclipse-muto/core
git clone https://github.com/eclipse-muto/agent
git clone https://github.com/eclipse-muto/messages
```

- Install dependencies with rosdep then build:
  ```bash
  source /opt/ros/<your-ros-distro>/setup.bash
  cd ~/path_to_ws
  rosdep install --from-path src --ignore-src -r -y --rosdistro ${ROS_DISTRO}
  colcon build --symlink-install
  ```

# Next steps
- Head to [Getting started](./step2-getting-started-with-f1tenth-gym.md) if installation was successful. If not, head to [Troubleshooting](./troubleshooting.md)