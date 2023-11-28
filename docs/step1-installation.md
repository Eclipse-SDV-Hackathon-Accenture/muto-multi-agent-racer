# Hack Challenge "ROS Racers"

## Template Setup

How To for getting the template up and running

# F1TENTH gym environment ROS2 communication bridge
This is a ROS communication bridge inspired from [f1tenth_gym_ros](https://github.com/f1tenth/f1tenth_gym_ros) with multiagent support (up to 4) for the F1TENTH gym environment that turns it into a ROS2 simulation. The project is primarily intended for ease of use with [Eclipse Muto](https://projects.eclipse.org/projects/automotive.muto) which is an adaptive framework and a runtime platform for dynamically composable model-driven software stacks for ROS

# Installation

**Supported Systems:**

- Ubuntu 20.04 native with ROS2 Foxy
- Ubuntu 22.04 native with ROS2 Humble
- Docker (tested on Ubuntu 22.04)

## Native on Ubuntu

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

- The simulation and an example race algorithm lives under this repository. To clone Muto to this workspace's src folder, you can use [vcstool](https://github.com/dirk-thomas/vcstool): 
```bash
cd /path/to/this/repo
vcs import src < autoware.repos
```

- Install dependencies with rosdep then build:
  ```bash
  source /opt/ros/<your-ros-distro>/setup.bash
  cd ~/path_to_ws
  rosdep install --from-path src --ignore-src -r -y --rosdistro ${ROS_DISTRO}
  colcon build --symlink-install
  ```

- Now head to [Getting started](./step2-getting-started-with-f1tenth-gym.md) if installation was successful. If not, head to [Troubleshooting](./troubleshooting.md)