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
- Clone the repo into your colcon workspace if you haven't already:
  ```bash
  cd ~/path_to_ws/src
  git clone https://gitlab.eteration.com/composiv/eclipse-muto/multi-agent/muto-multiagent-simulation.
  ```
- Update correct parameter for path to map file:
  Go to `sim.yaml` [/path_to_ws/src/f1tenth_gym_ros/config/sim.yaml](https://gitlab.eteration.com/composiv/eclipse-muto/multi-agent/muto-multiagent-simulation/-/blob/main/src/f1tenth_gym_ros/config/sim.yaml) in your cloned repo, change the `map_path` parameter to point to the correct location. It should be `~/path_to_ws/src/f1tenth_gym_ros/src/f1tenth_gym_ros/maps/levine'`. You could also use map of your choosing instead of levine. Make sure you specify the map image file extension via `map_img_ext`.  Don't specify the extension of yaml file on `map_path` parameter.

- Install dependencies with rosdep then build:
  ```bash
  source /opt/ros/<your-ros-distro>/setup.bash
  cd ~/path_to_ws
  rosdep install --from-path src --ignore-src -r -y --rosdistro ${ROS_DISTRO}
  colcon build --symlink-install
  ```

- Keep in mind that if you're using this repo in a remote computer, you're gonna need [display forwarding](https://unix.stackexchange.com/questions/12755/how-to-forward-x-over-ssh-to-run-graphics-applications-remotely) to see anything graphics related (Rviz in our case).

- Now head to [Quickstart](#quickstart) if installation was successful. If not, head to [Troubleshooting](#troubleshooting)

## With Docker:

**Install the following dependencies:**

- Follow the instructions [here](https://docs.docker.com/install/linux/docker-ce/ubuntu/) to install Docker. A short tutorial can be found [here](https://docs.docker.com/get-started/) if you're not familiar with Docker.

**Installing the simulation:**

1. Clone this repo
```bash
cd ~/path_to_ws/src
git clone https://gitlab.eteration.com/composiv/eclipse-muto/multi-agent/muto-multiagent-simulation.git 
```
2. To run the containerized environment, start a docker container by running below:
```bash
# (DISCLAIMER: This command could create a security breach on your system. DON'T run it on public and/or unsecure networks. Keep in mind that this project is for demonstration purposes.)
xhost +local:root  # You need this for display forwarding from docker container to your local. Without this, rviz won't work. 
```
3. To build the container
```bash
cd ~/path_to_ws/src/f1tenth_gym_ros
docker build -t sim:latest .
```
4. To run the container
```bash
 docker run -it --rm --privileged \
  --gpus all \  # remove this line if you have trouble with NVIDIA drivers.
  --net host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  sim:latest
```
 - Alternatively, run the `run_dev.sh` script under `~/path_to_ws/src/f1tenth_gym_ros/src/f1tenth_gym_ros`.

# Quickstart:
## Configuring the simulation
- The configuration file for the simulation is at `~/path_to_ws/src/f1tenth_gym_ros/src/f1tenth_gym_ros/config/sim.yaml`.
- Topic names and namespaces can be configured in `sim.yaml` but it's highly recommended to keep them unchanged.
- The map can be changed via the `map_path` parameter. You'll have to use the full path to the map file. The map follows the ROS convention. It is assumed that the image file and the `yaml` file for the map are in the same directory with the same name
- The `num_agent` parameter can be changed arbitrarily between [1, 4].

## 1. Create a racecar to spawn and use in simulator
- The baseline for a racecar (specific to this simulation) lives under [this](https://gitlab.eteration.com/composiv/eclipse-muto/multi-agent/racecar1) repository. If you'd like to test your own algorithm with your own racecar workspace, it is suggested that you read the README.md of the [racecar](https://gitlab.eteration.com/composiv/eclipse-muto/multi-agent/racecar1) repository.
  
## 2. Topics published by the simulation
- `{racecar_namespace}/{scan_topic}`: Where `racecar_namespace` and `scan_topic` are parameters from `sim.yaml` config file. These parameters could be but be configured from the `sim.yaml` file under the `config` directory but it is highly recommended that you keep them unchanged. If you wan't to change it for some specific reason, make sure that you update the corresponding racecar's `racecar_namespace` too. If left untouched, the simulation will publish a scan topic __*for each agent*__ with the following name: `/racecar{i}/scan` where {i} could be a number up to `num_agent` parameter specified in `sim.yaml`. If `num_agent` is 3, there will be `/racecar1/scan`, `/racecar2/scan`and `/racecar3/scan` with each containing different scan data based on the transforms and odometry of the specific racecar.
- For instance, if you change the `racecar_namespace` to `foo`, the simulation will publish topics like `/foo1/scan_topic`, `/foo2/odom_topic`. So in your racecar workspace, you'd need to change the `racecar_namespace` parameter to `foo1` or `foo2` in order for the simulation to successfully seperate the racecars from each other. The simulation automatically numerates the agent right after the `racecar_namespace` so while configuring your racecar's `racecar_namespace`, you'd need to give `namespace of your choosing + agent number (which should be unique between 1 and num_agent parameter)` meanwhile while configuring the simulation's `racecar_namespace` in the `sim.yaml` file, you'd need to give `namespace of your choosing` without the agent number to match it properly with your racecars. 

- `{racecar_namespace}/{odom_topic}`: Exact same logic as the above. e.g.: `/racecar2/odom`, `/foo3/odom` 

- A `tf` tree is also maintained. If you have `tf2_tools` installed, you could view the tree with:
```bash
source /opt/ros/<your-ros2-distro>/setup.bash
ros2 run tf2_tools view_frames.py
```
Above command is going to output you a `.pdf` file under the directory you've executed the command.

## 3. Topics subscribed by the simulation
 `/{racecar_namespace}/drive`: The topic to send `AckermannDriveStamped` messages where {i} is the agent number. 
> **IMPORTANT: For sending seperate drive commands to agents, you need to send an `AckermannDriveStamped` message with a `frame_id` of `{racecar_namespace}{i}/base_link` to the `{racecar_namespace}{i}/drive` topic where {i} is the number of the specific agent you want to choose. e.g. msg.header.frame_id = racecar2/base_link** 

Below command sends an AckermannDriveStamped message to the 4th agent. Notice the topic name and frame_id is unique for the chosen agent: 
```bash
ros2 topic pub racecar4/drive ackermann_msgs/msg/AckermannDriveStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'racecar4/base_link'}, drive: {steering_angle: 1.0, steering_angle_velocity: 1.0, speed: 1.0, acceleration: 0.0, jerk: 0.0}}"
```

- `/map`: The map of the environment

- `/initalpose`: This is the topic for resetting all of the agents' poses to their initial state via RViz's 2D Pose Estimate tool. You could accomplish resetting each agent's pose by clicking the brown `InteractiveMarkers` that are mapped to agent numbers. If you click 1, that agent will be active and when you send a `2D Pose Estimate` via Rviz, the active agent will spawn in that location 

# Launching the Simulation
1. To launch the simulation, make sure you source both the ROS2 setup script and the local workspace setup script. Run the following in the bash session from the terminal:
```bash
source /opt/ros/<your-ros-distro>/setup.bash
source install/local_setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```
An rviz window should pop up showing the simulation either on your host system or in the browser window depending on the display forwarding you chose.

# Troubleshooting

### Invalid frame ID "map" passed to canTransform
```bash
[rviz2-1] Warning: Invalid frame ID "map" passed to canTransform argument target_frame - frame does not exist
[rviz2-1] at line 133 in /tmp/binarydeb/ros-foxy-tf2-0.13.14/src/buffer_core.cpp
``` 
- Everytime you launch the simulation, you're gonna see this warning being printed out aggressively and as a result, you might fail to see the racecar `RobotModel`s. Just give it some time to receive the transforms from buffer. If the console doesn't stop outputting this message after up to a minute, try restarting.
### I can't see any map!
- Make sure you've configured the map_path parameter of `sim.yaml` correctly. It should look like `/path/to/map`. Notice how there is no extension after the map file. That is handled with the `map_img_ext` parameter. If your map file format is a .pgm instead of .png, set `map_img_ext` parameter accordingly.

### I can see the map but there are no racecars. There's only a white rectangle!
- Take a look at the [racecar](https://gitlab.eteration.com/composiv/eclipse-muto/multi-agent/racecar1) repository's `README.md` if you haven't already.
- Try to launch the racecar nodes first, then the simulation. After launching the simulation, wait up to a minute. 
- If you've changed the racecar_namespace parameter, there might be something broken because of that. I recommend changing it back to its original state.

### I can see both the map and the racecars, but the racecars don't move
In your control algorithm, make sure you're publishing and subscribing to correct topics. If you didn't change any parameters BOTH in this repo and racecar repo, the default topics that are published and subscribed by simulation will be like: 
- `/racecar1/scan` ...(and other topics under racecar1 namespace).
- `/racecar2/odom` ...(and other topics under racecar2 namespace).
- `/racecar3/drive` where `racecar3` is the `racecar_namespace` parameter and `drive` is the `drive_topic` parameter.

Notice that the topics above are the default values of these parameters. If you had set the `racecar_namespace` to be `convertible`

`drive_topic` to be `ride`,

`odom_topic` to be `xy`

 and `scan_topic` to be `scanner` in the `sim.yaml` file under the `config` directory, the topics that are being published by the simulation would be:
- `/convertible1/scanner` ...(and other topics under convertible1 namespace).
- `/convertible2/xy` ...(and other topics under convertible2 namespace).
- `/convertible3/ride` ...(and other topics under convertible3 namespace).

It is recommended to leave the parameters under `sim.yaml` with their default values (except map parameters). But if you insist on changing them, make sure you only use alphanumerical characters otherwise interactive markers might not work properly. And double check the topics you subscribe and publish to in your racecar workspace. 

- An important thing to remind is while publishing a drive message, your `AckermannDriveStamped` message has to have a `header.frame_id` of `{racecar_namespace}/base_link`. Otherwise the simulator won't be able to tell which racecar to drive. For `racecar1` it would be `racecar1/base_link`

### My changes aren't reflecting to the simulation!
- Make sure you rebuild the container or if you're using Ubuntu 20.04 native installation, rebuild the workspace with `colcon build`.


### Rviz is not showing up!
- If you're running the simulation via docker, don't forget to run the xhost command mentioned in the [installation](#with-an-nvidia-gpu) section and run the container with the command mentioned there