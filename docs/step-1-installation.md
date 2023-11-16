# Hack Challenge "ROS Racers"

## Template Setup

How To for getting the template up and running

### F1TENTH gym environment ROS2 communication bridge

This is a ROS communication bridge with multiagent support (up to 4) for the F1TENTH gym environment that turns it into a ROS2 simulation.

### Installation

**Supported Systems:**

- Ubuntu (tested on 20.04) native with ROS 2 Foxy
- Docker (tested on 22.04)

### Native on Ubuntu 20.04

**Install the following dependencies:**

- **ROS 2** Follow the instructions [here](https://docs.ros.org/en/foxy/Installation.html) to install ROS 2 Foxy.
- **F1TENTH Gym**
  ```bash
  cd $HOME
  git clone https://github.com/f1tenth/f1tenth_gym
  cd f1tenth_gym && pip3 install -e .
  ```

**Installing the simulation:**

- Create a workspace: ` cd $HOME && mkdir muto-multiagent-sim`
- Clone the repo into the workspace:
  ```bash
  cd $HOME/
  git clone https://gitlab.eteration.com/composiv/eclipse-muto/multi-agent/muto-multiagent-simulation.git muto-multiagent-sim
  ```
- Update correct parameter for path to map file:
  Go to `sim.yaml` [$HOME/muto-multiagent-simulation/src/f1tenth_gym_ros/config](https://gitlab.eteration.com/composiv/eclipse-muto/multi-agent/muto-multiagent-simulation/-/blob/main/src/f1tenth_gym_ros/config/sim.yaml) in your cloned repo, change the `map_path` parameter to point to the correct location. It should be `'<your_home_dir>/muto-multiagent-sim/src/f1tenth_gym_ros/maps/levine'`. You could also use Spielberg_map instead of levine

- Install dependencies with rosdep then build:
  ```bash
  source /opt/ros/foxy/setup.bash
  cd $HOME/muto-multiagent-sim && rosdep install -i --from-path src --rosdistro foxy -y && colcon build
  ```
- Now head to [Quickstart](#quickstart) if installation was successful. If not, head to [Troubleshooting](#troubleshooting)

### With Docker:

**Install the following dependencies:**

- **Docker** Follow the instructions [here](https://docs.docker.com/install/linux/docker-ce/ubuntu/) to install Docker. A short tutorial can be found [here](https://docs.docker.com/get-started/) if you're not familiar with Docker.

**Installing the simulation:**

1. Clone this repo

```bash
cd $HOME
git clone https://gitlab.eteration.com/composiv/eclipse-muto/multi-agent/muto-multiagent-simulation.git muto-multiagent-sim
```

2. To run the containerized environment, start a docker container by running below commands.

```bash
xhost +local:root  # You need this for display forwarding from docker container to your local. Without this, rviz won't work.
```

3. To build the container

```bash
cd $HOME/muto-multiagent-sim/src/f1tenth_gym_ros
docker build -t sim:latest .
```

4. To run the container

```bash
 docker run -it --rm --privileged \
  --gpus all \  # remove this line if you have troubles with NVIDIA drivers.
  --net host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  sim:latest
```

### Quickstart:

#### Configuring the simulation

- The configuration file for the simulation is at `$HOME/muto-multiagent-sim/src/f1tenth_multiagent_gym_ros/config/sim.yaml`.
- Topic names and namespaces can be configured in `sim.yaml` but it's highly recommended to keep them untouched
- The map can be changed via the `map_path` parameter. You'll have to use the full path to the map file. The map follows the ROS convention. It is assumed that the image file and the `yaml` file for the map are in the same directory with the same name
- The `num_agent` parameter can be changed arbitrarily (Between 1 and 4).
  The entire directory of the repo is mounted to a workspace `/muto-multiagent-sim/src` as a package.

#### 1. Create a racecar to spawn in simulator

- An example racecar live under different repositories. You could see the link to them below.
  If you'd like to test your own algorithm with your own racecar workspace, I suggest you read the README.md of the [racecar](https://gitlab.eteration.com/composiv/eclipse-muto/multi-agent/racecar1) repository.

#### 2. Topics published by the simulation

- `{racecar_namespace}{i}/{scan_topic}`: The laser scan data for a specific agent. These parameters are inside the `sim.yaml` file under the `config` directory. If left untouched, it will spawn a scan topic **_for each agent_** with the following name: `/racecar{i}/scan` where {i} is the value of `num_agent` parameter in `sim.yaml`. If `num_agent` is 3, there will be `/racecar1/scan`, `/racecar2/scan`and `/racecar3/scan` with each containing different scan data based on the odometry of the specific racecar.

- `{racecar_namespace}{i}/{odom_topic}`: Exact same logic as the above. e.g.: `/racecar{i}/odom`

- A `tf` tree is also maintained. If you have tf2_tools installed, you could view the tree with:

```bash
$ source /opt/ros/foxy/setup.bash
$ ros2 run tf2_tools view_frames.py
```

Above command is going to give you a `.pdf` file in the directory you've just executed the command.

#### 3. Topics subscribed by the simulation

`/{racecar_namespace}{i}`: The topic to send `AckermannDriveStamped` messages where {i} is the agent number.

> **IMPORTANT: For sending seperate drive commands to agents, you need to send an `AckermannDriveStamped` message with a `frame_id` of `{racecar_namespace}{i}/base_link` to the `{racecar_namespace}{i}/drive` topic where {i} is the number of the specific agent you want to choose. e.g. msg.header.frame_id = racecar2/base_link**

- `/map`: The map of the environment

An example drive message you could send from a bash session to the 1st agent in a yaml format would look like this:

```bash
$ ros2 topic pub racecar1/drive ackermann_msgs/msg/AckermannDriveStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'racecar4/base_link'}, drive: {steering_angle: 1.0, steering_angle_velocity: 1.0, speed: 1.0, acceleration: 0.0, jerk: 0.0}}"
```

Above command sends an AckermannDriveStamped message to the 4th agent. Notice the topic name and frame_id is unique for each agent

`/initalpose`: This is the topic for resetting all of the agents' poses to their initial state via RViz's 2D Pose Estimate tool. There will be a cleverer way to reset agent poses in the future. Do **NOT** publish directly to this topic unless you know what you're doing.

### Launching the Simulation

1. To launch the simulation, make sure you source both the ROS2 setup script and the local workspace setup script. Run the following in the bash session from the terminal:

```bash
source /opt/ros/foxy/setup.bash
source install/local_setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```

An rviz window should pop up showing the simulation either on your host system or in the browser window depending on the display forwarding you chose.

You can then run another node by creating another bash session

### Troubleshooting

### Invalid frame ID "map" passed to canTransform

```bash
[rviz2-1] Warning: Invalid frame ID "map" passed to canTransform argument target_frame - frame does not exist
[rviz2-1] at line 133 in /tmp/binarydeb/ros-foxy-tf2-0.13.14/src/buffer_core.cpp
```

- Everytime you launch the simulation, you're gonna see this warning being printed out aggressively. Just give it some time to receive the transforms from buffer. If the console doesn't stop outputting this message after up to a minute, check [this section](#i-can-see-the-map-but-there-are-no-racecars). This might be fixed later on.

### I can't see any map!

- Make sure you've configured the map_path parameter of `sim.yaml` correctly. It should look like `/path/to/map`. Notice how there is no extension after the map file. That is handled with the `map_img_ext` parameter. If your map file format is a .pgm instead of .png, set `map_img_ext` parameter accordingly.

### I can see the map but there are no racecars. There's only a white rectangle!

- Take a look at the [racecar](https://gitlab.eteration.com/composiv/eclipse-muto/multi-agent/racecar1) repository's `README.md` if you haven't already.
- Try to launch the racecar nodes first, then the simulation. After launching the simulation, wait up to a minute.
- If you've changed the racecar_namespace parameter, there might be something broken because of that. I recommend changing it back to its original state.

### I can see both the map and the racecars, but the racecars don't move

In your control algorithm, make sure you're publishing and subscribing to correct topics. If you didn't change any parameters, the default topics that are published and subscribed by simulation will be like:

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

It is recommended to leave the parameters under `sim.yaml` with their default values (except map parameters). But if you insist on changing them, make sure it's in a format like make sure you don't use `_`(underscore) character as it splits interactive markers based on them. And double check the topics you subscribe to in the racecar side

- An important thing to note is while publishing a drive message, your `AckermannDriveStamped` message has to have a `header.frame_id` of `{racecar_namespace}/base_link`. Otherwise the simulator won't be able to tell which racecar to drive. For namespace `racecar1` it would be `racecar1/base_link`

### My changes aren't reflecting to the simulation!

- Make sure you rebuild the container or if you work on native 20.04, rebuild the workspace with `colcon build`.

### Rviz is not showing up!

- If you're running the simulation via docker, don't forget to run the xhost command mentioned in the [installation](#with-an-nvidia-gpu) section and run the container with the command mentioned there. If it's still not showing up, I'm out of ideas.

## Links
