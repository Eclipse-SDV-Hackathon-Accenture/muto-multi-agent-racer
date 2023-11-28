# Launching the Simulation
1. To launch the simulation, make sure you source both the ROS2 setup script and the local workspace setup script. Run the following in the bash session from the terminal:
```bash
source /opt/ros/<your-ros-distro>/setup.bash
source install/local_setup.bash
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```
An rviz window should pop up showing the simulation either on your system

## Configuring the simulation
- The configuration file for the simulation is at `/path_to_ws/src/muto-multiagent-simulation/src/f1tenth_gym_ros/config/sim.yaml`.
- Topic names and namespaces can be configured in `sim.yaml` but it's highly recommended to keep them unchanged.
- The map can be changed via the `map_path` parameter. You'll have to use the full path to the map file. The map follows the ROS convention. It is assumed that the image file and the `yaml` file for the map are in the same directory with the same name
- The `num_agent` parameter can be changed arbitrarily between [1, 4].

## 1. Create a racecar to spawn and use in simulator
If you'd like to test your own algorithm with your own racecar workspace, it is suggested that you reference the README.md of the racecar package.
  
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

Below command sends an AckermannDriveStamped message to the 3rd agent. Notice the topic name and frame_id is unique for the chosen agent: 
```bash
ros2 topic pub racecar3/drive ackermann_msgs/msg/AckermannDriveStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'racecar3/base_link'}, drive: {steering_angle: 1.0, steering_angle_velocity: 1.0, speed: 1.0, acceleration: 0.0, jerk: 0.0}}"
```

- `/map`: The map of the environment

- `/initalpose`: This is the topic for resetting all of the agents' poses to their initial state via RViz's 2D Pose Estimate tool. (You need to choose the specific agent you'd like to reset via `MultiagentPanel`: an Rviz Plugin that comes with the simulation)



