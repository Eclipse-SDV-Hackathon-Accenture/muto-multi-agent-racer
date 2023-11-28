# Autonomous Racecar for F1tenth Gym Ros
This is a template for a racecar to test autonomous driving algorithms on the [F1tenth Gym Ros](https://gitlab.eteration.com/composiv/eclipse-muto/multi-agent/muto-multiagent-simulation) simulation

# Installation
**Supported Systems:**
- Ubuntu (tested on 22.04 with ROS2 Humble)
- Docker (tested on 22.04 with ROS2 Humble)


## With an NVIDIA gpu:
**Install the following dependencies:**
- **Docker** Follow the instructions [here](https://docs.docker.com/install/linux/docker-ce/ubuntu/) to install Docker. A short tutorial can be found [here](https://docs.docker.com/get-started/) if you're not familiar with Docker.

**Installing the simulation:**

1. Clone this repo
```bash
cd $HOME
git clone --recurse-submodules https://gitlab.eteration.com/composiv/eclipse-muto/multi-agent/racecar1 racecar
```
2. To build the container
```bash
cd $HOME/racecar/src
docker build -t racecar:latest .
```
3. To run the container
```bash
docker run -it racecar:latest
```
# Quickstart
## Step1: Configuration and usage:
- Vehicle name must be the same with the namespace in the simulation's `racecar_namespace` parameter
    - Don't forget to configure your vehicle name in sim.yaml. 
    - If you didn't mess with the naming configuration of the simulation, the parameters in racecar.yaml should look like below:
        * vehicle_name: 'racecar1' (other possible values with default configuration are racecar2, racecar3 or racecar4 depending on `num_agents` of simulation's `sim.yaml`) 
        * color: 'black' (other possible values are green, red, blue ) 
    - Above was the configuration for your racecar's base properties. 
    - Assuming the simulation's name settings are default; for your racing algorithm, you should keep the following in mind:
        * If your `vehicle_name` is racecar1:
            * Sim related topics would be:
                * Odometry data from `racecar1/odom`
                * LaserScan data from `racecar1/scan`
                * Drive message to `racecar1/drive`
        * A typical autonomous racing algorithm produces a control output and publishes it to a topic. 
            - Keeping above information in mind, your drive message might look like: 
                * topic: racecar1/drive 
                * msg_type: ackermann_msgs/msg/AckermannDriveStamped 
                * body: "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'racecar1/base_link'}, drive: {steering_angle: 1.0 steering_angle_velocity: 1.0, speed: 1.0, acceleration: 0.0, jerk: 0.0}}"
## Step2: Launching your racecar
- Source the workspace and launch the nodes:
```bash
. install/setup.bash
ros2 launch racecar racecar.launch.py
```

## Step3: Observe the results
- If you dont have an algorithm but you've managed to get a racecar up and running, you could see the results by manually publishing an ackermann  message to a racecar with `vehicle_name`: racecar1:
```bash
ros2 topic pub racecar1/drive ackermann_msgs/msg/AckermannDriveStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'racecar1/base_link'}, drive: {steering_angle: 1.0, steering_angle_velocity: 1.0, speed: 1.0, acceleration: 0.0, jerk: 0.0}}"
``` 


# Eclipse Muto configuration  (#TODO)

# More information
- The simulation lives under [this](https://gitlab.eteration.com/composiv/eclipse-muto/multi-agent/muto-multiagent-simulation) repository.

- You could use this as a template to create racecars to create more racecars for the simulation. The simulation supports up to 4 racecars. Whenever you copy this template, just replace every instance of `racecar1` with `racecar2`, `racecar3` or `racecar4`. If you want to use another name for your racecar (It's suggested you dont), you need to configure the racecar_namespace parameter in the simulation part.

- If you add another folder (an algorithm perhaps) to this workspace, make sure you copy it to the docker environment using the `Dockerfile`.
