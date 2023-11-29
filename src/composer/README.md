
[![Catkin Make (Build and Test)](https://github.com/eclipse-muto/composer/actions/workflows/catkin-build.yml/badge.svg?branch=main)](https://github.com/eclipse-muto/composer/actions/workflows/catkin-build.yml)

# Muto Composer

## Build
After you checkout the repository, source your ROS environment and make sure the additional dependencies such as the eclipse paho mqtt client library is installed.

```bash
source /opt/ros/humble/setup.bash
pip3 install paho-mqtt celery requests
```

## Muto Twins

Muto agent requires network connectivity to the Muto Twins (ditto) and MQTT servers. The address for the muto sandbox is: 
* The twin servers and API: https://sandbox.composiv.ai/
* The mqtt server: mqtt://sandbox.composiv.ai:1883

## Running agent

The muto agent must be launched and the target device and the devices should have access to all the packages required to launch the stacks.  For these purposes you can either choose to run a containerized version of muto or launch it direct from the devices after all the packages (such as the muto learning modules) are build and installed on the AV.

```bash
source install/setup.bash
ros2 launch agent agent.launch
```


You can get stacks from twin server via::
```bash
curl 'link_to_stack_url'
```
which returns something similar to the below structure:

```json
{
    "name": "Muto Learning Simulator with Gap Follwer",
    "context": "eteration_office",
    "stackId": "org.eclipse.muto.sandbox:f1tenth-multiagent-gym.launch",
    "stack": [
        {
            "thingId": "org.eclipse.muto.sandbox:racecar1.launch"
        }
    ],
    "node": [ 
        {
            "name": "reactive_gap_follower",
            "pkg": "reactive_gap_follower",
            "exec": "reactive_gap_follower",
            "param": [
              { "from": "$(find reactive_gap_follower)/params.yaml" }
            ]
        }
    ]
}
```
This is a stack with a single node, "cass_gap_follower".  However, it includes another stack (with many other nodes and parameters) that it requires with a stackId reference org.eclipse.muto.sandbox:f1tenth-multiagent-gym.launch. The elements of the stack model resembles a [ROS launch XML](https://wiki.ros.org/roslaunch/XML), so it should be fairly straightforward to understand if you have experience writing XML launch files

## Managing Stacks and Vehicles

New stack can be easily stored on the sandbox server using the things API (put). See the ditto documentation for many examples [https://www.eclipse.org/ditto/intro-overview.html].  For example, to add a new stack to the repository we can use the thing PUT api as follows:

```bash
$ curl -X PUT -H "Content-Type: application/json" -d ' 
{ 
    "name": "Muto Learning Simulator with Gap Follower", 
    "context": "eteration_office",
    "stackId": "org.eclipse.muto.sandbox:composiv_simulator_gf.launch", 
    "stack": [
        {
            "thingId": "org.eclipse.muto.sandbox:composiv_simulator.launch"
        }
    ],
    "node": [ 
        {
            "name": "cass_gap_follower",
            "pkg": "cass_gap_follower",
            "exec": "cass_gap_follower",
            "param": [
              { "from": "$(find cass_gap_follower)/params.yaml" }
            ]
        }
    ]
}
' http://sandbox.composiv.ai/api/2/things/org.eclipse.muto.sandbox:composiv_simulator_gf.launch

```

## Managing Stacks and Vehicles
We can use the TWINS to directly communicating commands to the vehicle itself. The twin server supports special mqtt channels for these purposes called **twin** and **live** channels. For example the following command can be published to the sandbox MQTT server to activate a stack on a car.  Each vehicle has its dedicated **twin** and **live** channels:

```yaml
topic: org.eclipse.muto.sandbox::simulator-monster-01/stack/commands/active
```
```yaml
payload: {
    "name": "Muto Learning Simulator with Gap Follwer",
    "context": "eteration_office",
    "stackId": "org.eclipse.muto.sandbox::composiv_simulator_gf.launch",
    "stack": [
        {
            "thingId": "org.eclipse.muto.sandbox::composiv_simulator.launch"
        }
    ],
    "node": [ 
        {
            "name": "cass_gap_follower",
            "pkg": "cass_gap_follower",
            "exec": "cass_gap_follower",
            "param": [
              { "from": "$(find cass_gap_follower)/params.yaml" }
            ]
        }
    ]
}
```

You can use any open-source mqtt client to issue these commands and monitor various muto twin messages [MQTTX](https://mqttx.app/). Another option is the [mosquitto_pub](https://mosquitto.org/man/mosquitto_pub-1.html), which is a simple MQTT version 5/3.1.1 client that will publish a single message on a topic and exit.  You can publish the message described above using the commandline:

```bash
  mosquitto_pub -d -h sandbox.composiv.ai -p 1883  -t "org.eclipse.muto.sandbox::simulator-monster-01/stack/commands/active" -m '{"name":"Composiv Learning Simulator with Gap Follwer","context":"eteration_office","stackId":"org.eclipse.muto.sandbox::composiv_simulator_gf.launch","stack":[{"thingId":"org.eclipse.muto.sandbox::composiv_simulator.launch"}],"node":[{"name":"cass_gap_follower","pkg":"cass_gap_follower","exec":"cass_gap_follower","param":[{"from":"$(find cass_gap_follower)/params.yaml"}]}]}'

```


## Controlling the F1Tenth Car (navigate on/off) 

```bash
ros2 topic pub --once /mux std_msgs/Int32MultiArray "{layout: { dim: [], data_offset: 0}, data: [0, 0, 0, 0, 1 , 0] }"
```

## More information
Check out the Eclipse Muto github.io page for further reading: [Eclipse Muto](https://eclipse-muto.github.io/docs/docs/muto)