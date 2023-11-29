# Subscribe to Simulation Data
- Assuming you haven't changed the racecar_namespace: 
- For `LaserScan` data, you could subscribe to:
  + `racecar1/scan`
  + `racecar2/scan`
  + `racecar3/scan`
- For `Odometry` data,:
  + `racecar1/odom`
  + `racecar2/odom`
  + `racecar3/odom`

# Example Drive for Racecar
- Below command sends an example AckermannDriveStamped message to the 1st racecar. 
```bash
ros2 topic pub -r 100 racecar1/drive ackermann_msgs/msg/AckermannDriveStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'racecar1/base_link'}, drive: {steering_angle: 1.0, steering_angle_velocity: 1.0, speed: 1.0, acceleration: 0.0, jerk: 0.0}}"
```

### Note: 
- While sending an `AckermannDriveStamped` message to a racecar, your `AckermannDriveStamped` message's `frame_id` should have the value of `racecar_namespace_of_the_racecar_you_want_to_control/base_link`. Consider the example above. You can observe that this message is being sent to `racecar1/drive` topic and within the message, it includes a `frame_id` of `racecar1/base_link` which should only cause `racecar1` to move. 

- If you'd like to send an `AckermannDriveStamped` to another racecar, you'd need to adjust the `topic` you'll send the message to and the `frame_id` of your corresponding `AckermannDriveStamped` message.

# Example Drive for Racecar using Eclipse Muto

## What is a stack?
- In the context of Eclipse Muto, a stack is a set of instructions written in a `json`-like structure to let Eclipse Muto know which ROS nodes it should introspect.

## Example Stack
```json
{
  "thingId": "org.eclipse.muto.sandbox:racecar01-gap-follower-fast-pace",
  "policyId": "org.eclipse.muto.sandbox:racecar01-gap-follower-fast-pace",
  "definition": "org.eclipse.muto:Stack:0.0.1",
  "attributes": {
    "type": "simulator"
  },
  "features": {
    "stack": {
      "properties": {
        "name": "Racecar1 Gap Follower (Fast Paced)",
        "context": "eteration_office",
        "stackId": "org.eclipse.muto.sandbox:racecar01-gap-follower-fast-pace",
        "arg": [],
        "node": [
          {
            "name": "cass_gap_follower_fast_pace",
            "pkg": "reactive_gap_follower",
            "exec": "cass_gap_follower",
            "param": [
              {
                "from": "$(find reactive_gap_follower)/config/params.yaml"
              }
            ]
          }
        ]
      }
    }
  }
}
```
- If you pay attention to the `"Node"` part, it includes an instruction to start the reactive gap follower algorithm. 
- You can start your own nodes in a similar manner and provide a different parameter file. 
- Notice that while you're providing parameters, you could go like: 
  ```json
  "param": [
    {
      "from": "$(find your_package_name)/path_to_your.yaml"
    }
  ]
  ```
-  or you could simply define your parameters in place like:
```json
  "param": [
    {
      "param1": "value1",
      "param2": "value2"
    }
  ]
```

## Pushing a stack to the twin server

If you have a stack like below:
```json
{
  "thingId": "org.eclipse.muto.sandbox:racecar01-gap-follower-fast-pace",
  "policyId": "org.eclipse.muto.sandbox:racecar01-gap-follower-fast-pace",
  "definition": "org.eclipse.muto:Stack:0.0.1",
  "attributes": {
    "type": "simulator"
  },
  "features": {
    "stack": {
      "properties": {
        "name": "Racecar1 Gap Follower (Fast Paced)",
        "context": "eteration_office",
        "stackId": "org.eclipse.muto.sandbox:racecar01-gap-follower-fast-pace",
        "arg": [],
        "node": [
          {
            "name": "cass_gap_follower_fast_pace",
            "pkg": "reactive_gap_follower",
            "exec": "cass_gap_follower",
            "param": [
              {
                "from": "$(find reactive_gap_follower)/config/params.yaml"
              }
            ]
          }
        ]
      }
    }
  }
}
```
You can push it to the [Twin Server](http://sandbox.composiv.ai) using `curl` and introspect it using [Muto Dashboard](dashboard.composiv.ai):
```bash
curl -X PUT -H "Content-Type: application/json" -d '
{
  "definition": "org.eclipse.muto:Stack:0.0.1",
  "attributes": {                              
    "type": "simulator"
  },                   
  "features": {
    "stack": {
      "properties": {
        "name": "Example Stack",
        "context": "eteration_office",
        "stackId": "org.eclipse.muto.sandbox:example-id",
        "arg": [],                                       
        "node": []
      }           
    }
  } 
}   
' https://ditto:ditto@sandbox.composiv.ai/api/2/things/org.eclipse.muto.sandbox:org.eclipse.muto.sandbox:example-id.launch
```
For further reading, refer to [API Docs](https://sandbox.composiv.ai/apidoc/)
