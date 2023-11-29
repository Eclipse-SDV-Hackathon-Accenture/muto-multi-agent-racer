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

# Send Data To Your Racecar
Below command sends an example AckermannDriveStamped message to the 1st racecar. 
```bash
ros2 topic pub -r 100 racecar1/drive ackermann_msgs/msg/AckermannDriveStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'racecar1/base_link'}, drive: {steering_angle: 1.0, steering_angle_velocity: 1.0, speed: 1.0, acceleration: 0.0, jerk: 0.0}}"
```

### Note: 
- While sending an `AckermannDriveStamped` message to a racecar, your `AckermannDriveStamped` message's `frame_id` should have the value of `racecar_namespace_of_the_racecar_you_want_to_control/base_link`. Consider the example above. You can observe that this message is being sent to `racecar1/drive` topic and within the message, it includes a `frame_id` of `racecar1/base_link` which should only cause `racecar1` to move. 

- If you'd like to send an `AckermannDriveStamped` to another racecar, you'd need to adjust the `topic` you'll send the message to and the `frame_id` of your corresponding `AckermannDriveStamped` message.
