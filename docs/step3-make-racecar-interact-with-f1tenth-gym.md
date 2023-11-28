# Get Sim Data
- Assuming you haven't changed the namespace related parameters: 
- For `LaserScan` data, you could subscribe to:
  + `racecar1/scan`
  + `racecar2/scan`
  + `racecar3/scan`
- For `Odometry` data,:
  + `racecar1/odom`
  + `racecar2/odom`
  + `racecar3/odom`

# Send Data To Your Racecar
- Assuming you haven't changed the namespace related parameters: 
  + `racecar1/drive`
  + `racecar2/drive`
  + `racecar3/drive`
- An example AckermanDriveStamped message would look like below:
```bash
ros2 topic pub racecar3/drive ackermann_msgs/msg/AckermannDriveStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'racecar3/base_link'}, drive: {steering_angle: 1.0, steering_angle_velocity: 1.0, speed: 1.0, acceleration: 0.0, jerk: 0.0}}"
```
  

