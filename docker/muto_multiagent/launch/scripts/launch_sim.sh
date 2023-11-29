source /opt/ros/humble/setup.bash
source /home/muto/sim_ws/install/local_setup.bash
sleep 5
# export DISPLAY=novnc:0.0
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
