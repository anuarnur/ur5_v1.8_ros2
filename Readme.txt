Based on https://github.com/ros-industrial-attic/ur_modern_driver

# To launch ur5 using ROS2:

ros2 run ur_ros2_driver ur_driver

#and wait until "DEBUG: Realtime port: Got connection" message


##############################

# To send commands from terminal:
ros2 topic pub /ur_driver/joint_speed std_msgs/msg/Float64MultiArray "data:
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0
- 0.0" -r 100

##############################

# Examples of how to control UR5:

python3 test_acceleration.py #using UR_Script
#or
python3 joint_tuner.py #Using joint veloicty

