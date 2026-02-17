####################################################################
# MAIN COMMANDS FOR WORKSHOP
####################################################################

# running the ROS2 setup file on your terminal (once per terminal)

source /opt/ros/jazzy/setup.bash

#running the twist publisher python code (in the same directory of the twist_publisher.py file)

python3 ./twist_publisher.py

#running the odom subscriber python code (in the same directory of the twist_odom.py file)

python3 ./twist_odom.py

#running the ROS2<->Gazebo Bridge with the config in the file bridge.yaml (in the same directory of the bridge.yaml file)

ros2 launch ros_gz_bridge ros_gz_bridge.launch.py bridge_name:=ros_gz_bridge config_file:=./bridge.yaml

#Running gazebo simulation with world.sdf 

gz sim world.sdf

######################################################################
# miscelanous useful commands
######################################################################
#List all ROS nodes
ros2 node list

#List all ROS topics
ros2 topic list

#List all Gazebo topics
gz topic -l

#Echo (print) contents of ROS topic (/odom in this example)
ros2 topic echo /odom

#Echo (print) contents of Gazebo topic (/cmd_vel in this example)
gz topic -e -t /cmd_vel

#Directly publish message on ROS topic with terminal command
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.3}}"

#See information (publishers, subscribers, message type) about gazebo topic (/model/vehicle_blue/odometry in this example) 
gz topic -i -t /model/vehicle_blue/odometry

#See information (publishers, subscribers, message type) about ROS topic (/cmd_vel in this example) 
ros2 topic info /cmd_vel

#See verbose (additional information about ROS topic (/cmd_vel in this example) 
ros2 topic info /cmd_vel -v
