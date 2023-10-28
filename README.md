# RL_exam  Micol Ricci P38/000109 
Materal for the Robotics Lab Exam


#  PROJECT GOAL
The goal is to simulate a typical application of Industry 4.0 .A TurtleBot will autonomously search for a specific tool (in this case a marker) within an environment. Once it locates the tool, the mobile robot will deliver it to a KUKA IIWA manipulator, which will simulate the process of grasping the tool. There are three different tools located in three separate rooms within the environment, while the manipulator is situated in a fourth room.

# IMPLEMENTATION IDEA

The TurtleBot initiates its mission by receiving input from the user specifying the marker code of the tool it needs to find. It then proceeds to navigate the environment, following a sequence of waypoints, each corresponding to a different room, with the central area as the starting point.

Upon reaching each waypoint, the TurtleBot begins to rotate in place, scanning the room for the designated marker. Once it identifies a marker, the robot pauses and verifies if it matches the desired marker code. If it's a match, the TurtleBot simulates the process of picking up the tool by remaining stationary for one second. Afterward, it heads for the waypoint within the manipulator room, signaling the KUKA IIWA via a ROS topic that the tool has been delivered and the grasping process can commence.

Upon startup, the KUKA IIWA manipulator proceeds to locate the delivery point, represented by a marker on the ground. This location is determined using a fixed camera positioned on top of the manipulator.

Then the KUKA IIWA enters a waiting state, monitoring a specific ROS topic for a signal from the TurtleBot. The manipulator subscribes to this topic, anticipating the result of the TurtleBot's search mission. If the TurtleBot successfully finds and delivers the tool, the manipulator takes action by positioning its end-effector at the center of the marker.

# SIMULATION EXECUTION

To start the simulation it is necessary to execute four steps:

- lunch _Gazebo_ with the custom world
- start the aruco marker publisher nodes, gmapping for SLAM and move base for turtlebot navigation
- start the turtlebot node
- start the kuka iiwa node

with the four following commands in four different terminal
```
roslaunch environment_pkg environment.launch
```
```
roslaunch high_level_control configuration.lunch
```
```
rosrun high_level_control turtlebot_controller_node
```
```
rosrun high_level_control kuka_controller_node
```


