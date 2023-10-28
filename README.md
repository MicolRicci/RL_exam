# RL_exam
Materal for the Robotics Lab Exam
# ROBOTICS LAB TECHNICAL  Micol Ricci P38/000109 

#  PROJECT GOAL
The goal is to simulate a typical application of Industry 4.0 .A TurtleBot will autonomously search for a specific tool (in this case a marker) within an environment. Once it locates the tool, the mobile robot will deliver it to a KUKA IIWA manipulator, which will simulate the process of grasping the tool. The manipulator will position itself over a marker representing the tool's pickup location. There are three different tools located in three separate rooms within the environment, while the manipulator is situated in a fourth room.

#  Turtlebot node

The TurtleBot initiates its mission by receiving input from the user specifying the marker code of the tool it needs to find. It then proceeds to navigate the environment, following a sequence of waypoints, each corresponding to a different room, with the central area as the starting point.

Upon reaching each waypoint, the TurtleBot begins to rotate in place, scanning the room for the designated marker. Once it identifies a marker, the robot pauses and verifies if it matches the desired marker code. If it's a match, the TurtleBot simulates the process of picking up the tool by remaining stationary for one second. Afterward, it heads for the waypoint within the manipulator room, signaling the KUKA IIWA via a ROS topic that the tool has been delivered and the grasping process can commence.

In the event that the TurtleBot spots a marker inside a room before reaching the corresponding waypoint, and it turns out not to be the desired one, the mobile robot will skip that room and proceed to the next waypoint. Conversely, if the marker is indeed the one it's looking for, the TurtleBot advances to the waypoint, aligns itself to have the marker within its field of view, simulates the tool's pickup, and then proceeds to the manipulator room.

Additionally, the scenario in which the required tool is not present in the environment has also been considered. In this case, the TurtleBot returns to the central area and notifies the KUKA IIWA via a ROS topic that there is nothing to grasp.

# Kuka iwaa node

Upon startup, the KUKA IIWA manipulator proceeds to locate the delivery point, represented by a marker on the ground. This location is determined using a fixed camera positioned on top of the manipulator, strategically placed to ensure that the x and y coordinates in the camera frame align with those in the KUKA operational space. This alignment eliminates the need for coordinate transformations between the two frames. The z-coordinate and orientation for the KUKA end-effector are arbitrarily chosen.

Subsequently, the KUKA IIWA enters a waiting state, monitoring a specific ROS topic for a signal from the TurtleBot. The manipulator subscribes to this topic, anticipating the result of the TurtleBot's search mission. If the TurtleBot successfully finds and delivers the tool, the manipulator takes action by positioning its end-effector at the center of the marker. However, if the tool has not been found, the manipulator remains inactive.



# SIMULATION EXECUTION

To start the simulation it is necessary to execute four steps:

- lunch _Gazebo_ with the custom world
- start the aruco marker publisher nodes, gmapping for SLAM and move base for turtlebot navigation
- start the turtlebot node
- start the kuka iiwa node

with the four following commands in four different terminal
```
roslaunch environment_pkg environment.launch
roslaunch high_level_control configuration.lunch
rosrun high_level_control turtlebot_controller_node
rosrun high_level_control kuka_controller_node
```

At last it is necessary to start the _Gazebo_ simulation and write in input for what marker the turtlebot should search. 

