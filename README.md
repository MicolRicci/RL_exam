# RL_exam  Micol Ricci P38/000109 
Materal for the Robotics Lab Exam

##  PROJECT OVERVIEW

The purpose of the project is to simulate a typical industry4.0 application. Within an environment there are two robots, a mobile one (a turlebot3) and a fixed manipulator (Kuka iiwa7). the Turtlebot must be able to autonomously navigate the environment and go in search of a specific tool (represented by a marker). Once found, it takes it to the manipulator, which will simulate a grasping process. 


### IMPLEMENTATION IDEA

The turtlebot begins its mission by receiving as keyboard input an identifier corresponding to the desired tool marker. 

At this point it begins navigation in the environment by following a set of waypoints representing the center of one of three rooms containing the objects.

Once it reaches a waypoint the turtlebot begins to rotate on itself in search of the marker to be scanned and verified:

if the marker is not the desired one it proceeds to the next waypont

and if there is a match instead it simulates grabbing the object by remaining stationary for 1 second and then heads toward the manipulator, whose position is known.

When it arrives at its destination, it starts the process of the manipulator, which had previously located the arrival point of the object represented by a marker on the ground.

It remains in a wait state until the turtlebot arrives and only at that point begins the process of taking the object by going to place the end effector in the center of the marker.


## SIMULATION EXECUTION

####  PREREQUISITES:

To run up the simulations,a version of ROS Noetic is needed. Other packages like the Gmapping package, the SLAM package, the ARUCO package and the MOVE-BASE are mandatory to make the simulation working 

#### COMMAND
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

