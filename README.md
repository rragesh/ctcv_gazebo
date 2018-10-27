# ctcv_gazebo v0.2 (July 2018)

----
## General Info

Author: David Portugal, Ingeniarius Ltd.

This package contains the basic framework to run a robot in the Gazebo Simulator with ROS.

This is part of the extra tasks of [RobotCraft, class of 2018](http://robotcraft.ingeniarius.pt/).

----
## Assumption

You should have ROS Kinetic and Gazebo 7 installed on Ubuntu 16.04.
Install the following dependencies:

```
sudo apt install ros-kinetic-move-base ros-kinetic-nav-core ros-kinetic-amcl ros-kinetic-map-server
```

Clone this packages into your workspace (assuming it is at */home/your_user/catkin_ws*):

```
cd ~/catkin_ws/src
git clone https://github.com/ingeniarius-ltd/ctcv_gazebo
```
Install the following dependencies:

```
sudo apt install ros-kinetic-move-base ros-kinetic-nav-core ros-kinetic-amcl ros-kinetic-map-server
```
Then compile it
```
cd ~/catkin_ws
catkin_make
```


----
## Usage

To start up the Gazebo environment with the CTCV and 1 Robot do:

```
roslaunch ctcv_gazebo ctcv.launch
```

And in a separate terminal, start the localization and navigation software of the robot:

```
roslaunch ctcv_gazebo robot.launch
```

To start up the Gazebo environment with the CTCV and 1 Robot for autonomous docking.
```
roslaunch ctcv_gazebo recharge.launch
```
To send the location of the docking station as service request
```
rosservice call /dock_service "pose:
x: 0.0
y: 0.0
theta: 0.0
offset_distance: 0.0
battery_thresh: 0.0
robot_width: 0.0"
```
