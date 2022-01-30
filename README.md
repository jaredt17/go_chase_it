# go_chase_it
Udacity Robotics Software Engineering Nanodegree Project 2: Go Chase It, A ROS+GAZEBO project.

## Pre-requisites
- ROS Noetic (Ubuntu 20.04)

## Setup
```
$ mkdir -p /home/workspace/catkin_ws/src
$ cd /home/workspace/catkin_ws/src
$ catkin_init_workspace
```
- Make sure ball_chaser and my_robot are in the src directory of the catkin_ws
```
$ cd ../
$ catkin_make
```

## Running
```
$ source devel/setup.bash
$ roslaunch my_robot world.launch

# In a different terminal window
$ roslaunch ball_chaser ball_chaser.launch
```
- In Gazebo insert a white ball model in front of the robot, and it should start driving towards it as long as it is in the field of view of the camera. This can be checked with rviz camera image or rqt.
