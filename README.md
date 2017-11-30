# Udacity Self-Driving Car Engineer Nanodegree


## System Integration - Capstone project

This is the project repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car. For more information about the project, see the project introduction [here](https://classroom.udacity.com/nanodegrees/nd013/parts/6047fe34-d93c-4f50-8336-b70ef10cb4b2/modules/e1a23b06-329a-4684-a717-ad476f0d8dff/lessons/462c933d-9f24-42d3-8bdc-a08a5fc866e4/concepts/5ab4b122-83e6-436d-850f-9f4d26627fd9) (available to students).

![front-cover](https://i.imgur.com/2nIgpgQ.jpg)

## Introduction
This project consists of two parts:


- Program a self driving car using [ROS](http://www.ros.org/ "ROS-link") to navigate autonomously in a highway, detecting traffic lights and accelerate/deaccelarate accordingly to maintain a good and safe driving experience.
- Test the code on a real self-driving car 'Carla', a Lincoln MKZ model running Ubuntu.

## Team members

**Team name: ADAS 2.0**

<table>
  <tr>
    <th>Name</th>
    <th>email</th>
  </tr>
  <tr>
    <td>Anastasios Stathopoulos</td>
    <td>stathopoan@gmail.com</td>
  </tr>
  <tr>
    <td>Aruul Mozhi Varman S</td>
    <td>aruulmozhivarman@outlook.com</td>
  </tr>
  <tr>
    <td>Francesco Fantauzzi (lead)</td>
    <td>Francesco_Fantauzzi@yahoo.com</td>
  </tr>
</table>

## Overview

This project consists of several ROS nodes implementing functionalities such as traffic light detection, control and waypoint following. The overall architecture is displayed below and illustrates the three basic components: perception, planning, and control. 

![ROS-architecture](https://i.imgur.com/76fgoSK.png)

For every component a representative node has been implemented as illustrated below:

<p align="center"> <img src="https://i.imgur.com/sYvjBuS.png" width="350"> </p>

## Waypoint updater Node

This node is responsible for navigating the car in the road adjusting the velocity for every waypoint ahead based on the traffic light state. It receives data from the topics: 

- /base_waypoints, a complete list of waypoints the car will be following.
- /current_pose, the vehicle's current position
- /traffic_waypoint, the locations to stop for red traffic lights

and subscribes to the topic: `/final_waypoints` which is a list of waypoints head of the car. The code is located at the file `/ros/src/waypoint_updater/waypoint_updater.py`.


![waypoint-updater-ros-graph](https://i.imgur.com/xnGRgax.png)

This node receives all the waypoint positions of the track and stores them. Everytime it receives data from the current car position it tries to find the next closest waypoint to localize itself. If there is no traffic light ahead it adjusts the speed of the next specified waypoints (variable:`LOOKAHEAD_WPS`) making sure the speed limit is not exceeded while maintaining the acceleration and jerk values below maximum allowed values. If there is a yellow or red traffic light near, it deaccelerates the car's speed as smoothly as possible respecting the speed, acceleration and jerk limit. While the light color is red the car is not moving by sending an empty list to the `/final_waypoints` topic. When the traffic light color turns green the car accelerates to the maximum allowed speed until the next red traffic light.

## Traffic Light Detection Node

This node is responsible for detecting a yellow or red traffic light ahead and notify for its state. It receives data from the topics:

- /base_waypoints, a complete list of waypoints the car will be following.
- /current_pose, the vehicle's current position
- /image_color, the image taken from a camera in front of the car

and subscribes to the topic: `/traffic_waypoint` which is the index of the waypoint closest to the red light's stop line if near the car else -1. The code is located at the file `/ros/src/tl_detector/tl_detector.py`.

![tl-detector-ros-graph](https://i.imgur.com/DAnxuOH.png)

The node receives initially all the traffic light positions and the waypoint positions of the track and stores them after mapping each traffic light position with the nearest waypoint. Then it subscribes to the `/image_color` topic to be able to receive images from a camera in front of the car. If the light position is larger than the specified distance the node publices -1 to the  `/traffic_waypoint` topic. If the light position is smaller than the specified distance, the image taken is processed and a color classification is being conducted. If the color is yellow or red then the waypoint index closest to the traffic light position is being published indicating the car must fully stop at that specific waypoint otherwise -1 is being published.

## DBW Node

This node is responsible for controlling the car in terms of throttle, brake, and steering. Carla is equipped with a drive-by-wire (dbw) system, meaning the throttle, brake, and steering have electronic control. It receives data from the topics:

- /current_velocity, receive target linear and angular velocities
- /twist_cmd, receive target linear and angular velocities
- /vehicle/dbw_enabled, indicates if the car is under dbw or driver control

and subscribes to the topics: 

-  /vehicle/throttle_cmd,  throttle
-  /vehicle/brake_cmd, brake
-  /vehicle/steering_cmd, steering

The code is located at the files: 


- `/ros/src/twist_controller/twist_controller.py`
- `/ros/src/twist_controller/yaw_controller.py`

![dbw-ros-graph](https://i.imgur.com/TXEN4pI.png)



## Notes before running

This project requires the use of a GPU. Make sure you have available a Nvidia GPU. Traffic lights classification is very demanding and requires a lot of computational power. To ensure smooth and proper simulation please follow the above recommendation.

## Running Instructions

Run the ROS code and open simulator (see instructions at Usage section). In order for the car to move autonomously uncheck "*Manual*" checkbox. The car may take a few seconds before it begins moving due to waiting for the traffic classifier to load its model and start classification. This behavior only apllies to the first time you uncheck "*Manual*".

<br>
<img src="https://i.imgur.com/UWz5oMI.png" width="425"/> <img src="https://i.imgur.com/7QXgxVY.png" width="425"/>
<img src="https://i.imgur.com/8Wsytei.png" width="425"/> <img src="https://i.imgur.com/SGxZwQB.png" width="425"/>

## Installation Instructions

### Native Installation

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop).
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space

  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases/tag/v1.2).

### Docker Installation
[Install Docker](https://docs.docker.com/engine/installation/)

Build the docker container
```bash
docker build . -t capstone
```

Run the docker file
```bash
docker run -p 4567:4567 -v $PWD:/capstone -v /tmp/log:/root/.ros/ --rm -it capstone
```

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt
```
3. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
4. Run the simulator

### Real world testing
1. Download [training bag](https://drive.google.com/file/d/0B2_h37bMVw3iYkdJTlRSUlJIamM/view?usp=sharing) that was recorded on the Udacity self-driving car (a bag demonstraing the correct predictions in autonomous mode can be found [here](https://drive.google.com/open?id=0B2_h37bMVw3iT0ZEdlF4N01QbHc))
2. Unzip the file
```bash
unzip traffic_light_bag_files.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_files/loop_with_traffic_light.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```
5. Confirm that traffic light detection works on real life images
