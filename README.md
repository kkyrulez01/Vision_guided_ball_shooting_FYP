# FYP Project no: D091 - Development of a Vision System for visually guided ball shooting

## ℹ️ Overview  

The aim of this project is to develop a vision system to guide a USV (Unmanned Surface Vehicle) to shoot racquetballs into the target holes.
The motivation behind this project comes from one of the tasks included in the 2024 RobotX Challenge and that is the Scan Dock Deliver task.
The functionalities of the vision system will be demonstrated within the [VRX simulation environment](https://github.com/osrf/vrx) that is simulated
in Gazebo. For further details on the VRX simulation environment, please check out the [VRX wiki](https://github.com/osrf/vrx/wiki). 

The vision system implemented Binocular vision with the use of the front left and right cameras on the USV. The camera image topics are then bridged from Gazebo
to ROS2 and subscribed by a custom ROS2 Subscriber node. Here, perception is achieved with the use of computer vision algorithms from OpenCV2 library. This allows
us to obtain the coordinates of the targets. These coordinates will then be used to calculate the optimal ball trajectory and required ball shooter pose.

The vision system features 2 approaches to this problem:
1) Multi-scale template matching
2) Color filtering + Shape detection

## 🌟 Requirements

- Gazebo Harmonic
- ROS2 Jazzy
