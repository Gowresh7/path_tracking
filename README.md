# Path Tracking Controller Implementation in ROS

## Overview
This repository contains the implementation of a path tracking system using the following algorithm in ROS:
- Pure Pursuit Control

## Limitations
- The current implementation does not accommodate reverse motions.
- Sharp turns and U-turns require tuning of the parameters like lookahead distance and speed. This will be considered when adding lane centering.
- Steering is restricted within a certain range to prevent circular movements.

## Table of Contents
- [Overview](#overview)
- [Limitations](#limitations)
- [Requirements](#requirements)
- [Setup](#setup)
- [ToDo List](#todo-list)
- [Controllers Background](#controllers-background)
- [System Architecture](#system-architecture)
- [Design Choices](#design-choices)

## Requirements
- The controller is implemented on the Polaris GEM e2 Simulator found at [Polaris GEM e2](https://github.com/GEM-Illinois/POLARIS_GEM_e2).
- **System:** Ubuntu 20.04 + ROS Noetic

## Setup
- Assuming the same workspace as used by the Polaris GEM e2 Simulator, clone the repository in the workspace and compile:

  ```bash
  $ cd ~/gem_ws/src
  $ git clone https://github.com/Gowresh7/path_tracking.git
  $ cd ~/gem_ws
  $ catkin_make
  $ source devel/setup.bash
  ```

- Use the given launch file to launch the simulation along with path tracking node, tf publisher node, and Rviz. Around 5 -10 seconds after the simulation, a default path will be published for the robot to follow:

  ```bash
  $ roslaunch path_tracking path_tracking_sim.launch
  ```

## ToDo List
- [x] Implement Pure Pursuit Controller
- [x] Implement State Machines
- [x] Visualize States as markers
- [x] Visualize Path Trajectory taken by vehicle
- [ ] Implement Lane Centering
- [ ] Implement Stanley Controller
- [ ] Implement MPC

## Controllers Background

### Pure Pursuit Controller 
The Pure Pursuit algorithm is a popular path-tracking method for vehicles with Ackermann steering (like cars). It is designed to steer the vehicle towards a target point, known as the lookahead point, on a given path. The algorithm calculates the curvature of this arc to determine the appropriate steering angle using the following equation:

![Pure Pursuit Equation](https://raw.githubusercontent.com/Gowresh7/path_tracking/main/docs/PurePursuit_Eqn.png)

Where:

- δ is the steering angle of the vehicle.
- L is the wheelbase of the vehicle (the distance between the front and rear axles).
- y is the lateral offset of the lookahead point in the vehicle's coordinate frame.
- d is the distance between the vehicle and the lookahead point.

## System Architecture

### High-Level Architecture
- The path tracking system is built around three main components:

    - #### PathTrackingController Base Class: 
        - An abstract base class that defines the interface for any path-tracking controller.

    - #### PurePursuitController Derived Class: 
        - A specific implementation of the path-tracking controller using the Pure Pursuit algorithm.

    - #### PathTrackingNode Class: 
        - The main ROS node that handles subscribing to necessary topics, publishing control commands, and managing the state of the robot.

### ROS Topics & Params
- The system is designed to interact with the robot and environment through ROS topics:

    - #### Subscribed Topics:
        - **/gps_path**: Input Path of type **nav_msgs::Path** in 'world' frame for the robot to follow.
        - **/gem/base_footprint/odom**: Odometry data given by the gazebo simulator in 'world' frame.

    - #### Published Topics:
        - **/gem/ackermann_cmd**: Control data of type **ackermann_msgs::AckermannDrive** to send speed and steering angle to the robot.
        - **/state_marker**: Visualization marker of type **visualization_msgs::Marker** to showcase the robot's current state.

    - #### Params:
        - **lookahead_distance**: Determines how far ahead the robot looks on the path to calculate the steering angle. Default is 6.0 meters.
        - **vehicle_speed**: The speed at which the robot should travel while tracking the path. Default is 2.8 meters/second.
        - **wheelbase**: The distance between the front and rear axles of the robot. Default is 1.75 meters.
        - **controller_type**: Specifies the type of controller to use. Currently defaults to "PURE_PURSUIT".

## Design Choices

### Pure Pursuit Algorithm
- For the initial implementation, the Pure Pursuit Algorithm was chosen for its relatively simple logic, which calculates the steering angle based on a lookahead point on the input path.

### Modular Design
- Considering the future implementations of other controller algorithms like Stanley, MPC, etc., a modular architecture was followed to develop the code. This allows for easy implementation and integration of the different path tracking controllers. The **'PathTrackingController'** base class ensures that any derived controller can implement the **'computeControl'** function using the robot's odometry, path, and other parameters as input to output the necessary control commands.

### State Management
- To better manage the robot's behavior at various stages, an enumerated State was used to manage the robot's current state (IDLE, TRACKING, GOAL_REACHED, ERROR). This design allows the system to react appropriately based on the robot's progress along the path, handle errors gracefully, and stop the robot when the goal is reached.

### Dynamic Reconfigure & ROS Parameters
- Parameters such as **'lookahead_distance'**, **'vehicle_speed'**, **'wheelbase'**, and **'controller_type'** are loaded through ROS parameters, making it easy to tune the system for different robots and environments without modifying the code.
- Additionally **'lookahead_distance'**, **'vehicle_speed'**, **'goal_tolerance'** and **'steering_limits'** are exposed thorugh the dynamic reconfigure servers to enable parameter tuning during runtime

### Visualization
- In order to showcase the current state of the robot at various stages of its behavior, **visualization markers** are published and can be viewed in Rviz.
- The predicted trajectory, based on the calculated steering angle is published as a path to visualize in Rviz.

### Additional ROS Nodes
- To enable better visualization in Rviz, a python node was created to provide transformations between */world* frame and */base_footprint* frame based on odometry data
- Another python node was created to publish various path messages as input to the Controller.