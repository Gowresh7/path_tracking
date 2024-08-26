# Path Tracking Controller Implementation in ROS

## Overview
This repository contains the implementation of a path tracking system using the following algorithms in ROS:-
- Pure Pursuit Control

## Table of Contents
- [Overview](#overview)
- [Controllers Background](#controller-background)
- [System Architecture](#system-architecture)
- [Design Choices](#design-choices)
- [Class and Function Details](#class-and-function-details)
- [Execution Flow](#execution-flow)
- [Parameter Configuration](#parameter-configuration)
- [Potential Enhancements](#potential-enhancements)
- [Testing and Debugging](#testing-and-debugging)
- [Conclusion](#conclusion)

## Controllers Background

### Pure Pursuit Controller 

The Pure Pursuit algorithm is a popular path-tracking method for vehicles with Ackermann steering (like cars). It is designed to steer the vehicle towards a target point, known as the lookahead point, on a given path. The algorithm calculates the curvature of this arc to determine the appropriate steering angle using the following equation:-

![Pure Pursuit Equation](https://raw.githubusercontent.com/Gowresh7/path_tracking/main/docs/PurePursuit_Eqn.png)
Where:
    δ is the steering angle of the vehicle.
    L is the wheelbase of the vehicle (the distance between the front and rear axles).
    y is the lateral offset of the lookahead point in the vehicle's coordinate frame.
    d is the distance between the vehicle and the lookahead point.

## System Architecture

### High-Level Architecture

The path tracking system is built around three main components: