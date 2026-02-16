# Tiago_robot_manipulation

## Overview

This project implements a mobile manipulation task for the TIAGo robot, inspired by a RoboCup-style domestic service scenario.
The robot autonomously navigates from outside a house to a target room, waits for a door to open if it is closed, detects objects on a table using AprilTags, grasps them using its robotic arm, and relocates them to another table nearby.

The task is repeated until all objects are successfully moved.

## Task Description

The robot performs the following sequence:

1. Navigation

    Starts outside the house.
    Detects whether the door is open or closed.
    If closed → waits.
    If open → navigates into the room.

2. Object Detection

    Three objects are placed on a table.
    Each object has a corresponding AprilTag in front of it.
    The robot detects AprilTags using its onboard camera.

3. Pose Estimation & Manipulation

    Object poses are obtained in quaternion format from AprilTag detections.

    Quaternions are converted to Euler angles to enable precise end-effector orientation


    Pose transformations are computed relative to the robot’s base_link frame.

    The robot uses TIAGo grasping actions to:

      Pick objects one by one

      Navigate to a nearby table

      Place the objects

Returns to the original table and repeats for remaining objects.



##  Running the Project

### 1️⃣ Docker Setup

A Docker container is provided for a clean and reproducible environment.

    docker-compose -f clean-the_table_docker.yml up


### 2️⃣ Execute the Task

Inside the container, run:

    rosrun clean_the_table clean_the_table.py

This launches the autonomous clean-the-table behavior.


##  Actions & Services

### Actions

 Pick Object
 Place Object

### Services

 Door State Detection

