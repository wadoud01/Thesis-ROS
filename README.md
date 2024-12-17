# AR-based Control System for Franka Panda Robotic Arm

This project presents an augmented reality (AR)-based control system for the Franka Panda robotic arm. The system integrates Unity3D and Hololens 2 for the user interface, along with ROS and MoveIt for real-time motion planning. The project allows users to interact with the robot using an AR interface, sending real-time commands to control the robotic arm's movements and gripper, while providing immediate feedback.

## System Overview

The system is composed of multiple components that work together to provide seamless interaction between the user and the robot:
- **User Interface**: Designed in Unity3D and displayed on Hololens 2 using MRTK.
- **Robot Control**: ROS and MoveIt are used for real-time motion planning.
- **Synchronization**: A ROS-TCP communication protocol enables data transfer between ROS and Unity.
- **Real-Time Data Transfer**: Data is transferred via UDP to another PC that is connected to the real robot, waiting for user permission before execution.

## Components

### 1. **unity_coordinate.listener.py**
This Python script handles communication between Unity and the ROS system. It listens for 3D coordinates (representing target positions for the robot) and grasp signals (to control the robot’s fingers) from Unity.

- **Features**:
  - Moves the robot arm to target coordinates using MoveIt and inverse kinematics (IK).
  - Controls the robot's fingers (open or close) based on user input.
  - Provides feedback to Unity about the success or failure of commands.
  - Ensures that no duplicate commands are processed.

- **Key ROS Topics**:
  - **`/unity_coordinates`**: Receives the target position for the robot in 3D space.
  - **`/grasp_signal`**: Receives a boolean signal to control the robot's gripper (open/close).
  - **`unity_feedback`**: Publishes feedback messages to Unity.

### 2. **pos_udp_sender**
This C++ ROS node is responsible for sending the robot's joint positions over UDP to another PC that controls the real robot. It monitors the robot's joint states and sends the data only when the robot has stopped moving and an execute signal is received.

- **Features**:
  - Monitors joint states of the robot and tracks movement.
  - Sends joint positions to a remote system using UDP when the execute signal is received and the robot has stopped moving.
  - Publishes feedback to Unity to inform the user that the robot is executing the movement.

- **Key ROS Topics**:
  - **`/joint_states`**: Receives joint state data from the robot.
  - **`/execute_signal`**: Receives a signal to trigger the movement of the robot.
