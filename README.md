# Robot Delivery System for Food Delivery
## Introduction

The Robot Delivery System is a ROS-based project that manages food delivery in a simulated restaurant environment. The system automates order handling, robot navigation, and task execution with a focus on reliability, scalability, and innovation.

This repo contains three main packages:

1. **turtlebot3_simulation** : Used for starting the robot in a custom Gazebo world.  
2. **turtlebot3_navigation** : Enables navigation of the robot in an environment.  
3. **robot_delivery_system** : Manages orders for the robot.  

## Table of Contents
- [Environment Requirements](#environment-requirements)
- [Dependencies](#dependencies)
- [Installation](#installation)
- [Running the System](#running-the-system)
- [How to Test the System](#how-to-test-the-system)

---

## Environment Requirements

Ensure your system meets the following requirements:  
- **OS**: Ubuntu 20.04
- **ROS Distribution**: [ROS Noetic (Ubuntu 20.04)](#http://wiki.ros.org/noetic/Installation/Ubuntu)
- **Gazebo**: Version 11 or later  


---

## Dependencies

The following dependencies are required for the workspace to function correctly:  


  ```bash
  sudo apt-get install ros-noetic-map-server ros-noetic-amcl ros-noetic-move-base
   ```

## Installation
 - Clone the repository into your ROS workspace:

 ```bash
     cd ~/catkin_ws/src
     git clone https://github.com/Athilakshmi30/robot_delivery_system.git
  ```

- Build the workspace:
     ```bash
     cd ~/catkin_ws
     catkin_make
     ```
- Source the workspace:
```bash
    source devel/setup.bash
```

## Running the System
- To start the robot in Gazebo with cafe world

   ```bash
   roslaunch turtlebot3_gazebo turtlebot3_cafe_world.launch
   ```
- To start the navigation

   ```bash
   roslaunch turtlebot3_navigation turtlebot3_navigation.launch
   ```
- To start the order manager node for robot

   ```bash
   roslaunch robot_delivery_system order_manager.launch
   ```
- To test the robot order delivery system

   ```bash
   rosrun robot_delivery_system test_node
   ```
This node is responsible for getting user input like table number and action (order or cancel).
## How to Test the System  

## Ideal Case

### Scenario Overview

This system simulates a robot delivering food in a café environment. There are 3 tables in the café, and the robot can perform two possible actions: **order** or **cancel**.

**Tables:**

- Table 1
- Table 2
- Table 3

**Actions:**

- **order**
- **cancel**

### Steps to Place an Order

1. **Placing an Order for Table 1:**
   - In the terminal, navigate to the `test_node` and give the input `1` to specify the table number (Table 1).
   - Press **Enter** to continue.
   - The system will prompt you to choose an action.
   - Select **order** to place an order for Table 1.

2. **Robot Movement and Food Collection:**
   - The robot will automatically navigate to the Kitchen to collect the food.
   - Once the robot reaches the Kitchen, a pop-up will appear, asking the user to confirm receipt of food.

   - **If the user presses "OK" within 20 seconds**, the robot will proceed to Table 1 to deliver the food.

   - **If the user does not press "OK" within 20 seconds**, the robot will return to home (starting position).

3. **Delivery to Table 1:**
   - Once the robot reaches Table 1, another pop-up will appear asking if the customer has taken the food.

   - **If the user presses "OK" within 20 seconds**, the delivery will be confirmed, and the robot will complete the task.

   - **If the user does not press "OK" within 20 seconds**, the robot will return to the Kitchen to return the food, and then go back home.

### Testing Other Tables and Actions

- To test the system for **Table 2** or **Table 3**, simply repeat the process by providing the respective table number (2 or 3) when prompted in the terminal.

- You can also test the **cancel** action by selecting **cancel** instead of **order** at the action prompt. In this case, the robot will not proceed with the order and will return to home.


## Other Test Cases

### 1. Place Multiple Orders Before Robot Reaches the Kitchen (Before Pop-up Appears)

**Expected Behavior:**
- The robot will collect and deliver the food to the respective tables, following the order sequence.

### 2. Place Multiple Orders and Cancel One Order Before Robot Reaches the Kitchen

**Expected Behavior:**
- The robot will not collect food for the cancelled order. It will only collect and execute orders that are currently active.

### 3. Place Multiple Orders and Cancel One Order After Receiving Food from the Kitchen

**Expected Behavior:**
- The robot will not deliver food to the cancelled order's table. After delivering the food for the other orders, it will return the food for the cancelled order to the Kitchen and go back home.

### 4. Place Multiple Orders and Don't Accept the Pop-up in Any One of the Orders

**Expected Behavior:**
- If the customer doesn't provide confirmation at the table within the time limit, the robot will consider that order as cancelled and return the food to the Kitchen after completing the other orders.

### 5. Place an Order and While Executing (Moving from Kitchen to Table), Place Another Order

**Expected Behavior:**
- After completing the current order, the robot will return to the Kitchen to collect food for the next order and continue the process accordingly.
