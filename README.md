# Eddie Description

Robot description for the Eddie robot in form of URDF files, controllers and meshes.
Gripper not added.

## Environment

  Ubuntu: 24.04

  ROS2: Jazzy

## Setup

- Clone this repository into your workspace

  ```bash
  # Create workspace
  mkdir -p ~/eddie_ws/src && cd ~/eddie_ws/src

  # Clone repository
  git clone https://github.com/secorolab/eddie_description
  ```

- Build workspace

  ```bash
  cd ~/eddie_ws

  colcon build
  ```

- Clone ependendent packages

    ```bash
  cd ~/eddie_ws/src

  vcs import < eddie_description/dep.repos
  ```

## Usage

- View robot in rviz

  ```bash
  cd ~/eddie_ws

  # Source workspace
  source install/setup.bash

  # View robot in rviz
  ros2 launch eddie_description display_eddie.launch.py joint_state_gui:=false
  ```

- View robot in rviz with joint state gui

  ```bash
  ros2 launch eddie_description display_eddie.launch.py joint_state_gui:=true
  ```

## ROS Independent

- The URDF and the mesh files for the whole Eddie robot including arms and Robotiq-2F-85 gripper are
  available in the [eddie_urdf](eddie_urdf) folder.

## Eddie

TODO: Image of Eddie with arms in rviz to be added
