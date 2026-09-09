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

- A MuJoCo model of the mobile base, [mujoco/mobile_platform.xml](mujoco/mobile_platform.xml),
  built from the same meshes and the same kinematics and inertial data as the URDF. Its drive
  and wheel frames agree with the URDF's to below a micrometre. Export the meshes from the CAD
  with `mujoco/export_step_meshes.py` and `mujoco/export_torso.py`.

## Eddie

TODO: Image of Eddie with arms in rviz to be added
