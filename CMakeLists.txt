cmake_minimum_required(VERSION 3.12)
project(eddie_description)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install the robot description, including the MuJoCo model.
install(DIRECTORY urdf meshes launch config mujoco
  DESTINATION share/${PROJECT_NAME}
)

# Add ament package macros
ament_package()
