"""Show the mobile base on its own in RViz.

Expands urdf/eddie_base.urdf.xacro directly, so unlike display_eddie.launch.py it
needs neither the arm nor the gripper descriptions.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory("eddie_description")

    declare_joint_state_gui = DeclareLaunchArgument(
        "joint_state_gui",
        default_value="true",
        description="Launch the joint state gui publisher",
    )

    declare_use_kelo_tulip = DeclareLaunchArgument(
        "use_kelo_tulip",
        default_value="true",
        description="Make the pivot and wheel joints movable",
    )

    declare_rviz_config = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(package_share, "config", "rviz", "eddie.rviz"),
        description="RViz configuration to load",
    )

    base_xacro_file = os.path.join(package_share, "urdf", "eddie_base.urdf.xacro")

    robot_description = ParameterValue(
        Command(
            [
                FindExecutable(name="xacro"),
                " ",
                base_xacro_file,
                " use_kelo_tulip:=",
                LaunchConfiguration("use_kelo_tulip"),
            ]
        ),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # Qt apps; pinned to xcb because the wayland plugin misbehaves here.
    qt_xcb = {"QT_QPA_PLATFORM": "xcb"}

    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher",
        condition=IfCondition(LaunchConfiguration("joint_state_gui")),
        output="screen",
        additional_env=qt_xcb,
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        condition=UnlessCondition(LaunchConfiguration("joint_state_gui")),
        output="screen",
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        output="screen",
        additional_env=qt_xcb,
    )

    return LaunchDescription(
        [
            declare_joint_state_gui,
            declare_use_kelo_tulip,
            declare_rviz_config,
            robot_state_publisher_node,
            joint_state_publisher_gui_node,
            joint_state_publisher_node,
            rviz2_node,
        ]
    )
