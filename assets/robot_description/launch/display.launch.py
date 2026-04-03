from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable


def generate_launch_description():
    pkg_share = FindPackageShare("robot_description")

    # ---------------------------------------------------------
    # Robot Description (Xacro → URDF)
    # ---------------------------------------------------------
    robot_description = Command([
        FindExecutable(name="xacro"),
        " ",
        PathJoinSubstitution([
            pkg_share,
            "urdf",
            "panda.xacro"
        ])
    ])

    robot_description_param = {
        "robot_description": robot_description
    }

    # ---------------------------------------------------------
    # Robot State Publisher
    # ---------------------------------------------------------
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description_param],
    )

    # ---------------------------------------------------------
    # Joint State Publisher GUI
    # ---------------------------------------------------------
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen"
    )

    # ---------------------------------------------------------
    # RViz2
    # ---------------------------------------------------------
    rviz_config = PathJoinSubstitution([
        pkg_share,
        "rviz",
        "config.rviz"
    ])
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])