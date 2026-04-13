import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import FindExecutable
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ---------------------------------------------------------
    # Gazebo model path fix:
    # Gazebo Classic converts package:// → model:// when spawning
    # via topic. GAZEBO_MODEL_PATH must include the parent of the
    # package share dir so model://robot_description/... resolves.
    # GAZEBO_MODEL_DATABASE_URI="" prevents the network model-fetch
    # hang on startup.
    # ---------------------------------------------------------
    pkg_share_parent = os.path.dirname(get_package_share_directory("robot_description"))

    set_gazebo_model_path = SetEnvironmentVariable(
        name="GAZEBO_MODEL_PATH",
        value=[
            pkg_share_parent,
            ":",
            os.environ.get("GAZEBO_MODEL_PATH", ""),
        ],
    )

    set_gazebo_model_db_uri = SetEnvironmentVariable(
        name="GAZEBO_MODEL_DATABASE_URI",
        value="",
    )

    # ---------------------------------------------------------
    # Arguments
    # ---------------------------------------------------------
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    x_pos = LaunchConfiguration("x", default="0.0")
    y_pos = LaunchConfiguration("y", default="0.0")
    z_pos = LaunchConfiguration("z", default="0.05")

    # ---------------------------------------------------------
    # Robot Description — read the pre-compiled panda.urdf
    # (change to xacro pipeline below if you prefer)
    # ---------------------------------------------------------
    pkg_share = FindPackageShare("robot_description")

    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution([pkg_share, "urdf", "panda.urdf"]),
        ]),
        value_type=str,
    )

    robot_description_param = {"robot_description": robot_description}

    # ---------------------------------------------------------
    # Robot State Publisher
    # ---------------------------------------------------------
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description_param,
            {"use_sim_time": use_sim_time},
        ],
    )

    # ---------------------------------------------------------
    # Joint State Publisher  (publishes zero positions —
    # all joints start at their URDF zero angle)
    # ---------------------------------------------------------
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # ---------------------------------------------------------
    # Gazebo Classic — launch gzserver + gzclient separately.
    # gazebo.launch.py only forwards "gui" and "server" and silently
    # drops all other args (including pause), so we include the two
    # sub-launch files directly so that "pause:=true" is honoured.
    # ---------------------------------------------------------
    gazebo_share = get_package_share_directory("gazebo_ros")

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, "launch", "gzserver.launch.py")
        ),
        launch_arguments={
            "world":   "",
            "verbose": "true",
            "pause":   "true",   # physics paused at startup; press Play in GUI to run
        }.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_share, "launch", "gzclient.launch.py")
        ),
    )

    # ---------------------------------------------------------
    # Spawn robot in Gazebo
    # gazebo_ros spawn_entity reads /robot_description topic
    # ---------------------------------------------------------
    spawn_entity_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        name="spawn_panda",
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-entity", "panda",
            "-x", x_pos,
            "-y", y_pos,
            "-z", z_pos,
        ],
    )

    # ---------------------------------------------------------
    # How to view inertia in Gazebo Classic GUI:
    #   View menu → Inertia  (shows inertia ellipsoids on all links)
    #   View menu → Center of Mass  (shows COM markers)
    # ---------------------------------------------------------

    return LaunchDescription([
        set_gazebo_model_db_uri,
        set_gazebo_model_path,
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="Use simulation (Gazebo) clock"),
        DeclareLaunchArgument("x", default_value="0.0",
                              description="Spawn X position"),
        DeclareLaunchArgument("y", default_value="0.0",
                              description="Spawn Y position"),
        DeclareLaunchArgument("z", default_value="0.1",
                              description="Spawn Z position (slight offset avoids floor clipping)"),

        robot_state_publisher_node,
        joint_state_publisher_node,
        gzserver,
        gzclient,
        spawn_entity_node,
    ])
