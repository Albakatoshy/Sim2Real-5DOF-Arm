import os
import re

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def _read_urdf_strip_xml_decl(path: str) -> str:
    with open(path, "r", encoding="utf-8") as f:
        txt = f.read()
    # spawn_entity.py (lxml) يكره الـ xml declaration في unicode
    txt = re.sub(r'^\s*<\?xml[^>]*\?>\s*', '', txt, count=1)
    return txt


def generate_launch_description():
    pkg_share = get_package_share_directory("robotic_arm_description")
    urdf_path = os.path.join(pkg_share, "urdf", "robotic_arm.urdf")
    robot_desc = _read_urdf_strip_xml_decl(urdf_path)

    # args
    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    paused = LaunchConfiguration("paused")
    jsp_gui = LaunchConfiguration("jsp_gui")
    world = LaunchConfiguration("world")
    x = LaunchConfiguration("x")
    y = LaunchConfiguration("y")
    z = LaunchConfiguration("z")

    declare_args = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("paused", default_value="false"),
        DeclareLaunchArgument("jsp_gui", default_value="true"),
        DeclareLaunchArgument(
            "world",
            default_value="/usr/share/gazebo-11/worlds/empty.world"
        ),
        DeclareLaunchArgument("x", default_value="0.0"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("z", default_value="0.2"),
    ]

    # Gazebo env: خلي Gazebo يدور على models محليًا فقط + ~/.gazebo/models
    # (هناك حاطين model folder للـ robotic_arm_description)
    gazebo_model_path = ":".join([
        "/usr/share/gazebo-11/models",
        os.path.join(os.path.expanduser("~"), ".gazebo", "models"),
    ])

    env = [
        SetEnvironmentVariable("GAZEBO_MODEL_DATABASE_URI", ""),   # يمنع download
        SetEnvironmentVariable("GAZEBO_MODEL_PATH", gazebo_model_path),
    ]

    # robot_state_publisher
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time, "robot_description": robot_desc}],
    )

    # joint_state_publisher_gui (مفيد ل RViz فقط، مش ضروري ل Gazebo)
    jsp = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
        condition=IfCondition(jsp_gui),
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # Gazebo (عن طريق gazebo_ros launch الرسمي)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gazebo.launch.py")
        ),
        launch_arguments={
            "world": world,
            "gui": gui,
            "paused": paused,
        }.items(),
    )

    # spawn robot from robot_description topic
    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-entity", "robotic_arm",
            "-topic", "robot_description",
            "-x", x, "-y", y, "-z", z,
            "-unpause",
        ],
    )

    # RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=[],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(declare_args + env + [gazebo_launch, rsp, jsp, spawn, rviz])
