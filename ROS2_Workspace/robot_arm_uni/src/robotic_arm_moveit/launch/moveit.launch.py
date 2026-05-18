import os
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ─────────────────────────────────────────
    # PACKAGE PATHS
    # ─────────────────────────────────────────
    robot_description_pkg = get_package_share_directory("robotic_arm_description")
    robot_moveit_pkg      = get_package_share_directory("robotic_arm_moveit")

    # ─────────────────────────────────────────
    # MOVEIT CONFIG
    # ─────────────────────────────────────────
    moveit_config = (
        MoveItConfigsBuilder("robotic_arm", package_name="robotic_arm_moveit")
        .robot_description(
            file_path=os.path.join(
                robot_description_pkg, "urdf", "robotic_arm.urdf.xacro"
            )
        )
        .robot_description_semantic(
            file_path=os.path.join(robot_moveit_pkg, "config", "robotic_arm.srdf")
        )
        .robot_description_kinematics(
            file_path=os.path.join(robot_moveit_pkg, "config", "kinematics.yaml")
        )
        .joint_limits(
            file_path=os.path.join(robot_moveit_pkg, "config", "joint_limits.yaml")
        )
        .trajectory_execution(
            file_path=os.path.join(robot_moveit_pkg, "config", "moveit_controllers.yaml")
        )
        .pilz_cartesian_limits(
            file_path=os.path.join(robot_moveit_pkg, "config", "pilz_cartesian_limits.yaml")
        )
        .planning_pipelines(
            pipelines=["ompl", "pilz_industrial_motion_planner"]
        )
        .to_moveit_configs()
    )

    # ─────────────────────────────────────────
    # GAZEBO
    # ─────────────────────────────────────────
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("robotic_arm_description"),
                "launch",
                "gazebo.launch.py",
            ])
        ]),
    )

    # ─────────────────────────────────────────
    # STATIC TRANSFORM — world → base_link
    # ─────────────────────────────────────────
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
        parameters=[{"use_sim_time": True}],
    )

    # ─────────────────────────────────────────
    # ROBOT STATE PUBLISHER
    # computes mimic joint TF from URDF <mimic> tags
    # ─────────────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": True},
        ],
    )

    # ─────────────────────────────────────────
    # JOINT STATE PUBLISHER
    # reads /joint_states from Gazebo and
    # fills in mimic joints automatically
    # ─────────────────────────────────────────
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": True},
            {"source_list": ["/joint_states"]},
        ],
    )

    # ─────────────────────────────────────────
    # MOVE GROUP
    # ─────────────────────────────────────────
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"use_sim_time": True},
            {"publish_robot_description_semantic": True},
        ],
    )

    # ─────────────────────────────────────────
    # RVIZ
    # ─────────────────────────────────────────
    rviz_config = os.path.join(robot_moveit_pkg, "config", "moveit.rviz")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            moveit_config.planning_pipelines,
            {"use_sim_time": True},
        ],
    )

    # ─────────────────────────────────────────
    # DELAYS — wait for Gazebo to fully start
    # ─────────────────────────────────────────
    delayed_move_group = TimerAction(period=5.0, actions=[move_group_node])
    delayed_rviz       = TimerAction(period=7.0, actions=[rviz_node])

    # ─────────────────────────────────────────
    # LAUNCH — all nodes included ✅
    # ─────────────────────────────────────────
    return LaunchDescription([
        gazebo_launch,
        static_tf_node,
        robot_state_publisher,
        joint_state_publisher_node,   # ← was missing before!
        delayed_move_group,
        delayed_rviz,
    ])