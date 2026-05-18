#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    urdf_file = os.path.join(
        get_package_share_directory("robotic_arm_description"),
        "urdf",
        "robotic_arm.urdf.xacro"
    )

    model_arg = DeclareLaunchArgument(
        "model",
        default_value=urdf_file,
        description="Path to the robot URDF file"
    )

    env_var = SetEnvironmentVariable(
        "GAZEBO_MODEL_PATH",
        os.path.join(get_package_prefix("robotic_arm_description"), "share")
    )

    robot_description_content = ParameterValue(Command(["xacro ", urdf_file]), value_type=str)

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description_content, 
            "use_sim_time": True  # <--- CRITICAL FOR RVIZ
        }]
    )

    # Use the unified gazebo launch file which automatically loads the ROS Factory plugins!
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gazebo.launch.py"
            )
        )
    )

    spawn_robot = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "robotic_arm"
        ],
        output="screen"
    )

    spawn_robot_delayed = TimerAction(
        period=5.0,  
        actions=[spawn_robot]
    )

    # =================================================================
    # NEW: SPAWN CONTROLLERS FOR ROS 2
    # =================================================================
    
    # List of controllers we defined in your controllers.yaml
    controllers = [
        "joint_state_controller",
        "rotating_base_controller", # <--- ADD THIS
        "shoulder_controller",
        "upper_arm_controller",
        "wrist_controller",
        "gripper_base_controller",
        "gripper_controller"
    ]
    controller_spawner_nodes = []
    for controller in controllers:
        controller_spawner_nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[controller, "--controller-manager", "/controller_manager"],
                output="screen"
            )
        )

    # CRITICAL: We must wait for the robot to actually exist in Gazebo 
    # before we try to attach controllers to it. This event handler waits 
    # for the spawn_robot node to finish before triggering the controllers.
    load_controllers_event = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=controller_spawner_nodes,
        )
    )

    rviz_config_file = os.path.join(get_package_share_directory('robotic_arm_description'), 'rviz', 'urdf.rviz')
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    )

    return LaunchDescription([
            model_arg,
            env_var,
            robot_state_publisher_node,
            gazebo,
            spawn_robot_delayed,  
            load_controllers_event,
            rviz_node, # <--- Added here
        ])