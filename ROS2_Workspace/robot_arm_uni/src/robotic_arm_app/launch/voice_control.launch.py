import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("robotic_arm", package_name="robotic_arm_moveit_config")
        .to_moveit_configs()
    )

    moveit_params = moveit_config.to_dict()
    moveit_params['use_sim_time'] = True  # Merge into a single dict

    voice_to_moveit_node = Node(
        package='robotic_arm_app',
        executable='moveit_interface',
        name='voice_to_moveit',
        output='screen',
        parameters=[moveit_params],  # Single parameters list with everything
        remappings=[
            ('joint_states', '/joint_states')
        ]
    )

    return LaunchDescription([
        voice_to_moveit_node,
    ])