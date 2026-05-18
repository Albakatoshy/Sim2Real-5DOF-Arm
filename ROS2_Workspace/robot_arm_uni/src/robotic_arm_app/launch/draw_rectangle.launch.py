from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Load full MoveIt config (includes kinematics.yaml)
    moveit_config = (
        MoveItConfigsBuilder(
            "robotic_arm",
            package_name="robotic_arm_moveit_config"
        )
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )

    draw_rectangle_node = Node(
        package="robotic_arm_app",
        executable="draw_rectangle",
        output="screen",
        parameters=[
            moveit_config.robot_description,            # URDF
            moveit_config.robot_description_semantic,   # SRDF
            moveit_config.robot_description_kinematics, # kinematics.yaml ← KEY
            {"use_sim_time": False},
        ],
    )

    return LaunchDescription([draw_rectangle_node])