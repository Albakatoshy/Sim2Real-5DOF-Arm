from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('robotic_arm_description')
    urdf_path = os.path.join(pkg, 'urdf', 'robotic_arm.urdf.xacro')

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': open(urdf_path).read()}]
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen'
    )

    rviz_config = os.path.join(pkg, 'rviz', 'view.rviz')
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config] if os.path.exists(rviz_config) else []
    )

    return LaunchDescription([
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz2
    ])
