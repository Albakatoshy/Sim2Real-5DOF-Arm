import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    AppendEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import SetParameter, Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ── Launch Arguments ─────────────────────────────────────────────────────
    is_sim_arg = DeclareLaunchArgument(
        'is_sim',
        default_value='true',
        description='Set to true for Gazebo simulation, false for real robot'
    )
    is_sim = LaunchConfiguration('is_sim')

    # ── Package Paths ────────────────────────────────────────────────────────
    ros_gz_sim_pkg    = FindPackageShare('ros_gz_sim').find('ros_gz_sim')
    moveit_config_pkg = FindPackageShare('robotic_arm_moveit_config').find('robotic_arm_moveit_config')
    description_pkg   = get_package_share_directory('robotic_arm_description')
    controllers_yaml  = os.path.join(description_pkg, 'config', 'controllers.yaml')

    # ── Shared: use_sim_time for all nodes ───────────────────────────────────
    sim_time_param = SetParameter(name='use_sim_time', value=is_sim)

    # ── SIM ONLY: make Gazebo able to find robot meshes ─────────────────────
    workspace_install_dir = os.path.abspath(os.path.join(description_pkg, '..'))
    set_model_path = AppendEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH', # Updated for Ignition Fortress
            value=workspace_install_dir,
            condition=IfCondition(is_sim)
        )

    # ── SIM ONLY: launch Gazebo ──────────────────────────────────────────────
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_pkg, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': 'empty.sdf -r'}.items(),
        condition=IfCondition(is_sim)
    )

    # ── SHARED: Robot State Publisher ────────────────────────────────────────
    # Publishes robot_description to the /robot_description topic.
    # spawn_entity listens to this topic, so RSP must start before it.
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_pkg, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={
            'use_sim_time': is_sim,
        }.items()
    )

    # ── SIM ONLY: spawn the robot from the robot_description topic ───────────
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'robotic_arm'],
        output='screen',
        condition=IfCondition(is_sim)
    )

    # ── SIM ONLY: bridge Gazebo clock → ROS 2 ───────────────────────────────
# ── SIM ONLY: bridge Gazebo → ROS 2 ───────────────────────────────
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/joint_states@sensor_msgs/msg/JointState[ignition.msgs.Model' # Add this line!
        ],
        output='screen',
        condition=IfCondition(is_sim)
    )

    # ── REAL ROBOT ONLY: standalone ros2_control_node ────────────────────────
    # Gazebo runs its own controller_manager internally via gz_ros2_control,
    # so we only need this node when running on real hardware.
    real_robot_control = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controllers_yaml],
        output='screen',
        condition=UnlessCondition(is_sim)
    )

    # ── REAL ROBOT ONLY: spawn controllers ───────────────────────────────────
    # In sim, Gazebo's gz_ros2_control plugin spawns these automatically.
# ── Spawn controllers (MUST RUN IN SIM TOO) ──────────────────────────────
    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
        # Remove the UnlessCondition so it runs in simulation!
    )

    spawn_arm_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller'],
        output='screen',
    )

    spawn_gripper_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller'],
        output='screen',
        condition=UnlessCondition(is_sim)
    )

    # ── SHARED: MoveIt move_group ────────────────────────────────────────────
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_pkg, 'launch', 'move_group.launch.py')
        ),
        launch_arguments={
            'use_sim_time': is_sim,
        }.items()
    )

    # ── SHARED: RViz with MoveIt plugin ─────────────────────────────────────
    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_pkg, 'launch', 'moveit_rviz.launch.py')
        ),
        launch_arguments={
            'use_sim_time': is_sim,
        }.items()
    )

    # ── Assemble ─────────────────────────────────────────────────────────────
    return LaunchDescription([
        # Arguments first
        is_sim_arg,

        # Shared
        sim_time_param,

        # Sim-only
        set_model_path,
        gazebo,

        # RSP must come before spawn_entity so robot_description is available
        rsp,
        spawn_entity,
        bridge,

        # Real-robot-only
        real_robot_control,
        spawn_jsb,
        spawn_arm_controller,
        spawn_gripper_controller,

        # Shared (MoveIt)
        move_group,
        moveit_rviz,
    ])