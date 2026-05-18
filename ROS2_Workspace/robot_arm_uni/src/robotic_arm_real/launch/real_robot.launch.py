import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import SetParameter, Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # ── Launch Arguments ─────────────────────────────────────────────────────
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyACM0',
        description='Serial port the Arduino is connected to'
    )
    port = LaunchConfiguration('port')

    # ── Package Paths ────────────────────────────────────────────────────────
    moveit_config_pkg = FindPackageShare('robotic_arm_moveit_config').find('robotic_arm_moveit_config')
    description_pkg   = get_package_share_directory('robotic_arm_description')
    urdf_file         = os.path.join(description_pkg, 'urdf', 'robotic_arm.urdf.xacro')
    controllers_yaml  = os.path.join(description_pkg, 'config', 'controllers.yaml')

    # ── Generate robot_description from xacro ────────────────────────────────
    # ros2_control_node needs robot_description as a direct parameter.
    # It subscribes to ~/robot_description (a different topic than /robot_description),
    # so we must pass it explicitly here.
    robot_description_content = ParameterValue(
        Command([
            FindExecutable(name='xacro'), ' ',
            urdf_file,
            ' is_sim:=false',
        ]),
        value_type=str
    )
    robot_description = {'robot_description': robot_description_content}

    # ── No sim time on real hardware ─────────────────────────────────────────
    sim_time_param = SetParameter(name='use_sim_time', value=False)

    # ── Robot State Publisher ────────────────────────────────────────────────
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_pkg, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
        }.items()
    )

    # ── ros2_control node ────────────────────────────────────────────────────
    # robot_description passed directly so the hardware plugin loads immediately
    # without waiting on a topic.
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_yaml],
        output='screen'
    )

    # ── Spawn controllers ────────────────────────────────────────────────────
    # joint_state_broadcaster must come first so arm/gripper controllers
    # can read joint states when they activate.
    spawn_jsb = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    spawn_arm = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['arm_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    spawn_gripper = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['gripper_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # ── MoveIt move_group ────────────────────────────────────────────────────
    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_pkg, 'launch', 'move_group.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
        }.items()
    )

    # ── RViz with MoveIt plugin ──────────────────────────────────────────────
    moveit_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_config_pkg, 'launch', 'moveit_rviz.launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
        }.items()
    )

    # ── Assemble ─────────────────────────────────────────────────────────────
    return LaunchDescription([
        port_arg,
        sim_time_param,

        # RSP publishes /robot_description for MoveIt and RViz
        rsp,

        # ros2_control_node gets robot_description as a parameter (not via topic)
        ros2_control_node,

        # Controllers — JSB must activate before arm/gripper
        spawn_jsb,
        spawn_arm,
        spawn_gripper,

        # MoveIt
        move_group,
        moveit_rviz,
    ])