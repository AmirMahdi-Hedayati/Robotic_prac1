from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():

    pkg_share = FindPackageShare(package='prac1_robot_description').find('prac1_robot_description')
    default_model_path = os.path.join(pkg_share, 'src', 'description', 'robot.urdf')
    default_rviz_config_path = os.path.join(pkg_share, 'rviz', 'display.rviz')

    # Read URDF file
    with open(default_model_path, 'r') as f:
        robot_desc = f.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'use_sim_time': False},
            {'robot_description': robot_desc}   # FIXED PARAMETER NAME
        ],
        output='both'
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='True'),
        DeclareLaunchArgument('model', default_value=default_model_path),
        DeclareLaunchArgument('rvizconfig', default_value=default_rviz_config_path),

        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
    ])