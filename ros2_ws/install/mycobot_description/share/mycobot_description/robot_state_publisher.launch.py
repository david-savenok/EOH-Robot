#!/usr/bin/env python3
#
# Author: Addison Sears-Collins
# Date: November 10, 2024
# Description: Display the robotic arm with RViz
#
# This file launches the robot state publisher, joint state publisher,
# and RViz2 for visualizing the mycobot robot.
 
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
 
# Define the arguments for the XACRO file
ARGUMENTS = [
    DeclareLaunchArgument('prefix', default_value='',
                          description='Prefix for robot joints and links'),
    DeclareLaunchArgument('add_world', default_value='true',
                          choices=['true', 'false'],
                          description='Whether to add world link'),
    DeclareLaunchArgument('base_link', default_value='base_link',
                          description='Name of the base link'),
    DeclareLaunchArgument('base_type', default_value='g_shape',
                          description='Type of the base'),
    DeclareLaunchArgument('flange_link', default_value='link6_flange',
                          description='Name of the flange link'),
    DeclareLaunchArgument('gripper_type', default_value='adaptive_gripper',
                          description='Type of the gripper'),
    DeclareLaunchArgument('use_gazebo', default_value='false',
                          choices=['true', 'false'],
                          description='Whether to use Gazebo simulation'),
    DeclareLaunchArgument('use_gripper', default_value='true',
                          choices=['true', 'false'],
                          description='Whether to attach a gripper')
]
 
 
def generate_launch_description():
    # Define filenames
    urdf_package = 'mycobot_description'
    urdf_filename = 'mycobot_280.urdf.xacro'
    rviz_config_filename = 'mycobot_280_description.rviz'
 
    # Set paths to important files
    pkg_share_description = FindPackageShare(urdf_package)
    default_urdf_model_path = PathJoinSubstitution(
        [pkg_share_description, 'urdf', 'robots', urdf_filename])
    default_rviz_config_path = PathJoinSubstitution(
        [pkg_share_description, 'rviz', rviz_config_filename])
 
    # Launch configuration variables
    jsp_gui = LaunchConfiguration('jsp_gui')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    urdf_model = LaunchConfiguration('urdf_model')
    use_rviz = LaunchConfiguration('use_rviz')
    use_sim_time = LaunchConfiguration('use_sim_time')
 
    # Declare the launch arguments
    declare_jsp_gui_cmd = DeclareLaunchArgument(
        name='jsp_gui',
        default_value='true',
        choices=['true', 'false'],
        description='Flag to enable joint_state_publisher_gui')
 
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        name='rviz_config_file',
        default_value=default_rviz_config_path,
        description='Full path to the RVIZ config file to use')
 
    declare_urdf_model_path_cmd = DeclareLaunchArgument(
        name='urdf_model',
        default_value=default_urdf_model_path,
        description='Absolute path to robot urdf file')
 
    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Whether to start RVIZ')
 
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')
 
    robot_description_content = ParameterValue(Command([
        'xacro', ' ', urdf_model, ' ',
        'prefix:=', LaunchConfiguration('prefix'), ' ',
        'add_world:=', LaunchConfiguration('add_world'), ' ',
        'base_link:=', LaunchConfiguration('base_link'), ' ',
        'base_type:=', LaunchConfiguration('base_type'), ' ',
        'flange_link:=', LaunchConfiguration('flange_link'), ' ',
        'gripper_type:=', LaunchConfiguration('gripper_type'), ' ',
        'use_gazebo:=', LaunchConfiguration('use_gazebo'), ' ',
        'use_gripper:=', LaunchConfiguration('use_gripper')
    ]), value_type=str)
 
    # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
    start_robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description_content}])
 
    # Publish the joint state values for the non-fixed joints in the URDF file.
    start_joint_state_publisher_cmd = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=UnlessCondition(jsp_gui))
 
    # Depending on gui parameter, either launch joint_state_publisher or joint_state_publisher_gui
    start_joint_state_publisher_gui_cmd = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(jsp_gui))
 
    # Launch RViz
    start_rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}])
 
    # Create the launch description and populate
    ld = LaunchDescription(ARGUMENTS)
 
    # Declare the launch options
    ld.add_action(declare_jsp_gui_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_urdf_model_path_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_sim_time_cmd)
 
    # Add any actions
    ld.add_action(start_joint_state_publisher_cmd)
    ld.add_action(start_joint_state_publisher_gui_cmd)
    ld.add_action(start_robot_state_publisher_cmd)
    ld.add_action(start_rviz_cmd)
 
    return ld