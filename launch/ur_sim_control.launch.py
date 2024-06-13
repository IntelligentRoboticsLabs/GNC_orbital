# Copyright (c) 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the {copyright_holder} nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Denis Stogl
#
# Modified by: Juan Carlos Manzanares Serrano

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import (
    get_package_share_directory
)
import os
import yaml

def load_yaml_file(file_path):
    with open(file_path, 'r') as file:
        return yaml.safe_load(file)

def launch_setup(context, *args, **kwargs):

    # Initialize Arguments
    safety_limits = LaunchConfiguration('safety_limits')
    safety_pos_margin = LaunchConfiguration('safety_pos_margin')
    safety_k_position = LaunchConfiguration('safety_k_position')
    # General arguments
    runtime_config_package = LaunchConfiguration('runtime_config_package')
    description_file = LaunchConfiguration('description_file')
    launch_rviz = LaunchConfiguration('launch_rviz')
    gazebo_gui = LaunchConfiguration('gazebo_gui')

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), 'rviz', 'view_arms.rviz']
    )

    initial_joint_controllers = PathJoinSubstitution(
            [FindPackageShare(runtime_config_package), 'config',  'ur_controllers.yaml']
        )

    path_template = os.path.join(get_package_share_directory('gnc_orbital'), 'worlds', 'lab.sdf.template')
    path_world = os.path.join(get_package_share_directory('gnc_orbital'), 'worlds', 'lab.sdf')

    yaml_file_path = os.path.join(get_package_share_directory('gnc_orbital'), 'params', 'world_params.yaml')
    config = load_yaml_file(yaml_file_path)

    nodes_to_start = []

    with open(path_template, 'r') as file:
        world_content = file.read()
    
    for key, value in config.items():
        placeholder = f'{{{{{key}}}}}'
        world_content = world_content.replace(placeholder, str(value))

        if (key == 'spot1_z_light'):
            world_content = world_content.replace('{{spot1_cylinder_z}}', str((value-0.05)/2))
            world_content = world_content.replace('{{spot1_cylinder_length}}', str(value-0.05))

        if (key == 'spot2_z_light'):
            world_content = world_content.replace('{{spot2_cylinder_z}}', str((value-0.05)/2))
            world_content = world_content.replace('{{spot2_cylinder_length}}', str(value-0.05))

    with open(path_world, 'w') as file:
        file.write(world_content)

    arms_params_file = os.path.join(get_package_share_directory('gnc_orbital'), 'params', 'arms_params.yaml')
    with open(arms_params_file, "r") as stream:
        arms_params = (yaml.safe_load(stream))

    arm1 = arms_params['gnc_orbital']['arm1']
    arm2 = arms_params['gnc_orbital']['arm2']

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare(runtime_config_package), 'urdf/ur', description_file]
            ),
            ' ',
            'safety_limits:=',
            safety_limits,
            ' ',
            'safety_pos_margin:=',
            safety_pos_margin,
            ' ',
            'safety_k_position:=',
            safety_k_position,
            ' ',
            'name:=',
            'gnc_orbital',
            ' ',
            'name1:=',
            arm1['name'],
            ' ',
            'name2:=',
            arm2['name'],
            ' ',
            'ur_type1:=',
            arm1['type'],
            ' ',
            'ur_type2:=',
            arm2['type'],
            ' ',
            'tf_prefix1:=',
            arm1['tf_prefix'],
            ' ',
            'tf_prefix2:=',
            arm2['tf_prefix'],
            ' ',
            'arm1_pose:="', f"{arm1['position'][0]} {arm1['position'][1]} {arm1['position'][2]}", '"',
            ' ',
            'arm1_orientation:="', f"{arm1['orientation'][0]} {arm1['orientation'][1]} {arm1['orientation'][2]}", '"',
            ' ',
            'arm2_pose:="', f"{arm2['position'][0]} {arm2['position'][1]} {arm2['position'][2]}", '"',
            ' ',
            'arm2_orientation:="', f"{arm2['orientation'][0]} {arm2['orientation'][1]} {arm2['orientation'][2]}", '"',
            ' ',
            'sim_gazebo:=true',
            ' ',
            'simulation_controllers:=', initial_joint_controllers,
        ]
    )
    robot_description = {'robot_description': robot_description_content}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='both',
        parameters=[{'use_sim_time': True}, robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    ur3_joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ur3_joint_state_broadcaster'],
    )

    ur10_joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ur10_joint_state_broadcaster'],
    )

    ur3_joint_trajectory_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ur3_joint_trajectory_controller'],
    )

    ur10_joint_trajectory_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['ur10_joint_trajectory_controller'],
    )

    # Spawn robot
    gazebo_spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'ur', '-topic', '/robot_description'],
        output='screen',
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('gazebo_ros'), '/launch', '/gazebo.launch.py']
        ),
        launch_arguments={
            'gui': gazebo_gui
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='log',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(launch_rviz),
    )

    nodes_to_start.append(joint_state_broadcaster_spawner)
    nodes_to_start.append(robot_state_publisher_node)
    nodes_to_start.append(ur3_joint_state_broadcaster_spawner)
    nodes_to_start.append(ur10_joint_state_broadcaster_spawner)
    nodes_to_start.append(ur3_joint_trajectory_spawner)
    nodes_to_start.append(ur10_joint_trajectory_spawner)
    nodes_to_start.append(gazebo_spawn_robot)
    nodes_to_start.append(gazebo)
    nodes_to_start.append(rviz_node)

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    runtime_config_package = LaunchConfiguration('runtime_config_package')
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            'safety_limits',
            default_value='true',
            description='Enables the safety limits controller if true.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'safety_pos_margin',
            default_value='0.15',
            description='The margin to lower and upper limits in the safety controller.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'safety_k_position',
            default_value='20',
            description='k-position factor in the safety controller.',
        )
    )
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            'runtime_config_package',
            default_value='gnc_orbital',
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'description_file',
            default_value='ur.urdf.xacro',
            description='URDF/XACRO description file with the robot.',
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument('launch_rviz', default_value='true', description='Launch RViz?')
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'gazebo_gui', default_value='true', description='Start gazebo with GUI?'
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            'world', default_value=[PathJoinSubstitution(
                [FindPackageShare(runtime_config_package), 'worlds',  'lab.sdf'])]
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])
