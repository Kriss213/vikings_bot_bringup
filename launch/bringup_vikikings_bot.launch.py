from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

import rclpy
from rclpy.node import Node as RclpyNode


def launch_setup(context, *args, **kwargs):
    # check for running nodes

    nodes = []
    robot_nr = int(LaunchConfiguration('robot_nr').perform(context))
    
    # map server
    map_server_node = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('vikings_bot_map_server'),
                'launch',
                'map_server.launch.py'
            ])
        ]),
        launch_arguments={
            'use_rviz': 'False',
            'map_file': 'simulation_map.yaml',
        }.items()
    )

    if robot_nr == 1:
        nodes.append(map_server_node)  

    namespace = f'vikings_bot_{robot_nr}'
    
    # Spawn robot in gazebo
    spawn_node = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('vikings_bot_gazebo'),
                'launch',
                'spawn.launch.xml'
            ])
        ]),
        launch_arguments={
            'vikings_bot_name': namespace,
            #'robot_file': 'vikings_bot.xacro',
            'use_sim': 'true',
            'x_spawn': '-4', #'11.0',
            'y_spawn': str(-2+(robot_nr-1)*1.2), #str(-7+(2*(i-1))),
            'z_spawn': '0.2',
            'roll_spawn': '0.0',
            'pitch_spawn': '0.0',
            'yaw_spawn': '0'
        }.items()
    )

    

    # localization
    localization_node = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('vikings_bot_localization_server'),
                'launch',
                'spawn_localization.launch.py'
            ])
        ]),
        launch_arguments={
            'vikings_bot_name': namespace,
            'x_spawn': '-4',# 11.0',
            'y_spawn': str(-2+(robot_nr-1)*1.2),#str(-7+(2*(i-1))),
            'yaw_spawn': '0',
        }.items()
    )

    # path planner server
    path_planner_node = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('vikings_bot_path_planner_server'),
                'launch',
                'spawn_path_planner.launch.py'
            ])
        ]),
        launch_arguments={
            'namespace': namespace,
        }.items()
    )


    nodes.append(spawn_node)
    nodes.append(localization_node)
    nodes.append(path_planner_node)

    # rviz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_vikings_bot',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('vikings_bot_bringup'),
            'rviz',
            'vikings_bot.rviz'
        ])],
        parameters=[{
            'use_sim_time': True,
        }]
    )
    
    if robot_nr == 1:
        nodes.append(rviz_node)
    
    return nodes

def generate_launch_description():
    """
    Launch N robots in simulation.
    """
    
    #Robot count argument
    robot_nr_arg = DeclareLaunchArgument(
        'robot_nr',
        default_value='1',
        description='Robot number'
    )
    

    return LaunchDescription([
        robot_nr_arg,
        OpaqueFunction(function=launch_setup)
    ])