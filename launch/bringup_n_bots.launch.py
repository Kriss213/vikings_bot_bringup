from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource, AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    nodes = []
    
    robot_count = min(int(LaunchConfiguration('robot_count').perform(context)),5)

    positions = [
        (10, 6, -1.7),
        (10, -6, 1.7),
        (-11, -5, 0.0),
        (-7, 0.5, 0.0),
        (2.75, -0.6, 3.14)
    ]


    for i in range(1,robot_count+1):
        namespace = f'vikings_bot_{i}'

        # map server node
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
                'map_file': LaunchConfiguration('map_file'),
                'vikings_bot_name': namespace,
            }.items()
        )
        nodes.append(map_server_node)
        
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
                'x_spawn': str(positions[i-1][0]),#'-4', #'11.0',
                'y_spawn': str(positions[i-1][1]),#str(-2+(i-1)*1.5), #str(-7+(2*(i-1))),
                'z_spawn': '0.2',
                'roll_spawn': '0.0',
                'pitch_spawn': '0.0',
                'yaw_spawn': str(positions[i-1][2])
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
                'x_spawn': str(positions[i-1][0]),
                'y_spawn': str(positions[i-1][1]),#str(-2+(i-1)*1.5),#str(-7+(2*(i-1))),
                'yaw_spawn': str(positions[i-1][2]),
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
        name=f'rviz2_{namespace}',
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
    

    return [
        rviz_node,
    ]+nodes

def generate_launch_description():
    """
    Launch N robots in simulation.
    """
    
    #Robot count argument
    robot_count_arg = DeclareLaunchArgument(
        'robot_count',
        default_value='1',
        description='Number of robots to spawn'
    )
    map_arg = DeclareLaunchArgument(
        'map_file',
        default_value='warehouse_map.yaml',
        description='Map yaml file.'
    )

    return LaunchDescription([
        robot_count_arg,
        map_arg,
        OpaqueFunction(function=launch_setup)
    ])