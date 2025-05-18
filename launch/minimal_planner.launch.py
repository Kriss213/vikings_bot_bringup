from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # static publisher
    stat_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub',
            arguments=['0.0', '0.0', '0.0', '0', '0', '0', '1', 'map', 'base_link']
        )

    # map server
    map_server_node = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('vikings_bot_map_server'),
                'launch',
                'minimal_map_server.launch.py'
            ])
        ])
    )

    # path planner server for each robot
    # TODO robot_count as param
    robot_count=5
    path_planner_nodes = []
    for i in range(1,robot_count+1):
        # create obstacle topics omitting own obstacle topic
        obs_topics = [f"/R{r}/position" for r in range(1,robot_count+1) if r!=i]
        pos_topic = f"/R{i}/position"

        path_planner_node = IncludeLaunchDescription(
            AnyLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('vikings_bot_path_planner_server'),
                    'launch',
                    'minimal_planner.launch.py'
                ]) 
            ]),
            launch_arguments={
                'namespace': f"R{i}",
                'obstacle_topics': str(obs_topics),
                'position_topic': pos_topic,
            }.items()
        )
        path_planner_nodes.append(path_planner_node)

    # rviz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_vikings_bot',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('vikings_bot_bringup'),
            'rviz',
            'minimal_planner_debug.rviz'
        ])],
        parameters=[{
            'use_sim_time': False,
        }]
    )
    

    return LaunchDescription([
        rviz_node,
        map_server_node,
        stat_tf,
        #path_planner_node,
    ] + path_planner_nodes)