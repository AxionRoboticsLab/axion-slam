from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    maps_dir = LaunchConfiguration('maps_dir')
    map_topic = LaunchConfiguration('map_topic')

    return LaunchDescription([
        DeclareLaunchArgument(
            'maps_dir',
            default_value='~/data/maps',
            description='Directory for {name}_2dmap.pgm/.yaml',
        ),
        DeclareLaunchArgument(
            'map_topic',
            default_value='/map',
            description='OccupancyGrid topic for axion-console',
        ),
        Node(
            package='axion_slam',
            executable='map_manager_node',
            name='map_manager',
            output='screen',
            parameters=[{
                'maps_dir': maps_dir,
                'map_topic': map_topic,
                'publish_rate_hz': 2.0,
            }],
        ),
    ])
