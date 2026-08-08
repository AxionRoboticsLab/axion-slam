from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    maps_dir = LaunchConfiguration('maps_dir')
    map_topic = LaunchConfiguration('map_topic')

    fastdds_xml = PathJoinSubstitution([
        FindPackageShare('axion_slam'),
        'config',
        'fastdds_no_shm.xml',
    ])

    return LaunchDescription([
        SetEnvironmentVariable(name='FASTRTPS_DEFAULT_PROFILES_FILE', value=fastdds_xml),
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
