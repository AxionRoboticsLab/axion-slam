from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    maps_dir = LaunchConfiguration('maps_dir')
    map_topic = LaunchConfiguration('map_topic')
    rosbridge_address = LaunchConfiguration('rosbridge_address')
    rosbridge_port = LaunchConfiguration('rosbridge_port')

    fastdds_xml = PathJoinSubstitution([
        FindPackageShare('axion_slam'),
        'config',
        'fastdds_no_shm.xml',
    ])

    rosbridge_launch = PathJoinSubstitution([
        FindPackageShare('rosbridge_server'),
        'launch',
        'rosbridge_websocket_launch.xml',
    ])

    return LaunchDescription([
        # 云 VM / 受限 /dev/shm 下禁用 FastDDS SHM，避免 RTPS_TRANSPORT_SHM Error
        SetEnvironmentVariable(name='FASTRTPS_DEFAULT_PROFILES_FILE', value=fastdds_xml),
        DeclareLaunchArgument(
            'maps_dir',
            default_value='~/data/maps',
            description='Directory for {map_name}.pgm/.yaml',
        ),
        DeclareLaunchArgument(
            'map_topic',
            default_value='/map',
            description='OccupancyGrid topic for axion-console',
        ),
        DeclareLaunchArgument(
            'rosbridge_address',
            default_value='0.0.0.0',
            description='rosbridge bind address (use 127.0.0.1 behind nginx)',
        ),
        DeclareLaunchArgument(
            'rosbridge_port',
            default_value='9090',
            description='rosbridge websocket port',
        ),
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(rosbridge_launch),
            launch_arguments={
                'address': rosbridge_address,
                'port': rosbridge_port,
            }.items(),
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
