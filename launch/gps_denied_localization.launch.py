from launch import LaunchDescription
import launch_ros
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition   
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    dlio_path = get_package_share_directory('direct_lidar_inertial_odometry')
    pc_merger = get_package_share_directory('pc_merger')
    

    # Set default arguments
    rviz = LaunchConfiguration('rviz', default='false')
    pointcloud_topic = LaunchConfiguration('pointcloud_topic', default='/luminar_merged')
    imu_topic = LaunchConfiguration('imu_topic', default='/gps_bot/imu')
    gps_pose_topic = LaunchConfiguration('gps_pose_topic', default='/gps_bot/pose')
    gps_orientation_topic = LaunchConfiguration('gps_orientation_topic', default='/gps_bot/orientation')
    output_pose_topic = LaunchConfiguration('output_pose_topic', default='/dlio/odom_node/pose')
    gps_denied_topic = LaunchConfiguration('gps_denied_topic', default='/gps_is_denied')
    map_file = LaunchConfiguration('map_file', default='map.bin')
    run_mode = LaunchConfiguration('run_mode', default=True)

    # Define arguments
    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value=rviz,
        description='Launch RViz'
    )
    declare_pointcloud_topic_arg = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value=pointcloud_topic,
        description='Pointcloud topic name'
    )
    declare_imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value=imu_topic,
        description='IMU topic name'
    )
    declare_gps_pose_topic_arg = DeclareLaunchArgument(
        'gps_pose_topic',
        default_value=gps_pose_topic,
        description='GPS pose topic name'
    )
    declare_gps_orientation_topic_arg = DeclareLaunchArgument(
        'gps_orientation_topic',
        default_value=gps_orientation_topic,
        description='GPS orientation topic name'
    )
    declare_output_pose_topic_arg = DeclareLaunchArgument(
        'output_pose',
        default_value=output_pose_topic,
        description='Output pose topic name'
    )
    declare_gps_denied_topic_arg = DeclareLaunchArgument(
        'gps_denied_topic',
        default_value=gps_denied_topic,
        description='GPS denied topic name'
    )

    dlio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([dlio_path, '/launch/dlio.launch.py']),
        launch_arguments={'map_file': map_file, 'run_mode': run_mode}.items()
    )

    use_4_lidar = LaunchConfiguration('use_4_lidar', default=True)
    declare_use_4_lidar_arg = DeclareLaunchArgument(
        'use_4_lidar',
        default_value=use_4_lidar,
        description='Use a merged point cloud from 4 lidars'
    )

    pc_merger_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([pc_merger, '/launch/run.launch.py']),
    )
    
    gps_denied_node = Node(
        package='gps_denied_publisher',
        executable='gps_denied_publisher',
        name='gps_denied_publisher',
        parameters=[{'run_mode': run_mode}]
    )
    
    return LaunchDescription([
        launch_ros.actions.SetParameter(name='use_sim_time', value=True),
        declare_rviz_arg,
        declare_pointcloud_topic_arg,
        declare_imu_topic_arg,
        declare_gps_pose_topic_arg,
        declare_gps_orientation_topic_arg,
        declare_output_pose_topic_arg,
        declare_gps_denied_topic_arg,
        declare_use_4_lidar_arg,
        dlio_launch,
        pc_merger_launch,
        gps_denied_node
    ])