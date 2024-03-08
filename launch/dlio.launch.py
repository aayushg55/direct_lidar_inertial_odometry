#
#   Copyright (c)     
#
#   The Verifiable & Control-Theoretic Robotics (VECTR) Lab
#   University of California, Los Angeles
#
#   Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez
#   Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu
#
import launch_ros
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition   
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    current_pkg = FindPackageShare('direct_lidar_inertial_odometry')

    # Set default arguments
    rviz = LaunchConfiguration('rviz', default='false')
    pointcloud_topic = LaunchConfiguration('pointcloud_topic', default='/luminar_merged')
    imu_topic = LaunchConfiguration('imu_topic', default='/gps_bot/imu')
    gps_pose_topic = LaunchConfiguration('gps_pose_topic', default='/gps_bot/pose')
    gps_orientation_topic = LaunchConfiguration('gps_orientation_topic', default='/gps_bot/orientation')
    output_pose_topic = LaunchConfiguration('output_pose_topic', default='/dlio/odom_node/pose')
    gps_denied_topic = LaunchConfiguration('gps_denied_topic', default='/gps_is_denied')

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
    # Load parameters
    dlio_yaml_path = PathJoinSubstitution([current_pkg, 'cfg', 'dlio.yaml'])
    dlio_params_yaml_path = PathJoinSubstitution([current_pkg, 'cfg', 'params.yaml'])

    # DLIO Odometry Node
    dlio_odom_node = Node(
        package='direct_lidar_inertial_odometry',
        executable='dlio_odom_node',
        output='screen',
        parameters=[dlio_yaml_path, dlio_params_yaml_path],
        remappings=[
            ('pointcloud', pointcloud_topic),
            ('imu', imu_topic),
            ('gps_pose', gps_pose_topic),
            ('gps_orientation', gps_orientation_topic),            
            ('odom', 'dlio/odom_node/odom'),
            ('pose', output_pose_topic),
            ('gps_denied', gps_denied_topic),
            ('path', 'dlio/odom_node/path'),
            ('kf_pose', 'dlio/odom_node/keyframes'),
            ('kf_cloud', 'dlio/odom_node/pointcloud/keyframe'),
            ('deskewed', 'dlio/odom_node/pointcloud/deskewed'),
        ],
    )

    # DLIO Mapping Node
    dlio_map_node = Node(
        package='direct_lidar_inertial_odometry',
        executable='dlio_map_node',
        output='screen',
        parameters=[dlio_yaml_path, dlio_params_yaml_path],
        remappings=[
            ('keyframes', 'dlio/odom_node/pointcloud/keyframe'),
            ('map', 'dlio/map_node/map'),
        ],
    )

    # RViz node
    # rviz_config_path = PathJoinSubstitution([current_pkg, 'launch', 'dlio.rviz'])
    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='dlio_rviz',
    #     arguments=['-d', rviz_config_path],
    #     output='screen',
    #     condition=IfCondition(LaunchConfiguration('rviz'))
    # )

    return LaunchDescription([
        launch_ros.actions.SetParameter(name='use_sim_time', value=True),
        declare_rviz_arg,
        declare_pointcloud_topic_arg,
        declare_imu_topic_arg,
        declare_gps_pose_topic_arg,
        declare_gps_orientation_topic_arg,
        declare_output_pose_topic_arg,
        declare_gps_denied_topic_arg,
        dlio_odom_node,
        dlio_map_node
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments='0.0 0.0 0.0 0.0 0.0 0.0 map odom'.split(' '),
        #     #parameters=[parameter_file],
        #     output='screen'
        # )
        # rviz_node
    ])
