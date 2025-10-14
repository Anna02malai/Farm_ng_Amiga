from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory  # kept from original Relay file (unused)
import os  # kept from original Relay file (unused)


def generate_launch_description():

    # ----------------------------
    # Args from both files
    # ----------------------------

    # From pc2l.launch.py 

    declare_scanner = DeclareLaunchArgument(
        name='scanner', default_value='',
        description='Namespace for sample topics'
    )

    # From Relay_topics.launch.py

    sensor_offset = LaunchConfiguration('sensor_offset')
    do_offset = LaunchConfiguration('do_offset')
    pub_odom_tf = LaunchConfiguration('pub_odom_tf')

    declare_sensor_offset = DeclareLaunchArgument(
        'sensor_offset', default_value='0.2',
        description='The sensor offset from base_link to lidar position'
    )
    declare_do_offset = DeclareLaunchArgument(
        'do_offset', default_value='True',
        description='Using offset or not'
    )
    declare_pub_odom_tf = DeclareLaunchArgument(
        'pub_odom_tf', default_value='True',
        description='Publish odom base_link TF'
    )

    # ----------------------------
    # Nodes from pc2l.launch.py
    # ----------------------------

    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        remappings=[('cloud_in', '/cloud_registered_body'), ('scan', '/lidar_scan')],
        parameters=[{
            'target_frame': 'body',
            'transform_tolerance': 0.01,
            'min_height': -1.0,
            'max_height': 1.5,
            'angle_min': -3.141592654,
            'angle_max': 3.141592654,
            'angle_increment': 0.003141592,
            'scan_time': 0.03333,
            'range_min': 0.5,
            'range_max': 40.0,
            'use_inf': True,
            'inf_epsilon': 1.0
        }],
        name='pointcloud_to_laserscan'
    )

    # ----------------------------
    # Nodes from Relay_topics.launch.py
    # ----------------------------

    map_to_odom_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_map_odom',
        arguments=['0.0', '0', '0.0', '0', '0', '0', '1', 'map', 'odom'],
    )

    base_to_laser_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_imu',
        arguments=[sensor_offset, '0', '0', '0', '0', '0', '1', 'base_link', 'laser_frame'],
    )

    base_to_cloud_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_cloud',
        arguments=[sensor_offset, '0', '0', '0', '0', '0', '1', 'base_link', 'cloud_frame'],
    )

    lidar_mount_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_mount_tf',
        arguments=['0', '0', '0.6', '0', '0', '0', 'base_link', 'camera_init']
    )

    relay_topics_node = Node(
        package='fl_nav2_helper',
        executable='relay_topics',
        name='relay_topics',
        output='screen',
        parameters=[{
            'sensor_offset': sensor_offset,
            'do_offset': do_offset,
            'pub_odom_tf': pub_odom_tf
        }]
    )

    #-----------------------------
    # Nodes from amiga_launch
    #-----------------------------

    amiga_node = Node(
            package='slam_nav',
            executable='amiga_node',
            name='amiga_node',
            output='screen'
        )
    
    vel_smoother = Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
				# plugin='nav2_velocity_smoother::VelocitySmoother',
				name='velocity_smoother',
				parameters=[{'smoothing_frequency': 20.0,
                         'feedback': 'OPEN_LOOP',
                         'max_velocity': [0.7, 0.0, 0.6],
                         'max_accel':    [0.8, 0.0, 1.2],
                         'max_decel':    [-1.0, 0.0, -1.5],
                         'velocity_timeout': 0.4}],
				remappings= [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]
        ) 

    # ----------------------------
    # Assemble LD
    # ----------------------------

    ld = LaunchDescription()

    # Declarations (include all to preserve original behavior)
    ld.add_action(declare_scanner)
    ld.add_action(declare_sensor_offset)
    ld.add_action(declare_do_offset)
    ld.add_action(declare_pub_odom_tf)

    # Relay/TF nodes
    ld.add_action(map_to_odom_node)
    ld.add_action(base_to_laser_node)
    ld.add_action(base_to_cloud_node)
    ld.add_action(lidar_mount_tf)
    ld.add_action(relay_topics_node)

    # PointCloud → LaserScan node
    ld.add_action(pointcloud_to_laserscan_node)

    # Amiga + Velocity Smoother nodes
    ld.add_action(amiga_node)
    ld.add_action(vel_smoother)

    return ld
