from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        
        Node(
            package='slam_nav',
            executable='amiga_node',
            name='amiga_node',
            output='screen'
        ),

        # Node(
        #     package='slam_nav',
        #     executable='vel_node',
        #     name='vel_node',
        #     output='screen'
        # ),

        Node(
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
    ])
