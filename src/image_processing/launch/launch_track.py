from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # DeclareLaunchArgument(
    #     'use_sim_time',
    #     default_value='true',
    #     description='Use simulation (Gazebo) clock if true')

    # clik_cmd_pub = Node(
    #     package='panda_utils',
    #     executable='clik_cmd_pub',
    #     name='clik_cmd_pub',
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #         'ts': LaunchConfiguration('clik_ts'),
    #         'gamma': LaunchConfiguration('clik_gamma'),
    #     }],
    # )
    skeleton_tracking = Node(
        package='image_processing',
        executable='skeleton_track',
        name='skeleton_track',
        parameters=[{
            'use_sim_time': use_sim_time,
        }],
    )

    return LaunchDescription([
        skeleton_tracking
    ])
