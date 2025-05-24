from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    jacobian_calculator = Node(
        package='panda_utils',
        executable='calc_jacobian_server',
        parameters=[{
            'use_sim_time': False,
        }],
    )
    forward_kine = Node(
        package='panda_utils',
        executable='calc_forward_kine_server',
        parameters=[{
            'use_sim_time': False,
        }],
    )
    pos_cmds_joints = Node(
        package='panda_utils',
        executable='send_joints_cmd_server',
        parameters=[{
            'use_sim_time': False,
        }],
    )
    # clik = Node(
    #     package='panda_utils',
    #     executable='calc_clik_server',
    #     parameters=[{
    #         'use_sim_time': False,
    #     }],
    #     # arguments=['--ros-args', '--log-level', 'DEBUG'],
    # )

    return LaunchDescription([
        forward_kine,
        jacobian_calculator,
        # clik,
        pos_cmds_joints
    ])
