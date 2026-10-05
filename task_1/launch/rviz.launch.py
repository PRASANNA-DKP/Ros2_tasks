from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():

    rviz_config = os.path.join(
        get_package_share_directory('task_1'),
        'Rviz',
        'rviz_open.rviz'
    )

    return LaunchDescription([

        Node(
            package='task_1',
            executable='goal_tf',
            name='frame_broadcaster',
            output='screen'
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )

    ])