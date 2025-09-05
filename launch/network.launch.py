#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='diagnosis_pkg_acu_pkg',
            executable='network_diag',
            name='network_diagnostics',
            output='screen',
            parameters=[{
                # set your defaults here:
                'use_sim_time': False,
                'publish_hz': 2.0,
                # hardcoded absolute path (works in your current workspace):
                'config_file': '/home/bullwork/Trailblazer_hamdan/src/diagnosis_pkg_acu_pkg/diagnosis_pkg_acu_pkg/config_file/network_diag.yaml',
                # if you prefer portable path resolution, replace with:
                # 'config_file': None  # and set it in code using get_package_share_directory
            }],
        )
    ])
