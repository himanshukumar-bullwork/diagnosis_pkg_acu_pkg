#!/usr/bin/env python3
import os
import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context)
    publish_hz   = float(LaunchConfiguration('publish_hz').perform(context))
    platform_fp  = LaunchConfiguration('platform_file').perform(context)
    configs_dir  = LaunchConfiguration('configs_dir').perform(context)

    # Defaults if platform file can’t be read
    enabled_defaults = {
        'can': True,
        'network': True,
        'gps_base': True,
        'gps_rover': True,
        'lidar': True,
        'zed': False,
        'rl': True,
        'nav2': True,
        # 'vehicle': False,  # no known node mapping; we’ll just log if present
    }

    try:
        with open(platform_fp, 'r') as f:
            platform = yaml.safe_load(f) or {}
        enabled = (platform.get('platform', {}).get('enabled', {})) or {}
    except Exception as e:
        print(f"[platform_launch] WARN: failed to read '{platform_fp}': {e}")
        enabled = enabled_defaults

    # Helper to read a bool with a default
    def is_true(key, default=False):
        val = enabled.get(key, default)
        return bool(val)

    # Build nodes conditionally
    nodes = []

    def node_params(config_filename):
        return [{
            'use_sim_time': (use_sim_time.lower() == 'true'),
            'publish_hz': publish_hz,
            'config_file': os.path.join(configs_dir, config_filename),
        }]

    # CAN
    if is_true('can', False):
        nodes.append(Node(
            package='diagnosis_pkg_acu_pkg',
            executable='can',
            name='can_diagnostics',
            output='screen',
            respawn=True, respawn_delay=2.0,
            parameters=node_params('can_diag.yaml'),
        ))

    # GPS → launch if either base or rover is enabled
    if is_true('gps_base', False) or is_true('gps_rover', False):
        nodes.append(Node(
            package='diagnosis_pkg_acu_pkg',
            executable='gps',
            name='gsp_diagnostics',
            output='screen',
            respawn=True, respawn_delay=2.0,
            parameters=node_params('gps_diag.yaml'),
        ))

    # Nav2
    if is_true('nav2', False):
        nodes.append(Node(
            package='diagnosis_pkg_acu_pkg',
            executable='nav2_diagnosis',
            name='nav2_diagnostics',
            output='screen',
            respawn=True, respawn_delay=2.0,
            parameters=node_params('nav2_diag.yaml'),
        ))

    # Network
    if is_true('network', False):
        nodes.append(Node(
            package='diagnosis_pkg_acu_pkg',
            executable='network_diag',
            name='network_diagnostics',
            output='screen',
            respawn=True, respawn_delay=2.0,
            parameters=node_params('network_diag.yaml'),
        ))

    # Robot Localisation (EKF core)
    if is_true('rl', False):
        nodes.append(Node(
            package='diagnosis_pkg_acu_pkg',
            executable='robot_localization_ekf_core',
            name='robot_localization_ekf',
            output='screen',
            respawn=True, respawn_delay=2.0,
            parameters=node_params('ekf_diag.yaml'),
        ))

    # ZED
    if is_true('zed', False):
        nodes.append(Node(
            package='diagnosis_pkg_acu_pkg',
            executable='zed_x_diag',
            name='zed_diagnostics',
            output='screen',
            respawn=True, respawn_delay=2.0,
            parameters=node_params('zed_min_diag.yaml'),
        ))

    # LiDAR (no YAML/parameters)
    if is_true('lidar', False):
        nodes.append(Node(
            package='diagnosis_pkg_acu_pkg',
            executable='lidar',
            name='lidar_diagnostics',
            output='screen',
            respawn=True, respawn_delay=2.0,
            # No parameters on purpose
        ))

    # Vehicle flag present but no known node mapping in this package
    if is_true('vehicle', False):
        print("[platform_launch] NOTE: 'vehicle' is enabled, but no node mapping is defined. Skipping.")

    # Pretty log of what we’re launching
    def onoff(k):
        return "ON " if is_true(k, enabled_defaults.get(k, False)) else "OFF"

    print("[platform_launch] enabled flags:")
    print(f"  can={onoff('can')}  network={onoff('network')}  gps_base={onoff('gps_base')}  gps_rover={onoff('gps_rover')}")
    print(f"  lidar={onoff('lidar')}  zed={onoff('zed')}  rl={onoff('rl')}  nav2={onoff('nav2')}  vehicle={onoff('vehicle')}")

    return nodes


def generate_launch_description():
    # Defaults you gave:
    default_platform = "/home/bullwork/Trailblazer_hamdan/src/core/core/platform_configs/platform.yaml"
    default_configs  = "/home/bullwork/Trailblazer_hamdan/src/diagnosis_pkg_acu_pkg/diagnosis_pkg_acu_pkg/config_file"

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use /clock for all diagnostics nodes'
        ),
        DeclareLaunchArgument(
            'publish_hz', default_value='2.0',
            description='Diagnostics publish frequency for nodes that accept it'
        ),
        DeclareLaunchArgument(
            'platform_file', default_value=default_platform,
            description='Absolute path to platform.yaml'
        ),
        DeclareLaunchArgument(
            'configs_dir', default_value=default_configs,
            description='Directory that holds the various *_diag.yaml files'
        ),

        # Build nodes after reading platform.yaml
        OpaqueFunction(function=_launch_setup),
        LogInfo(msg="platform-driven diagnostics launch started"),
    ])
