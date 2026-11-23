"""Start nav_data_recorder, the one-shot boot info dump (nav_data_session) and the retention /
nightly report loop (nav_data_maintenance). Nothing else needs to be installed on the robot.

Usage (from the robot's start script, after nav2 has been launched):
  nice -n 10 ionice -c2 -n7 ros2 launch nav_data_recorder nav_data_recorder.launch.py robot_id:=robot01

Optional: profile:=<name or path> (default: $NAV_DATA_PROFILE, else 'default' = plain Nav2; names are
the directories installed by nav_data_profiles), topics_file:= / params_file:= to override one file
of the profile, ws_src:=<git source tree>, map_name:=<map in use> (default: $NAV_DATA_MAP_NAME),
maintenance:=false to not start nav_data_maintenance (its settings: params.yaml of the profile).

The session name (YYYYMMDD_HHMMSS_<boot>) is computed once here so that the bag directory
(rec/YYYYMMDD/HHMMSS_<boot>) and the boot info directory (sessions/<session>) match.
"""
import hashlib
import os
import subprocess
import time

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav_data_tools.profile import profile_dir, profile_file


def _boot_short():
    try:
        with open('/proc/sys/kernel/random/boot_id') as f:
            return ''.join(c for c in f.read() if c in '0123456789abcdef')[:4] or '0000'
    except OSError:
        return '0000'


def _git_rev(path):
    if not path:
        return ''
    path = os.path.expanduser(path)
    try:
        rev = subprocess.run(['git', '-C', path, 'rev-parse', '--short=12', 'HEAD'],
                             capture_output=True, text=True, timeout=5).stdout.strip()
        dirty = subprocess.run(['git', '-C', path, 'status', '--porcelain', '-uno'],
                               capture_output=True, text=True, timeout=10).stdout.strip()
        return rev + ('-dirty' if dirty else '') if rev else ''
    except (OSError, subprocess.SubprocessError):
        return ''


def _param_from_file(path, name):
    """nav_data_recorder.ros__parameters.<name> from a ROS params yaml, or ''."""
    try:
        import yaml
        with open(os.path.expanduser(path)) as f:
            doc = yaml.safe_load(f) or {}
        for key, val in doc.items():
            if key.lstrip('/').endswith('nav_data_recorder') and isinstance(val, dict):
                return str(val.get('ros__parameters', {}).get(name, '') or '')
    except (OSError, ValueError, AttributeError, ImportError):
        pass
    return ''


def _sha256(path):
    try:
        with open(path, 'rb') as f:
            return hashlib.sha256(f.read()).hexdigest()
    except OSError:
        return ''


def _setup(context):
    lc = lambda n: LaunchConfiguration(n).perform(context)  # noqa: E731
    profile = lc('profile')
    topics_file = lc('topics_file') or profile_file('topics.yaml', profile)
    params_file = lc('params_file') or profile_file('params.yaml', profile)
    pdir = profile_dir(profile)
    print(f'[nav_data] profile {pdir}: {topics_file}, {params_file}', flush=True)
    root_dir = lc('root_dir') or _param_from_file(params_file, 'root_dir') or '~/nav_data'
    ws_src = lc('ws_src')
    session = time.strftime('%Y%m%d_%H%M%S') + '_' + _boot_short()
    git_rev = _git_rev(ws_src)
    map_name = lc('map_name') or os.environ.get('NAV_DATA_MAP_NAME', '')

    recorder = Node(
        package='nav_data_recorder',
        executable='nav_data_recorder_node',
        name='nav_data_recorder',
        output='screen',
        respawn=True,
        respawn_delay=5.0,
        parameters=[params_file, {
            'root_dir': root_dir,
            'topics_file': topics_file,
            'robot_id': lc('robot_id'),
            'session_name': session,
            'ws_git_rev': git_rev,
            'map_name': map_name,
            'config_sha256': _sha256(topics_file),
            'use_sim_time': lc('use_sim_time').lower() == 'true',
        }],
    )
    actions = [recorder]
    if lc('session_dump').lower() == 'true':
        actions.append(Node(
            package='nav_data_tools',
            executable='nav_data_session',
            name='nav_data_session',
            output='screen',
            arguments=['--root', root_dir, '--session', session, '--ws-src', ws_src,
                       '--recorder-config', topics_file, '--git-rev', git_rev,
                       '--map-name', map_name],
        ))
    if lc('maintenance').lower() == 'true':
        actions.append(Node(
            package='nav_data_tools',
            executable='nav_data_maintenance',
            name='nav_data_maintenance',
            output='screen',
            respawn=True,
            respawn_delay=60.0,
            arguments=['--root', root_dir, '--params-file', params_file, '--profile', pdir],
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value='robot'),
        DeclareLaunchArgument('root_dir', default_value='',
                              description='default: root_dir in params_file, else ~/nav_data'),
        DeclareLaunchArgument('profile', default_value='',
                              description="name or path; default: $NAV_DATA_PROFILE, else 'default'"),
        DeclareLaunchArgument('topics_file', default_value='', description='default: from profile'),
        DeclareLaunchArgument('params_file', default_value='', description='default: from profile'),
        DeclareLaunchArgument('ws_src', default_value=''),
        DeclareLaunchArgument('map_name', default_value=''),
        DeclareLaunchArgument('session_dump', default_value='true'),
        DeclareLaunchArgument('maintenance', default_value='true',
                              description='retention and nightly report (nav_data_maintenance)'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        OpaqueFunction(function=_setup),
    ])
