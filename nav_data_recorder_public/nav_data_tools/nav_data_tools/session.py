"""nav_data_session: dump what this boot actually runs with, once, then exit.

Waits until the nav2 lifecycle nodes are active, then writes <root>/sessions/<session>/:
  nav2_params_merged.yaml   parameters of the nav2 servers as they are now (ros2 param dump format)
  bt_xml/                   the BT xml files bt_navigator points to
  bt_xml_names.txt
  ws_git.txt                rev, branch, dirty files of --ws-src (if given)
  recorder_config.yaml      the recorder topic list
  files/                    copies of --copy-file / $NAV_DATA_SESSION_FILES (e.g. map config)
  env.txt                   ROS_DOMAIN_ID, RMW_IMPLEMENTATION, CYCLONEDDS_URI, map name, NAV_DATA_*
Started by nav_data_recorder.launch.py; it never touches the nav2 processes.
"""
import argparse
import os
import shutil
import subprocess
import sys
import time

import yaml

from nav_data_tools.common import DEFAULT_ROOT, append_event, expand

DEFAULT_NODES = [
    'bt_navigator', 'controller_server', 'planner_server', 'smoother_server', 'behavior_server',
    'velocity_smoother', 'collision_monitor', 'local_costmap/local_costmap',
    'global_costmap/global_costmap', 'docking_server', 'route_server', 'waypoint_follower',
]
WAIT_ACTIVE = ['bt_navigator', 'controller_server', 'planner_server']


def _value(pv):
    from rcl_interfaces.msg import ParameterType as T
    return {
        T.PARAMETER_BOOL: lambda: pv.bool_value,
        T.PARAMETER_INTEGER: lambda: pv.integer_value,
        T.PARAMETER_DOUBLE: lambda: pv.double_value,
        T.PARAMETER_STRING: lambda: pv.string_value,
        T.PARAMETER_BYTE_ARRAY: lambda: list(pv.byte_array_value),
        T.PARAMETER_BOOL_ARRAY: lambda: list(pv.bool_array_value),
        T.PARAMETER_INTEGER_ARRAY: lambda: list(pv.integer_array_value),
        T.PARAMETER_DOUBLE_ARRAY: lambda: list(pv.double_array_value),
        T.PARAMETER_STRING_ARRAY: lambda: list(pv.string_array_value),
    }.get(pv.type, lambda: None)()


def _nest(flat):
    out = {}
    for k in sorted(flat):
        d = out
        parts = k.split('.')
        for p in parts[:-1]:
            d = d.setdefault(p, {})
            if not isinstance(d, dict):
                break
        else:
            d[parts[-1]] = flat[k]
    return out


class Dumper:
    def __init__(self, node, timeout=5.0):
        self.node = node
        self.timeout = timeout

    def _call(self, srv_type, name, req):
        import rclpy
        cli = self.node.create_client(srv_type, name)
        try:
            if not cli.wait_for_service(timeout_sec=self.timeout):
                return None
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(self.node, fut, timeout_sec=self.timeout)
            return fut.result()
        finally:
            self.node.destroy_client(cli)

    def lifecycle_state(self, node_name):
        from lifecycle_msgs.srv import GetState
        r = self._call(GetState, f'/{node_name}/get_state', GetState.Request())
        return r.current_state.label if r else None

    def params(self, node_name):
        from rcl_interfaces.srv import GetParameters, ListParameters
        lr = self._call(ListParameters, f'/{node_name}/list_parameters', ListParameters.Request())
        if lr is None:
            return None
        names = list(lr.result.names)
        flat = {}
        for i in range(0, len(names), 200):
            req = GetParameters.Request()
            req.names = names[i:i + 200]
            gr = self._call(GetParameters, f'/{node_name}/get_parameters', req)
            if gr is None:
                return None
            for n, v in zip(req.names, gr.values):
                flat[n] = _value(v)
        return flat


def _git_info(ws_src):
    ws_src = expand(ws_src)

    def g(*args):
        try:
            return subprocess.run(['git', '-C', ws_src, *args], capture_output=True, text=True,
                                  timeout=20).stdout.strip()
        except (OSError, subprocess.SubprocessError):
            return ''
    dirty = g('status', '--porcelain', '-uno')
    lines = [f'path: {ws_src}', f'rev: {g("rev-parse", "HEAD")}',
             f'branch: {g("rev-parse", "--abbrev-ref", "HEAD")}',
             f'describe: {g("describe", "--always", "--dirty")}',
             f'dirty_files: {len(dirty.splitlines()) if dirty else 0}']
    if dirty:
        lines += ['', '# git status --porcelain -uno (first 200)'] + dirty.splitlines()[:200]
    return '\n'.join(lines) + '\n'


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument('--root', default=DEFAULT_ROOT)
    p.add_argument('--session', required=True)
    p.add_argument('--ws-src', default='', help='source tree to record the git revision of')
    p.add_argument('--copy-file', action='append', default=[],
                   help='extra file to keep with the session (repeatable); also $NAV_DATA_SESSION_FILES '
                        '(colon separated)')
    p.add_argument('--map-name', default='')
    p.add_argument('--recorder-config', default='')
    p.add_argument('--git-rev', default='')
    p.add_argument('--nodes', nargs='*', default=DEFAULT_NODES)
    p.add_argument('--wait', type=float, default=600.0, help='max seconds to wait for nav2 active')
    a, _ = p.parse_known_args(argv)  # ignore --ros-args from launch

    root = expand(a.root)
    out = os.path.join(root, 'sessions', a.session)
    os.makedirs(out, exist_ok=True)

    # files that do not need ROS first, so they exist even if nav2 never comes up
    if a.ws_src:
        with open(os.path.join(out, 'ws_git.txt'), 'w') as f:
            f.write(_git_info(a.ws_src))
    if a.recorder_config and os.path.isfile(a.recorder_config):
        shutil.copy(a.recorder_config, os.path.join(out, 'recorder_config.yaml'))
    extra = a.copy_file + [x for x in os.environ.get('NAV_DATA_SESSION_FILES', '').split(':') if x]
    for src in extra:
        src = expand(src)
        if os.path.isfile(src):
            os.makedirs(os.path.join(out, 'files'), exist_ok=True)
            shutil.copy(src, os.path.join(out, 'files', os.path.basename(src)))
    with open(os.path.join(out, 'env.txt'), 'w') as f:
        f.write(f'map_name={a.map_name}\n')
        keys = sorted(k for k in os.environ if k.startswith(('ROS_', 'RMW_', 'NAV_DATA_')))
        for k in keys + ['CYCLONEDDS_URI', 'FASTRTPS_DEFAULT_PROFILES_FILE', 'HOSTNAME']:
            f.write(f'{k}={os.environ.get(k, "")}\n')

    import rclpy
    rclpy.init(args=None)
    node = rclpy.create_node('nav_data_session')
    d = Dumper(node)
    try:
        t0 = time.monotonic()
        while True:
            states = {n: d.lifecycle_state(n) for n in WAIT_ACTIVE}
            if all(s == 'active' for s in states.values()):
                break
            if time.monotonic() - t0 > a.wait:
                node.get_logger().warn(f'nav2 not active after {a.wait:.0f} s: {states}; dumping anyway')
                break
            time.sleep(5.0)

        dump, missing = {}, []
        for n in a.nodes:
            flat = d.params(n)
            if flat is None:
                missing.append(n)
                continue
            dump['/' + n] = {'ros__parameters': _nest(flat)}
        with open(os.path.join(out, 'nav2_params_merged.yaml'), 'w') as f:
            f.write(f'# dumped by nav_data_session at {time.strftime("%Y-%m-%d %H:%M:%S")}\n')
            if missing:
                f.write(f'# not reachable: {", ".join(missing)}\n')
            yaml.safe_dump(dump, f, sort_keys=False, allow_unicode=True, width=120)

        bt = dump.get('/bt_navigator', {}).get('ros__parameters', {})
        names = []
        for k in ('default_nav_to_pose_bt_xml', 'default_nav_through_poses_bt_xml'):
            v = bt.get(k)
            if v:
                names.append(f'{k}: {v}')
                if os.path.isfile(v):
                    os.makedirs(os.path.join(out, 'bt_xml'), exist_ok=True)
                    shutil.copy(v, os.path.join(out, 'bt_xml', os.path.basename(v)))
        with open(os.path.join(out, 'bt_xml_names.txt'), 'w') as f:
            f.write('\n'.join(names) + '\n')
        append_event(root, 'recorder', 'session_dumped', session=a.session,
                     nodes=len(dump), missing=missing, git=a.git_rev)
        node.get_logger().info(f'session info written to {out} ({len(dump)} nodes)')
    except KeyboardInterrupt:
        return 0
    except Exception:  # noqa: BLE001
        if not rclpy.ok():        # stopped (Ctrl+C, SIGTERM, shutdown) before nav2 was up: fine
            return 0
        raise
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
