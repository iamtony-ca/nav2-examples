"""nav_data_nightly: index unusual intervals of one day and write a daily report.

  ros2 run nav_data_tools nav_data_nightly --root ~/nav_data [--date 20261120]

Reads only small topics (no scans, no costmaps). Output:
  index/<date>.jsonl          one line per interval: start, end, rule, score, file, detail
  index/<date>_report.md      counts per rule and the top intervals
  pinned/<time>_<reason>/     +-5 min around each /nav_data/mark, exported as standard mcap
Run by nav_data_maintenance after 02:30 (lowest CPU / IO priority); waits while the robot drives.
"""
import argparse
import datetime as _dt
import json
import math
import os
import re
import shutil
import sys
import time
from collections import deque

import numpy as np
import yaml

from nav_data_tools.common import (DEFAULT_ROOT, append_event, day_range_ns, expand, iso_local,
                                   list_rec_files)
from nav_data_tools.profile import profile_file

EXIT_GAVE_UP = 3   # robot kept driving for --max-wait-h; nothing written

GOAL_STATUS = {0: 'UNKNOWN', 1: 'ACCEPTED', 2: 'EXECUTING', 3: 'CANCELING', 4: 'SUCCEEDED',
               5: 'CANCELED', 6: 'ABORTED'}


class Hit:
    __slots__ = ('rule', 'name', 'start', 'end', 'detail', 'count', 'file')

    def __init__(self, rule, name, start, end, detail, file):
        self.rule, self.name, self.start, self.end = rule, name, start, end
        self.detail, self.count, self.file = detail or {}, 1, file

    def duration(self):
        return (self.end - self.start) / 1e9


class Cond:
    """Condition that must hold for min_s; reports [start, end] when it stops holding."""

    def __init__(self, min_s=0.0, file_fn=None):
        self.min_ns = int(min_s * 1e9)
        self.start = None
        self.last = None
        self.detail = None
        self.file_fn = file_fn or (lambda: '')
        self.file = ''

    def update(self, t, on, detail=None):
        if on:
            if self.start is None:
                self.start = t
                self.detail = detail
                self.file = self.file_fn()
            self.last = t
            return None
        return self.close(t)

    def close(self, t=None):
        if self.start is None:
            return None
        # the condition ends when the first "off" sample arrives (t); at end of data, last "on"
        s, e = self.start, (t if t is not None else self.last)
        d = self.detail
        self.start = self.last = None
        if e - s >= self.min_ns:
            return s, e, d, self.file
        return None


def _as_list(v):
    if not v:
        return []
    return list(v) if isinstance(v, (list, tuple)) else [v]


def _yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def _speed(msg):
    tw = getattr(msg, 'twist', msg)
    tw = getattr(tw, 'twist', tw)       # Odometry.twist.twist / TwistStamped.twist
    return math.hypot(tw.linear.x, tw.linear.y)


def _point_to_polyline(p, pts):
    if pts is None or len(pts) == 0:
        return None
    if len(pts) == 1:
        return float(np.hypot(*(pts[0] - p)))
    a, b = pts[:-1], pts[1:]
    ab = b - a
    L2 = np.einsum('ij,ij->i', ab, ab)
    L2[L2 == 0] = 1e-12
    u = np.clip(np.einsum('ij,ij->i', p - a, ab) / L2, 0, 1)
    proj = a + ab * u[:, None]
    return float(np.min(np.hypot(*(proj - p).T)))


class Indexer:
    def __init__(self, cfg, log=print):
        self.cfg = cfg
        self.log = log
        T = cfg['topics']
        self.T = T
        R = cfg['rules']
        self.R = {k: v for k, v in R.items() if v.get('enabled', True)}
        self.active_states = set(cfg.get('active_states', []))
        # without a robot status topic, "the robot should be moving" = a nav action goal is
        # executing (navigate_to_pose / navigate_through_poses ...)
        self.use_status = bool(T.get('robot_status'))
        # static transforms complete the map -> base chain (e.g. base_footprint -> base_link)
        self.tf_static = T.get('tf_static', '/tf_static' if T.get('tf') else None)
        self.recovery_states = set(cfg.get('recovery_states', []))
        self.nav_exec = {}
        self.frames = cfg.get('frames', {'map': 'map', 'base': 'base_link'})
        self.hits = []
        self.file = ''
        self.status = None
        self.active = False
        self.tf = {}
        self.plan = None
        self.plan_frame = None
        self.last_sample = 0
        self.cmd_sm = 0.0
        ff = lambda: self.file  # noqa: E731
        self.c = {
            'R1': Cond(R.get('R1', {}).get('min_s', 10), ff),
            'R2': Cond(R.get('R2', {}).get('min_s', 2), ff),
            'R3': Cond(0, ff), 'R5p': Cond(0, ff), 'R5r': Cond(0, ff), 'R5c': Cond(0, ff),
            'R6': Cond(R.get('R6', {}).get('min_s', 3), ff), 'R7': Cond(0, ff),
        }
        # one condition per progress checker (each publishes its own latched is_stuck)
        self.stuck = {t: Cond(0, ff) for t in _as_list(T.get('is_stuck'))}
        # action names; a status topic (/x/_action/status) is accepted too
        self.result_actions = {a[:-len('/_action/status')] if a.endswith('/_action/status') else a
                               for a in T.get('result_actions', [])}
        self.beh = {t: Cond(0, ff) for t in T.get('behavior_actions', [])}
        self.goal_status = {}
        self.plan_times = deque()
        self.replan_times = deque()
        self.planner_times = deque()
        self.last_bt_err = 0
        self.prev_bits = {}
        self.prev_codes = {}

    # ---------------------------------------------------------------- output helpers
    def _hit(self, rule, start, end, detail=None, file=None):
        if rule not in self.R:
            return
        self.hits.append(Hit(rule, self.R[rule].get('name', rule), start, end, detail,
                             self.file if file is None else file))

    def _closed(self, rule, res):
        if res:
            self._hit(rule, res[0], res[1], res[2], res[3])

    def topics(self):
        T = self.T
        names = [T[k] for k in ('robot_status', 'odom', 'tf', 'cmd_smoothed', 'cmd_adjusted',
                                'bt_error_code', 'plan', 'replan_request', 'current_planner',
                                'stop_command', 'safety', 'monitoring', 'mark', 'action_result')
                 if T.get(k)]
        if self.tf_static:
            names.append(self.tf_static)
        return (names + list(self.stuck) + list(T.get('nav_actions', [])) +
                list(T.get('behavior_actions', [])))

    # ---------------------------------------------------------------- message handlers
    def on(self, topic, msg, t):
        T = self.T
        # independent of the dispatch below: one status message may carry both kinds of fields
        if topic == T.get('safety'):
            self._safety(msg, t)
        if topic == T.get('monitoring'):
            self._monitoring(msg, t)
        if topic == self.tf_static:
            self._tf_cache(msg)
            return
        if topic == T.get('robot_status'):
            self._status(msg.data, t)
        elif topic == T.get('odom'):
            if 'R1' in self.R:
                v = _speed(msg)
                self._closed('R1', self.c['R1'].update(
                    t, self.active and v < self.R['R1'].get('speed_below', 0.02),
                    {'status': self.status}))
        elif topic == T.get('tf'):
            self._tf(msg, t)
        elif topic == T.get('cmd_smoothed'):
            self.cmd_sm = abs(_speed(msg))
        elif topic == T.get('cmd_adjusted'):
            if 'R2' in self.R:
                r = self.R['R2']
                a = abs(_speed(msg))
                on = self.cmd_sm > r.get('min_cmd', 0.05) and a < r.get('ratio_below', 0.5) * self.cmd_sm
                self._closed('R2', self.c['R2'].update(
                    t, on, {'cmd_smoothed': round(self.cmd_sm, 3), 'cmd_adjusted': round(a, 3)}))
        elif topic == T.get('bt_error_code'):
            v = int(msg.data)
            if v != 0 and v != self.last_bt_err:
                self._hit('R4', t, t, {'bt_error_code': v})
            self.last_bt_err = v
        elif topic == T.get('plan'):
            self._plan(msg, t)
        elif topic == T.get('replan_request'):
            self._rate('R5r', self.replan_times, t, self.R.get('R5', {}).get('replan_per_min', 10),
                       'replan_request')
        elif topic == T.get('current_planner'):
            # the BT publishes /current_planner once per path computation it starts
            self._rate('R5c', self.planner_times, t,
                       self.R.get('R5', {}).get('compute_per_min', 20), 'compute_path')
        elif topic == T.get('action_result'):
            self._result(msg.data, t)
        elif topic == T.get('stop_command'):
            vals = set(self.R.get('R7', {}).get('stop_values', []))
            self._closed('R7', self.c['R7'].update(t, int(msg.data) in vals,
                                                   {'stop_command': int(msg.data)}))
        elif topic in self.stuck:
            self._closed('R7', self.stuck[topic].update(
                t, bool(msg.data), {'is_stuck': topic.rsplit('/', 1)[0]}))
        elif topic == T.get('mark'):
            self._hit('R10', t, t, {'mark': msg.data})
        elif topic in T.get('nav_actions', []):
            self._goals(topic, msg, t, nav=True)
        elif topic in self.beh:
            self._goals(topic, msg, t, nav=False)

    def _safety(self, msg, t):
        for spec in self.R.get('R8', {}).get('fields', []):
            # "!name": active-low bit (true while OK), the hit is when it goes false
            f = spec.lstrip('!')
            if not hasattr(msg, f):
                continue
            v = bool(getattr(msg, f)) != spec.startswith('!')
            if v and not self.prev_bits.get(f, False):
                self._hit('R8', t, t, {'field': f, 'value': bool(getattr(msg, f))})
            self.prev_bits[f] = v

    def _monitoring(self, msg, t):
        # R9: a flag field goes true, or a code field becomes non-zero (field names in R9)
        r = self.R.get('R9', {})
        for f in r.get('flag_fields', []):
            v = bool(getattr(msg, f, False))
            if v and not self.prev_bits.get('R9:' + f, False):
                self._hit('R9', t, t, {f: True})
            self.prev_bits['R9:' + f] = v
        for f in r.get('code_fields', []):
            v = int(getattr(msg, f, 0) or 0)
            if v != 0 and v != self.prev_codes.get(f, 0):
                self._hit('R9', t, t, {f: v})
            self.prev_codes[f] = v

    def _set_active(self, active, t):
        self.active = active
        if not active:
            for k in ('R1', 'R6'):
                self._closed(k, self.c[k].close(t))

    def _status(self, value, t):
        prev = self.status
        self.status = value
        self._set_active(value in self.active_states, t)
        rec = value in self.recovery_states
        self._closed('R3', self.c['R3'].update(t, rec, {'robot_status': value, 'prev': prev}))

    def _result(self, data, t):
        # /nav_data/action_result (recorder get_result): failure decided by the recorder rule
        try:
            r = json.loads(data)
        except ValueError:
            return
        if r.get('action') in self.result_actions and r.get('fail'):
            self._hit('R4', t, t, {k: v for k, v in r.items() if k not in ('fail', 'goal')})

    def _goals(self, topic, msg, t, nav):
        executing = False
        for st in msg.status_list:
            gid = bytes(st.goal_info.goal_id.uuid).hex()[:12]
            key = (topic, gid)
            prev = self.goal_status.get(key)
            if prev != st.status:
                self.goal_status[key] = st.status
                if nav and st.status == 6:
                    self._hit('R4', t, t, {'action': topic.split('/_action')[0], 'status': 'ABORTED'})
            executing |= st.status == 2
        if nav and not self.use_status:
            self.nav_exec[topic] = executing
            self._set_active(any(self.nav_exec.values()), t)
        if not nav:
            self._closed('R3', self.beh[topic].update(t, executing,
                                                      {'behavior': topic.split('/_action')[0]}))
        if len(self.goal_status) > 4096:
            self.goal_status.clear()

    def _rate(self, key, times, t, per_min, label):
        times.append(t)
        while times and t - times[0] > 60e9:
            times.popleft()
        self._closed('R5', self.c[key].update(t, len(times) >= per_min,
                                              {label + '_per_min': len(times)}))

    def _plan(self, msg, t):
        if msg.poses:
            self.plan = np.array([[p.pose.position.x, p.pose.position.y] for p in msg.poses])
        else:
            self.plan = None
        self.plan_frame = msg.header.frame_id
        if 'R5' in self.R:
            self._rate('R5p', self.plan_times, t, self.R['R5'].get('plan_per_min', 30), 'plan')

    def _tf_cache(self, msg):
        for tr in msg.transforms:
            q = tr.transform.rotation
            self.tf[tr.child_frame_id] = (tr.header.frame_id, tr.transform.translation.x,
                                          tr.transform.translation.y, _yaw(q))

    def _tf(self, msg, t):
        self._tf_cache(msg)
        if 'R6' not in self.R:
            return
        r = self.R['R6']
        if t - self.last_sample < 1e9 / r.get('sample_hz', 5.0):
            return
        self.last_sample = t
        pose = self.pose()
        if pose is None:
            return
        d = None
        if self.active and self.plan is not None and self.plan_frame in ('', self.frames['map']):
            d = _point_to_polyline(np.array(pose[:2]), self.plan)
        on = d is not None and d > r.get('dist_m', 0.3)
        self._closed('R6', self.c['R6'].update(t, on, {'dist_m': round(d, 3) if d else None,
                                                       'pose': [round(v, 2) for v in pose]}))

    def pose(self):
        """map -> base 2D pose by composing cached transforms."""
        x = y = yaw = 0.0
        f = self.frames['base']
        for _ in range(8):
            if f == self.frames['map']:
                return x, y, yaw
            e = self.tf.get(f)
            if e is None:
                return None
            parent, tx, ty, tyaw = e
            c, s = math.cos(tyaw), math.sin(tyaw)
            x, y, yaw = tx + c * x - s * y, ty + s * x + c * y, tyaw + yaw
            f = parent
        return None

    # ---------------------------------------------------------------- finish
    def finish(self, t_end):
        for k, c in self.c.items():
            rule = 'R5' if k.startswith('R5') else 'R7' if k.startswith('R7') else k
            self._closed(rule, c.close())
        for c in self.beh.values():
            self._closed('R3', c.close())
        for c in self.stuck.values():
            self._closed('R7', c.close())
        return merge(self.hits, int(self.cfg.get('merge_gap_s', 5.0) * 1e9))


def merge(hits, gap_ns):
    hits = sorted(hits, key=lambda h: (h.rule, h.start))
    out = []
    for h in hits:
        if out and out[-1].rule == h.rule and h.start - out[-1].end <= gap_ns:
            o = out[-1]
            o.end = max(o.end, h.end)
            o.count += h.count
            if h.rule in ('R8', 'R4', 'R10') and h.detail != o.detail:
                o.detail = dict(o.detail)
                o.detail.setdefault('more', []).append(h.detail)
            continue
        out.append(h)
    return sorted(out, key=lambda h: h.start)


def score(h, cfg):
    w = cfg['rules'].get(h.rule, {}).get('weight', 1)
    return round(w * (1 + math.log10(1 + h.duration())) * (1 + math.log10(h.count)), 2)


# -------------------------------------------------------------------- driving check
def robot_status_now(topic='/robot_status', timeout=3.0):
    try:
        import rclpy
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from rclpy.signals import SignalHandlerOptions
        from std_msgs.msg import String
    except ImportError:
        return None
    rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('nav_data_nightly_probe')
    got = []
    # a latched (transient_local) publisher that publishes on change only gives its last value
    # to a transient_local subscription; a volatile publisher only matches a volatile one
    t_disc = time.monotonic()
    pubs = []
    while not pubs and time.monotonic() - t_disc < timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
        pubs = node.get_publishers_info_by_topic(topic)
    latched = bool(pubs) and all(p.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL
                                 for p in pubs)
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL if latched
                     else DurabilityPolicy.VOLATILE)
    node.create_subscription(String, topic, lambda m: got.append(m.data), qos)
    t0 = time.monotonic()
    try:
        while not got and time.monotonic() - t0 < timeout:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return got[-1] if got else None


def nav_goal_executing_now(status_topics, timeout=3.0):
    """True if any of the action status topics (latched) lists a goal in EXECUTING."""
    if not status_topics:
        return False
    try:
        import rclpy
        from action_msgs.msg import GoalStatusArray
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from rclpy.signals import SignalHandlerOptions
    except ImportError:
        return False
    rclpy.init(args=None, signal_handler_options=SignalHandlerOptions.NO)
    node = rclpy.create_node('nav_data_nightly_probe')
    got = {}
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)
    for t in status_topics:
        node.create_subscription(
            GoalStatusArray, t,
            lambda m, t=t: got.__setitem__(t, any(s.status == 2 for s in m.status_list)), qos)
    t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < timeout:
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return any(got.values())


# -------------------------------------------------------------------- main flow
def index_day(root, date, cfg, log=print, files=None):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    d0, d1 = day_range_ns(date)
    if files is None:
        files = [f for f in list_rec_files(root) if f.end_ns >= d0 and f.start_ns < d1]
    ix = Indexer(cfg, log)
    wanted = ix.topics()
    classes = {}
    t_last = d0
    for f in files:
        r = rosbag2_py.SequentialReader()
        try:
            r.open(rosbag2_py.StorageOptions(uri=f.path, storage_id='mcap'),
                   rosbag2_py.ConverterOptions('cdr', 'cdr'))
        except RuntimeError as e:
            log(f'  ! {f.path}: {e}')
            continue
        types = {t.name: t.type for t in r.get_all_topics_and_types()}
        present = [t for t in wanted if t in types]
        if not present:
            continue
        r.set_filter(rosbag2_py.StorageFilter(topics=present))
        ix.file = os.path.relpath(f.path, root)
        while r.has_next():
            topic, data, t = r.read_next()
            if t < d0 or t >= d1:
                continue
            ty = types[topic]
            if ty not in classes:
                try:
                    classes[ty] = get_message(ty)
                except (AttributeError, ModuleNotFoundError, ValueError):
                    classes[ty] = None
                    log(f'  ! cannot load {ty}; {topic} ignored')
            if classes[ty] is None:
                continue
            ix.on(topic, deserialize_message(data, classes[ty]), t)
            t_last = t
    return ix.finish(t_last), files


def write_outputs(root, date, hits, cfg, files):
    idx = os.path.join(root, 'index')
    os.makedirs(idx, exist_ok=True)
    rows = []
    for h in hits:
        rows.append({'start': iso_local(h.start), 'end': iso_local(h.end), 'start_ns': h.start,
                     'end_ns': h.end, 'rule': h.rule, 'name': h.name, 'score': score(h, cfg),
                     'duration_s': round(h.duration(), 1), 'count': h.count, 'file': h.file,
                     'detail': h.detail})
    counts = {}
    for r in rows:
        counts[(r['rule'], r['name'])] = counts.get((r['rule'], r['name']), 0) + 1
    top = sorted(rows, key=lambda r: -r['score'])[:int(cfg.get('report_top', 20))]
    hours = sum(max(0, f.end_ns - f.start_ns) for f in files) / 3.6e12
    lines = [f'# nav_data 일일 리포트 {date}', '',
             f'- 기록 파일 {len(files)}개, 기록 시간 약 {hours:.1f} h',
             f'- 색인 구간 {len(rows)}개', '', '## 규칙별 건수', '', '| 규칙 | 이름 | 건수 |',
             '|---|---|---|']
    for k in sorted(cfg['rules'], key=lambda s: int(s[1:])):
        name = cfg['rules'][k].get('name', k)
        lines.append(f'| {k} | {name} | {counts.get((k, name), 0)} |')
    lines += ['', f'## 상위 {len(top)}개 구간', '',
              '| 시작 | 길이(s) | 규칙 | 점수 | 내용 | 파일 |', '|---|---|---|---|---|---|']
    for r in top:
        det = json.dumps(r['detail'], ensure_ascii=False, default=str)
        if len(det) > 120:
            det = det[:117] + '...'
        lines.append(f"| {r['start'][11:19]} | {r['duration_s']} | {r['rule']} {r['name']} | "
                     f"{r['score']} | {det.replace('|', '/')} | {r['file']} |")
    if top:
        r = top[0]
        lines += ['', '구간 내보내기 예:', '', '```',
                  f"ros2 run nav_data_tools nav_data_export --root {root} --at \"{r['start'][:19]}\" "
                  f"--before 60 --after 60 -o ~/nav_export/{date}_{r['start'][11:19].replace(':', '')}.mcap",
                  '```']
    with open(os.path.join(idx, f'{date}_report.md'), 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines) + '\n')
    # index/<date>.jsonl last and atomically: its existence means "this day is done"
    tmp = os.path.join(idx, f'.{date}.jsonl.tmp')
    with open(tmp, 'w', encoding='utf-8') as f:
        for r in rows:
            f.write(json.dumps(r, ensure_ascii=False, default=str) + '\n')
        f.flush()
        os.fsync(f.fileno())                  # a power cut must not leave an empty "done" marker
    os.replace(tmp, os.path.join(idx, f'{date}.jsonl'))
    _fsync_dir(idx)
    return rows


def _fsync_file(path):
    try:
        fd = os.open(path, os.O_RDONLY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)
    except OSError:
        pass


def _fsync_dir(path):
    try:
        fd = os.open(path, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(fd)
        finally:
            os.close(fd)
    except OSError:
        pass


def pin_marks(root, hits, cfg, log=print):
    from nav_data_tools.export import run_export
    r10 = cfg['rules'].get('R10', {})
    before = int(r10.get('pin_before_s', 300) * 1e9)
    after = int(r10.get('pin_after_s', 300) * 1e9)
    made = []
    for h in hits:
        if h.rule != 'R10':
            continue
        reason = re.sub(r'[^\w가-힣-]+', '_', str(h.detail.get('mark', 'mark')))[:40].strip('_')
        stamp = _dt.datetime.fromtimestamp(h.start / 1e9).strftime('%Y%m%d_%H%M%S')
        d = os.path.join(root, 'pinned', f'{stamp}_{reason or "mark"}')
        if os.path.exists(os.path.join(d, 'note.json')):
            continue
        if os.path.exists(d):                     # interrupted earlier: start this pin again
            shutil.rmtree(d, ignore_errors=True)
        os.makedirs(d)
        try:
            run_export(root, h.start - before, h.end + after, os.path.join(d, 'export.mcap'),
                       log=log)
        except (SystemExit, Exception) as e:  # noqa: BLE001
            log(f'  ! pin {d}: {e}')
            append_event(root, 'recorder', 'pin_failed', dir=os.path.relpath(d, root),
                         error=str(e)[:200])
            shutil.rmtree(d, ignore_errors=True)
            continue
        _fsync_file(os.path.join(d, 'export.mcap'))   # note.json = "pin complete": data first
        with open(os.path.join(d, 'note.json'), 'w', encoding='utf-8') as f:
            json.dump({'mark': h.detail.get('mark'), 'time': iso_local(h.start),
                       'count': h.count, 'file': h.file}, f, ensure_ascii=False, indent=1)
            f.flush()
            os.fsync(f.fileno())
        _fsync_dir(d)
        made.append(d)
    return made


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument('--root', default=DEFAULT_ROOT)
    p.add_argument('--date', help='YYYYMMDD (default: yesterday)')
    p.add_argument('--profile', default='',
                   help="profile name or path (default: $NAV_DATA_PROFILE, else 'default')")
    p.add_argument('--rules', default='',
                   help="rules yaml. Default: the --profile's nightly_rules.yaml, else $NAV_DATA_RULES,\n"
                        "else that of $NAV_DATA_PROFILE, else of the 'default' profile")
    p.add_argument('--no-wait', action='store_true', help='do not wait while the robot drives')
    p.add_argument('--max-wait-h', type=float, default=6.0)
    p.add_argument('--no-pin', action='store_true')
    a = p.parse_args(argv)
    root = expand(a.root)
    rules = a.rules or (profile_file('nightly_rules.yaml', a.profile) if a.profile else
                        os.environ.get('NAV_DATA_RULES') or profile_file('nightly_rules.yaml'))
    print(f'rules: {rules}')
    with open(rules, encoding='utf-8') as f:
        cfg = yaml.safe_load(f)
    date = a.date or (_dt.date.today() - _dt.timedelta(days=1)).strftime('%Y%m%d')

    if not a.no_wait:
        driving = set(cfg.get('driving_states', []))
        t0 = time.monotonic()
        while True:
            try:
                if cfg['topics'].get('robot_status'):
                    st = robot_status_now(cfg['topics']['robot_status'])
                    if st is None:                # no status yet (e.g. right after boot): wait
                        st = 'unknown (no message on robot_status)'
                        busy = True
                    else:
                        busy = st in driving
                else:
                    st = 'nav goal executing'
                    busy = nav_goal_executing_now(cfg['topics'].get('nav_actions', []))
            except Exception as e:  # noqa: BLE001  (no ROS environment: cannot tell, run now)
                print(f'cannot check whether the robot is busy ({e}); indexing now')
                busy = False
            if not busy:
                break
            if time.monotonic() - t0 > a.max_wait_h * 3600:
                print(f'robot still driving ({st}) after {a.max_wait_h} h; giving up for today')
                append_event(root, 'recorder', 'nightly_skipped', date=date, status=st)
                return EXIT_GAVE_UP
            print(f'robot busy ({st}); checking again later', flush=True)
            time.sleep(min(1800, max(60.0, a.max_wait_h * 3600 - (time.monotonic() - t0))))

    t0 = time.monotonic()
    hits, files = index_day(root, date, cfg)
    pins = [] if a.no_pin else pin_marks(root, hits, cfg)     # before the index (= done marker)
    rows = write_outputs(root, date, hits, cfg, files)
    msg = f'{date}: {len(files)} files, {len(rows)} intervals, {len(pins)} pinned, ' \
          f'{time.monotonic() - t0:.0f} s'
    print(msg)
    append_event(root, 'recorder', 'nightly_done', date=date, intervals=len(rows), pinned=len(pins),
                 files=len(files))
    return 0


if __name__ == '__main__':
    sys.exit(main())
