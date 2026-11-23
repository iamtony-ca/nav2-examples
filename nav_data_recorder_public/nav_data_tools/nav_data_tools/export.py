"""nav_data_export: cut a time range out of the recording and restore standard messages.

  ros2 run nav_data_tools nav_data_export --root ~/nav_data \\
      --from "2026-11-20 15:00:30" --to "2026-11-20 15:04:00" -o ~/nav_export/20261120_1500.mcap

  CompactScan            -> sensor_msgs/LaserScan on the original topic
  CompactGrid            -> nav2_msgs/Costmap on the original topic + nav_msgs/OccupancyGrid
                            (/local_costmap/costmap_raw -> /local_costmap/costmap)
  global diff + keyframe -> nav_msgs/OccupancyGrid /global_costmap/costmap (1 Hz)
  everything else        -> unchanged bytes
  latched topics and the last restored grids are put at the start of the range.

The output opens in rviz2 (ros2 bag play <file> --clock), Lichtblick/Foxglove and PlotJuggler.
Custom message types are read from this machine's ament index, so source the workspace that
builds them (the robot's message packages).
"""
import argparse
import array
import json
import os
import re
import shutil
import subprocess
import sys
import time

import numpy as np

from nav_data_tools import codecs
from nav_data_tools.common import DEFAULT_ROOT, expand, iso_local, list_rec_files, parse_time

INFO_TOPIC = '/nav_data/recorder_info'
COMPACT_SCAN = 'nav_data_recorder_msgs/msg/CompactScan'
COMPACT_GRID = 'nav_data_recorder_msgs/msg/CompactGrid'
COSTMAP = 'nav2_msgs/msg/Costmap'
COSTMAP_UPDATE = 'nav2_msgs/msg/CostmapUpdate'
OCC = 'nav_msgs/msg/OccupancyGrid'
SCAN = 'sensor_msgs/msg/LaserScan'
# latched topics of files without /nav_data/recorder_info; newer files list them there
DEFAULT_LATCHED = {'/tf_static', '/map', '/costmap_filter_info'}
ACCUMULATE = {'/tf_static'}   # several latched publishers: keep every distinct message
# topics not needed to rebuild state before the range (skipped during pre-roll)
PREROLL_EXCLUDE = r'^/(tf|odom|rosout|cmd_vel.*|nav_data/compact/.*|local_plan|motion_target)$'


def occupancy_topic(costmap_topic: str) -> str:
    # /local_costmap/costmap_raw -> /local_costmap/costmap (also for a layer-scoped diff stream:
    # /global_costmap/<layer>/costmap_raw -> /global_costmap/<layer>/costmap)
    if costmap_topic.endswith('/costmap_raw'):
        return costmap_topic[:-len('/costmap_raw')] + '/costmap'
    return costmap_topic + '_grid'


class _Msgs:
    """Lazy message classes and (de)serialization."""

    def __init__(self):
        from rclpy.serialization import deserialize_message, serialize_message
        from rosidl_runtime_py.utilities import get_message
        self._get = get_message
        self.de = deserialize_message
        self.ser = serialize_message
        self._cache = {}

    def cls(self, type_name):
        if type_name not in self._cache:
            self._cache[type_name] = self._get(type_name)
        return self._cache[type_name]


class Exporter:
    def __init__(self, out_path, t_from, t_to, global_rate=1.0, include=None, exclude=None,
                 log=print, rename=None):
        import rosbag2_py
        self.rb = rosbag2_py
        self.m = _Msgs()
        self.t_from, self.t_to = t_from, t_to
        self.global_period = int(1e9 / global_rate) if global_rate > 0 else 0
        self.rename = dict(rename or {})   # output topic -> new name
        self.include = re.compile(include) if include else None
        self.exclude = re.compile(exclude) if exclude else None
        self.log = log
        self.out_path = expand(out_path)
        self.single_file = self.out_path.endswith('.mcap')
        self.bag_uri = self.out_path[:-5] + '.tmpbag' if self.single_file else self.out_path
        if os.path.exists(self.out_path):
            raise SystemExit(f'output exists: {self.out_path}')
        if self.single_file and os.path.isdir(self.bag_uri):
            # left behind by an interrupted export (ours to remove), unless another export that is
            # still running writes there (its pid is in the lock file)
            pid = _lock_pid(self.bag_uri)
            if pid and _alive(pid):
                raise SystemExit(f'another export (pid {pid}) is writing {self.out_path}')
            log(f'  removing leftover {self.bag_uri}')
            shutil.rmtree(self.bag_uri)
        os.makedirs(os.path.dirname(self.bag_uri) or '.', exist_ok=True)
        self.w = rosbag2_py.SequentialWriter()
        self.w.open(rosbag2_py.StorageOptions(uri=self.bag_uri, storage_id='mcap',
                                              storage_preset_profile='zstd_fast'),
                    rosbag2_py.ConverterOptions('cdr', 'cdr'))
        if self.single_file:
            with open(os.path.join(self.bag_uri, LOCK), 'w') as f:
                f.write(str(os.getpid()))
        self.created = {}
        self.latched = set(DEFAULT_LATCHED)
        self.global_key_topic = '/nav_data/global_costmap_key'
        self.global_diff_topic = ''   # set from /nav_data/recorder_info when a diff stream is recorded
        self.global_occ_topic = ''    # optional output name for the rebuilt global OccupancyGrid
        self.scan_dec = {}
        self.grid_dec = {}
        self.mirror = codecs.GlobalMirror()
        self.last_global_emit = None
        self.latched_store = {}      # topic -> list[(type, bytes)]
        # /tf_static from all publishers merged into one TFMessage (child frame -> transform).
        # Lichtblick backfills only the last message of a topic after a seek, so every
        # /tf_static message written must carry all static transforms.
        self.static_tf = {}
        self.static_type = 'tf2_msgs/msg/TFMessage'
        self.latched_emitted = {}    # topic -> set(bytes) emitted already
        self.pending_grids = {}      # source topic -> (costmap_bytes, occ_bytes, frame time)
        self.snapshot_done = False
        self.counts = {}
        self.decode_failures = 0

    # ------------------------------------------------------------- output
    def _topic(self, name, type_name, latched=False):
        if name in self.created:
            return
        qos = self.rb._storage.QoS(1 if latched else 10).reliable()
        if latched:
            qos = qos.transient_local()
        self.w.create_topic(self.rb.TopicMetadata(0, name, type_name, 'cdr', [qos], ''))
        self.created[name] = type_name

    def _keep(self, topic):
        if self.include and not self.include.search(topic):
            return False
        if self.exclude and self.exclude.search(topic):
            return False
        return True

    def _emit(self, topic, type_name, data, t):
        latched = topic in self.latched or type_name == OCC   # by the recorded name
        topic = self.rename.get(topic, topic)
        if not self._keep(topic):
            return
        self._topic(topic, type_name, latched)
        self.w.write(topic, data, t)
        self.counts[topic] = self.counts.get(topic, 0) + 1

    def _emit_latched(self, topic, type_name, data, t):
        seen = self.latched_emitted.setdefault(topic, set())
        if data in seen:
            return
        if topic not in ACCUMULATE:
            seen.clear()
        seen.add(data)
        self._emit(topic, type_name, data, t)

    def _merge_static(self, data):
        try:
            msg = self.m.de(data, self.m.cls(self.static_type))
        except Exception:  # noqa: BLE001
            return False
        changed = False
        for tr in msg.transforms:
            old = self.static_tf.get(tr.child_frame_id)
            if old is None or self.m.ser(old) != self.m.ser(tr):
                self.static_tf[tr.child_frame_id] = tr
                changed = True
        return changed

    def _static_msg(self):
        msg = self.m.cls(self.static_type)()
        msg.transforms = [self.static_tf[k] for k in sorted(self.static_tf)]
        return self.m.ser(msg)

    # ------------------------------------------------------------- restore
    def _laser(self, cs, ranges):
        LaserScan = self.m.cls(SCAN)
        s = LaserScan()
        s.header = cs.header
        s.angle_min = cs.angle_min
        s.angle_increment = cs.angle_increment
        s.angle_max = cs.angle_min + cs.angle_increment * max(0, cs.beam_count - 1)
        s.time_increment = cs.time_increment
        s.scan_time = cs.scan_time
        s.range_min = cs.range_min
        s.range_max = cs.range_max
        s.ranges = array.array('f', ranges.astype(np.float32).tobytes())
        return s

    def _costmap_msgs(self, header, layer, resolution, sx, sy, origin, data):
        Costmap = self.m.cls(COSTMAP)
        Occ = self.m.cls(OCC)
        cm = Costmap()
        cm.header = header
        cm.metadata.layer = layer
        cm.metadata.resolution = resolution
        cm.metadata.size_x = sx
        cm.metadata.size_y = sy
        cm.metadata.origin = origin
        cm.metadata.update_time = header.stamp
        cm.data = array.array('B', data.tobytes())
        og = Occ()
        og.header = header
        og.info.map_load_time = header.stamp
        og.info.resolution = resolution
        og.info.width = sx
        og.info.height = sy
        og.info.origin = origin
        og.data = array.array('b', codecs.costmap_to_occupancy(data).tobytes())
        return self.m.ser(cm), self.m.ser(og)

    def _global_occ(self, t):
        if self.mirror.grid is None:
            return None
        Occ = self.m.cls(OCC)
        res, sx, sy, origin, frame = self.mirror.meta
        og = Occ()
        og.header.frame_id = frame
        og.header.stamp.sec = t // 1_000_000_000
        og.header.stamp.nanosec = t % 1_000_000_000
        og.info.resolution = res
        og.info.width = sx
        og.info.height = sy
        og.info.origin = origin
        og.data = array.array('b', codecs.costmap_to_occupancy(self.mirror.grid.reshape(-1)).tobytes())
        return self.m.ser(og)

    def _maybe_global(self, t, force=False):
        if self.mirror.grid is None or t < self.t_from:
            return
        due = (self.last_global_emit is None or force or
               (self.mirror.dirty and self.global_period and
                t - self.last_global_emit >= self.global_period))
        if not due:
            return
        data = self._global_occ(t)
        occ = self.global_occ_topic or (occupancy_topic(self.global_diff_topic)
                                        if self.global_diff_topic else '/global_costmap/costmap')
        self._emit(occ, OCC, data, t)
        self.last_global_emit = t
        self.mirror.dirty = False

    def _snapshot(self):
        """State at t_from: latched topics, last decoded local grids, global grid."""
        if self.snapshot_done:
            return
        self.snapshot_done = True
        t = self.t_from
        if self.static_tf:
            self._emit('/tf_static', self.static_type, self._static_msg(), t)
        for topic, items in self.latched_store.items():
            for type_name, data in items:
                self._emit_latched(topic, type_name, data, t)
        for src, (cm, og) in self.pending_grids.items():
            self._emit(src, COSTMAP, cm, t)
            self._emit(occupancy_topic(src), OCC, og, t)
        self.pending_grids.clear()
        self._maybe_global(t, force=True)

    # ------------------------------------------------------------- input
    def process(self, topic, type_name, data, t):
        if t > self.t_to:
            return False
        if t >= self.t_from:
            self._snapshot()
        live = t >= self.t_from

        if topic == INFO_TOPIC:
            try:
                info = json.loads(self.m.de(data, self.m.cls('std_msgs/msg/String')).data)
                self.latched |= set(info.get('latched', []))
                self.global_key_topic = info.get('global_keyframe') or self.global_key_topic
                self.global_diff_topic = info.get('global_diff') or self.global_diff_topic
                self.global_occ_topic = info.get('global_occupancy') or self.global_occ_topic
            except (ValueError, AttributeError):
                pass
            return True

        if type_name == COMPACT_SCAN:
            cs = self.m.de(data, self.m.cls(COMPACT_SCAN))
            dec = self.scan_dec.setdefault(cs.source_topic, codecs.ScanDecoder())
            ranges = dec.decode(cs.encoding, cs.beam_count, cs.payload)
            if ranges is None:
                self.decode_failures += live  # before the range this is just the pre-roll
            elif live:
                self._emit(cs.source_topic, SCAN, self.m.ser(self._laser(cs, ranges)), t)
            return True

        if type_name == COMPACT_GRID:
            cg = self.m.de(data, self.m.cls(COMPACT_GRID))
            dec = self.grid_dec.setdefault(cg.source_topic, codecs.GridDecoder())
            grid = dec.decode(cg.encoding, cg.size_x, cg.size_y, cg.shift_x, cg.shift_y, cg.payload)
            if grid is None:
                self.decode_failures += live
                return True
            if live or not self.snapshot_done:
                cm, og = self._costmap_msgs(cg.header, cg.layer, cg.resolution, cg.size_x,
                                            cg.size_y, cg.origin, grid)
                if live:
                    self._emit(cg.source_topic, COSTMAP, cm, t)
                    self._emit(occupancy_topic(cg.source_topic), OCC, og, t)
                else:
                    self.pending_grids[cg.source_topic] = (cm, og)
            return True

        if topic == self.global_key_topic:
            self.mirror.apply_full(self.m.de(data, self.m.cls(COSTMAP)))
            self._maybe_global(t)
            return True
        if topic == self.global_diff_topic and type_name == COSTMAP:
            self.mirror.apply_full(self.m.de(data, self.m.cls(COSTMAP)))
        elif topic == self.global_diff_topic + '_updates' and type_name == COSTMAP_UPDATE:
            self.mirror.apply_update(self.m.de(data, self.m.cls(COSTMAP_UPDATE)))

        if topic == '/tf_static':
            self.static_type = type_name or self.static_type
            if self._merge_static(data) and live and self.snapshot_done:
                self._emit(topic, self.static_type, self._static_msg(), t)
        elif topic in self.latched:
            if live:
                self._emit_latched(topic, type_name, data, t)
            else:
                items = self.latched_store.setdefault(topic, [])
                if topic in ACCUMULATE:
                    if all(d != data for _, d in items):
                        items.append((type_name, data))
                        del items[:-64]
                else:
                    items[:] = [(type_name, data)]
        elif live:
            self._emit(topic, type_name, data, t)
        self._maybe_global(t)
        return True

    def read_file(self, path, start_full):
        """Pre-roll (state only, small topics) until start_full, then everything."""
        r = self.rb.SequentialReader()
        r.open(self.rb.StorageOptions(uri=path, storage_id='mcap'),
               self.rb.ConverterOptions('cdr', 'cdr'))
        types = {t.name: t.type for t in r.get_all_topics_and_types()}
        if not types:
            self.log(f'  ! {path}: no topic list (file not closed?). Skipped; see README "recover".')
            return True
        if start_full is not None:
            r.set_filter(self.rb.StorageFilter(regex_to_exclude=PREROLL_EXCLUDE))
            while r.has_next():
                topic, data, t = r.read_next()
                if t >= start_full:
                    break
                self.process(topic, types.get(topic, ''), data, t)
            r.reset_filter()
            r.seek(start_full)
        while r.has_next():
            topic, data, t = r.read_next()
            if not self.process(topic, types.get(topic, ''), data, t):
                return False
        return True

    def close(self):
        self._snapshot()
        self.w.close()
        self._fix_schemas()
        if self.single_file:
            produced = [f for f in os.listdir(self.bag_uri) if f.endswith('.mcap')]
            if len(produced) == 1:
                shutil.move(os.path.join(self.bag_uri, produced[0]), self.out_path)
                shutil.rmtree(self.bag_uri)
                return self.out_path
        return self.bag_uri


    def _fix_schemas(self):
        # rosbag2 0.26.9 leaves the schema of some types empty (action_msgs/msg/GoalStatusArray);
        # Lichtblick cannot decode those topics. rosbag2_py cannot pass a definition, so the files
        # are rewritten by nav_data_fix_schema (C++, same writer API with a definition).
        bad = [t for t in set(self.created.values()) if _schema_missing(self.rb, t)]
        if not bad:
            return
        tool = _fix_schema_tool()
        if not tool:
            self.log(f'  ! no schema for {", ".join(sorted(bad))} (nav_data_fix_schema not found); '
                     'Lichtblick will not decode these topics')
            return
        for name in sorted(os.listdir(self.bag_uri)):
            if not name.endswith('.mcap'):
                continue
            src = os.path.join(self.bag_uri, name)
            tmp = src + '.fix'
            r = subprocess.run([tool, src, tmp], capture_output=True, text=True)
            fixed = [f for f in os.listdir(tmp) if f.endswith('.mcap')] if os.path.isdir(tmp) else []
            if r.returncode != 0 or len(fixed) != 1:
                self.log(f'  ! schema fix failed for {name}: {r.stderr.strip()}')
                shutil.rmtree(tmp, ignore_errors=True)
                continue
            os.replace(os.path.join(tmp, fixed[0]), src)
            shutil.rmtree(tmp)


def _schema_missing(rb, type_name):
    try:
        d = rb.LocalMessageDefinitionSource().get_full_text(type_name)
        return d.encoding == 'unknown' or not d.encoded_message_definition
    except Exception:  # noqa: BLE001
        return True


def _fix_schema_tool():
    try:
        from ament_index_python.packages import get_package_prefix
        p = os.path.join(get_package_prefix('nav_data_recorder'), 'lib', 'nav_data_recorder',
                         'nav_data_fix_schema')
        return p if os.access(p, os.X_OK) else None
    except Exception:  # noqa: BLE001
        return None


LOCK = '.nav_data_export.pid'


def _lock_pid(bag_dir):
    try:
        with open(os.path.join(bag_dir, LOCK)) as f:
            return int(f.read().strip() or 0)
    except (OSError, ValueError):
        return 0


def _alive(pid):
    try:
        os.kill(pid, 0)
        return True
    except ProcessLookupError:
        return False
    except PermissionError:
        return True


def select_files(files, t_from, t_to):
    sel = [f for f in files if f.end_ns >= t_from and f.start_ns <= t_to]
    return sel


def run_export(root, t_from, t_to, out, preroll_s=12.0, files=None, global_rate=1.0,
               include=None, exclude=None, log=print, rename=None):
    if t_to <= t_from:
        raise SystemExit('--to must be after --from')
    if files is None:
        files = select_files(list_rec_files(root), t_from, t_to)
    if not files:
        raise SystemExit('no recording covers that range')
    ex = Exporter(out, t_from, t_to, global_rate, include, exclude, log, rename)
    t0 = time.monotonic()
    for i, f in enumerate(files):
        start_full = None
        if i == 0:
            s = max(f.start_ns, t_from - int(preroll_s * 1e9))
            start_full = s if s > f.start_ns else None
        log(f'  reading {f.path}')
        if not ex.read_file(f.path, start_full):
            break
    path = ex.close()
    n = sum(ex.counts.values())
    log(f'wrote {path}: {n} messages, {len(ex.counts)} topics, {time.monotonic() - t0:.1f} s')
    if ex.decode_failures:
        log(f'  note: {ex.decode_failures} compact messages had no keyframe before them (skipped)')
    return path, ex.counts


class _FileArg:
    def __init__(self, path):
        from nav_data_tools.common import RecFile, _read_range
        s, e, idx = _read_range(path)
        st = os.stat(path)
        self.f = RecFile(path, os.path.dirname(path), 0, s or 0, e or int(st.st_mtime * 1e9),
                         idx, st.st_size)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    p.add_argument('--root', default=DEFAULT_ROOT)
    p.add_argument('--from', dest='t_from', help='local time "YYYY-MM-DD HH:MM:SS", ISO or epoch')
    p.add_argument('--to', dest='t_to')
    p.add_argument('--at', help='center time; use with --before/--after')
    p.add_argument('--before', type=float, default=60.0, help='seconds before --at')
    p.add_argument('--after', type=float, default=60.0, help='seconds after --at')
    p.add_argument('-o', '--output', required=True, help='.mcap file or a bag directory')
    p.add_argument('--files', nargs='*', help='explicit mcap files instead of searching --root')
    p.add_argument('--preroll', type=float, default=12.0,
                   help='seconds read before --from to find keyframes (>= grid keyframe_s)')
    p.add_argument('--global-rate', type=float, default=1.0, help='global costmap output Hz')
    p.add_argument('--include', help='regex: only these output topics')
    p.add_argument('--exclude', help='regex: drop these output topics')
    p.add_argument('--rename', action='append', default=[], metavar='SRC=DST',
                   help='write output topic SRC as DST (repeatable), e.g. '
                        '/global_costmap/my_layer/costmap=/global_costmap/costmap')
    a = p.parse_args(argv)

    if a.at:
        c = parse_time(a.at)
        t_from, t_to = c - int(a.before * 1e9), c + int(a.after * 1e9)
    elif a.t_from and a.t_to:
        t_from, t_to = parse_time(a.t_from), parse_time(a.t_to)
    else:
        p.error('give --from and --to, or --at')
    files = None
    if a.files:
        files = sorted((_FileArg(f).f for f in a.files), key=lambda f: f.start_ns)
        files = select_files(files, t_from, t_to)
    print(f'export {iso_local(t_from)} .. {iso_local(t_to)}')
    rename = {}
    for r in a.rename:
        src, sep, dst = r.partition('=')
        if not sep or not src or not dst:
            p.error(f'--rename needs SRC=DST, got {r!r}')
        rename[src] = dst
    run_export(expand(a.root), t_from, t_to, a.output, a.preroll, files, a.global_rate,
               a.include, a.exclude, rename=rename)
    return 0


if __name__ == '__main__':
    sys.exit(main())
