# nav_data_recorder

Always-on navigation data recorder for ROS 2 (Jazzy) and Nav2. It keeps the last days of
navigation data in compressed rosbag2 (mcap) files, at a few percent of one CPU core, so a problem
noticed later can be found and replayed in rviz2 or Lichtblick.

- Scans and costmaps are stored in compact form (`CompactScan`, `CompactGrid`); everything else is
  recorded as is. `nav_data_export` rebuilds standard messages for any time range.
- Every 10-minute file can be read on its own (latched topics and keyframes at its start).
- An event log (goal results, errors, safety bits, manual marks) and a nightly report point to the
  moments worth looking at; marked moments are kept for 90 days.
- Old data is deleted by age, total size and free disk space.

Measured on an x86 industrial robot standing still: 39 KB/s on disk (520 KB/s before zstd),
6.5 % of one core, 52 MB RAM. On the Nav2 TurtleBot3 simulation, driving: about 17 KB/s, 3.5 %.

Korean operation guide (setup, daily use, analysis, troubleshooting): [docs/operation_guide.md](docs/operation_guide.md).

| Package | Kind | Contents |
|---|---|---|
| `nav_data_recorder_msgs` | rosidl | `CompactScan`, `CompactGrid` |
| `nav_data_recorder` | C++ | recorder node, launch, `nav_data_fix_schema` |
| `nav_data_tools` | Python | `nav_data_session`, `nav_data_maintenance` (runs `nav_data_prune` and `nav_data_nightly`), `nav_data_export`, `nav_data_profile` |
| `nav_data_profiles` (`profiles/`) | data | robot profiles: topics, recorder parameters, nightly rules, Lichtblick layout; `default/` = plain Nav2 |

## Requirements

ROS 2 Jazzy (`ros-base` or more) and Nav2 message packages; no pip packages. Check a robot without
internet with `nav_data_recorder/scripts/check_deps.sh` (see [Offline install](#offline-install)).

## Build

```bash
cd ~/ros2_ws/src && git clone <this repository> nav_data_recorder
cd ~/ros2_ws && colcon build --packages-up-to nav_data_recorder nav_data_tools   # includes nav_data_profiles
```

Build in the workspace that also builds your robot's message packages (or source it first), so the
recorded type definitions match what the robot publishes. `--cmake-args -DBUILD_TESTING=OFF`
skips the tests.

## Quick start (plain Nav2)

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch nav_data_recorder nav_data_recorder.launch.py robot_id:=robot01
# data goes to ~/nav_data; Ctrl+C closes the current file
```

The `default` profile records the standard Nav2 topics: `/plan`, `/local_plan`, `/tf`,
`/odom`, `/cmd_vel*`, `/scan`, both costmaps, `/map`, `/behavior_tree_log`, the navigate and
behavior action states and results, `/rosout`. Topics a robot does not have are reported once and
ignored.

## Adapting to your robot

The packages contain no robot-specific code. Everything robot-specific is a profile: one directory
`profiles/<name>/` with up to four files. Missing files are taken from `profiles/default/`.

```bash
cp -r profiles/default profiles/myrobot      # edit, then: colcon build --packages-select nav_data_profiles
ros2 launch nav_data_recorder nav_data_recorder.launch.py robot_id:=robot01 profile:=myrobot
```

| What | Where |
|---|---|
| Topics, rates, scan/costmap compaction, event rules | `topics.yaml` (launch `topics_file:=` overrides it) |
| Root dir, split period, writer cache | `params.yaml` (launch `params_file:=` / `root_dir:=` override it) |
| Nightly rules R1..R10, thresholds, robot status values (`active_states`, `driving_states`, `recovery_states`), safety bit names and polarity | `nightly_rules.yaml` (`--rules` / `NAV_DATA_RULES` override it) |
| Lichtblick view | `nav_data_analysis.json` |
| Map name, source tree for the git revision | launch `map_name:=…` (or `$NAV_DATA_MAP_NAME`), `ws_src:=…` |
| Extra files kept with each boot (map config ...) | `$NAV_DATA_SESSION_FILES` (colon separated) |
| Retention (7 days, 60 GB, free disk), nightly report time | `nav_data_maintenance` section of `params.yaml` |
| Output topic names of an export | `nav_data_export --rename SRC=DST` (or `occupancy_output` in `global_keyframe`) |

`profile:=` also takes a directory path, so a profile can live outside this repository. Only
`profiles/default/` is part of this repository (`.gitignore` ignores the others); to keep a private
profile in a private copy, `git add -f` it and run `tools/check_publish.sh` before any public push.
See [profiles/README.md](profiles/README.md).

Custom messages need no build dependency: event rules read fields of any message type at run time
(`field: data`, `fields: [a, b.c]`, `'!name'` for active-low bits), and an action's result is
fetched with its `get_result` service. See the commented examples in `topics.yaml` and
`nightly_rules.yaml`.

## Deploy on a robot

1. `check_deps.sh` (pass `~/ros2_ws/install/setup.bash` if Nav2 is built from source), then build as above.
2. Start the recorder from the robot's start script, after Nav2, in the same environment as the
   stack (same `ROS_DOMAIN_ID`, RMW and DDS config; otherwise it sees no topics):

   ```bash
   nice -n 10 ionice -c2 -n7 ros2 launch nav_data_recorder nav_data_recorder.launch.py \
       robot_id:=robot01 profile:=<your profile> ws_src:=$HOME/ros2_ws/src map_name:=<map>
   ```

   The launch restarts the recorder if it dies, runs `nav_data_session` once per boot and starts
   `nav_data_maintenance` (lowest CPU/IO priority): retention every 15 min, and after 02:30 the
   nightly report for the previous day (days missed while the robot was off are done at the next
   start; the report waits while the robot drives). Nothing is installed in the OS: no systemd
   units, no sudo. `maintenance:=false` leaves it out.
3. After a few minutes: `ls ~/nav_data/rec/$(date +%Y%m%d)/`, and in
   `~/nav_data/events/<date>.jsonl` look for `missing_topic` and `type_load_failed`.

### Offline install

Run `check_deps.sh` only if packages may be missing. It prints, from the robot's own apt lists, the
.deb URLs of anything missing. The robot's ROS packages must stay at their installed version (a
newer `ros-jazzy-rosbag2-*` pulls newer core packages), so:

1. Save the printed URLs to `urls.txt` and run `wget -i urls.txt` on a PC with internet.
   packages.ros.org keeps only the latest sync; for an older version take the same path from the
   first dated snapshot after the build date in the version (`...-1noble.20260126...` is in
   `http://snapshots.ros.org/jazzy/2026-01-28/ubuntu/`; dates: `http://snapshots.ros.org/jazzy/`).
2. Copy the .deb files to the robot and `sudo apt install ./*.deb`.

## Data layout

```
~/nav_data/
  rec/20261120/083015_ab12/          one recorder start = one rosbag2 directory
      083015_ab12_0.mcap ...           10 min per file, each file self-contained
  sessions/20261120_083015_ab12/     Nav2 params, BT xml, extra files, git rev of that boot
  events/20261120.jsonl              online events (state, error, safety, mark, recorder)
  index/20261120.jsonl, *_report.md  nightly index and report
  pinned/<time>_<reason>/            +-5 min around each mark, kept 90 days
```

Topics recorded under other names: `/nav_data/compact/<topic>` (CompactScan / CompactGrid),
`/nav_data/recorder_info` (latched topic list and compaction map, at each file start),
`/nav_data/action_result` (std_msgs/String JSON, one per finished goal of an action with a `result`
rule), and in diff-stream mode `/nav_data/global_costmap_key` (get_costmap at each file start).

## Find and replay a problem

```bash
# mark the moment on the robot (kept 90 days by the nightly job)
ros2 topic pub --once /nav_data/mark std_msgs/msg/String "{data: 'stopped at the corner'}"

# find a time: index/<date>_report.md, events/<date>.jsonl, or what people remember
ros2 run nav_data_tools nav_data_export --root ~/nav_data \
    --at "2026-11-20 15:02:10" --before 60 --after 60 -o ~/nav_export/20261120_1502.mcap

# rviz2
ros2 bag play ~/nav_export/20261120_1502.mcap --clock
rviz2 --ros-args -p use_sim_time:=true          # or your rviz launch with use_sim_time:=true

# Lichtblick (once: copy profiles/<profile>/nav_data_analysis.json to ~/.lichtblick-suite/layouts/)
lichtblick --defaultLayout=nav_data_analysis ~/nav_export/20261120_1502.mcap
```

Export restores `LaserScan` on the original scan topics and each costmap both as `nav2_msgs/Costmap`
and as `OccupancyGrid` (`/local_costmap/costmap`, `/global_costmap/costmap`). To export on another
PC, copy the `rec/` files, pass `--files`, and source a workspace with `nav_data_recorder_msgs`,
`nav_data_recorder` (for `nav_data_fix_schema`), `nav_data_tools` and the robot's message packages.

Notes on viewers:
- rosbag2 Jazzy leaves the schema of `action_msgs/msg/GoalStatusArray` empty, so Lichtblick could
  not show action status. The recorder writes schemas itself from the installed `.msg` files, and
  `nav_data_export` rewrites its output with `nav_data_fix_schema`.
- After a seek Lichtblick reads only the last message of each topic: export writes `/tf_static` as
  one message, and the layout turns on transform preloading for `/tf`.
- Scans are colored by range: Lichtblick 1.31 draws LaserScan "flat" colors almost black.

A file that was being written at a power cut has no index; it is usually still read (rosbag2 scans
it). If export skips it, recover it with the mcap CLI (`mcap recover in.mcap -o out.mcap`).

## Settings

- `profiles/<profile>/topics.yaml`: topic list, rates, encodings, event rules.
- `profiles/<profile>/params.yaml`: root, split period, cache size.
- `nav_data_recorder/config/mcap_writer.yaml`: mcap chunk size. Data reaches the disk one chunk
  (and one writer cache) at a time; what is still in memory is lost at a power cut. Keep both
  small (128 KiB chunk, 1 MiB cache) so the loss stays at seconds even when little is recorded.
- `profiles/<profile>/nightly_rules.yaml`: nightly rules R1..R10 and thresholds. `frames.base`
  must be reachable from `map` through `/tf` and `/tf_static`.

## Tests

```bash
colcon test --packages-select nav_data_recorder nav_data_tools && colcon test-result --verbose
# the config tests also parse every profile found in profiles/
# load / end-to-end: nav_data_tools/test/fake_nav_pub.py publishes busy-robot-like traffic
```

## License

Apache-2.0, see [LICENSE](LICENSE).
