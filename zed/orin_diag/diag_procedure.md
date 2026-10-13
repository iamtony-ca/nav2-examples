# Diagnostic procedure for the ZED restarts on the Orin

Everything below is read-only, except for the A/B YAML toggles in Step 6. Commands assume the robot user is `orin` with the workspace at `~/zed_ws` (`scripts/run_all_auto.sh:47`).

## Facts from the code that shape the procedure

1. **The watchdog log names only the first stale topic, not all of them.** `check_topic_health()` returns the first topic over `msg_timeout` in the order front cloud, rear cloud, right gray image (`zed_watchdog/zed_watchdog/zed_watchdog_node.py:219-224`). So "front cloud stopped" can mean every topic stopped. Step 1 adds an independent logger to tell these apart.
2. **The timing is fixed by the watchdog code.**
   - The failure fires when a topic's last message is more than 3.0 s old. The FSM timer runs every 0.1 s (`:94, :167-170`, `msg_timeout: 3.0` in `config/zed_watchdog_default.yaml:21`).
   - The kill happens right away (`:215-216, :263-275`), plus a 1 s sleep (`:260-261`).
   - The relaunch comes after `cooldown_sec` of 10 s (`:177-181`). The cameras then start staggered at 0, 0.5 and 1.0 s (`zed_multi_camera_delay_separate_three_cams.launch.py:115, 216-218`).
   - So in absolute times: last message T₀, detection about T₀+3 s, relaunch about T₀+13 s. GPU load from engine loading and NEURAL init after a relaunch would start at about T₀+13 s.
3. **The watchdog uses `time.time()`, which is wall clock, not monotonic** (`zed_watchdog_node.py:111, 114, 220`). If chrony steps the clock forward by more than 3 s, every topic looks stale at once. That restart would be a false positive.
4. **The wrapper kills the whole process on unknown grab errors.**
   - `CAMERA_REBOOTING` logs "Connection issue detected" and continues.
   - `CAMERA_NOT_INITIALIZED` and `FAILURE` log "Camera issue detected" and continue.
   - `CORRUPTED_FRAME` logs "Grab status degraded".
   - Any other error logs "Critical camera error: … NODE KILLED." and calls `exit(EXIT_FAILURE)` (`zed_components/src/zed_camera/src/zed_camera_component_main.cpp:4528-4609`).
   - All 3 cameras share one `component_container_isolated` process, which uses `respawn=True` with `respawn_delay=10.0` (launch file `:87-107`). One camera's fatal error therefore takes all three down. This fits "without the watchdog the node simply died".
   - UNVERIFIED: whether launch_ros reloads the composable nodes into a respawned container. My understanding is that it does not.
5. **Object detection runs inside the front camera's grab thread** (`processDetectedObjects`, `zed_camera_component_main.cpp:4752`). Extra DEIM latency therefore lengthens `zed_front`'s grab loop directly.
6. **A point cloud is dropped silently when TF is missing, apart from a 1 Hz WARN.** The patched `publishPointCloud()` calls `lookupTransform("base_link", frame, <camera stamp>, 0.1 s)`. On failure it logs `"Could not transform %s to base_link: %s"` (throttled to 1 s) and returns without publishing (`zed_camera_component_video_depth.cpp:2515-2529`).
   - The TF arrives from the IPC over zenoh (`assets/zenoh/zenoh_config.json5`, the subscribers list contains `/tf` and `/tf_static`).
   - The right gray image does not depend on TF.
7. **The diagnostics already measure overload.** The wrapper publishes on `/diagnostics` at about 1 Hz, per camera:
   - "Data Capture Mean Frequency" and "Tot. Processing Time … (Max. 0.067 sec)"
   - "Frame Drop rate"
   - "Object detection Processing Time"
   - a "System overloaded" WARN once processing time exceeds the frame period 10 times (`zed_camera_component_main.cpp:7540-7575, 7719-7728`)
   
   Each camera also has a 1 Hz wall-timer heartbeat at `~/status/heartbeat` that is independent of the grab thread (`:2058, :3424-3433, :8945+`). It is only published while someone subscribes.
8. **Logs that already exist:**
   - `ROS_LOG_DIR=$HOME/ros2_logs` (`run_all_auto.sh:50, 69`).
   - jetson_monitor already logs the full jtop stats dict at INFO, throttled to 1/s (`jetson_monitor/jetson_monitor/jetson_monitor_node.py:138`). That gives an existing 1 Hz GPU time series in its ROS log file.
   - **ZED SDK console output (`[ZED]…` lines) only reaches the gnome-terminal, and the watchdog closes that window** (`zed_watchdog_node.py:203-205, 267-275`).
   - UNVERIFIED: whether `ROS_LOG_DIR` reaches the gnome-terminal child. Check both `~/ros2_logs` and `~/.ros/log`.
9. **The watchdog may not kill the container.** It matches process names `zed_*_main` or command lines containing both "zed_multi_camera" and "launch.py" (`zed_watchdog_node.py:227-250`). The container's command line is `component_container_isolated … __node:=zed_multi_container` and matches neither, so only `ros2 launch` gets the SIGKILL. A leftover container could keep the cameras open after a restart.
   - UNVERIFIED: whether closing the terminal SIGHUPs the container.
10. **jetson_monitor spawns about 20 short processes per second.** It runs `ping` and `chronyc` subprocesses every 0.1 s (`jetson_monitor_node.py:19, 48, 85-99`). That costs CPU only, not GPU, but keep it in mind when reading the CPU columns.

## Step 0: Shell setup

Run this in each new terminal, using the same values as `run_all_auto.sh`:
```bash
source /opt/ros/humble/setup.bash; source ~/zed_ws/install/setup.bash
export ROS_DOMAIN_ID=52 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp   # match the running stack (run_all_auto.sh:48-49)
echo $CYCLONEDDS_URI    # must match what the stack uses
mkdir -p ~/diag
```
Snapshot state that can be compared after an event:
```bash
sudo jetson_clocks --show > ~/diag/clocks.txt; nvpmodel -q >> ~/diag/clocks.txt
grep Udp: /proc/net/snmp > ~/diag/udp_before.txt
chronyc tracking > ~/diag/chrony_before.txt
```
- If `jetson_clocks --show` reports GPU min freq below max, DVFS is active, so `jetson_clocks` is not applied. `scripts/setup_hardware.sh:13-15` only runs it when that script is executed as root at boot. UNVERIFIED whether it runs.

## Step 1: Continuous logging (about 1% CPU; start it, then wait for an event)

**1a. tegrastats.** Option list from the [r36.4 Tegrastats doc](https://docs.nvidia.com/jetson/archives/r36.4/DeveloperGuide/AT/JetsonLinuxDevelopmentTools/TegrastatsUtility.html): `--interval`, `--logfile`, `--stop`, `--verbose`.
```bash
sudo tegrastats --interval 500 --logfile ~/diag/tegra.log &
head -2 ~/diag/tegra.log      # JP6 lines start with "MM-DD-YYYY HH:MM:SS" (forum logs); stop later with: sudo tegrastats --stop
```
The fields to read:
- **`GR3D_FREQ X%@[f1,f2]`**: X is the share of time the GPU is busy. f1 and f2 are the two GPC clocks in MHz.
  - 99% with f at max (about 1300 MHz; UNVERIFIED exact value) means real GPU saturation.
  - 99% with f falling means throttling or DVFS: the GPU is busy because its clock was lowered.
- **`EMC_FREQ X%@f`**: X close to 100% means the shared LPDDR5 bandwidth is saturated. That stalls the GPU and CPU together.
- **`CPU [x%@f,…]`**: watch for one core at 100%. The patched per-point cloud loop and voxel grid run single-threaded (`zed_camera_component_video_depth.cpp:2556-2597`).
- **`RAM`/`SWAP`/`lfb`**: memory pressure.
- **`VIC_FREQ`**: VIC usage. Whether the ZED X path uses VIC for image conversion is UNVERIFIED.
- **`tj@`, `gpu@`, `cpu@`**: temperatures. A tj near the throttle point together with falling clocks means thermal throttling.
- **`VDD_GPU_SOC`, `VDD_CPU_CV`, `VIN_SYS_5V0` (mW)**: power rails.

**1b. Per-topic gap logger.** It covers 3 watched topics, 3 heartbeats, OD and `/tf` in one process, one line per second, with epoch time. Type it once:
```bash
cat > ~/diag/gap.py <<'EOF'
import time, rclpy
from rclpy.qos import qos_profile_sensor_data as Q
from sensor_msgs.msg import PointCloud2, Image
from tf2_msgs.msg import TFMessage
from zed_msgs.msg import Heartbeat, ObjectsStamped
p='/zed_multi/'
T={'F':(p+'zed_front/point_cloud/cloud_registered',PointCloud2),'R':(p+'zed_rear/point_cloud/cloud_registered',PointCloud2),
   'G':(p+'zed_right/left/gray/rect/image',Image),'OD':(p+'zed_front/obj_det/objects',ObjectsStamped),
   'hF':(p+'zed_front/status/heartbeat',Heartbeat),'hR':(p+'zed_rear/status/heartbeat',Heartbeat),
   'hG':(p+'zed_right/status/heartbeat',Heartbeat),'tf':('/tf',TFMessage)}
rclpy.init(); n=rclpy.create_node('diag_gap')
t0=time.time(); last={k:t0 for k in T}; c={k:0 for k in T}; g={k:0. for k in T}; lag={k:0. for k in T}
def cb(k):
    def f(m):
        t=time.time(); g[k]=max(g[k],t-last[k]); last[k]=t; c[k]+=1
        h=getattr(m,'header',None)
        if h: lag[k]=t-h.stamp.sec-h.stamp.nanosec*1e-9
    return f
for k,(tp,ty) in T.items(): n.create_subscription(ty,tp,cb(k),Q)
def tick():
    t=time.time(); print('%.2f '%t+' '.join('%s:%d/%d/%.2f'%(k,c[k],1000*max(g[k],t-last[k]),lag[k]) for k in T),flush=True)
    for k in T: c[k]=0; g[k]=0.
n.create_timer(1.0,tick); rclpy.spin(n)
EOF
nohup python3 ~/diag/gap.py >> ~/diag/gap.log 2>&1 &
```
- Each field reads `name:count/maxgap_ms/lag_s`. Lag is wall time minus the header stamp, which is the pipeline latency.
- **Fallback with no file:** run `ros2 topic hz <topic> -w 15 | while read l; do echo "$EPOCHREALTIME $l"; done >> ~/diag/hz_F.log &`, once per topic.

**1c. Diagnostics stream:**
```bash
nohup sh -c 'ros2 topic echo /diagnostics | grep --line-buffered -E "^  name:|sec:|Processing|Frequency|Drop|overloaded|message:"' >> ~/diag/diag.log 2>&1 &
```

**1d. Optional: capture the SDK console output that the watchdog normally discards.** This is a YAML-only change in `zed_watchdog.yaml`. The watchdog appends `cam_serials:=…` as `$0` (`zed_watchdog_node.py:201-203`).
```yaml
launch_cmd: ["bash","-c","ros2 launch zed_multi_camera zed_multi_camera_delay_separate_three_cams.launch.py \"$0\" 2>&1 | while IFS= read -r l; do echo \"$EPOCHREALTIME $l\"; done >> $HOME/diag/zed_console.log"]
```
UNVERIFIED in the field. The command line still contains "zed_multi_camera" and "launch.py", so the kill logic still matches it. Python output buffering can delay lines; add `PYTHONUNBUFFERED=1 ` in front if they are late.

## Step 2: After an event, collect evidence

All ROS file logs use epoch timestamps. Convert one with `date -d @<epoch>`.
```bash
cd ~/ros2_logs; ls -t | head                     # also: ls -t ~/.ros/log | head
grep -hE "Runtime Failure|Unstable|Boot Timeout|Terminal window|Attempt|Killing|Max attempts" zed_watchdog_node_*.log | tail -20
grep -hE "process has died|exit code|respawn" */launch.log | tail -20
grep -hE "Could not transform|Connection issue|Camera issue|Grab status degraded|Critical camera error|NODE KILLED|overloaded|ERROR" component_container_isolated_*.log | tail -40
grep -h "###" jetson_monitor_node_*.log | tail -60 > ~/diag/jtop_tail.txt   # 1 Hz jtop stats (jetson_monitor_node.py:138)
pgrep -af component_container                     # must be exactly ONE after a relaunch (orphan check, fact 9)
```
Kernel, daemons, time and network:
```bash
sudo journalctl -k --since "-15min" -o short-precise | grep -iE "nvgpu|max9|gmsl|camrtc|nvcsi|vi5|capture|oom|killed process|throttl|soctherm|over.?current"
sudo journalctl -u zed_x_daemon --since "-15min" -o short-precise | tail -40
sudo journalctl -u nvargus-daemon --since "-15min" -o short-precise | tail -20   # UNVERIFIED that ZED X uses argus here
journalctl -u chrony --since "-15min" | grep -iE "step|jump|offset"; chronyc tracking
grep Udp: /proc/net/snmp      # compare RcvbufErrors with udp_before.txt
grep . /sys/class/hwmon/hwmon*/oc*_event_cnt 2>/dev/null      # over-current throttle counters (UNVERIFIED path on R36.4)
ls -lt /usr/local/zed/resources | head                         # a fresh engine file on every launch means re-optimisation (UNVERIFIED cache location)
```
If the console capture (1d) is on, also run `grep -iE "optimiz|error|warn|timeout" ~/diag/zed_console.log | tail`. A ZED X restart storm combined with fresh engine files would mean the SDK keeps rebuilding the DEIM engine.

## Step 3: Build the timeline and read the outcome

Find T₀, the last good message of each topic in `gap.log`. The topic's count goes to 0 and its gap grows. Then compare the other logs in the window from T₀−10 s to T₀+20 s.

**Is the GPU spike the cause or a side effect?**
- 99% from about T₀−5 s to T₀, with lag in `gap.log` rising (0.1 s, then 0.5 s and up) before T₀, and diagnostics "Tot. Processing Time" above 0.067 s or "System overloaded": the GPU is saturating and the grab loop falls behind. That makes it a cause.
- 99% only from about T₀+13 s onward: this is the relaunch (camera open, NEURAL and OD engine load). It is an effect of the restart, not the cause.
- 99% at T₀ with the GR3D clock falling and tj or OC counters rising: power or thermal throttling is the cause.

**Which topics stopped:**

| Pattern in gap.log at T₀ | Meaning | What to check |
|---|---|---|
| All of F, R, G, OD and hF, hR, hG stop | The whole process is dead or hung | launch.log "process has died [… exit code N]" before the watchdog kill. −11 is SIGSEGV, −6 is abort, 1 is `exit(EXIT_FAILURE)` "NODE KILLED", −9 is the watchdog's own kill (it comes after detection). |
| All data topics stop, heartbeats continue | The grab threads of all 3 cameras stalled together, which points to a shared resource: GPU, EMC, one MAX96712 deserializer, or `zed_x_daemon` | tegrastats around T₀, kernel capture/GMSL messages, `zed_x_daemon` journal, "Connection issue detected" |
| Only F (and OD) stop, hF continues, R and G fine | A front-only pipeline stalled (NEURAL HD1200 + DEIM + tracking) | diagnostics front "Object detection Processing Time", lag on F and OD |
| F and/or R stop, G continues, `tf` count goes to 0 or "Could not transform" appears | TF or zenoh loss; the GPU spike is a coincidence | zenoh_client container logs, the IPC side |
| Nothing stops in gap.log, but the watchdog fires anyway | Watchdog false positive | chrony step (fact 3), `RcvbufErrors` rising (BEST_EFFORT fragments dropped, `zed_watchdog_node.py:76-80`), watchdog process starved |
| Lag jumps the same amount on every topic at once | A system clock step | chrony journal |

**`ros2_logs` shows a second `component_container_isolated`, or a "camera already in use" error on relaunch.** The cleanup left a container behind (fact 9). The restart loop is then partly self-inflicted.

## Step 4: Provocation test (only if events are rare; robot stationary)

This tests whether extra GPU load produces the stall within minutes, using the `trtexec` that is already on the Orin:
```bash
/usr/src/tensorrt/bin/trtexec --onnx=<deim.onnx> --saveEngine=$HOME/diag/deim.plan   # one-time build, GPU-heavy
/usr/src/tensorrt/bin/trtexec --loadEngine=$HOME/diag/deim.plan --duration=120       # extra load while the stack runs
```
- A stall pattern from Step 3 within minutes means GPU contention is sufficient to cause it.
- No stall under extra load means the GPU is unlikely to be the root cause, so look at TF, GMSL or the clock.

## Step 5: Watchdog tolerance test (see if the cameras recover by themselves)

In `zed_watchdog.yaml`, set `msg_timeout: 10.0` and `max_attempts: 50`. `attempt_count` is never reset (`zed_watchdog_node.py:188-193`).

Safety: this means up to 10 s without a fresh obstacle cloud. Run it supervised or at low speed.

- If gap.log shows gaps of 3–8 s that recover with no restart: the stall is transient back-pressure. The fix is to remove the load source (Step 6) and/or tolerate the stall, not to restart.
- If the gaps never recover: it is a real hang or death, and Step 3 tells you which one.

## Step 6: A/B isolation (one toggle per run; same duration, route and logging; record events per hour and max gap)

Keep `msg_timeout` at 10 s and count "gaps over 1 s" from gap.log rather than restarts.

| # | Toggle (file:line) | If events drop, the cause is |
|---|---|---|
| A0 | Baseline with DEIM, plus the identical config with `custom_onnx_file` back on `v26n_260327_260714.onnx` (`custom_object_detection.yaml:15`) | Whether DEIM itself increased the rate (cleanest A/B) |
| A1 | Front `od_enabled: false` (`common_stereo_zed_front.yaml:156`) | OD load as a whole |
| A2 | Front OD `enable_tracking: false` (`:157`) | SDK object tracking |
| A3 | Front `pos_tracking_enabled: false` (`:101`), only together with A2. UNVERIFIED: the SDK may require positional tracking for OD tracking | Positional tracking GEN_1 |
| A4 | Front `depth_mode: 'NEURAL_LIGHT'` (`:84`) | NEURAL depth at HD1200 (the largest GPU consumer) |
| A5 | Rear `depth_mode: 'NONE'` (`common_stereo_zed_rear.yaml:84`). The rear cloud then stops, so the watchdog must stop watching it: set `check_topics` to [front cloud, front cloud, right image], because the 3 types are hard-coded `[PointCloud2, PointCloud2, Image]` (`zed_watchdog_node.py:42`) | Rear NEURAL_LIGHT GPU share |
| A6 | `point_cloud_freq: 5.0` front and rear (`:87`) | CPU per-point loop + XYZ retrieve |
| A7 | Remove `depth_registered` and the `color/rect/image` topics from the zenoh publishers allow-list (`assets/zenoh/zenoh_config.json5`), since the IPC subscription makes the wrapper produce them | Image retrieve and DDS/zenoh bandwidth load |
| A8 | `sudo jetson_clocks` (runtime only, reverts on reboot; user decision under the "keep Orin as-is" rule), only if `clocks.txt` showed DVFS active | DVFS ramp lag under bursts |

**Rules for reading the A/B results:**
- **A0 differs and A1 fixes it:** reduce OD cost. Options are DEIM input 512, OD at a lower rate, or the YOLO-like raw `[1,5,300]` layout.
- **Only A4 or A5 fixes it:** GPU budget. Take depth off the GPU (the `stereo_scan` CPU/OFA plan) or lighten the front depth.
- **Nothing fixes it and the Step 3 pattern is TF, clock, GMSL or process death:** the GPU spike is a relaunch artifact. Fix the TF path, chrony stepping (`makestep`), GMSL/`zed_x_daemon`, or the container cleanup (kill `component_container_isolated` too).

Sources:
- [NVIDIA Tegrastats Utility r36.4](https://docs.nvidia.com/jetson/archives/r36.4/DeveloperGuide/AT/JetsonLinuxDevelopmentTools/TegrastatsUtility.html)
- [NVIDIA Tegrastats Utility r39.2.1 (GR3D_FREQ per-GPC meaning)](https://docs.nvidia.com/jetson/archives/r39.2.1/DeveloperGuide/AT/JetsonLinuxDevelopmentTools/TegrastatsUtility.html)
- [NVIDIA forum: JP6 tegrastats log format with timestamp and GR3D_FREQ @[f1,f2]](https://forums.developer.nvidia.com/t/gpu-utilization-from-tegrastats-in-jetpack-6/286782)
- [NVIDIA forum: JP6 tegrastats with tj@ and EMC_FREQ](https://forums.developer.nvidia.com/t/jetpack-6-tegrastats-missing-gpu-temperature/309539)

Files read (all under `/home/tony/backup/jazzy_gz/datasets/src/orin_260729/`):
- `zed_watchdog/zed_watchdog/zed_watchdog_node.py`
- `zed_watchdog/config/zed_watchdog_default.yaml`
- `jetson_monitor/jetson_monitor/jetson_monitor_node.py`
- `scripts/run_all_auto.sh`
- `scripts/setup_hardware.sh`
- `zed-ros2-examples/tutorials/zed_multi_camera/launch/zed_multi_camera_delay_separate_three_cams.launch.py`
- `zed-ros2-wrapper/zed_components/src/zed_camera/src/zed_camera_component_main.cpp`
- `zed-ros2-wrapper/zed_components/src/zed_camera/src/zed_camera_component_video_depth.cpp`
- `zed-ros2-wrapper/zed_wrapper/config/common_stereo_zed_{front,rear,right}.yaml`
- `zed-ros2-wrapper/zed_wrapper/config/zedx_zed_*.yaml`
- `zed-ros2-wrapper/zed_wrapper/config/custom_object_detection.yaml`
- `assets/zenoh/zenoh_config.json5`
- `assets/dds/cyclonedds.xml`