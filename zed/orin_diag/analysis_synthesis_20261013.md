# ZED restarts with GPU at ~99.7% on the Orin: ranked causes and fix plan

**Short answer.** The GPU number alone cannot show that GPU load causes the restarts. The current setup also turns any short stall of 3 s or more into a kill-and-reload of all 3 cameras, and that reload is itself GPU-heavy. The actual cause of the stall has to be found by lining up the timestamps of a few logs. Until then, there are config-only mitigations and a watchdog redesign that are safe to start on.

All paths are relative to `/home/tony/backup/jazzy_gz/datasets/src/orin_260729/`. `W` = `zed-ros2-wrapper/zed_components/src/zed_camera/src/` and `C` = `zed-ros2-wrapper/zed_wrapper/config/`. Nothing was modified.

## 0. What I checked myself, and corrections to the input reports

**Confirmed in the source:**
- **Watchdog trigger.** Any one of the 3 watched topics silent for more than 3 s triggers a restart (`zed_watchdog/zed_watchdog/zed_watchdog_node.py:164-171, 219-224`).
- **Wall clock.** Times are taken with `time.time()` (`:111, :114, :220`).
- **Blocking kill.** The kill runs inside the timer callback: kill, then `sleep(1.0)`, then `terminate()` and `wait(2.0)` (`:211-217, :226-275`).
- **The kill does not match the container.** It only matches process names `zed_*_main`, or a command line containing both "zed_multi_camera" and "launch.py" (`:227-250`). The container's command line contains neither.
- **Retry counter.** `attempt_count` is never reset (`:188-193`).
- **Max attempts is 5, not 50.** `launch/zed_watchdog.launch.py:10-14` loads `config/zed_watchdog.yaml`, which sets `max_attempts: 5`. The 50 is only in `zed_watchdog_default.yaml`. After 5 restarts in one power cycle the watchdog sits in FATAL_ERROR and the robot has no cameras.
- **Grab-loop error handling** (`W/zed_camera_component_main.cpp:4580-4611`):
  - `CAMERA_REBOOTING`, `CAMERA_NOT_INITIALIZED` and `FAILURE` sleep 1 s and retry.
  - `CORRUPTED_FRAME` only logs a warning.
  - Every other error calls `exit(EXIT_FAILURE)`, which kills all 3 cameras because they share one process.
  - The positional-tracking start code also has an exit path (`:4436-4446`).
- **Camera recovery runs in the background.** `async_grab_camera_recovery = true` is hard-coded (`:2484-2486`). `grab_compute_capping_fps = 0` is also hard-coded (`:2454`).
- **Heartbeat.** Each camera has a 1 Hz heartbeat on its own wall timer, independent of the grab thread (`:3424-3433`).
- **Point clouds are dropped when TF is missing.** The cloud publisher looks up TF at the image timestamp with a 0.1 s timeout and returns without publishing on failure (`W/zed_camera_component_video_depth.cpp:2514-2530`).
- **OD parameters are sent every frame.** `retrieveCustomObjects(objects, custom_params, id)` is called on every frame (`W/zed_camera_component_objdet.cpp:~1080-1088`). OD only runs when the topic has a subscriber (`:1009-1023`), and the zenoh bridge always subscribes (`assets/zenoh/zenoh_config.json5:10`).
- **The SDK runs OD asynchronously.** I checked the real 5.1.1 header I extracted earlier: `retrieveObjects` "immediately returns the last objects detected … is_new = false" (`scratchpad/zedsdk511/include/sl/Camera.hpp:12734-12737`). The header also documents the background recovery (`:9964-9974`) and that the validity check "requires some computations" inside `grab()` (`:9996-10004`).
- **Camera configs:**
  - All three cameras have `enable_image_validity_check: 1` (line 31), `auto_exposure_gain: true` (line 43) and `auto_whitebalance: true` (line 46).
  - Front: `depth_mode` NEURAL (line 84), point cloud COMPACT at 15 Hz (lines 87-88), positional tracking on with GEN_1 (lines 101-102), OD on with tracking on (lines 156-157), thread scheduling SCHED_BATCH (line 190).
  - Rear: NEURAL_LIGHT, no positional tracking, no OD. Right: depth NONE.
  - Front grab resolution is HD1200 at 15 fps (`zedx_zed_front.yaml:9-10`).
- **Container restart does not reload the cameras.** The container is declared with `respawn=True`, `respawn_delay=10` (`zed-ros2-examples/tutorials/zed_multi_camera/launch/zed_multi_camera_delay_separate_three_cams.launch.py:98-107`).
- **Turning off OD tracking also allows turning off positional tracking:**
  - The wrapper forces positional tracking on only for `pos_tracking_enabled`, `publish_tf`, `depth_stabilization > 0`, mapping, or OD with tracking (`main.cpp:6432-6490`).
  - `publish_tf` is forced off (`zed_wrapper/launch/zed_camera.launch.py:341-342`).
  - No odom or pose topic is bridged to the IPC (zenoh publishers list).
- **`jetson_clocks` is not run at boot.** It appears only in `scripts/setup_hardware.sh:13-15`, which `scripts/run_all_auto*.sh` never calls. No `CYCLONEDDS_URI` is set either.

**Corrections and contradictions resolved:**
- **The "diag" report is wrong that DEIM latency directly lengthens the front grab loop** (its fact 5). Inference runs in parallel inside the SDK (Camera.hpp:12734). DEIM therefore adds GPU contention, not direct blocking. The exception is the first `enableObjectDetection`, which builds or loads the TensorRT engine inside the front grab thread (`main.cpp:4474-4483`, `objdet.cpp:950-957`).
- **The "watchdog" report is wrong that `cpu_temp`/`fan_rpm` are unbound when jtop is down.** In `jetson_monitor/jetson_monitor/jetson_monitor_node.py` they are assigned only inside the `try`, so a `KeyError` before them also leaves them unbound. Its conclusion (a crash risk) still holds.
- **jetson_monitor still forks `ping` and `chronyc` every 0.1 s.** The calls in `update_stats` are commented out, but `update_network_stats` on the 0.1 s `network_timer` still runs them, each with a 0.5 s timeout (`jetson_monitor_node.py:~48, 77-118`).
- **What "99.7" is depends on which field the user read:**
  - `usage_percent` is an integer (`int(gpu_cur)`), so it cannot show 99.7.
  - "99.7" is either the raw jtop value in the 1 Hz INFO log of the whole stats dict, or `avg_60s`.
  - `avg_60s` is really a 60 × 0.1 s ≈ **6 s** average, so a value of 99.7 there means about 6 s of saturation.
  - Either way, the value is GPU busy-time and is not normalised to clock speed. The jtop sample rate depends on jtop's default interval, which I did not check.
  - **Ask the user which field they read.**
- **How the container dies on restart is not fully known.** It is not SIGKILLed directly. It most likely dies from SIGHUP when `ros2 launch`, the session leader of the gnome-terminal pty, is killed. rclcpp handles only SIGINT and SIGTERM, so the process ends with no `sl::Camera::close()`. **UNVERIFIED:** whether an orphaned container can survive the restart.

## 1. Ranked root-cause hypotheses

Notation: T0 = time of the last message on the stalled topic.

### H1. The GPU spike comes after the stall, not before it (most likely explanation of the GPU reading itself)

**Mechanism.** The watchdog detects the stall at about T0+3 s and kills everything. After the 10 s cooldown it relaunches at about T0+13 s, loading 3 cameras in turn (0, 0.5 and 1.0 s apart, launch file `:115, 216-218`). The relaunch:
- opens 3 cameras and loads the NEURAL and NEURAL_LIGHT depth engines,
- deserialises the DEIM TensorRT engine,
- starts GEN_1 tracking.

That is a multi-second burst at about 100% busy time. Because the cameras were killed without a clean close, the next open may be slower or fail, which adds attempts and burns through `max_attempts: 5`.

**For:**
- Event-type GPU load is the only plausible source of a 2-3 s plateau. Stereolabs' published steady-state depth load is well below 100%: NEURAL at 53% for 4 cameras at 30 fps on AGX Orin (https://docs.stereolabs.com/docs/depth-sensing/depth-modes). This setup is about a quarter to a third of that. DEIM-N is estimated at 5.6-7.6 ms per 66.7 ms frame. All of these figures are UNVERIFIED for SDK 5.1.1 on this robot.
- The user's phrase "at that moment" is ambiguous about order.

**Against:** it says nothing about why the first stall happened. It also does not apply if the spike came before T0.

**Quickest check (read-only).** Line up the watchdog's "🚨 Runtime Failure: X stopped" and "🚀 [Attempt N]" log lines with the jetson_monitor "####" INFO lines in `~/ros2_logs`. Both are epoch-stamped.
- Spike in [T0+11 s, T0+60 s] → it is the relaunch, and H1 holds.
- Spike in [T0-3 s, T0] → go to H3.

### H2. A camera stall inside the SDK or driver; the watchdog kills the camera before the SDK's own recovery can finish

There are three possible sub-causes:
- **(a) GMSL communication hiccup.** `grab()` returns `CAMERA_REBOOTING` and the wrapper loops with 1 s sleeps (`main.cpp:4581-4587`). Stereolabs staff say "20-30 sec is the expected period" for this recovery and that "overloading the Jetson … could cause the ZED X Driver to timeout and issue a camera reboot" (https://community.stereolabs.com/t/error-code-camera-rebooting-when-streaming-multiple-cameras-over-jetson/9445).
- **(b) A `grab()` lock in the corrupted-frame detector.** `enable_image_validity_check: 1` is on for all cameras. The 5.2.1 release notes say "Fixed a rare lock … using Camera::grab() when corrupted frame detector was enabled" (https://www.stereolabs.com/developers/release/5.2). The robot runs 5.1.1, so the fix is missing.
- **(c) Auto-exposure I2C frame drops on ZED X drivers older than 1.4.0.** The robot has 1.3.2, and AE is on for all cameras (https://community.stereolabs.com/t/62-frame-drops-with-auto-exposure-enabled-on-4x-zed-x-gmsl2-data-attached/10853).

**For:**
- It fits the pattern "short GPU-heavy period, then topics silent for 3 s or more" (SDK re-initialisation).
- The 3 s timeout is far shorter than the SDK's own recovery time.
- The exact fixes are absent on 5.1.1.

**Against:** whether it happens at all is unknown until the logs are checked.

**Quickest check.** In the camera gnome-terminal or `~/ros2_logs`, grep `Connection issue detected|Camera issue detected|Grab status degraded` within the 5 s before "Runtime Failure". For (b): no wrapper error is logged, and the heartbeat of the stalled camera keeps running. That needs a subscriber, because the heartbeat is only published when someone listens.

### H3. Real GPU saturation, possibly at a low DVFS clock, with DEIM making it worse

**Mechanism.**
- Front NEURAL at HD1200 + GEN_1 + OD tracking + FP32 DEIM (estimated 1.34× YOLO26n on GPU), rear NEURAL_LIGHT, and validity checks on all 3 cameras run in one process with time-sliced CUDA contexts.
- `jetson_clocks` is not applied at boot (setup_hardware.sh is not called), so the GPU governor can sit at a low clock.
- jtop % is busy time, so a busy GPU at a low clock still reads about 99%.
- Grab latency then stretches and can push the GMSL driver into a timeout, which leads into H2a.

**For:**
- Restarts already happened with YOLO26n, and the user says they increased after the DEIM swap.
- No clock or throttle state is recorded anywhere.

**Against:** at full clock the steady load should be far below 100% (H1 numbers). The DEIM latency on the Orin is estimated, not measured.

**Quickest check.** `cat /sys/class/devfreq/17000000.gpu/{governor,cur_freq,min_freq,max_freq}` (path is UNVERIFIED on R36.4; locate it with `ls /sys/class/devfreq/`), plus `tegrastats --interval 100 --logfile ~/tegra.log` around an event. Look at `GR3D_FREQ X%@[MHz]`.
- 99% at about 1300 MHz before T0 → true saturation.
- 99% at a low clock → DVFS or throttling.

### H4. The process dies

**Mechanism.** Either the wrapper calls `exit()` on a grab error code outside its handled list (e.g. `CAMERA_NOT_DETECTED`, `CUDA_ERROR`, `NOT_ENOUGH_GPU_MEMORY`; `main.cpp:4604-4611`), or the SDK 5.1.1 custom-OD code crashes. The 5.1.2 release notes list "random crash … Custom Object Detection and repeatedly setting custom parameters" and "random crash when tracking objects whose estimators failed to initialize". This wrapper re-sends the parameters every frame, and OD tracking is on. `respawn=True` then brings back an empty container: launch_ros does not reload composable nodes (https://github.com/ros2/launch_ros/issues/361).

**For:** this is exactly "without the watchdog, the node just died".

**Against:** whether the bug actually triggers when the parameters are unchanged is UNVERIFIED.

**Quickest check.** In the launch output, look for "process has died [pid …, exit code N]":
- N=1 together with "NODE KILLED" → wrapper `exit()`.
- N=-11 or -6 with no "Critical" line before it → SDK crash.
- N=-9 → the watchdog's own kill.

With the watchdog running, the launch process is SIGKILLed, so this line may only appear in runs without the watchdog.

### H5. False trigger from missing TF; the GPU spike is coincidental

**Mechanism.** If the `base_link→zed_*_left_camera_frame` chain from the IPC is on `/tf` rather than `/tf_static`, or arrives late or with clock skew, every point cloud is dropped (`video_depth.cpp:2518-2529`). Two of the three watched topics are clouds, so this alone trips the watchdog.

**Quickest check.** Grep `Could not transform` in the logs, and confirm that the right gray image kept flowing during the stall.

### H6. First-time engine build for the new ONNX takes longer than `boot_timeout` (60 s)

**Mechanism.** The SDK builds a TensorRT engine for a new ONNX inside the front grab thread. If the build takes more than 60 s, the watchdog kills it, the build never finishes, and every attempt rebuilds at 100% GPU.

**Applies only** right after the model swap or to a cache that keeps being invalidated.

**Quickest check.** `ls -la --time-style=full-iso /usr/local/zed/resources` for a dot-file created at the first DEIM launch. The cache location is UNVERIFIED for 5.1.1.

### H7. Watchdog false positive (low probability)

**Mechanism.** A forward chrony step on the wall clock, starvation of the single-threaded executor, or loss of large BEST_EFFORT clouds made of many fragments.

**Quickest check.** `journalctl -u chrony | grep -i step`. Run `ros2 topic hz` on the same topic during an event and compare: does the topic really stop?

## 2. Fix plan

### Step 0 (before any change, read-only)
For the next 2-3 incidents, collect:
- `tegrastats --interval 100 --logfile`
- the GPU devfreq values from the H3 check
- the watchdog log
- jetson_monitor's "####" lines
- the camera-terminal output with these strings grepped: `Connection issue|Camera issue|Grab status degraded|Critical camera error|Could not transform|Object Detection error|process has died`
- `dmesg -T` / `journalctl -k` lines for GMSL (max9*), nvcsi/vi, nvgpu and OOM

Setting `debug.sdk_verbose_log_file` (`C/common_stereo_zed_*.yaml:197`) captures SDK messages to a file, but it redirects all stdout of the shared process.

### (a) Config-only mitigations on the Orin, by benefit/risk

1. **`config/zed_watchdog.yaml`: `max_attempts: 5` → 50.** This removes the "no cameras for the rest of the shift" failure. Zero risk.
2. **Pre-build the DEIM engine once with the watchdog stopped.** Launch by hand, wait for "=== Starting Object Detection ===" and the dot-file in `/usr/local/zed/resources`, then start the watchdog again. This eliminates H6. Zero risk.
3. **`enable_image_validity_check: 1` → 0 in all three `C/common_stereo_zed_*.yaml:31`.** This removes the 5.1.1 `grab()` lock path (H2b) and some per-grab compute. Cost: no `CORRUPTED_FRAME` warnings and no `status/health` topic. Low risk.
4. **Front: `object_detection.enable_tracking: false` (`:157`), then `pos_tracking_enabled: false` (`:101`).**
   - Removes one of the two 5.1.1 OD crash paths (H4) and GEN_1 GPU work.
   - Positional tracking is then no longer forced on (`main.cpp:6432-6490`, `publish_tf` forced False). Odom and pose are not bridged to the IPC.
   - Cost: OD tracking IDs and velocities are lost. Check first whether the IPC consumer of `/obj_det/objects` uses them.
5. **Front `depth_mode: 'NEURAL'` → `'NEURAL_LIGHT'` (`C/common_stereo_zed_front.yaml:84`).** This is the largest documented depth-GPU saving, and Stereolabs recommends it for multi-camera setups. Cost: some depth and cloud quality at 0.3-3 m (UNVERIFIED for this scene). Validate the laserscan and the OD 3D positions.
6. **Lock exposure and white balance on all 3 cameras.** Set `auto_exposure_gain: false` (`:43`) and `auto_whitebalance: false` (`:46`), with `exposure_time`/`analog_gain` (`C/zedx_zed_*.yaml:13,17`) and `whitebalance_temperature` (`:47`) tuned on site. This targets H2c. Cost: image quality if the lighting varies, which may affect DEIM accuracy.
7. **Reduce load created by subscribers.**
   - If the IPC does not need them, drop `/zed_multi/zed_front/depth/depth_registered` (about 34 MB/s of float32) and the `*/left/color/rect/image` topics from the zenoh publisher list (`assets/zenoh/zenoh_config.json5:9-16`). The wrapper only retrieves these when they have subscribers.
   - Optionally set `point_cloud_res: 'REDUCED'` on front and rear (`:88`), which is mainly a CPU saving.
8. **Watchdog `msg_timeout` 3 → about 8 s, only as a stopgap together with items 1-2.**
   - A restart already means at least 13 s plus boot time with no data, so tolerating an 8 s gap does not reduce data availability.
   - It does give a slow-but-alive grab loop a chance to recover.
   - The nav stack must already treat a stale `/scan_zed_cloud_*` as unsafe. Check this on the IPC (UNVERIFIED).
9. **`jetson_clocks` only with the user's explicit approval.** It is a runtime clock change, not a JetPack/SDK/driver change, and it reverts on reboot. Use it only if the H3 check shows the GPU at a low clock. Afterwards, watch power and the over-current counters (`/sys/class/hwmon/hwmon*/oc*_event_cnt`; path UNVERIFIED).

### (b) Watchdog redesign

**Signals per camera:**
- `/zed_multi/<cam>/status/heartbeat` (`zed_msgs/Heartbeat`, 1 Hz wall timer; `main.cpp:3424-3433`)
- one small data topic (`left/gray/rect/camera_info`, or a header-only subscription to the cloud)
- the container PID

**Timing and threading:**
- Use `time.monotonic()`.
- Put subscriptions in their own callback group under a MultiThreadedExecutor.
- Run kills in a worker thread, never inside the timer callback.

**Policy:**
- **DEAD:** PID gone, or all heartbeats older than 5 s → restart now.
- **SLOW:** data older than 3 s but heartbeat alive → log a snapshot only (per-topic ages for all topics, last `/diagnostics` of that camera, tegrastats tail, dmesg tail, camera log tail).
- **STALLED:** data older than 30 s with heartbeat alive → restart. 30 s matches the SDK's 20-30 s async recovery.
- Backoff: `min(10·2^n, 120)` s.
- Reset `attempt_count` after 10 min in RUNNING.
- Never stop permanently in FATAL: keep retrying every 120 s and publish the watchdog state for the IPC.

**Process handling:**
- Launch without gnome-terminal, using `Popen(..., start_new_session=True)` with stdout written to a timestamped log file.
- Stop with SIGINT to the process group, wait 10 s, then SIGTERM, wait 5 s, then SIGKILL. This lets the wrapper join its threads and close the cameras.
- Match `component_container_isolated … __node:=zed_multi_container` explicitly, and check that no leftover container exists before relaunching.
- Remove `respawn=True` from the container (launch file `:105-106`). The watchdog owns restarts.

### (c) Structural changes

1. **One container/process per camera.** Leave `container_name` empty so `zed_camera.launch.py` creates its own container (`zed_wrapper/launch/zed_camera.launch.py:289-309`). This gives fault isolation and real per-camera restart. Measure GPU throughput on the Orin first; whether each `sl::Camera` already has its own CUDA context is UNVERIFIED.
2. **Small wrapper patches (no SDK change):**
   - Call `setCustomObjectDetectionRuntimeParameters` once, and again on dynamic parameter change, then use `retrieveObjects(objects, mObjDetInstID)` per frame (`objdet.cpp:~1080-1088`; Camera.hpp marks the old call deprecated).
   - Replace the `exit()` at `main.cpp:4604-4611` with log, back off, and close/reopen of that one camera.
   - Look up TF with `tf2::TimePointZero`, or cache the static camera transform (`video_depth.cpp:2518-2522`).
3. **OD cost.**
   - Measure DEIM-N on the Orin with `/usr/src/tensorrt/bin/trtexec` (TRT 10.3, FP32) while all 3 cameras run, and compare against YOLO26n.
   - If needed, use a 512-input export (about 0.64× FLOPs; needs re-fine-tuning, accuracy impact UNVERIFIED).
   - Running OD less often than every frame needs wrapper code: no 5.1.1 or wrapper parameter does it, and `grab_compute_capping_fps` is hard-coded to 0.
4. **Move rear depth off the GPU** (the stereo_scan CPU/OFA plan). This is a modest gain: rear is NEURAL_LIGHT at SVGA 15 fps, a fraction of front NEURAL. Front depth has to stay, because SDK OD needs depth for 3D positions.
5. **Record for the user's later decision:** SDK 5.1.2, 5.2.1 and 5.5 fix the OD crash, the `grab()` lock and the GMSL stuck-REBOOTING/freeze issues. Upgrading is out of scope under the current constraint.

## 3. What NOT to do

- **Don't raise `msg_timeout` to a large value by itself** (e.g. 30 s) without the heartbeat/PID check. A dead process would then go unnoticed for 30 s. Don't tighten it either.
- **Don't run `jetson_clocks`, `nvpmodel` or devfreq writes without the user's explicit approval.** Never run the SDK or ZED Link installers or the `.deb`, and don't upgrade JetPack, the SDK or the driver.
- **Don't conclude that the GPU is the cause from jtop % alone.** It is 1 s busy time without clock speed, and the restart itself produces it.
- **Don't make kills more brutal** (e.g. `pkill -9` on the container). Abrupt kills without `close()` are a likely contributor to failed re-opens.
- **Don't rely on the container's `respawn=True`.** It brings back an empty container.
- **Don't switch to `pos_tracking_mode` GEN_2/GEN_3 on 5.1.1.** Staff identified a crash with GEN_3 (UNVERIFIED for GEN_2).
- **Don't set `allow_reduced_precision_inference: true` for DEIM.** DEIM-family FP16 needs TensorRT ≥ 10.6 and the Orin has 10.3.
- **Don't move to DEIM-S** (about 2.2× N's latency), and don't re-export at opset ≥ 20.
- **Don't change several toggles at once.** Use one A/B change per run, keep logging running, and compare gaps over 1 s per hour, not restart counts.
- **Don't run git in `src/`.**