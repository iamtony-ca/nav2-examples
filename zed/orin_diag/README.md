# orin_diag: ZED restart analysis (2026-10-13)

## Symptom
After the DEIM model was applied, `zed_watchdog` restarted the ZED nodes. jetson_monitor showed GPU at about 99.7% for 2–3 s. The watchdog log always reads `Runtime Failure: /zed_multi/zed_front/point_cloud/cloud_registered stopped`. The same thing happened with YOLO26n, and without the watchdog the node simply died.

## Key facts (source-verified)
- **"Always front" proves nothing by itself.** `check_topic_health()` returns only the *first* stale topic in `check_topics` order (front cloud → rear cloud → right image) (`zed_watchdog_node.py:219-224`). If all three stop together, front is still the one named. Whether only front stopped or everything stopped has to be measured separately: use `gap.py`, or log the age of every topic.
- **The restart itself saturates the GPU.** Timeline: stall at T0, kill at T0+3 s, relaunch at T0+13 s, which loads 3 cameras, the NEURAL/NEURAL_LIGHT engines, the DEIM engine and tracking. A spike *before* T0 points to a cause; a spike *after* it is an effect.
- **`max_attempts` is 5.** The launch loads `zed_watchdog.yaml`, which sets 5, not the default 50. `attempt_count` is never reset. After 5 restarts the watchdog stays in FATAL_ERROR with no cameras.
- **The wrapper `exit()`s on unhandled grab errors** ("Critical camera error … NODE KILLED", `main.cpp:4604-4611`). The three cameras share one process, so all three die. The container's `respawn=True` brings back an empty container that does not reload the components, which is why the node "just died" without the watchdog.
- **The SDK recovers on its own in the background** (`async_grab_camera_recovery=true`, typically 20–30 s according to Stereolabs). A 3 s timeout kills the camera before that recovery can finish.
- **The watchdog's kill misses the container process.** It is ended only indirectly (likely SIGHUP), with no `close()`. That can slow down the next open or make it fail.
- **Known bugs in 5.1.1 that are fixed in later versions:**
  - custom OD + setting the parameters every frame can crash (fixed in 5.1.2); this wrapper sets them every frame;
  - `grab()` can lock with the corrupted-frame detector on (fixed in 5.2.1);
  - auto-exposure frame drops with ZED X driver < 1.4.0.
- **The jetson_monitor GPU % is busy time only, with no clock.** At a low DVFS clock it can still read 99%. `jetson_clocks` is not applied at boot.

## Hypotheses (most likely first; confirm with diag_procedure.md)
1. **H1:** the GPU spike is the restart's effect, not its cause.
2. **H2:** an SDK/GMSL recovery (CAMERA_REBOOTING) or a grab lock, killed by the 3 s timeout.
3. **H3:** real GPU saturation (front NEURAL HD1200 + DEIM FP32 + tracking + rear NEURAL_LIGHT), possibly at a low clock.
4. **H4:** the process dies (wrapper `exit()` or a 5.1.1 OD crash).
5. **H5:** TF loss drops the clouds. Less likely, because the rear cloud does not stop.
6. **H6:** first-time engine build longer than the 60 s boot timeout. Only right after a model swap.

## Files
- `diag_procedure.md`: read-only diagnostic steps for the Orin (logging, post-event grep, timeline reading, A/B toggles).
- `analysis_synthesis_20261013.md`: full synthesis, fix plan (config / watchdog redesign / structural) and what not to do.
- `gap.py`: a per-topic gap, count and latency logger (3 watched topics, heartbeats, OD, /tf), one line per second.
- `_raw/`: raw investigation JSON.
