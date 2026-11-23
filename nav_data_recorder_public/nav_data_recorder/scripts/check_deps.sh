#!/usr/bin/env bash
# Offline check of what nav_data_recorder needs on a robot. Run before building:
#   bash ~/ros2_ws/src/nav_data_recorder/nav_data_recorder/scripts/check_deps.sh [workspace setup.bash]
# A workspace setup.bash (e.g. ~/ros2_ws/install/setup.bash) lets it find nav2_msgs built from source.
# Prints OK/MISSING per item and, if something is missing, the exact .deb URLs apt would fetch
# (from this robot's own apt lists) so they can be downloaded on another PC.
[ -f /opt/ros/jazzy/setup.bash ] || { echo "MISSING /opt/ros/jazzy"; exit 1; }
source /opt/ros/jazzy/setup.bash
[ -n "${1:-}" ] && [ -f "$1" ] && source "$1"
set -u
missing=()
check_pkg() {
  local v; v=$(dpkg-query -W -f='${Status} ${Version}' "$1" 2>/dev/null)
  if [[ "$v" == "install ok installed"* ]]; then printf 'OK       %-45s %s\n' "$1" "${v##* }"
  else printf 'MISSING  %s\n' "$1"; missing+=("$1"); fi
}
echo "== apt packages (build + run)"
for p in ros-jazzy-rclcpp ros-jazzy-rclpy ros-jazzy-rosbag2-cpp ros-jazzy-rosbag2-storage \
         ros-jazzy-rosbag2-storage-mcap ros-jazzy-rosbag2-py ros-jazzy-rosidl-typesupport-introspection-cpp \
         ros-jazzy-rosidl-runtime-py ros-jazzy-yaml-cpp-vendor ros-jazzy-ament-index-cpp \
         ros-jazzy-ament-index-python ros-jazzy-tf2 ros-jazzy-tf2-msgs ros-jazzy-action-msgs \
         ros-jazzy-sensor-msgs ros-jazzy-nav-msgs ros-jazzy-geometry-msgs ros-jazzy-std-msgs \
         ros-jazzy-lifecycle-msgs ros-jazzy-rcl-interfaces ros-jazzy-launch-ros \
         python3-numpy python3-yaml; do
  check_pkg "$p"
done
echo "== nav2_msgs (apt or built in your workspace)"
if v=$(dpkg-query -W -f='${Status} ${Version}' ros-jazzy-nav2-msgs 2>/dev/null) && [[ "$v" == "install ok installed"* ]]; then
  printf 'OK       %-45s %s\n' ros-jazzy-nav2-msgs "${v##* }"
elif p=$(ros2 pkg prefix nav2_msgs 2>/dev/null); then
  printf 'OK       %-45s %s\n' nav2_msgs "$p"
else
  printf 'MISSING  %s\n' ros-jazzy-nav2-msgs; missing+=(ros-jazzy-nav2-msgs)
fi
echo "== only to build the tests (or build with -DBUILD_TESTING=OFF)"
test_missing=()
for p in ros-jazzy-ament-cmake-gtest python3-pytest; do
  n=${#missing[@]}; check_pkg "$p"; ((${#missing[@]} > n)) && { test_missing+=("$p"); unset "missing[$n]"; }
done
echo "== runtime checks"
tmp=$(mktemp -d)
if python3 - "$tmp" <<'PY' 2>/dev/null
import sys, rosbag2_py
w = rosbag2_py.SequentialWriter()
w.open(rosbag2_py.StorageOptions(uri=sys.argv[1] + '/b', storage_id='mcap',
                                 storage_preset_profile='zstd_fast'),
       rosbag2_py.ConverterOptions('cdr', 'cdr'))
PY
then echo "OK       mcap writer with zstd_fast"; else echo "MISSING  mcap writer (rosbag2_storage_mcap)"; fi
rm -rf "$tmp"
free_gb=$(df -BG --output=avail "$HOME" | tail -1 | tr -dc 0-9)
echo "INFO     free disk in \$HOME: ${free_gb} GB (prune keeps 7 days, cap 60 GB, deletes more below 15% free)"
if ((${#missing[@]})); then
  echo
  echo "== missing: ${missing[*]}"
  echo "URLs from this robot's apt lists (download elsewhere, copy, then: sudo apt install ./*.deb):"
  apt-get install --print-uris -qq "${missing[@]}" 2>/dev/null | cut -d"'" -f2
  exit 2
fi
echo; echo "nothing to install"
