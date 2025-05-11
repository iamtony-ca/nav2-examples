# nav2_custom_bt_node
this is cusom bt decorator node for nav2.
whenn flag is true, then navigation works, or, navigation doesn't work.


ros2 jazzy version

ref, 250511: https://github.com/ros-navigation/navigation2/tree/jazzy


# test
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False

ros2 topic pub -r 2 /mission_flag std_msgs/msg/Bool "{data: true}" \
  --qos-reliability best_effort \
  --qos-history keep_last \
  --qos-depth 10