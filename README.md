# nav2_custom_bt_node
This is custom bt decorator node for nav2.
when flag(topic) is true, then navigation works, or, navigation doesn't work.

This is nav2 action pause/resume codes. when controller subscribes pause_flag(true), then motion gets paused, and then, when controller subscribes pause_flag(false), then motion resumes and heads to the goal. 


ros2 jazzy version.

ref, 250511: https://github.com/ros-navigation/navigation2/tree/jazzy


# test
```
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False

ros2 launch nav2_bringup tb4_simulation_launch.py headless:=False

```

```
ros2 topic pub -r 2 /mission_flag std_msgs/msg/Bool "{data: true}" \
  --qos-reliability best_effort \
  --qos-history keep_last \
  --qos-depth 10


ros2 topic pub -r 2 /mission_flag std_msgs/msg/Bool "{data: false}" \
  --qos-reliability best_effort \
  --qos-history keep_last \
  --qos-depth 10  


ros2 topic pub -r 2 /nav_pause_flag std_msgs/msg/Bool "{data: false}" \
  --qos-reliability reliable \
  --qos-durability transient_local \
  --qos-history keep_last \
  --qos-depth 10


ros2 topic pub -r 2 /nav_pause_flag std_msgs/msg/Bool "{data: true}" \
  --qos-reliability reliable \
  --qos-durability transient_local \
  --qos-history keep_last \
  --qos-depth 10



ros2 topic pub --once /decor_flag std_msgs/msg/Bool "{data: true}" \
  --qos-reliability best_effort \
  --qos-history keep_last \
  --qos-depth 10

ros2 run tf2_ros tf2_echo map base_link


```


# changes in nav2
navigation2/nav2_bringup/params/nav2_params.yaml
```
bt_navigator:
  ros__parameters:
    plugin_lib_names:
      - "run_if_flag_true_decorator"
```


navigation2/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml
```
    <RunIfFlagTrueDecorator name="CheckFlag" node="{node}" flag_topic="/mission_flag">
        <FollowPath path="{path}" controller_id="{selected_controller}" error_code_id="{follow_path_error_code}"/>
    </RunIfFlagTrueDecorator>
```

controller_server.hpp/cpp/main.cpp



# to do
.. planner_server.hpp/cpp : if pause and want to not publish /plan, then need to modify in planner_server.hpp/cpp.  
.. controller_ser.hpp/cpp : in order to assure if pause_flag topic gets subscribed in coontroller, then topic structure should be like service structure. That is, add topic response(publisher) from controller to pause flag's publisher node.  