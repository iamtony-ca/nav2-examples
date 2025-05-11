# nav2_custom_bt_node
this is custom bt decorator node for nav2.
when flag(topic) is true, then navigation works, or, navigation doesn't work.

ros2 jazzy version

ref, 250511: https://github.com/ros-navigation/navigation2/tree/jazzy


# test
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False

ros2 topic pub -r 2 /mission_flag std_msgs/msg/Bool "{data: true}" \
  --qos-reliability best_effort \
  --qos-history keep_last \
  --qos-depth 10



# changes in nav2
navigation2/nav2_bringup/params/nav2_params.yaml
bt_navigator:
  ros__parameters:
    plugin_lib_names:
      - "run_if_flag_true_decorator"

navigation2/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml
    <RunIfFlagTrueDecorator name="CheckFlag" node="{node}" flag_topic="/mission_flag">
        <FollowPath path="{path}" controller_id="{selected_controller}" error_code_id="{follow_path_error_code}"/>
    </RunIfFlagTrueDecorator>