ros2 topic pub --once --qos-durability transient_local /velocity_modifier/control robot_interfaces/msg/ModifierControl 'command_type: 1
linear_value: 0.35
angular_value: 0.5
' 



ros2 interface show action_msgs/msg/GoalStatusArray
# An array of goal statuses.
GoalStatus[] status_list
        #
        int8 STATUS_UNKNOWN   = 0
        int8 STATUS_ACCEPTED  = 1
        int8 STATUS_EXECUTING = 2
        int8 STATUS_CANCELING = 3
        int8 STATUS_SUCCEEDED = 4
        int8 STATUS_CANCELED  = 5
        int8 STATUS_ABORTED   = 6
        GoalInfo goal_info
                unique_identifier_msgs/UUID goal_id
                        #
                        uint8[16] uuid
                builtin_interfaces/Time stamp
                        int32 sec
                        uint32 nanosec
        int8 status




# memo
/navigate_to_pose/_action/status : status가 누적되어 pub, 2,3,4,5,6


/compute_path_to_pose/_action/status : 2,4
/follow_path/_action/status : 2,3,4,5

# real status logic
enum class RobotStatus : uint8_t {
    IDLE, -> /navigate_to_pose/_action/status (0,1,2,3 이 아님 && 4,5,6 상태) && /bt_navigator/get_state, /bt_navigator/transition_event,  planner_server, controller_server (active) && motor stop 상태.
    RECEIVED_GOAL, -> /behavior_tree_log 의 최상위 node 실행시 조건(IDLE -> SUCCESS || Running)
    PLANNING, -> /compute_path_to_pose/_action/status (2) && recovery bt node 아닐시 && /navigate_to_pose/_action/status (2)
    DRIVING, -> /follow_path/_action/status (2) && recovery bt node 아닐시 && /navigate_to_pose/_action/status (2)
    FOLLOWING_WAYPOINTS, -> /follow_waypoints/_action/status (2) && recovery bt node 아닐시 && /navigate_to_pose/_action/status (2)
    PAUSED, -> (/behavior_tree_log 로 판별 || pasue_flag로 판별) && /navigate_to_pose/_action/status (2)
    COLLISION_IMMINENT, -> /collision_flag topic으로 판별.
    RECOVERY_CLEARING, -> /behavior_tree_log 로 판별 && /navigate_to_pose/_action/status (2)
    RECOVERY_PLANNING, -> /behavior_tree_log의 recovery용 compute_path_to_pose 실행여부로 판별. && /navigate_to_pose/_action/status (2)
    RECOVERY_DRIVING, -> /behavior_tree_log의 recovery용 follow_path 실행여부로 판별 && /navigate_to_pose/_action/status (2)
    SUCCEEDED, -> /navigate_to_pose/_action/status (4) array의 마지막 값만 확인.
    FAILED, -> /navigate_to_pose/_action/status (6) array의 마지막 값만 확인.
    CANCELED -> -> /navigate_to_pose/_action/status (5) array의 마지막 값만 확인.
};

# temp status logic
enum class RobotStatus : uint8_t {
    IDLE, -> /navigate_to_pose/_action/status (0,1,2,3 이 아님 && 4,5,6 상태) && /bt_navigator/get_state, /bt_navigator/transition_event,  planner_server, controller_server (active) && motor stop 상태.
    RECEIVED_GOAL, -> /behavior_tree_log 의 최상위 node 실행시 조건(IDLE -> SUCCESS || Running)
    PLANNING, -> /compute_path_to_pose/_action/status (2) && recovery bt node 아닐시 && /navigate_to_pose/_action/status (2)
    DRIVING, -> /follow_path/_action/status (2) && recovery bt node 아닐시 && /navigate_to_pose/_action/status (2)
    FOLLOWING_WAYPOINTS, -> /follow_waypoints/_action/status (2) && recovery bt node 아닐시 && /navigate_to_pose/_action/status (2)
    PAUSED, -> (/behavior_tree_log 로 판별 || pasue_flag로 판별) && /navigate_to_pose/_action/status (2)
    COLLISION_IMMINENT, -> /collision_flag topic으로 판별.
    RECOVERY_CLEARING, -> /behavior_tree_log 로 판별 && /navigate_to_pose/_action/status (2)
    RECOVERY_PLANNING, -> /behavior_tree_log의 recovery용 compute_path_to_pose 실행여부로 판별. && /navigate_to_pose/_action/status (2)
    RECOVERY_DRIVING, -> /behavior_tree_log의 recovery용 follow_path 실행여부로 판별 && /navigate_to_pose/_action/status (2)
    SUCCEEDED, -> /navigate_to_pose/_action/status (4) .
    FAILED, -> /navigate_to_pose/_action/status (6) .
    CANCELED -> -> /navigate_to_pose/_action/status (5)  확인.
};





# examples
# ros2 topic list --include-hidden-topics -t | grep transition
/amcl/transition_event [lifecycle_msgs/msg/TransitionEvent]
/behavior_server/transition_event [lifecycle_msgs/msg/TransitionEvent]
/bt_navigator/transition_event [lifecycle_msgs/msg/TransitionEvent]
/collision_monitor/transition_event [lifecycle_msgs/msg/TransitionEvent]
/controller_server/transition_event [lifecycle_msgs/msg/TransitionEvent]
/docking_server/transition_event [lifecycle_msgs/msg/TransitionEvent]
/global_costmap/global_costmap/transition_event [lifecycle_msgs/msg/TransitionEvent]
/local_costmap/local_costmap/transition_event [lifecycle_msgs/msg/TransitionEvent]
/map_server/transition_event [lifecycle_msgs/msg/TransitionEvent]
/planner_server/transition_event [lifecycle_msgs/msg/TransitionEvent]
/smoother_server/transition_event [lifecycle_msgs/msg/TransitionEvent]
/velocity_smoother/transition_event [lifecycle_msgs/msg/TransitionEvent]
/waypoint_follower/transition_event [lifecycle_msgs/msg/TransitionEvent]

  
# ros2 topilist --include-hidden-topics -t | grep status
/assisted_teleop/_action/status [action_msgs/msg/GoalStatusArray]
/backup/_action/status [action_msgs/msg/GoalStatusArray]
/compute_path_through_poses/_action/status [action_msgs/msg/GoalStatusArray]
/compute_path_to_pose/_action/status [action_msgs/msg/GoalStatusArray]
/dock_robot/_action/status [action_msgs/msg/GoalStatusArray]
/drive_on_heading/_action/status [action_msgs/msg/GoalStatusArray]
/follow_gps_waypoints/_action/status [action_msgs/msg/GoalStatusArray]
/follow_path/_action/status [action_msgs/msg/GoalStatusArray]
/follow_waypoints/_action/status [action_msgs/msg/GoalStatusArray]
/navigate_through_poses/_action/status [action_msgs/msg/GoalStatusArray]
/navigate_to_pose/_action/status [action_msgs/msg/GoalStatusArray]
/robot_status [std_msgs/msg/String]
/smooth_path/_action/status [action_msgs/msg/GoalStatusArray]
/spin/_action/status [action_msgs/msg/GoalStatusArray]
/undock_robot/_action/status [action_msgs/msg/GoalStatusArray]
/wait/_action/status [action_msgs/msg/GoalStatusArray]



# ros2 topic list --include-hidden-topics -t | grep log
/behavior_tree_log [nav2_msgs/msg/BehaviorTreeLog]



# ros2 service list | grep state
/amcl/change_state
/amcl/get_available_states
/amcl/get_state
/behavior_server/change_state
/behavior_server/get_available_states
/behavior_server/get_state
/bt_navigator/change_state
/bt_navigator/get_available_states
/bt_navigator/get_state
/collision_monitor/change_state
/collision_monitor/get_available_states
/collision_monitor/get_state
/controller_server/change_state
/controller_server/get_available_states
/controller_server/get_state
/docking_server/change_state
/docking_server/get_available_states
/docking_server/get_state
/global_costmap/global_costmap/change_state
/global_costmap/global_costmap/get_available_states
/global_costmap/global_costmap/get_state
/local_costmap/local_costmap/change_state
/local_costmap/local_costmap/get_available_states
/local_costmap/local_costmap/get_state
/map_server/change_state
/map_server/get_available_states
/map_server/get_state
/planner_server/change_state
/planner_server/get_available_states
/planner_server/get_state
/robot_state_publisher/describe_parameters
/robot_state_publisher/get_parameter_types
/robot_state_publisher/get_parameters
/robot_state_publisher/get_type_description
/robot_state_publisher/list_parameters
/robot_state_publisher/set_parameters
/robot_state_publisher/set_parameters_atomically
/smoother_server/change_state
/smoother_server/get_available_states
/smoother_server/get_state
/velocity_smoother/change_state
/velocity_smoother/get_available_states
/velocity_smoother/get_state
/waypoint_follower/change_state
/waypoint_follower/get_available_states
/waypoint_follower/get_state