# PathStaticCollisionInfo.msg

std_msgs/Header header
bool replan_request
bool is_goal_occupied          # Goal 점유 여부
bool is_last_goal_occupied          # last Goal 점유 여부
float64 hit_x                  # 충돌 지점 X
float64 hit_y                  # 충돌 지점 Y
geometry_msgs/Pose target_goal # 목표 지점(Goal) 좌표