# PathAgentCollisionInfo.msg
# 코스트 셀과 결부된 "로봇 유래" 메타정보 (희소 형태)
# 좌표: map 좌표계(world) 

std_msgs/Header header         # frame_id: map
uint16[] machine_id
string[] type_id

# 수치 요약(있으면 채움)
float64[] x                      # 경로상의 최초 충돌 지점 x
float64[] y                      # 경로상의 최초 충돌 지점 y 
float32[]  ttc_first                     # 최초 추돌 예상시각 [s], 모르면 음수
string[]   note 

bool is_goal_occupied          # Goal 점유 여부
bool is_last_goal_occupied          # Goal 점유 여부
geometry_msgs/Pose target_goal # 목표 지점(Goal) 좌표