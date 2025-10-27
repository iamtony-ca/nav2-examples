# frame_id: 보통 "map"
std_msgs/Header header

# --- 식별자 ---
uint16 machine_id
string type_id

# --- 현재 상태 ---
geometry_msgs/PoseStamped current_pose
# 현재 명령 속도(cmd_vel). 기본 프레임: base_link
geometry_msgs/Twist       current_twist

# 발자국(다각형)
geometry_msgs/PolygonStamped footprint

# 상위 상태
multi_agent_msgs/AgentStatus status

bool   reroute

# 웨이포인트(0~255)
uint8  current_waypoint
uint8  next_waypoint

# 0~5 m 프리뷰 (타임스탬프/프레임 포함)
# truncated_path.header.frame_id 는 MultiAgentInfo.header.frame_id 와 동일해야 함
nav_msgs/Path truncated_path

# --- 상호 관계/교차 인지 ---
uint16[] cross_agent_ids

# --- 운용 컨텍스트 ---
string mode

# --- 에어리어(협소/교차 섹션 등) ---
uint16 area_id
bool   occupancy

# --- 작업/리플랜 상태 플래그 ---
bool   transferring
bool   re_path_search

# --- 품질/불확실도 ---
float32 pos_std_m      # XY 위치 표준편차 [m], unknown이면 -1
float32 yaw_std_rad    # 요 표준편차 [rad], unknown이면 -1