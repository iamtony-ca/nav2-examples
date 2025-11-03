# MultiRobotState.msg
# frame_id: 보통 "map"
std_msgs/Header header

# --- 식별자 ---
uint16 machine_id             # 로봇 고유 숫자 ID (ex: 1001)
string type_id                # 로봇/플랫폼 타입 ID (ex: "MIR250", "CUSTOM_AMR_A")

# --- 현재 상태 ---
geometry_msgs/PoseStamped current_pose
geometry_msgs/Twist       current_twist   # 없으면 (0,0,0)로 발행 권장

# 발자국(원/다각형)
geometry_msgs/msg/PolygonStamped footprint

# 상위 상태(phase 등): 기존 RobotStatus를 그대로 사용
# (주의) 아래 mode/reroute/re_path_search/transferring와 중복될 수 있음.
multi_robot_msgs/RobotStatus status

bool   reroute                # 기존 goal cancel 후 새 waypoints 할당~첫 waypoint 도달 전까지 true

# 웨이포인트 인덱스(0~255)
uint8  current_waypoint
uint8  next_waypoint

# 0~5 m 전방 경로 프리뷰(선택)
geometry_msgs/PoseStamped[] truncated_path  # frame_id: header.frame_id

# 근미래 예측 궤적(있으면 RobotLayer가 우선 사용)
multi_robot_msgs/TimedTrajectory trajectory  # frame_id: header.frame_id

# --- 상호 관계/교차 인지 ---
uint16[] cross_agent_ids      # 교차 가능성이 높은 다른 AGV/AMR의 machine_id 목록


# --- 운용 컨텍스트 ---
string mode                   # "auto" | "manual" | 기타(플릿 표준 문자열)


# --- 에어리어(협소구간/교차로 등) ---
uint16 area_id                # 해당 없으면 0 또는 정의된 NONE 코드 사용
bool   occupancy              # 해당 area 점유(1) 중인지

# --- 작업/리플랜 상태 플래그 ---
bool   transferring           # 부하 이송 중(스테이션 상호작용 등)
bool   re_path_search         # 상위/플래너가 경로 재탐색 상태인지

# --- 품질/불확실도 ---
float32 pose_covariance       # [m] 표준편차 등가치(간단화용). 정교화 필요 시 Covariance로 대체 권장.
