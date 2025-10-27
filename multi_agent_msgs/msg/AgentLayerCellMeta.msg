std_msgs/Header header              # frame_id: map

uint16 machine_id
uint8  phase                        # AgentStatus.phase 값
string mode                         # "auto"/"manual"/...
bool   reroute
bool   re_path_search
bool   transferring

uint16 area_id                      # 없으면 0

# 위치 (world 또는 grid index; 둘 다 있으면 world 우선)
geometry_msgs/Point position        # 월드 좌표[m] (셀 센터)
uint32 mx
uint32 my

# 시간/불확실도
float32 t_first_hit                 # [s] 최초 점유/교차 예상 시각(모르면 음수)
float32 sigma                       # [m] 불확실도 등가 반경