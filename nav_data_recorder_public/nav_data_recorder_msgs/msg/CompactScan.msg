# Compact LaserScan record (nav_data_recorder).
# ranges are millimetres in uint16. Special codes:
#   0     = NaN (invalid measurement)
#   65533 = -inf (object closer than range_min, REP-117)
#   65534 = +inf (no return within range)
#   65535 = finite range >= 65.533 m (saturated)
#   1..65532 = finite range, round(r * 1000) clamped to at least 1
# ENC_ABS_MM   : payload = uint16 LE x beam_count (keyframe)
# ENC_DELTA_MM : payload = zigzag(int16(code - prev_code)) as uint16 LE x beam_count,
#                relative to the previous CompactScan of the same source_topic.
# Only add fields to this message; never change or remove existing ones.
uint8 ENC_ABS_MM=0
uint8 ENC_DELTA_MM=1
uint16 CODE_NAN=0
uint16 CODE_NEG_INF=65533
uint16 CODE_POS_INF=65534
uint16 CODE_SATURATED=65535

std_msgs/Header header        # original header
string source_topic
float32 angle_min
float32 angle_increment       # after decimation
float32 range_min
float32 range_max
float32 scan_time
float32 time_increment        # after decimation
uint16 decimation
uint16 beam_count
uint8 encoding
uint8[] payload
