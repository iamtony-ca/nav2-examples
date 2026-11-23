# Compact costmap record (nav_data_recorder). Values 0..255 are kept losslessly.
# ENC_RAW      : payload = size_x*size_y bytes, row-major (keyframe)
# ENC_XOR_PREV : payload = cur XOR aligned(prev). aligned(prev)[y][x] = prev[y+shift_y][x+shift_x]
#                when inside the previous grid, else 0. shift = origin move in cells.
# Only add fields to this message; never change or remove existing ones.
uint8 ENC_RAW=0
uint8 ENC_XOR_PREV=1

std_msgs/Header header
string source_topic
string layer
float32 resolution
uint32 size_x
uint32 size_y
geometry_msgs/Pose origin
int32 shift_x
int32 shift_y
uint8 encoding
uint8[] payload
