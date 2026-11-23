// CompactGrid encoder/decoder for nav2_msgs/Costmap.
//   Keyframe (raw bytes) every keyframe_s and on force; otherwise XOR against the previous frame
//   shifted by the origin move (whole cells). Unchanged cells become 0 and zstd removes them.
#ifndef NAV_DATA_RECORDER__GRID_CODEC_HPP_
#define NAV_DATA_RECORDER__GRID_CODEC_HPP_

#include <cstdint>
#include <vector>

#include "nav2_msgs/msg/costmap.hpp"
#include "nav_data_recorder_msgs/msg/compact_grid.hpp"

namespace nav_data_recorder
{

struct GridGeometry
{
  uint32_t size_x{0};
  uint32_t size_y{0};
  float resolution{0.f};
  double ox{0.0};
  double oy{0.0};
  double oz{0.0};
  double qx{0.0}, qy{0.0}, qz{0.0}, qw{1.0};
};

// aligned[y][x] = prev[y+sy][x+sx] if inside prev else 0. prev and out have the same size.
void alignShift(
  const std::vector<uint8_t> & prev, uint32_t size_x, uint32_t size_y, int32_t sx, int32_t sy,
  std::vector<uint8_t> & out);

class GridEncoder
{
public:
  explicit GridEncoder(double keyframe_s);

  void encode(
    const nav2_msgs::msg::Costmap & in, const std::string & source_topic, double now_s,
    nav_data_recorder_msgs::msg::CompactGrid & out);

  void forceKeyframe() {force_key_ = true;}
  uint64_t keyframes() const {return keyframes_;}
  uint64_t deltas() const {return deltas_;}

private:
  double keyframe_s_;
  bool force_key_{true};
  double last_key_s_{0.0};
  GridGeometry prev_geom_;
  std::vector<uint8_t> prev_;
  std::vector<uint8_t> aligned_;
  uint64_t keyframes_{0};
  uint64_t deltas_{0};
};

class GridDecoder
{
public:
  bool decode(const nav_data_recorder_msgs::msg::CompactGrid & in, nav2_msgs::msg::Costmap & out);

private:
  std::vector<uint8_t> prev_;
  uint32_t size_x_{0};
  uint32_t size_y_{0};
  bool have_key_{false};
};

}  // namespace nav_data_recorder

#endif  // NAV_DATA_RECORDER__GRID_CODEC_HPP_
