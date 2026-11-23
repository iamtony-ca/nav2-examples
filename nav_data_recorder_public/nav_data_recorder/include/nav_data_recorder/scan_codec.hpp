// CompactScan encoder/decoder.
//   ranges -> uint16 mm codes (see CompactScan.msg), beam decimation, keyframe every keyframe_s,
//   delta (zigzag int16) against the previous scan in between. Intensities are dropped.
#ifndef NAV_DATA_RECORDER__SCAN_CODEC_HPP_
#define NAV_DATA_RECORDER__SCAN_CODEC_HPP_

#include <cstdint>
#include <vector>

#include "nav_data_recorder_msgs/msg/compact_scan.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

namespace nav_data_recorder
{

uint16_t rangeToCode(float r);
float codeToRange(uint16_t code);

inline uint16_t zigzagEncode(int16_t v)
{
  return static_cast<uint16_t>((static_cast<uint16_t>(v) << 1) ^ static_cast<uint16_t>(v >> 15));
}
inline int16_t zigzagDecode(uint16_t v)
{
  return static_cast<int16_t>((v >> 1) ^ static_cast<uint16_t>(-static_cast<int16_t>(v & 1)));
}

class ScanEncoder
{
public:
  ScanEncoder(uint16_t decimation, double keyframe_s);

  // now_s: monotonic-ish time in seconds used only for the keyframe interval.
  void encode(
    const sensor_msgs::msg::LaserScan & in, const std::string & source_topic, double now_s,
    nav_data_recorder_msgs::msg::CompactScan & out);

  // Next encode() writes a keyframe (file split, restart).
  void forceKeyframe() {force_key_ = true;}

  uint64_t keyframes() const {return keyframes_;}
  uint64_t deltas() const {return deltas_;}

private:
  uint16_t decimation_;
  double keyframe_s_;
  bool force_key_{true};
  double last_key_s_{0.0};
  std::vector<uint16_t> prev_;
  float prev_angle_min_{0.f};
  float prev_angle_inc_{0.f};
  uint64_t keyframes_{0};
  uint64_t deltas_{0};
};

class ScanDecoder
{
public:
  // Returns false (and leaves out untouched) when a delta arrives without a matching keyframe.
  bool decode(const nav_data_recorder_msgs::msg::CompactScan & in, sensor_msgs::msg::LaserScan & out);

private:
  std::vector<uint16_t> prev_;
  bool have_key_{false};
};

}  // namespace nav_data_recorder

#endif  // NAV_DATA_RECORDER__SCAN_CODEC_HPP_
