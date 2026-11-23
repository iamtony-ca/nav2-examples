#include "nav_data_recorder/scan_codec.hpp"

#include <cmath>
#include <cstring>
#include <limits>
#include <string>

namespace nav_data_recorder
{

using nav_data_recorder_msgs::msg::CompactScan;

uint16_t rangeToCode(float r)
{
  if (std::isnan(r)) {
    return CompactScan::CODE_NAN;
  }
  if (std::isinf(r)) {
    return r > 0 ? CompactScan::CODE_POS_INF : CompactScan::CODE_NEG_INF;
  }
  const double mm = std::round(static_cast<double>(r) * 1000.0);
  if (mm >= 65533.0) {
    return CompactScan::CODE_SATURATED;
  }
  if (mm < 1.0) {
    return 1;
  }
  return static_cast<uint16_t>(mm);
}

float codeToRange(uint16_t code)
{
  switch (code) {
    case CompactScan::CODE_NAN: return std::numeric_limits<float>::quiet_NaN();
    case CompactScan::CODE_NEG_INF: return -std::numeric_limits<float>::infinity();
    case CompactScan::CODE_POS_INF: return std::numeric_limits<float>::infinity();
    case CompactScan::CODE_SATURATED: return 65.535f;
    default: return static_cast<float>(code) / 1000.0f;
  }
}

static void putU16(std::vector<uint8_t> & buf, size_t i, uint16_t v)
{
  buf[2 * i] = static_cast<uint8_t>(v & 0xff);
  buf[2 * i + 1] = static_cast<uint8_t>(v >> 8);
}

static uint16_t getU16(const std::vector<uint8_t> & buf, size_t i)
{
  return static_cast<uint16_t>(buf[2 * i] | (static_cast<uint16_t>(buf[2 * i + 1]) << 8));
}

ScanEncoder::ScanEncoder(uint16_t decimation, double keyframe_s)
: decimation_(decimation == 0 ? 1 : decimation), keyframe_s_(keyframe_s) {}

void ScanEncoder::encode(
  const sensor_msgs::msg::LaserScan & in, const std::string & source_topic, double now_s,
  CompactScan & out)
{
  const size_t n_in = in.ranges.size();
  const size_t n = (n_in + decimation_ - 1) / decimation_;

  std::vector<uint16_t> codes(n);
  for (size_t i = 0; i < n; ++i) {
    codes[i] = rangeToCode(in.ranges[i * decimation_]);
  }

  out.header = in.header;
  out.source_topic = source_topic;
  out.angle_min = in.angle_min;
  out.angle_increment = in.angle_increment * static_cast<float>(decimation_);
  out.range_min = in.range_min;
  out.range_max = in.range_max;
  out.scan_time = in.scan_time;
  out.time_increment = in.time_increment * static_cast<float>(decimation_);
  out.decimation = decimation_;
  out.beam_count = static_cast<uint16_t>(n);

  const bool geometry_same = prev_.size() == n && prev_angle_min_ == out.angle_min &&
    prev_angle_inc_ == out.angle_increment;
  const bool key = force_key_ || !geometry_same || (now_s - last_key_s_) >= keyframe_s_ ||
    now_s < last_key_s_;

  out.payload.resize(2 * n);
  if (key) {
    out.encoding = CompactScan::ENC_ABS_MM;
    for (size_t i = 0; i < n; ++i) {
      putU16(out.payload, i, codes[i]);
    }
    last_key_s_ = now_s;
    force_key_ = false;
    ++keyframes_;
  } else {
    out.encoding = CompactScan::ENC_DELTA_MM;
    for (size_t i = 0; i < n; ++i) {
      const int16_t d = static_cast<int16_t>(static_cast<uint16_t>(codes[i] - prev_[i]));
      putU16(out.payload, i, zigzagEncode(d));
    }
    ++deltas_;
  }
  prev_.swap(codes);
  prev_angle_min_ = out.angle_min;
  prev_angle_inc_ = out.angle_increment;
}

bool ScanDecoder::decode(const CompactScan & in, sensor_msgs::msg::LaserScan & out)
{
  const size_t n = in.beam_count;
  if (in.payload.size() != 2 * n) {
    return false;
  }
  std::vector<uint16_t> codes(n);
  if (in.encoding == CompactScan::ENC_ABS_MM) {
    for (size_t i = 0; i < n; ++i) {
      codes[i] = getU16(in.payload, i);
    }
  } else if (in.encoding == CompactScan::ENC_DELTA_MM) {
    if (!have_key_ || prev_.size() != n) {
      return false;
    }
    for (size_t i = 0; i < n; ++i) {
      codes[i] = static_cast<uint16_t>(prev_[i] + static_cast<uint16_t>(zigzagDecode(getU16(in.payload, i))));
    }
  } else {
    return false;
  }

  out.header = in.header;
  out.angle_min = in.angle_min;
  out.angle_increment = in.angle_increment;
  out.angle_max = in.angle_min + in.angle_increment * static_cast<float>(n == 0 ? 0 : n - 1);
  out.time_increment = in.time_increment;
  out.scan_time = in.scan_time;
  out.range_min = in.range_min;
  out.range_max = in.range_max;
  out.ranges.resize(n);
  for (size_t i = 0; i < n; ++i) {
    out.ranges[i] = codeToRange(codes[i]);
  }
  out.intensities.clear();
  prev_.swap(codes);
  have_key_ = true;
  return true;
}

}  // namespace nav_data_recorder
