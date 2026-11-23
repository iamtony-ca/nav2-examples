#include "nav_data_recorder/grid_codec.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace nav_data_recorder
{

using nav_data_recorder_msgs::msg::CompactGrid;

void alignShift(
  const std::vector<uint8_t> & prev, uint32_t size_x, uint32_t size_y, int32_t sx, int32_t sy,
  std::vector<uint8_t> & out)
{
  const int64_t w = size_x;
  const int64_t h = size_y;
  out.assign(static_cast<size_t>(w * h), 0);
  // destination x range whose source x+sx is inside [0, w)
  const int64_t x0 = std::max<int64_t>(0, -static_cast<int64_t>(sx));
  const int64_t x1 = std::min<int64_t>(w, w - sx);
  if (x1 <= x0) {
    return;
  }
  for (int64_t y = 0; y < h; ++y) {
    const int64_t ys = y + sy;
    if (ys < 0 || ys >= h) {
      continue;
    }
    std::memcpy(&out[y * w + x0], &prev[ys * w + x0 + sx], static_cast<size_t>(x1 - x0));
  }
}

static GridGeometry geometryOf(const nav2_msgs::msg::Costmap & in)
{
  GridGeometry g;
  g.size_x = in.metadata.size_x;
  g.size_y = in.metadata.size_y;
  g.resolution = in.metadata.resolution;
  g.ox = in.metadata.origin.position.x;
  g.oy = in.metadata.origin.position.y;
  g.oz = in.metadata.origin.position.z;
  g.qx = in.metadata.origin.orientation.x;
  g.qy = in.metadata.origin.orientation.y;
  g.qz = in.metadata.origin.orientation.z;
  g.qw = in.metadata.origin.orientation.w;
  return g;
}

GridEncoder::GridEncoder(double keyframe_s)
: keyframe_s_(keyframe_s) {}

void GridEncoder::encode(
  const nav2_msgs::msg::Costmap & in, const std::string & source_topic, double now_s,
  CompactGrid & out)
{
  const GridGeometry g = geometryOf(in);
  out.header = in.header;
  out.source_topic = source_topic;
  out.layer = in.metadata.layer;
  out.resolution = g.resolution;
  out.size_x = g.size_x;
  out.size_y = g.size_y;
  out.origin = in.metadata.origin;
  out.shift_x = 0;
  out.shift_y = 0;

  const size_t n = static_cast<size_t>(g.size_x) * g.size_y;
  const bool size_ok = in.data.size() == n;

  bool key = force_key_ || !size_ok || prev_.size() != n || g.size_x != prev_geom_.size_x ||
    g.size_y != prev_geom_.size_y || g.resolution != prev_geom_.resolution ||
    g.qx != prev_geom_.qx || g.qy != prev_geom_.qy || g.qz != prev_geom_.qz ||
    g.qw != prev_geom_.qw || g.oz != prev_geom_.oz || g.resolution <= 0.f ||
    (now_s - last_key_s_) >= keyframe_s_ || now_s < last_key_s_;

  int32_t sx = 0, sy = 0;
  if (!key) {
    const double fx = (g.ox - prev_geom_.ox) / g.resolution;
    const double fy = (g.oy - prev_geom_.oy) / g.resolution;
    const double rx = std::round(fx);
    const double ry = std::round(fy);
    // origin must move by whole cells; costmap rolling windows always do
    if (std::fabs(fx - rx) > 1e-3 || std::fabs(fy - ry) > 1e-3 ||
      std::fabs(rx) >= g.size_x || std::fabs(ry) >= g.size_y)
    {
      key = true;
    } else {
      sx = static_cast<int32_t>(rx);
      sy = static_cast<int32_t>(ry);
    }
  }

  if (key) {
    out.encoding = CompactGrid::ENC_RAW;
    out.payload = in.data;
    last_key_s_ = now_s;
    force_key_ = false;
    ++keyframes_;
  } else {
    out.encoding = CompactGrid::ENC_XOR_PREV;
    out.shift_x = sx;
    out.shift_y = sy;
    const uint8_t * src;
    if (sx == 0 && sy == 0) {
      src = prev_.data();
    } else {
      alignShift(prev_, g.size_x, g.size_y, sx, sy, aligned_);
      src = aligned_.data();
    }
    out.payload.resize(n);
    const uint8_t * cur = in.data.data();
    for (size_t i = 0; i < n; ++i) {
      out.payload[i] = cur[i] ^ src[i];
    }
    ++deltas_;
  }

  if (size_ok) {
    prev_ = in.data;
    prev_geom_ = g;
  } else {
    prev_.clear();
    force_key_ = true;
  }
}

bool GridDecoder::decode(const CompactGrid & in, nav2_msgs::msg::Costmap & out)
{
  const size_t n = static_cast<size_t>(in.size_x) * in.size_y;
  if (in.payload.size() != n) {
    return false;
  }
  std::vector<uint8_t> cur;
  if (in.encoding == CompactGrid::ENC_RAW) {
    cur = in.payload;
  } else if (in.encoding == CompactGrid::ENC_XOR_PREV) {
    if (!have_key_ || in.size_x != size_x_ || in.size_y != size_y_) {
      return false;
    }
    std::vector<uint8_t> aligned;
    alignShift(prev_, in.size_x, in.size_y, in.shift_x, in.shift_y, aligned);
    cur.resize(n);
    for (size_t i = 0; i < n; ++i) {
      cur[i] = in.payload[i] ^ aligned[i];
    }
  } else {
    return false;
  }
  out.header = in.header;
  out.metadata.layer = in.layer;
  out.metadata.resolution = in.resolution;
  out.metadata.size_x = in.size_x;
  out.metadata.size_y = in.size_y;
  out.metadata.origin = in.origin;
  out.metadata.update_time = in.header.stamp;
  out.data = cur;
  prev_.swap(cur);
  size_x_ = in.size_x;
  size_y_ = in.size_y;
  have_key_ = true;
  return true;
}

}  // namespace nav_data_recorder
