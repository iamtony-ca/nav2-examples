// CompactScan and CompactGrid round trips.
#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <random>

#include "nav_data_recorder/grid_codec.hpp"
#include "nav_data_recorder/scan_codec.hpp"

using nav_data_recorder::GridDecoder;
using nav_data_recorder::GridEncoder;
using nav_data_recorder::ScanDecoder;
using nav_data_recorder::ScanEncoder;
using nav_data_recorder_msgs::msg::CompactGrid;
using nav_data_recorder_msgs::msg::CompactScan;

static sensor_msgs::msg::LaserScan makeScan(size_t n, std::mt19937 & rng)
{
  sensor_msgs::msg::LaserScan s;
  s.angle_min = -2.0f;
  s.angle_increment = 0.001f;
  s.angle_max = s.angle_min + s.angle_increment * (n - 1);
  s.range_min = 0.05f;
  s.range_max = 40.f;
  s.scan_time = 0.033f;
  s.time_increment = 1e-6f;
  std::uniform_real_distribution<float> d(0.05f, 40.f);
  s.ranges.resize(n);
  for (auto & r : s.ranges) {
    r = d(rng);
  }
  return s;
}

static void expectSame(float a, float b)
{
  if (std::isnan(a)) {
    EXPECT_TRUE(std::isnan(b));
  } else if (std::isinf(a)) {
    EXPECT_EQ(a, b);
  } else {
    EXPECT_LE(std::fabs(a - b), 0.0005f + 1e-6f) << a << " vs " << b;
  }
}

TEST(ScanCodec, RoundTripKeyAndDelta)
{
  std::mt19937 rng(1);
  ScanEncoder enc(1, 1.0);
  ScanDecoder dec;
  auto scan = makeScan(7200, rng);
  scan.ranges[0] = std::numeric_limits<float>::quiet_NaN();
  scan.ranges[1] = std::numeric_limits<float>::infinity();
  scan.ranges[2] = -std::numeric_limits<float>::infinity();
  scan.ranges[3] = 70.f;  // saturated
  int deltas = 0;
  for (int k = 0; k < 40; ++k) {
    std::normal_distribution<float> noise(0.f, 0.01f);
    for (size_t i = 4; i < scan.ranges.size(); ++i) {
      scan.ranges[i] = std::max(0.06f, scan.ranges[i] + noise(rng));
    }
    CompactScan c;
    enc.encode(scan, "/scan", k * 0.1, c);
    if (c.encoding == CompactScan::ENC_DELTA_MM) {
      ++deltas;
    }
    sensor_msgs::msg::LaserScan out;
    ASSERT_TRUE(dec.decode(c, out));
    ASSERT_EQ(out.ranges.size(), scan.ranges.size());
    for (size_t i = 0; i < 3; ++i) {
      expectSame(scan.ranges[i], out.ranges[i]);
    }
    EXPECT_NEAR(out.ranges[3], 65.535f, 1e-4);
    for (size_t i = 4; i < scan.ranges.size(); ++i) {
      expectSame(scan.ranges[i], out.ranges[i]);
    }
  }
  EXPECT_GT(deltas, 30);  // keyframe once per second at 10 Hz
  EXPECT_EQ(enc.keyframes(), 4u);
}

TEST(ScanCodec, Decimation)
{
  std::mt19937 rng(2);
  ScanEncoder enc(4, 1.0);
  ScanDecoder dec;
  auto scan = makeScan(7200, rng);
  CompactScan c;
  enc.encode(scan, "/scan", 0.0, c);
  EXPECT_EQ(c.beam_count, 1800);   // ceil(7200 / 4)
  EXPECT_FLOAT_EQ(c.angle_increment, scan.angle_increment * 4);
  sensor_msgs::msg::LaserScan out;
  ASSERT_TRUE(dec.decode(c, out));
  for (size_t i = 0; i < out.ranges.size(); ++i) {
    expectSame(scan.ranges[i * 4], out.ranges[i]);
  }
}

TEST(ScanCodec, DeltaWithoutKeyframeRejected)
{
  std::mt19937 rng(3);
  ScanEncoder enc(1, 10.0);
  auto scan = makeScan(100, rng);
  CompactScan k, d;
  enc.encode(scan, "/scan", 0.0, k);
  enc.encode(scan, "/scan", 0.1, d);
  ASSERT_EQ(d.encoding, CompactScan::ENC_DELTA_MM);
  ScanDecoder fresh;
  sensor_msgs::msg::LaserScan out;
  EXPECT_FALSE(fresh.decode(d, out));
  EXPECT_TRUE(fresh.decode(k, out));
  EXPECT_TRUE(fresh.decode(d, out));
}

TEST(ScanCodec, ForceKeyAndGeometryChange)
{
  std::mt19937 rng(4);
  ScanEncoder enc(1, 100.0);
  auto scan = makeScan(100, rng);
  CompactScan c;
  enc.encode(scan, "/s", 0.0, c);
  enc.encode(scan, "/s", 0.1, c);
  EXPECT_EQ(c.encoding, CompactScan::ENC_DELTA_MM);
  enc.forceKeyframe();
  enc.encode(scan, "/s", 0.2, c);
  EXPECT_EQ(c.encoding, CompactScan::ENC_ABS_MM);
  scan.ranges.resize(90);
  enc.encode(scan, "/s", 0.3, c);
  EXPECT_EQ(c.encoding, CompactScan::ENC_ABS_MM);
}

static nav2_msgs::msg::Costmap makeGrid(uint32_t w, uint32_t h, double ox, double oy, float res)
{
  nav2_msgs::msg::Costmap m;
  m.metadata.size_x = w;
  m.metadata.size_y = h;
  m.metadata.resolution = res;
  m.metadata.origin.position.x = ox;
  m.metadata.origin.position.y = oy;
  m.metadata.origin.orientation.w = 1.0;
  m.data.assign(static_cast<size_t>(w) * h, 0);
  return m;
}

// world-fixed obstacle field, sampled into a rolling window
static uint8_t world(int64_t gx, int64_t gy)
{
  const uint64_t h = static_cast<uint64_t>(gx * 73856093LL) ^ static_cast<uint64_t>(gy * 19349663LL);
  return (h % 17 == 0) ? 254 : (h % 23 == 0 ? 128 : 0);
}

TEST(GridCodec, RollingWindowRoundTrip)
{
  const float res = 0.02f;
  GridEncoder enc(10.0);
  GridDecoder dec;
  std::mt19937 rng(5);
  int64_t cx = 0, cy = 0;
  int xors = 0;
  for (int k = 0; k < 60; ++k) {
    cx += static_cast<int64_t>(rng() % 7) - 2;  // move 0..4 cells, sometimes back
    cy += static_cast<int64_t>(rng() % 5) - 2;
    auto m = makeGrid(500, 500, cx * res, cy * res, res);
    for (uint32_t y = 0; y < 500; ++y) {
      for (uint32_t x = 0; x < 500; ++x) {
        m.data[y * 500 + x] = world(cx + x, cy + y);
      }
    }
    m.data[(k * 31) % m.data.size()] = static_cast<uint8_t>(k);  // something changes
    CompactGrid c;
    enc.encode(m, "/local_costmap/costmap_raw", k * 0.5, c);
    if (c.encoding == CompactGrid::ENC_XOR_PREV) {
      ++xors;
    }
    nav2_msgs::msg::Costmap out;
    ASSERT_TRUE(dec.decode(c, out));
    ASSERT_EQ(out.data, m.data) << "frame " << k;
    EXPECT_DOUBLE_EQ(out.metadata.origin.position.x, m.metadata.origin.position.x);
  }
  EXPECT_GT(xors, 50);
  EXPECT_EQ(enc.keyframes(), 3u);  // t = 0, 10, 20 s ... 29.5 s
}

TEST(GridCodec, XorIsMostlyZeroWhenStatic)
{
  GridEncoder enc(10.0);
  auto m = makeGrid(200, 100, 1.0, 2.0, 0.05f);
  for (size_t i = 0; i < m.data.size(); ++i) {
    m.data[i] = static_cast<uint8_t>(i * 7);
  }
  CompactGrid a, b;
  enc.encode(m, "/g", 0.0, a);
  m.data[5] = 99;
  enc.encode(m, "/g", 0.5, b);
  ASSERT_EQ(b.encoding, CompactGrid::ENC_XOR_PREV);
  size_t nz = 0;
  for (auto v : b.payload) {
    nz += v != 0;
  }
  EXPECT_EQ(nz, 1u);
}

TEST(GridCodec, SizeChangeForcesKeyframe)
{
  GridEncoder enc(100.0);
  GridDecoder dec;
  auto a = makeGrid(10, 10, 0, 0, 0.1f);
  auto b = makeGrid(12, 10, 0, 0, 0.1f);
  CompactGrid ca, cb;
  enc.encode(a, "/g", 0.0, ca);
  enc.encode(b, "/g", 0.1, cb);
  EXPECT_EQ(cb.encoding, CompactGrid::ENC_RAW);
  nav2_msgs::msg::Costmap out;
  EXPECT_TRUE(dec.decode(cb, out));
}

TEST(GridCodec, FractionalOriginMoveForcesKeyframe)
{
  GridEncoder enc(100.0);
  auto a = makeGrid(10, 10, 0, 0, 0.1f);
  auto b = makeGrid(10, 10, 0.05, 0, 0.1f);
  CompactGrid ca, cb;
  enc.encode(a, "/g", 0.0, ca);
  enc.encode(b, "/g", 0.1, cb);
  EXPECT_EQ(cb.encoding, CompactGrid::ENC_RAW);
}
