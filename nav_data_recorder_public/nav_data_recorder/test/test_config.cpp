// The shipped profiles parse; the default profile (plain Nav2) has the expected entries.
// Runs in the profiles/ source directory (see CMakeLists.txt).
#include <gtest/gtest.h>

#include <algorithm>
#include <cstdio>
#include <filesystem>

#include "nav_data_recorder/config.hpp"

using namespace nav_data_recorder;

TEST(Config, ShippedTopicsYaml)
{
  const auto cfg = loadConfig("default/topics.yaml");
  auto find = [&](const std::string & t) {
      return std::find_if(cfg.topics.begin(), cfg.topics.end(),
               [&](const TopicCfg & c) {return c.topic == t;});
    };
  ASSERT_NE(find("/tf"), cfg.topics.end());
  EXPECT_EQ(find("/tf")->mode, Mode::Tf);
  EXPECT_DOUBLE_EQ(find("/tf")->rate, 50.0);
  EXPECT_TRUE(find("/map")->latched);
  EXPECT_EQ(find("/scan")->mode, Mode::Scan);
  EXPECT_EQ(find("/scan")->output, "/nav_data/compact/scan");
  EXPECT_DOUBLE_EQ(find("/scan")->keyframe_s, 1.0);
  EXPECT_EQ(find("/local_costmap/costmap_raw")->mode, Mode::Grid);
  EXPECT_EQ(find("/global_costmap/costmap_raw")->mode, Mode::Grid);
  EXPECT_EQ(find("/nav_data/mark")->type, "std_msgs/msg/String");
  EXPECT_FALSE(cfg.global_keyframe.enabled);   // diff-stream mode is opt-in
  EXPECT_FALSE(cfg.events.empty());
  // the OccupancyGrid views are rebuilt by nav_data_export, not recorded
  EXPECT_EQ(find("/local_costmap/costmap"), cfg.topics.end());
  EXPECT_EQ(find("/global_costmap/costmap"), cfg.topics.end());
}

TEST(Config, EveryProfileParses)
{
  int n = 0;
  for (const auto & d : std::filesystem::directory_iterator(".")) {
    const auto f = d.path() / "topics.yaml";
    if (!d.is_directory() || !std::filesystem::exists(f)) {continue;}
    SCOPED_TRACE(f.string());
    EXPECT_NO_THROW({
      const auto cfg = loadConfig(f.string());
      EXPECT_FALSE(cfg.topics.empty());
    });
    ++n;
  }
  EXPECT_GE(n, 1);
}

TEST(Config, GlobalKeyframeOptions)
{
  const char * p = "/tmp/nav_data_recorder_test_gk.yaml";
  FILE * f = std::fopen(p, "w");
  std::fputs("global_keyframe:\n  service: global_costmap/my_layer/get_costmap\n"
    "  output: nav_data/key\n  occupancy_output: global_costmap/costmap\n", f);
  std::fclose(f);
  const auto cfg = loadConfig(p);
  EXPECT_TRUE(cfg.global_keyframe.enabled);
  // relative names become absolute (the recorded topics and recorder_info use absolute names)
  EXPECT_EQ(cfg.global_keyframe.service, "/global_costmap/my_layer/get_costmap");
  EXPECT_EQ(cfg.global_keyframe.output, "/nav_data/key");
  EXPECT_EQ(cfg.global_keyframe.occupancy_output, "/global_costmap/costmap");
  std::remove(p);
}

TEST(Config, RejectsDuplicates)
{
  const char * p = "/tmp/nav_data_recorder_test_dup.yaml";
  FILE * f = std::fopen(p, "w");
  std::fputs("passthrough:\n  - {topic: /a}\n  - {topic: a}\n", f);
  std::fclose(f);
  EXPECT_THROW(loadConfig(p), std::runtime_error);
}
