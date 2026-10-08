// Copyright (c) 2020, Samsung Research America
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License. Reserved.

#include <math.h>
#include <cmath>  // [10-07 Q40]
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_costmap_2d/costmap_subscriber.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_core/planner_exceptions.hpp"  // [10-07 Q40]
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "ammr_smac_hybrid_planner/node_hybrid.hpp"
#include "ammr_smac_hybrid_planner/a_star.hpp"
#include "ammr_smac_hybrid_planner/collision_checker.hpp"
#include "ammr_smac_hybrid_planner/smac_planner_hybrid.hpp"
#include "ammr_smac_hybrid_planner/smac_planner_2d.hpp"

class RclCppFixture
{
public:
  RclCppFixture() {rclcpp::init(0, nullptr);}
  ~RclCppFixture() {rclcpp::shutdown();}
};
RclCppFixture g_rclcppfixture;

// SMAC smoke tests for plugin-level issues rather than algorithms
// (covered by more extensively testing in other files)
// System tests in nav2_system_tests will actually plan with this work

TEST(SmacTest, test_smac_se2)
{
  rclcpp_lifecycle::LifecycleNode::SharedPtr nodeSE2 =
    std::make_shared<rclcpp_lifecycle::LifecycleNode>("SmacSE2Test");
  nodeSE2->declare_parameter("test.debug_visualizations", rclcpp::ParameterValue(true));

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros =
    std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");
  costmap_ros->on_configure(rclcpp_lifecycle::State());

  nodeSE2->declare_parameter("test.downsample_costmap", true);
  nodeSE2->set_parameter(rclcpp::Parameter("test.downsample_costmap", true));
  nodeSE2->declare_parameter("test.downsampling_factor", 2);
  nodeSE2->set_parameter(rclcpp::Parameter("test.downsampling_factor", 2));

  auto dummy_cancel_checker = []() {
      return false;
    };

  geometry_msgs::msg::PoseStamped start, goal;
  start.pose.position.x = 0.0;
  start.pose.position.y = 0.0;
  start.pose.orientation.w = 1.0;
  goal.pose.position.x = 1.0;
  goal.pose.position.y = 1.0;
  goal.pose.orientation.w = 1.0;
  auto planner = std::make_unique<ammr_smac_hybrid_planner::SmacPlannerHybrid>();
  planner->configure(nodeSE2, "test", nullptr, costmap_ros);
  planner->activate();

  try {
    planner->createPlan(start, goal, dummy_cancel_checker);
  } catch (...) {
  }

  // corner case where the start and goal are on the same cell
  goal.pose.position.x = 0.01;
  goal.pose.position.y = 0.01;

  nav_msgs::msg::Path plan = planner->createPlan(start, goal, dummy_cancel_checker);
  EXPECT_EQ(plan.poses.size(), 1);  // single point path

  planner->deactivate();
  planner->cleanup();

  planner.reset();
  costmap_ros->on_cleanup(rclcpp_lifecycle::State());
  costmap_ros.reset();
  nodeSE2.reset();
}

TEST(SmacTest, test_smac_se2_reconfigure)
{
  rclcpp_lifecycle::LifecycleNode::SharedPtr nodeSE2 =
    std::make_shared<rclcpp_lifecycle::LifecycleNode>("SmacSE2Test");

  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros =
    std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");
  costmap_ros->on_configure(rclcpp_lifecycle::State());

  auto planner = std::make_unique<ammr_smac_hybrid_planner::SmacPlannerHybrid>();
  planner->configure(nodeSE2, "test", nullptr, costmap_ros);
  planner->activate();

  nodeSE2->declare_parameter("resolution", 0.05);

  auto rec_param = std::make_shared<rclcpp::AsyncParametersClient>(
    nodeSE2->get_node_base_interface(), nodeSE2->get_node_topics_interface(),
    nodeSE2->get_node_graph_interface(),
    nodeSE2->get_node_services_interface());

  auto results = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("test.downsample_costmap", true),
      rclcpp::Parameter("test.downsampling_factor", 2),
      rclcpp::Parameter("test.angle_quantization_bins", 100),
      rclcpp::Parameter("test.allow_unknown", false),
      rclcpp::Parameter("test.max_iterations", -1),
      rclcpp::Parameter("test.minimum_turning_radius", 1.0),
      rclcpp::Parameter("test.cache_obstacle_heuristic", true),
      rclcpp::Parameter("test.reverse_penalty", 5.0),
      rclcpp::Parameter("test.change_penalty", 1.0),
      rclcpp::Parameter("test.non_straight_penalty", 2.0),
      rclcpp::Parameter("test.cost_penalty", 2.0),
      rclcpp::Parameter("test.tolerance", 0.2),
      rclcpp::Parameter("test.retrospective_penalty", 0.2),
      rclcpp::Parameter("test.analytic_expansion_ratio", 4.0),
      rclcpp::Parameter("test.max_planning_time", 10.0),
      rclcpp::Parameter("test.lookup_table_size", 30.0),
      rclcpp::Parameter("test.smooth_path", false),
      rclcpp::Parameter("test.analytic_expansion_max_length", 42.0),
      rclcpp::Parameter("test.max_on_approach_iterations", 42),
      rclcpp::Parameter("test.terminal_checking_interval", 42),
      rclcpp::Parameter("test.motion_model_for_search", std::string("REEDS_SHEPP"))});

  rclcpp::spin_until_future_complete(
    nodeSE2->get_node_base_interface(),
    results);

  EXPECT_EQ(nodeSE2->get_parameter("test.downsample_costmap").as_bool(), true);
  EXPECT_EQ(nodeSE2->get_parameter("test.downsampling_factor").as_int(), 2);
  EXPECT_EQ(nodeSE2->get_parameter("test.angle_quantization_bins").as_int(), 100);
  EXPECT_EQ(nodeSE2->get_parameter("test.allow_unknown").as_bool(), false);
  EXPECT_EQ(nodeSE2->get_parameter("test.max_iterations").as_int(), -1);
  EXPECT_EQ(nodeSE2->get_parameter("test.minimum_turning_radius").as_double(), 1.0);
  EXPECT_EQ(nodeSE2->get_parameter("test.cache_obstacle_heuristic").as_bool(), true);
  EXPECT_EQ(nodeSE2->get_parameter("test.reverse_penalty").as_double(), 5.0);
  EXPECT_EQ(nodeSE2->get_parameter("test.change_penalty").as_double(), 1.0);
  EXPECT_EQ(nodeSE2->get_parameter("test.non_straight_penalty").as_double(), 2.0);
  EXPECT_EQ(nodeSE2->get_parameter("test.cost_penalty").as_double(), 2.0);
  EXPECT_EQ(nodeSE2->get_parameter("test.retrospective_penalty").as_double(), 0.2);
  EXPECT_EQ(nodeSE2->get_parameter("test.tolerance").as_double(), 0.2);
  EXPECT_EQ(nodeSE2->get_parameter("test.analytic_expansion_ratio").as_double(), 4.0);
  EXPECT_EQ(nodeSE2->get_parameter("test.smooth_path").as_bool(), false);
  EXPECT_EQ(nodeSE2->get_parameter("test.max_planning_time").as_double(), 10.0);
  EXPECT_EQ(nodeSE2->get_parameter("test.lookup_table_size").as_double(), 30.0);
  EXPECT_EQ(nodeSE2->get_parameter("test.analytic_expansion_max_length").as_double(), 42.0);
  EXPECT_EQ(nodeSE2->get_parameter("test.max_on_approach_iterations").as_int(), 42);
  EXPECT_EQ(nodeSE2->get_parameter("test.terminal_checking_interval").as_int(), 42);
  EXPECT_EQ(
    nodeSE2->get_parameter("test.motion_model_for_search").as_string(),
    std::string("REEDS_SHEPP"));

  auto results2 = rec_param->set_parameters_atomically(
    {rclcpp::Parameter("resolution", 0.2)});
  rclcpp::spin_until_future_complete(
    nodeSE2->get_node_base_interface(),
    results2);
  EXPECT_EQ(nodeSE2->get_parameter("resolution").as_double(), 0.2);
}

// ===================================================================================
// [10-07 Q40] same-spot corner case 시험
//   through-poses 의 다음 구간 start(앞 구간 path 끝)가 goal 과 ~1e-6 cell 차이로 cell 경계
//   반대편에 놓이면 floor 비교만으로는 같은 칸 shortcut 이 빗나갔다. 새 조건은
//   "floor 같은 칸 OR start-goal 거리 < 0.5 cell". 아래 시험은
//   (1) 경계 너머 ~1e-6 cell, heading 다름 → pose 1개 + goal heading
//   (2) 다른 칸 0.6 cell → shortcut 아님 (0.5 cell 보다 넓히지 않았음)
//   (3) 원래 upstream 같은 칸 경우 (거리 0.5 이상이어도 floor 같으면 shortcut)
//   (4) sim net_grid34 costmap 기하에서 예전 MISS 노드 10곳 실제 연쇄 재현
// ===================================================================================
namespace
{
geometry_msgs::msg::PoseStamped q40Pose(double x, double y, double yaw)
{
  geometry_msgs::msg::PoseStamped p;
  p.header.frame_id = "map";
  p.pose.position.x = x;
  p.pose.position.y = y;
  p.pose.orientation.z = std::sin(yaw / 2.0);
  p.pose.orientation.w = std::cos(yaw / 2.0);
  return p;
}

struct Q40Rig
{
  rclcpp_lifecycle::LifecycleNode::SharedPtr node;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros;
  std::unique_ptr<ammr_smac_hybrid_planner::SmacPlannerHybrid> planner;

  explicit Q40Rig(bool field_map)
  {
    node = std::make_shared<rclcpp_lifecycle::LifecycleNode>("SmacQ40Test");
    // 현장 GridBased / StaticGridBased 값 (nav2_params.yaml)
    node->declare_parameter("test.tolerance", 0.01);
    node->declare_parameter("test.minimum_turning_radius", 0.3);
    node->declare_parameter("test.angle_quantization_bins", 144);
    node->declare_parameter("test.motion_model_for_search", std::string("DUBIN"));
    costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");
    costmap_ros->on_configure(rclcpp_lifecycle::State());
    if (field_map) {
      // sim net_grid34 global costmap 기하 ("Resizing costmap to 382 X 243 at 0.050000 m/pix")
      costmap_ros->getCostmap()->resizeMap(382, 243, 0.05, -20.05, -6.05);
    }
    planner = std::make_unique<ammr_smac_hybrid_planner::SmacPlannerHybrid>();
    planner->configure(node, "test", nullptr, costmap_ros);
    planner->activate();
  }

  ~Q40Rig()
  {
    planner->deactivate();
    planner->cleanup();
    planner.reset();
    costmap_ros->on_cleanup(rclcpp_lifecycle::State());
    costmap_ros.reset();
    node.reset();
  }

  // 기존(floor) 조건 결과와 map 좌표 거리
  bool floorSame(
    const geometry_msgs::msg::PoseStamped & s, const geometry_msgs::msg::PoseStamped & g,
    float & dist)
  {
    auto * cm = costmap_ros->getCostmap();
    float mxs, mys, mxg, myg;
    EXPECT_TRUE(cm->worldToMapContinuous(s.pose.position.x, s.pose.position.y, mxs, mys));
    EXPECT_TRUE(cm->worldToMapContinuous(g.pose.position.x, g.pose.position.y, mxg, myg));
    dist = std::hypot(mxs - mxg, mys - myg);
    return std::floor(mxs) == std::floor(mxg) && std::floor(mys) == std::floor(myg);
  }

  // shortcut 결과인가: pose 1개, start 위치, goal heading
  static bool isShortcut(
    const nav_msgs::msg::Path & plan, const geometry_msgs::msg::PoseStamped & s,
    const geometry_msgs::msg::PoseStamped & g)
  {
    return plan.poses.size() == 1u &&
           plan.poses[0].pose.position.x == s.pose.position.x &&
           plan.poses[0].pose.position.y == s.pose.position.y &&
           plan.poses[0].pose.orientation.z == g.pose.orientation.z &&
           plan.poses[0].pose.orientation.w == g.pose.orientation.w;
  }
};

auto q40_no_cancel = []() {return false;};
}  // namespace

// (1) 경계 너머 ~1e-6 cell, 같은 위치·다른 heading → shortcut
TEST(SmacTest, test_q40_boundary_same_spot_shortcut)
{
  Q40Rig rig(false);
  auto * cm = rig.costmap_ros->getCostmap();
  ASSERT_GT(cm->getSizeInCellsX(), 25u);
  ASSERT_GT(cm->getSizeInCellsY(), 40u);
  const double res = cm->getResolution(), ox = cm->getOriginX(), oy = cm->getOriginY();
  // goal 은 cell 경계 (mx=20, my=33), start 는 y 로 4e-6 cell 아래 (float 32.999996, 실측값)
  auto goal = q40Pose(ox + 20.0 * res, oy + 33.0 * res, M_PI_2);
  auto start = q40Pose(ox + 20.0 * res, oy + (33.0 - 4e-6) * res, 0.0);
  float dist;
  ASSERT_FALSE(rig.floorSame(start, goal, dist)) << "전제: 기존 floor 조건이 빗나가야 한다";
  ASSERT_LT(dist, 1e-4f);
  auto plan = rig.planner->createPlan(start, goal, q40_no_cancel);
  ASSERT_EQ(plan.poses.size(), 1u);
  EXPECT_TRUE(Q40Rig::isShortcut(plan, start, goal));

  // x 쪽 경계도 같은지 (start mx = 20 - 4e-6 → 19.999996)
  auto start_x = q40Pose(ox + (20.0 - 4e-6) * res, oy + 33.0 * res, M_PI);
  ASSERT_FALSE(rig.floorSame(start_x, goal, dist));
  plan = rig.planner->createPlan(start_x, goal, q40_no_cancel);
  EXPECT_TRUE(Q40Rig::isShortcut(plan, start_x, goal));

  // 0.49 cell 경계 너머 (새 범위 안쪽 끝) → shortcut
  auto start_49 = q40Pose(ox + (20.0 - 0.49) * res, oy + 33.0 * res, 0.0);
  ASSERT_FALSE(rig.floorSame(start_49, goal, dist));
  ASSERT_LT(dist, 0.5f);
  plan = rig.planner->createPlan(start_49, goal, q40_no_cancel);
  EXPECT_TRUE(Q40Rig::isShortcut(plan, start_49, goal));
}

// (2) 다른 칸 0.6 cell → shortcut 아님 (범위를 0.5 cell 보다 넓히지 않았다)
TEST(SmacTest, test_q40_other_cell_not_shortcut)
{
  Q40Rig rig(false);
  auto * cm = rig.costmap_ros->getCostmap();
  const double res = cm->getResolution(), ox = cm->getOriginX(), oy = cm->getOriginY();
  auto goal = q40Pose(ox + 20.0 * res, oy + 33.0 * res, 0.0);
  const std::vector<std::pair<double, double>> offsets = {
    {-0.6, 0.0},    // x 로 0.6 cell
    {-0.4, -0.4},   // 대각 0.566 cell
  };
  for (const auto & o : offsets) {
    auto start = q40Pose(
      ox + (20.0 + o.first) * res, oy + (33.0 + o.second) * res, 0.0);
    float dist;
    ASSERT_FALSE(rig.floorSame(start, goal, dist));
    ASSERT_GE(dist, 0.5f);
    bool planned = false;
    nav_msgs::msg::Path plan;
    try {
      plan = rig.planner->createPlan(start, goal, q40_no_cancel);
      planned = true;
    } catch (const nav2_core::PlannerException &) {
      // A* 로 넘어가 실패해도 shortcut 이 아니라는 뜻이므로 통과
    }
    if (planned) {
      EXPECT_FALSE(Q40Rig::isShortcut(plan, start, goal));
      EXPECT_GT(plan.poses.size(), 1u) << "offset " << o.first << "," << o.second;
    }
  }
}

// (3) 원래 upstream 같은 칸 경우: floor 같으면 거리 0.5 이상이어도 shortcut
TEST(SmacTest, test_q40_original_same_cell_kept)
{
  Q40Rig rig(false);
  auto * cm = rig.costmap_ros->getCostmap();
  const double res = cm->getResolution(), ox = cm->getOriginX(), oy = cm->getOriginY();
  auto start = q40Pose(ox + 20.1 * res, oy + 10.1 * res, 0.0);
  auto goal = q40Pose(ox + 20.9 * res, oy + 10.9 * res, M_PI_2);
  float dist;
  ASSERT_TRUE(rig.floorSame(start, goal, dist));
  ASSERT_GE(dist, 0.5f);
  auto plan = rig.planner->createPlan(start, goal, q40_no_cancel);
  EXPECT_TRUE(Q40Rig::isShortcut(plan, start, goal));
}

// (4) sim net_grid34 기하: 예전 MISS 노드 10곳. start = 앞 구간 path 끝 (getWorldCoords)
TEST(SmacTest, test_q40_field_geometry_prev_miss_nodes)
{
  Q40Rig rig(true);
  auto * cm = rig.costmap_ros->getCostmap();
  struct Wp {const char * name; double x, y, yaw_deg;};
  const std::vector<Wp> wps = {
    {"r3_H3E", -16.90, -4.40, 0}, {"r3_H3E2", -6.90, -4.40, 0},
    {"r3_H3E_J", -18.40, -4.40, -90}, {"r3_V1S2", -18.40, -3.40, -90},
    {"r3_V4N2", -2.60, -3.40, 90}, {"r3_V4N2_J", -2.60, -4.40, 0},
    {"r3_V4N2_S", -2.60, -3.40, 90}, {"r4_H3E4", -9.90, -4.40, 0},
    {"r4_H3E4_J", -13.40, -4.40, -90}, {"r4_V3N4_J", -7.60, -4.40, 0},
  };
  int shortcut = 0;
  for (const auto & w : wps) {
    float mxg, myg;
    ASSERT_TRUE(cm->worldToMapContinuous(w.x, w.y, mxg, myg));
    // 앞 구간 path 끝 = getWorldCoords(goal map 좌표) (smac_planner_hybrid.cpp 의 path 변환)
    geometry_msgs::msg::Pose end = ammr_smac_hybrid_planner::getWorldCoords(mxg, myg, cm);
    const double yaw_in = w.yaw_deg * M_PI / 180.0;
    auto start = q40Pose(end.position.x, end.position.y, yaw_in);
    auto goal = q40Pose(w.x, w.y, yaw_in + M_PI_2);  // 같은 위치, heading 90 도 다름
    float dist;
    ASSERT_FALSE(rig.floorSame(start, goal, dist)) << w.name << " 전제: 예전 MISS";
    auto plan = rig.planner->createPlan(start, goal, q40_no_cancel);
    EXPECT_TRUE(Q40Rig::isShortcut(plan, start, goal)) << w.name;
    shortcut += Q40Rig::isShortcut(plan, start, goal) ? 1 : 0;
  }
  EXPECT_EQ(shortcut, 10);
}
