// [V2.36d 09-26] yield_bt_nodes 단위시험 — 조정 계층(fleet_decision)과 같은 QoS 로 쏠 때
// YieldFlagFresh / GetYieldGoalAction 이 여러 상황(시작·재시작·죽음·정상 종료·늦은 합류·목표 교체)에서 의도대로 동작하는가.
// QoS 원칙(사용자): 1회성 트리거는 TRANSIENT_LOCAL, 주기 하트비트는 VOLATILE. 두 노드는 2 Hz 하트비트를 읽으므로 VOLATILE.
// 통합 판 전에 돌린다.
//   colcon build --packages-select yield_bt_nodes && colcon test --packages-select yield_bt_nodes
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>

#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

using namespace std::chrono_literals;
using BT::NodeStatus;

namespace
{
rclcpp::QoS latched()
{
  rclcpp::QoS q(rclcpp::KeepLast(1));
  q.reliable().transient_local();
  return q;
}
rclcpp::QoS volatile_qos()
{
  rclcpp::QoS q(rclcpp::KeepLast(1));
  q.reliable().durability_volatile();
  return q;
}
std::atomic<int> g_topic_seq{0};
}  // namespace

class YieldNodes : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    // 시험마다 토픽 이름을 달리해 앞 시험의 잔여값(TRANSIENT_LOCAL)이 섞이지 않게 한다
    const int k = g_topic_seq++;
    flag_topic_ = "/request_yield_t" + std::to_string(k);
    goal_topic_ = "/yield_goal_t" + std::to_string(k);
    node_ = std::make_shared<rclcpp::Node>("yield_bt_test_" + std::to_string(k));
    factory_.registerFromPlugin(PLUGIN_PATH);
  }

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr flag_pub(const rclcpp::QoS & q = volatile_qos())
  {
    return node_->create_publisher<std_msgs::msg::Bool>(flag_topic_, q);
  }

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub()
  {
    return node_->create_publisher<geometry_msgs::msg::PoseStamped>(goal_topic_, volatile_qos());
  }

  // BT 쪽 구독은 트리를 만들 때 생긴다 → 트리를 늦게 만들면 '늦게 붙는 구독자' 상황이다
  BT::Tree flag_tree()
  {
    auto bb = BT::Blackboard::create();
    bb->set<rclcpp::Node::SharedPtr>("node", node_);
    auto t = factory_.createTreeFromText(
      "<root BTCPP_format=\"4\"><BehaviorTree ID=\"T\"><YieldFlagFresh flag_topic=\"" + flag_topic_ +
      "\" node=\"{node}\" max_age_sec=\"1.5\"/></BehaviorTree></root>", bb);
    std::this_thread::sleep_for(400ms);          // 발견·잔여값 전달 대기
    return t;
  }

  BT::Tree goal_tree(BT::Blackboard::Ptr & bb, bool with_same_as)
  {
    bb = BT::Blackboard::create();
    bb->set<rclcpp::Node::SharedPtr>("node", node_);
    const std::string same = with_same_as ? " same_as=\"{ref}\"" : "";
    auto t = factory_.createTreeFromText(
      "<root BTCPP_format=\"4\"><BehaviorTree ID=\"G\"><GetYieldGoalAction topic_name=\"" + goal_topic_ +
      "\" node=\"{node}\" timeout_sec=\"1.0\" goal=\"{g}\"" + same + "/></BehaviorTree></root>", bb);
    std::this_thread::sleep_for(400ms);
    return t;
  }

  static void pub(const rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr & p, bool v)
  {
    std_msgs::msg::Bool m;
    m.data = v;
    p->publish(m);
    std::this_thread::sleep_for(100ms);
  }

  geometry_msgs::msg::PoseStamped pose(double x)
  {
    geometry_msgs::msg::PoseStamped ps;
    ps.header.frame_id = "map";
    ps.header.stamp = node_->now();
    ps.pose.position.x = x;
    ps.pose.orientation.w = 1.0;
    return ps;
  }

  std::string flag_topic_, goal_topic_;
  rclcpp::Node::SharedPtr node_;
  BT::BehaviorTreeFactory factory_;
};

// ---------------- 기본 동작 ----------------
TEST_F(YieldNodes, FlagFailsWithoutMessage)
{
  auto t = flag_tree();
  EXPECT_EQ(t.tickOnce(), NodeStatus::FAILURE);
}

TEST_F(YieldNodes, FlagTrueIsNotConsumedByTicks)
{
  auto p = flag_pub();
  auto t = flag_tree();
  pub(p, true);
  for (int i = 0; i < 10; ++i) {   // 현장 CheckFlagCondition(비래치)은 두 번째 tick 부터 FAILURE 였다
    EXPECT_EQ(t.tickOnce(), NodeStatus::SUCCESS) << "tick " << i;
  }
}

TEST_F(YieldNodes, FlagHeartbeatKeepsAlive)
{
  auto p = flag_pub();
  auto t = flag_tree();
  for (int i = 0; i < 8; ++i) {    // 2 Hz 하트비트 4 s
    pub(p, true);
    std::this_thread::sleep_for(400ms);
    EXPECT_EQ(t.tickOnce(), NodeStatus::SUCCESS) << "beat " << i;
  }
}

TEST_F(YieldNodes, FlagExpiresWhenHeartbeatStops)
{
  auto p = flag_pub();
  auto t = flag_tree();
  pub(p, true);
  std::this_thread::sleep_for(1000ms);
  EXPECT_EQ(t.tickOnce(), NodeStatus::SUCCESS) << "발행 1.1 s 뒤: 아직 유효";
  std::this_thread::sleep_for(600ms);
  EXPECT_EQ(t.tickOnce(), NodeStatus::FAILURE) << "발행 1.7 s 뒤: 만료";
}

TEST_F(YieldNodes, FlagFalseFailsImmediately)
{
  auto p = flag_pub();
  auto t = flag_tree();
  pub(p, true);
  EXPECT_EQ(t.tickOnce(), NodeStatus::SUCCESS);
  pub(p, false);
  EXPECT_EQ(t.tickOnce(), NodeStatus::FAILURE);
}

// ---------------- 시작·재시작·종료 상황 ----------------
TEST_F(YieldNodes, Init_NoReplayToLateJoiner)
{
  // 노드가 true 를 보낸 뒤 멈췄고(하트비트 끊김), 그 뒤 BT 가 새로 뜬다 → VOLATILE 이라 옛 true 가 재전달되지 않는다
  // (09-20 '옛 true 재전달' 사고가 구조적으로 생기지 않는다)
  auto p = flag_pub();
  pub(p, true);
  std::this_thread::sleep_for(500ms);
  auto t = flag_tree();
  EXPECT_EQ(t.tickOnce(), NodeStatus::FAILURE);
}

TEST_F(YieldNodes, Init_LateJoinDuringLiveHeartbeat)
{
  // 후퇴 진행 중(하트비트 살아 있음)에 BT 가 새로 뜬다 → 다음 하트비트(0.5 s 안)부터 SUCCESS
  auto p = flag_pub();
  std::atomic<bool> run{true};
  std::thread beat([&]() {
      while (run) {
        std_msgs::msg::Bool m;
        m.data = true;
        p->publish(m);
        std::this_thread::sleep_for(500ms);
      }
    });
  std::this_thread::sleep_for(1200ms);
  auto t = flag_tree();                      // 생성 + 0.4 s 대기
  std::this_thread::sleep_for(300ms);        // 늦게 붙은 뒤 최소 한 번의 하트비트
  EXPECT_EQ(t.tickOnce(), NodeStatus::SUCCESS);
  run = false;
  beat.join();
}

TEST_F(YieldNodes, Init_PublisherGoneMidYield)
{
  // 노드가 후퇴 중에 죽는다(false 도 못 보냄) → BT 가 들고 있던 true 는 1.5 s 뒤 만료
  auto t = flag_tree();
  {
    auto p = flag_pub();
    std::this_thread::sleep_for(300ms);
    pub(p, true);
    EXPECT_EQ(t.tickOnce(), NodeStatus::SUCCESS);
  }   // 발행자 소멸
  std::this_thread::sleep_for(1700ms);
  EXPECT_EQ(t.tickOnce(), NodeStatus::FAILURE) << "하트비트가 끊기면 만료";
}

TEST_F(YieldNodes, Init_FinishFalseLostStillExpires)
{
  // 종료 false 가 유실돼도(하트비트라 허용) 1.5 s 뒤 만료된다
  auto p = flag_pub();
  auto t = flag_tree();
  pub(p, true);
  EXPECT_EQ(t.tickOnce(), NodeStatus::SUCCESS);
  std::this_thread::sleep_for(1700ms);       // false 를 보내지 않음
  EXPECT_EQ(t.tickOnce(), NodeStatus::FAILURE);
}

TEST_F(YieldNodes, Guard_HeartbeatQosMatchesFleetDecision)
{
  // 방침 확인: 조정 계층은 하트비트를 VOLATILE 로 쏜다 → 받아야 한다.
  // TRANSIENT_LOCAL 발행도 VOLATILE 구독은 받는다(호환). 이 시험이 실패하면 QoS 전제가 바뀐 것이다.
  auto pv = flag_pub(volatile_qos());
  auto t = flag_tree();
  pub(pv, true);
  EXPECT_EQ(t.tickOnce(), NodeStatus::SUCCESS) << "VOLATILE 발행";
  pub(pv, false);
  auto pl = flag_pub(latched());
  std::this_thread::sleep_for(300ms);
  pub(pl, true);
  EXPECT_EQ(t.tickOnce(), NodeStatus::SUCCESS) << "TRANSIENT_LOCAL 발행도 호환";
}

// ---------------- 후퇴 목표 ----------------
TEST_F(YieldNodes, GoalFreshIsReceived)
{
  auto p = goal_pub();
  BT::Blackboard::Ptr bb;
  auto t = goal_tree(bb, false);
  EXPECT_EQ(t.tickOnce(), NodeStatus::FAILURE) << "목표 없음";
  p->publish(pose(-1.5));
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(t.tickOnce(), NodeStatus::SUCCESS);
  EXPECT_DOUBLE_EQ(bb->get<geometry_msgs::msg::PoseStamped>("g").pose.position.x, -1.5);
}

TEST_F(YieldNodes, GoalExpiresWhenHeartbeatStops)
{
  // 이전 후퇴의 목표 → 하트비트가 끊긴 지 1.0 s 넘으면 쓰지 않는다(새 요청이 옛 목표로 출발하지 않게)
  auto p = goal_pub();
  BT::Blackboard::Ptr bb;
  auto t = goal_tree(bb, false);
  p->publish(pose(-2.0));
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(t.tickOnce(), NodeStatus::SUCCESS);
  std::this_thread::sleep_for(1200ms);
  EXPECT_EQ(t.tickOnce(), NodeStatus::FAILURE);
}

TEST_F(YieldNodes, GoalChangeIsDetected)
{
  // 후퇴 중 노드가 다음 후보로 목표를 바꾸면 same_as 비교가 FAILURE → 가지가 끝나고 새 목표로 다시 계획
  auto p = goal_pub();
  BT::Blackboard::Ptr bb;
  auto t = goal_tree(bb, true);
  bb->set<geometry_msgs::msg::PoseStamped>("ref", pose(-1.5));
  p->publish(pose(-1.5));
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(t.tickOnce(), NodeStatus::SUCCESS) << "같은 목표";
  p->publish(pose(-1.53));
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(t.tickOnce(), NodeStatus::SUCCESS) << "3 cm 흔들림은 같은 목표";
  p->publish(pose(-0.5));
  std::this_thread::sleep_for(100ms);
  EXPECT_EQ(t.tickOnce(), NodeStatus::FAILURE) << "다른 목표";
}
