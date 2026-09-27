// [sim 실험군, 2026-09-20] GetYieldGoalAction
//
// 조정 계층(fleet_decision v2) 이 정한 **양보 후퇴 목표 자세**를 토픽에서 받아 블랙보드로 준다.
// 이걸로 BT 는 직선 BackUp 대신 recovery maneuver 와 같은 재료(ReverseReedsShepp 플래너 +
// ReverseRPP 컨트롤러) 로 곡선 후진 경로를 만들어 따라갈 수 있다 (사용자 지적 09-20).
//
// 동기 노드다. ReactiveFallback/PipelineSequence 안에서 RUNNING 을 만들지 않는다.
//  - 메시지가 없거나 timeout_sec 보다 오래됐으면 FAILURE (→ 평소 경로로 주행)
//  - 있으면 goal 포트에 넣고 SUCCESS
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "rclcpp/rclcpp.hpp"

namespace yield_bt_nodes
{

// [V2.36d 09-26 사용자] QoS 원칙: BT 가 구독하는 **1회성 트리거** 토픽은 TRANSIENT_LOCAL(유실 금지 + 초기화 설계),
// **주기적으로 계속 올라오는 하트비트·상태** 토픽은 VOLATILE 로 충분하다(다음 주기에 다시 온다, 잔여값·초기화 문제 없음).
// 이 파일의 두 노드는 조정 계층의 **2 Hz 하트비트**(/request_yield, /yield_goal)를 읽으므로 VOLATILE 로 구독한다.
//   - 발행 쪽도 반드시 VOLATILE 이어야 한다. (참고: VOLATILE 발행 → TRANSIENT_LOCAL 구독은 전달되지 않는다.
//     09-20 ~ 09-26 에 BT 가 후퇴 요청을 한 번도 못 받은 원인이 이 불일치였다.)
//   - 신선도는 DDS 발행 시각(source_timestamp, 발행자 벽시계)으로 잰다. 하트비트가 끊기면 저절로 무효가 되고,
//     sim 시각(use_sim_time)과 섞이지 않는다. 같은 PC 의 발행자·구독자를 전제로 skew 0.5 s 허용.
inline rclcpp::QoS heartbeat_qos()
{
  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.reliable().durability_volatile();
  return qos;
}

inline int64_t wall_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

// 발행 시각(없으면 수신 시각)으로 잰 나이 [s]
inline double sample_age(int64_t src_ns, int64_t rx_ns)
{
  const int64_t t = src_ns > 0 ? src_ns : rx_ns;
  return static_cast<double>(wall_ns() - t) * 1e-9;
}

constexpr double kClockSkewSec = 0.5;

class GetYieldGoalAction : public BT::SyncActionNode
{
public:
  GetYieldGoalAction(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
    if (!getInput("node", node_)) {
      throw BT::RuntimeError("[GetYieldGoalAction] Missing required input [node]");
    }
    getInput("topic_name", topic_name_);
    getInput("timeout_sec", timeout_sec_);

    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = callback_group_;
    // [V2.36d] 하트비트 → VOLATILE 구독 + 발행 시각으로 신선도 판정 (위 공통 설명).
    sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
      topic_name_, heartbeat_qos(),
      [this](std::shared_ptr<const geometry_msgs::msg::PoseStamped> msg, const rclcpp::MessageInfo & info) {
        last_goal_ = *msg;
        src_ns_ = info.get_rmw_message_info().source_timestamp;
        rx_ns_ = wall_ns();
        have_goal_ = true;
      },
      sub_options);
    callback_group_executor_.spin_some(std::chrono::seconds(0));   // Jazzy warm-up spin
    RCLCPP_INFO(node_->get_logger(), "[GetYieldGoalAction] subscribed: %s", topic_name_.c_str());
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<rclcpp::Node::SharedPtr>("node", "Shared ROS2 node"),
      BT::InputPort<std::string>("topic_name", "/yield_goal", "양보 후퇴 목표 토픽"),
      BT::InputPort<double>("timeout_sec", 1.0, "발행 시각 기준으로 이보다 오래된 목표는 무시(하트비트 0.5 s)"),
      BT::InputPort<geometry_msgs::msg::PoseStamped>("same_as",
        "주면: 최신 목표가 이것과 same_tol 넘게 다르면 FAILURE (후퇴 중 노드가 다음 후보로 바꿨음을 알린다)"),
      BT::InputPort<double>("same_tol", 0.05, "same_as 비교 허용 오차 [m]"),
      BT::OutputPort<geometry_msgs::msg::PoseStamped>("goal", "후퇴 목표 자세"),
    };
  }

  BT::NodeStatus tick() override
  {
    callback_group_executor_.spin_some();
    if (!have_goal_) {
      return BT::NodeStatus::FAILURE;
    }
    getInput("timeout_sec", timeout_sec_);
    const double age = sample_age(src_ns_, rx_ns_);
    if (timeout_sec_ > 0.0 && (age > timeout_sec_ || age < -kClockSkewSec)) {
      RCLCPP_WARN_THROTTLE(node_->get_logger(), *node_->get_clock(), 2000,
        "[GetYieldGoalAction] 목표가 발행된 지 %.1f s (> %.1f) → 무시(옛 값)", age, timeout_sec_);
      return BT::NodeStatus::FAILURE;
    }
    geometry_msgs::msg::PoseStamped ref;
    if (getInput("same_as", ref)) {
      double tol = 0.05;
      getInput("same_tol", tol);
      if (std::hypot(ref.pose.position.x - last_goal_.pose.position.x,
                     ref.pose.position.y - last_goal_.pose.position.y) > tol) {
        RCLCPP_INFO(node_->get_logger(), "[GetYieldGoalAction] 후퇴 목표가 바뀌었다 → 경로를 다시 만든다");
        return BT::NodeStatus::FAILURE;
      }
    }
    setOutput("goal", last_goal_);
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::string topic_name_{"/yield_goal"};
  double timeout_sec_{1.0};
  int64_t src_ns_{0};
  int64_t rx_ns_{0};
  bool have_goal_{false};
  geometry_msgs::msg::PoseStamped last_goal_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
};

// [V2.36b/c 09-26] YieldFlagFresh — 하트비트 플래그 조건.
// 현장 CheckFlagCondition 의 비래치 모드는 메시지 하나를 tick 한 번에 **소비**한다 → 2 Hz 하트비트 사이의 tick 은
// 전부 FAILURE 라 ReactiveSequence(플래그, FollowPath) 가 곧바로 FollowPath 를 끊는다(구성요소 시험에서 드러남).
// 여기서는 마지막 값이 true 이고 **발행 시각**이 max_age_sec 안이면 SUCCESS 다(읽어도 지우지 않는다).
// 하트비트 토픽이라 VOLATILE 로 구독한다. 노드가 끝낼 때 보낸 false 는 즉시 FAILURE,
// 하트비트가 끊기면(노드가 죽었거나 false 를 놓쳤어도) max_age_sec 뒤 FAILURE.
class YieldFlagFresh : public BT::ConditionNode
{
public:
  YieldFlagFresh(const std::string & name, const BT::NodeConfiguration & config)
  : BT::ConditionNode(name, config)
  {
    if (!getInput("node", node_)) {
      throw BT::RuntimeError("[YieldFlagFresh] Missing required input [node]");
    }
    getInput("flag_topic", topic_);
    getInput("max_age_sec", max_age_sec_);
    callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
    callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = callback_group_;
    sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      topic_, heartbeat_qos(),
      [this](std::shared_ptr<const std_msgs::msg::Bool> msg, const rclcpp::MessageInfo & info) {
        value_ = msg->data;
        src_ns_ = info.get_rmw_message_info().source_timestamp;
        rx_ns_ = wall_ns();
        have_ = true;
      },
      sub_options);
    callback_group_executor_.spin_some(std::chrono::seconds(0));   // Jazzy warm-up spin
    RCLCPP_INFO(node_->get_logger(), "[YieldFlagFresh] subscribed: %s (max_age %.1fs)", topic_.c_str(), max_age_sec_);
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<rclcpp::Node::SharedPtr>("node", "Shared ROS2 node"),
      BT::InputPort<std::string>("flag_topic", "/request_yield", "하트비트 플래그 토픽"),
      BT::InputPort<double>("max_age_sec", 1.5, "이보다 오래된 true 는 무효"),
    };
  }

  BT::NodeStatus tick() override
  {
    callback_group_executor_.spin_some();
    if (!have_ || !value_) {
      return BT::NodeStatus::FAILURE;
    }
    const double age = sample_age(src_ns_, rx_ns_);
    return (age <= max_age_sec_ && age >= -kClockSkewSec) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::string topic_{"/request_yield"};
  double max_age_sec_{1.5};
  bool have_{false};
  bool value_{false};
  int64_t src_ns_{0};
  int64_t rx_ns_{0};
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr sub_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
};

}  // namespace yield_bt_nodes

extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<yield_bt_nodes::GetYieldGoalAction>("GetYieldGoalAction");
  factory.registerNodeType<yield_bt_nodes::YieldFlagFresh>("YieldFlagFresh");
}
