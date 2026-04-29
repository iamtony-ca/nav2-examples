// check_pause_reset_condition.cpp

#include "amr_bt_nodes/check_pause_reset_condition.hpp"

namespace amr_bt_nodes
{

CheckPauseResetCondition::CheckPauseResetCondition(const std::string& name,
                                       const BT::NodeConfiguration& config)
: BT::ConditionNode(name, config)
{
  // 필수 입력: node
  if (!getInput("node", node_)) {
    throw BT::RuntimeError("[CheckPauseResetCondition] Missing required input [node]");
  }

  // 선택 입력들
  getInput("flag_topic", flag_topic_);
  getInput("latch", latch_);
  getInput("transient_local", transient_local_);

  // 콜백 그룹 & 전용 executor
  callback_group_ = node_->create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  // 구독 옵션 및 QoS
  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = callback_group_;

  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local().reliable();
  if (!transient_local_) {
    qos.reliable(); // DDS latched QoS
  }

  // flag 구독
  flag_sub_ = node_->create_subscription<std_msgs::msg::Bool>(
      flag_topic_, qos,
      std::bind(&CheckPauseResetCondition::flagCallback, this, std::placeholders::_1),
      sub_options);

  // 초기 latched 샘플 처리
  callback_group_executor_.spin_some();

  RCLCPP_INFO(node_->get_logger(),
              "[CheckPauseResetCondition] Subscribed to topic: %s", flag_topic_.c_str());
}

void CheckPauseResetCondition::flagCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  last_flag_.store(msg->data, std::memory_order_relaxed);
  RCLCPP_DEBUG(node_->get_logger(), "[CheckPauseResetCondition] flag=%s",
               msg->data ? "true" : "false");
}

BT::NodeStatus CheckPauseResetCondition::tick()
{
  // 콜백 처리
  callback_group_executor_.spin_some();



    // tick() 안
    bool reset_in = false;
    getInput("reset", reset_in);
    if (reset_in) { 
    // if (reset_in && !prev_reset_) {                // false -> true 상승 에지에서만
    last_flag_.store(false, std::memory_order_relaxed);
    return BT::NodeStatus::FAILURE;
    }
    // prev_reset_ = reset_in;


//   // --- 블랙보드 리셋 트리거 처리 (level-triggered) ---
//   bool do_reset = false;
//   if (getInput("reset", do_reset) && do_reset) {
//     // latch 여부와 무관하게 내부 latched 상태를 즉시 초기화
//     last_flag_.store(false, std::memory_order_relaxed);
//   }

  // --- 논리적 latch 동작 ---
  if (latch_) {
    // true가 유지되는 동안 SUCCESS
    if (last_flag_.load(std::memory_order_relaxed)) {
      return BT::NodeStatus::SUCCESS;
    }
  } else {
    // 1회성 이벤트 소비: true를 한번 보고 바로 false로 소모
    if (last_flag_.exchange(false, std::memory_order_acq_rel)) {
      return BT::NodeStatus::SUCCESS;
    }
  }

  return BT::NodeStatus::FAILURE;
}

} // namespace amr_bt_nodes



#include "behaviortree_cpp/bt_factory.h"

extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory &factory)
{
  factory.registerNodeType<amr_bt_nodes::CheckPauseResetCondition>("CheckPauseResetCondition");
}






