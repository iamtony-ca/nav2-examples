#include "amr_bt_nodes/is_goals_occupied_agent_condition.hpp"

#include "nav2_util/node_utils.hpp"
#include "nav2_costmap_2d/cost_values.hpp"
#include "rclcpp/qos.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace amr_bt_nodes
{

IsGoalsOccupiedAgentCondition::IsGoalsOccupiedAgentCondition(
  const std::string & condition_name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(condition_name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  tf_buffer_ = config().blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(callback_group_, node_->get_node_base_interface());

  std::string costmap_topic;
  getInput("costmap_topic", costmap_topic);
  if (costmap_topic.empty()) {
    
    costmap_topic = "local_costmap/costmap_raw";
    RCLCPP_WARN(node_->get_logger(), "costmap_topic is empty and set as deafult : %s", costmap_topic.c_str());
  }

  rclcpp::QoS qos(rclcpp::KeepLast(1));
  qos.transient_local().reliable();

  rclcpp::SubscriptionOptions sub_option;
  sub_option.callback_group = callback_group_;

  costmap_sub_ = node_->create_subscription<nav2_msgs::msg::Costmap>(
    costmap_topic, qos,
    std::bind(&IsGoalsOccupiedAgentCondition::costmapCallback, this, std::placeholders::_1),
    sub_option);

  // // PlannerSelector 패턴 적용: 초기 latched 메시지 처리를 위함
  // callback_group_executor_.spin_some(std::chrono::seconds(0)); // timeout 0은 즉시 반환

}

BT::PortsList IsGoalsOccupiedAgentCondition::providedPorts()
{
  return {
    BT::InputPort<std::vector<geometry_msgs::msg::PoseStamped>>("goals", "Vector of goals to check"),
    BT::InputPort<int>("occupied_cost_threshold", nav2_costmap_2d::LETHAL_OBSTACLE, "Cost value to consider a goal as occupied"),
    BT::InputPort<std::string>("costmap_topic", "Topic of the costmap to subscribe to"),
    // ## 출력 포트 2개 추가 ##
    BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("occupied_goals", "List of goals that are in occupied space"),
    BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("unoccupied_goals", "List of goals that are in free space")
  };
}

BT::NodeStatus IsGoalsOccupiedAgentCondition::tick()
{
  callback_group_executor_.spin_some();

  std::shared_ptr<nav2_costmap_2d::Costmap2D> costmap;
  std::string costmap_frame;
  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    costmap = latest_costmap_;
    costmap_frame = costmap_frame_id_;
  }

  if (!costmap || costmap_frame.empty()) {
    RCLCPP_WARN(node_->get_logger(), "Costmap or its frame ID is not available yet.");
    // return BT::NodeStatus::FAILURE;
    return BT::NodeStatus::RUNNING;
  }

  std::vector<geometry_msgs::msg::PoseStamped> goals;
  if (!getInput<std::vector<geometry_msgs::msg::PoseStamped>>("goals", goals) || goals.empty()) {
    return BT::NodeStatus::FAILURE;
  }
  int threshold;
  getInput("occupied_cost_threshold", threshold);

  // ## 결과를 담을 벡터 생성 ##
  std::vector<geometry_msgs::msg::PoseStamped> occupied_goals;
  std::vector<geometry_msgs::msg::PoseStamped> unoccupied_goals;

  std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*(costmap->getMutex()));

  for (const auto & goal : goals) {
    geometry_msgs::msg::PoseStamped transformed_goal;
    try {
      tf_buffer_->transform(goal, transformed_goal, costmap_frame);
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(node_->get_logger(), "Failed to transform goal pose to %s frame: %s",
        costmap_frame.c_str(), ex.what());
      // TF 변환 실패 시 안전을 위해 점유된 것으로 간주
      occupied_goals.push_back(goal);
      continue;
    }

    unsigned int mx, my;
    bool is_occupied = false;
    if (!costmap->worldToMap(transformed_goal.pose.position.x, transformed_goal.pose.position.y, mx, my))
    {
      // Costmap 경계 밖은 비점유로 간주하고, unoccupied_goals에 추가
      unoccupied_goals.push_back(goal);
      continue;
    }

    if (costmap->getCost(mx, my) >= threshold)
    {
      is_occupied = true;
    }

    // ## 검사 결과에 따라 적절한 벡터에 추가 ##
    if (is_occupied) {
      occupied_goals.push_back(goal);
    } else {
      unoccupied_goals.push_back(goal);
    }
  }

  // ## 루프 종료 후, 분류된 결과를 블랙보드에 설정 ##
  setOutput("occupied_goals", occupied_goals);
  setOutput("unoccupied_goals", unoccupied_goals);

  if (!occupied_goals.empty()) {
    RCLCPP_INFO(node_->get_logger(), "%zu goals are occupied. %zu goals are unoccupied.",
      occupied_goals.size(), unoccupied_goals.size());
    return BT::NodeStatus::SUCCESS; // 점유된 goal이 하나라도 있으면 SUCCESS
  } else {
    RCLCPP_INFO(node_->get_logger(), "All %zu goals are unoccupied.", unoccupied_goals.size());
    return BT::NodeStatus::FAILURE; // 모든 goal이 비점유 상태이면 FAILURE
  }
}

void IsGoalsOccupiedAgentCondition::costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg)
{
  auto new_costmap = std::make_shared<nav2_costmap_2d::Costmap2D>(
    msg->metadata.size_x, msg->metadata.size_y,
    msg->metadata.resolution, msg->metadata.origin.position.x, msg->metadata.origin.position.y);

  unsigned char * char_map = new_costmap->getCharMap();
  memcpy(char_map, &msg->data[0], msg->data.size() * sizeof(unsigned char));

  {
    std::lock_guard<std::mutex> lock(costmap_mutex_);
    latest_costmap_ = new_costmap;
    costmap_frame_id_ = msg->header.frame_id;
  }
}

}  // namespace amr_bt_nodes


#include "behaviortree_cpp/bt_factory.h"

extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory & factory)
{
  factory.registerNodeType<amr_bt_nodes::IsGoalsOccupiedAgentCondition>("IsGoalsOccupiedAgentCondition");
}
