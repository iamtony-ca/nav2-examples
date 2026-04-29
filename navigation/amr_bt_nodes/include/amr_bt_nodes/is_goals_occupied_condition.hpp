#ifndef AMR_BT_NODES__IS_GOALS_OCCUPIED_CONDITION_HPP_
#define AMR_BT_NODES__IS_GOALS_OCCUPIED_CONDITION_HPP_

#include <string>
#include <vector>
#include <memory>
#include <mutex>

#include "rclcpp/node.hpp"
#include "rclcpp/executors.hpp"
#include "behaviortree_cpp/condition_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "tf2_ros/buffer.h"

namespace amr_bt_nodes
{

class IsGoalsOccupiedCondition : public BT::ConditionNode
{
public:
  IsGoalsOccupiedCondition(
    const std::string & condition_name,
    const BT::NodeConfiguration & conf);

  static BT::PortsList providedPorts();

protected:
  BT::NodeStatus tick() override;

private:
  void costmapCallback(const nav2_msgs::msg::Costmap::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  rclcpp::Subscription<nav2_msgs::msg::Costmap>::SharedPtr costmap_sub_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  std::shared_ptr<nav2_costmap_2d::Costmap2D> latest_costmap_;
  std::string costmap_frame_id_;
  std::mutex costmap_mutex_;
};

}  // namespace amr_bt_nodes

#endif  // AMR_BT_NODES__IS_GOALS_OCCUPIED_CONDITION_HPP_