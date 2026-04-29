/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2020 Shivang Patel
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Shivang Patel
 *
 * Reference tutorial:
 * https://navigation.ros.org/tutorials/docs/writing_new_nav2planner_plugin.html
 *********************************************************************/

#include <cmath>
#include <string>
#include <memory>
#include "nav2_util/node_utils.hpp"

#include "ammr_straightline_planner/straight_line_planner.hpp"

namespace ammr_straightline_planner
{

void StraightLine::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;


  costmap_ros_ = costmap_ros; // [중요] 멤버 변수에 저장, costmap_ros 포인터 저장
  costmap_ = costmap_ros->getCostmap();
  // costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();
  

  // [추가] Collision Checker 초기화
  collision_checker_ = std::make_unique<nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *>>(costmap_);







  // Parameter initialization
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(
      0.1));
  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);
}

void StraightLine::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
    name_.c_str());
}

void StraightLine::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
    name_.c_str());
}

void StraightLine::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
    name_.c_str());
}

nav_msgs::msg::Path StraightLine::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal,
  std::function<bool()> /*cancel_checker*/)
{
  nav_msgs::msg::Path global_path;

  // Checking if the goal and start state is in the global frame
  if (start.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      node_->get_logger(), "Planner will only except start position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  if (goal.header.frame_id != global_frame_) {
    RCLCPP_INFO(
      node_->get_logger(), "Planner will only except goal position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  global_path.poses.clear();
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;


// 현재 로봇의 Footprint 가져오기 (매번 갱신된 footprint를 가져오는 것이 안전함)
  std::vector<geometry_msgs::msg::Point> footprint = costmap_ros_->getRobotFootprint();

  double dist = std::hypot(
    goal.pose.position.x - start.pose.position.x,
    goal.pose.position.y - start.pose.position.y);



  // calculating the number of loops for current value of interpolation_resolution_
  int total_number_of_loop = std::hypot(
    goal.pose.position.x - start.pose.position.x,
    goal.pose.position.y - start.pose.position.y) /
    interpolation_resolution_;
  double x_increment = (goal.pose.position.x - start.pose.position.x) / total_number_of_loop;
  double y_increment = (goal.pose.position.y - start.pose.position.y) / total_number_of_loop;


// 방향(Orientation) 계산 (직선 주행이므로 start->goal 방향으로 고정하거나, start의 방향 유지)
  // 여기서는 간단하게 Goal을 바라보는 방향으로 설정
  double yaw = std::atan2(
    goal.pose.position.y - start.pose.position.y,
    goal.pose.position.x - start.pose.position.x);



  for (int i = 0; i < total_number_of_loop; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = start.pose.position.x + x_increment * i;
    pose.pose.position.y = start.pose.position.y + y_increment * i;
    pose.pose.position.z = 0.0;


  //   pose.pose.orientation.x = 0.0;
  //   pose.pose.orientation.y = 0.0;
  //   pose.pose.orientation.z = 0.0;
  //   pose.pose.orientation.w = 1.0;
  //   pose.header.stamp = node_->now();
  //   pose.header.frame_id = global_frame_;
  //   global_path.poses.push_back(pose);
  // }

// Quaternion 변환 (tf2 사용)
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    
    pose.header.stamp = node_->now();
    pose.header.frame_id = global_frame_;

    // *** 핵심: 충돌 검사 로직 추가 ***
    // footprintCost는 해당 위치에 로봇을 놓았을 때의 Cost를 반환합니다.
    // Cost 값: 0~253 (Free~Inscribed), 254 (Lethal), 255 (Unknown)
    // 우리는 좁은 곳을 지나가야 하므로, 254(LETHAL_OBSTACLE)만 아니면 통과시킵니다.
    // 만약 더 안전하게 하려면 253(INSCRIBED_INFLATED_OBSTACLE)도 체크할 수 있지만,
    // 협소 구간 통과가 목적이므로 LETHAL만 체크하는 것이 맞습니다.
    
    double footprint_cost = collision_checker_->footprintCostAtPose(pose.pose.position.x, pose.pose.position.y, yaw, footprint);

    if (footprint_cost >= nav2_costmap_2d::LETHAL_OBSTACLE && footprint_cost != nav2_costmap_2d::NO_INFORMATION) {
        RCLCPP_WARN(node_->get_logger(), "Path is blocked by obstacle at index %d", i);
        // 충돌이 감지되면 빈 경로를 반환하여 Planning 실패를 알림 (또는 가능한 곳까지만 반환)
        return nav_msgs::msg::Path(); 
    }

    global_path.poses.push_back(pose);
  }

  // Goal 포즈 추가 및 최종 검사
  double goal_footprint_cost = collision_checker_->footprintCostAtPose(goal.pose.position.x, goal.pose.position.y, yaw, footprint);
  if (goal_footprint_cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
      RCLCPP_WARN(node_->get_logger(), "Goal is in collision");
      return nav_msgs::msg::Path();
  }



  geometry_msgs::msg::PoseStamped goal_pose = goal;
  goal_pose.header.stamp = node_->now();
  goal_pose.header.frame_id = global_frame_;
  global_path.poses.push_back(goal_pose);

  return global_path;
}

}  // namespace ammr_straightline_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ammr_straightline_planner::StraightLine, nav2_core::GlobalPlanner)
