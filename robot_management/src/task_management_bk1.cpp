#include "robot_management/task_management.hpp"

TaskManagement::TaskManagement() : Node("task_management")
{
  nav_to_pose_client_ = rclcpp_action::create_client<NavigateToPose>(
    this, "navigate_to_pose");

  trigger_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/nav2_trigger", 10,
    std::bind(&TaskManagement::trigger_callback, this, std::placeholders::_1));
}

void TaskManagement::trigger_callback(const std_msgs::msg::String::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "/nav2_trigger 수신: '%s'", msg->data.c_str());
  if (msg->data == "go") {
    send_goal();
  } else {
    RCLCPP_WARN(this->get_logger(), "알 수 없는 명령: '%s'", msg->data.c_str());
  }
}

void TaskManagement::send_goal()
{
  if (!nav_to_pose_client_->wait_for_action_server(std::chrono::seconds(3))) {
    RCLCPP_ERROR(this->get_logger(), "navigate_to_pose 서버 연결 실패");
    return;
  }

  NavigateToPose::Goal goal_msg;
  goal_msg.pose.header.frame_id = "map";
  goal_msg.pose.header.stamp = this->now();
  goal_msg.pose.pose.position.x = 1.0;
  goal_msg.pose.pose.position.y = 2.0;
  goal_msg.pose.pose.orientation.w = 1.0;

  auto options = rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
  options.goal_response_callback = std::bind(&TaskManagement::goal_response_callback, this, std::placeholders::_1);
  options.feedback_callback = std::bind(&TaskManagement::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
  options.result_callback = std::bind(&TaskManagement::result_callback, this, std::placeholders::_1);

  nav_to_pose_client_->async_send_goal(goal_msg, options);
  RCLCPP_INFO(this->get_logger(), "NavigateToPose goal 전송 요청");
}

void TaskManagement::goal_response_callback(GoalHandleNav::SharedPtr goal_handle)
{
  // auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "목표가 거부됨");
  } else {
    RCLCPP_INFO(this->get_logger(), "목표 수락됨");
  }
}

void TaskManagement::feedback_callback(
  GoalHandleNav::SharedPtr,
  const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
  RCLCPP_INFO(this->get_logger(), "남은 거리: %.2f m", feedback->distance_remaining);
}

void TaskManagement::result_callback(const GoalHandleNav::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "목표 도달 성공");
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "중단됨");
      break;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_WARN(this->get_logger(), "취소됨");
      break;
    default:
      RCLCPP_ERROR(this->get_logger(), "알 수 없는 결과");
      break;
  }
}
