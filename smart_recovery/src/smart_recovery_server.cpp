// smart_recovery_server.cpp
// =============================================================
#include "smart_recovery/smart_recovery_server.hpp"

SmartRecoveryServer::SmartRecoveryServer(const rclcpp::NodeOptions & opts)
: Node("smart_recovery_server", opts)
{
  // --- Callback groups --------------------------------------
  cb_action_          = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  cb_service_client_  = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  cb_topic_           = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // --- Action server ----------------------------------------
  action_server_ = rclcpp_action::create_server<SmartRecovery>(
    this,
    "smart_recovery",
    std::bind(&SmartRecoveryServer::handle_goal,    this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&SmartRecoveryServer::handle_cancel,  this, std::placeholders::_1),
    std::bind(&SmartRecoveryServer::handle_accepted,this, std::placeholders::_1),
    rcl_action_server_get_default_options(),
    cb_action_);

  // --- Planner service client --------------------------------
  planner_client_ = this->create_client<SmartRecoveryPath>(
    "generate_recovery_path", rclcpp::QoS(10), cb_service_client_);

  // --- Path publisher ---------------------------------------
  path_pub_ = this->create_publisher<nav_msgs::msg::Path>("/recovery_path", 1);

  // --- Done flag subscriber ---------------------------------
  rclcpp::SubscriptionOptions sub_opts;
  sub_opts.callback_group = cb_topic_;
  done_sub_ = this->create_subscription<std_msgs::msg::Bool>(
    "/recovery_done", 1,
    std::bind(&SmartRecoveryServer::doneCallback, this, std::placeholders::_1),
    sub_opts);
}

// ----------------- Action Callbacks -------------------------

rclcpp_action::GoalResponse SmartRecoveryServer::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  const std::shared_ptr<const SmartRecovery::Goal> goal)
{
  (void)uuid;
  RCLCPP_INFO(this->get_logger(), "SmartRecovery goal received: backup=%.2f speed=%.2f",
              goal->backup_distance, goal->speed);
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SmartRecoveryServer::handle_cancel(
  const std::shared_ptr<GoalHandleRecovery> handle)
{
  (void)handle;
  cancel_requested_ = true;
  RCLCPP_WARN(this->get_logger(), "SmartRecovery cancel requested");
  return rclcpp_action::CancelResponse::ACCEPT;
}

void SmartRecoveryServer::handle_accepted(const std::shared_ptr<GoalHandleRecovery> handle)
{
  RCLCPP_INFO(this->get_logger(), "SmartRecovery handle_accepted");
  std::thread{std::bind(&SmartRecoveryServer::execute, this, handle)}.detach();
}

void SmartRecoveryServer::execute(const std::shared_ptr<GoalHandleRecovery> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing SmartRecovery action");
  auto result = std::make_shared<SmartRecovery::Result>();

  // 1) Call planner service ----------------------------------
  if (!planner_client_->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(this->get_logger(), "Planner service not available");
    result->success = false;
    result->message = "Planner unavailable";
    goal_handle->abort(result);
    return;
  }

  auto req = std::make_shared<SmartRecoveryPath::Request>();
  req->goal = goal_handle->get_goal()->goal_pose;  // assumes action goal has goal_pose

  auto future = planner_client_->async_send_request(req);
  if (future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
    RCLCPP_ERROR(this->get_logger(), "Planner timeout");
    result->success = false;
    result->message = "Planner timeout";
    goal_handle->abort(result);
    return;
  }
  auto resp = future.get();
  if (!resp->success) {
    RCLCPP_ERROR(this->get_logger(), "Planner failed: %s", resp->message.c_str());
    result->success = false;
    result->message = resp->message;
    goal_handle->abort(result);
    return;
  }

  // 2) Publish recovery path ---------------------------------
  path_pub_->publish(resp->path);

  // 3) Wait for done or cancel -------------------------------
  rclcpp::Rate r(20);
  done_flag_ = false;
  cancel_requested_ = false;
  while (rclcpp::ok() && !done_flag_) {
    if (cancel_requested_) {
      RCLCPP_WARN(this->get_logger(), "Recovery aborted by client");
      result->success = false;
      result->message = "Canceled by client";
      goal_handle->canceled(result);
      return;
    }
    r.sleep();
  }

  result->success = true;
  result->message = "Recovery complete";
  goal_handle->succeed(result);
  RCLCPP_INFO(this->get_logger(), "SmartRecovery action completed successfully");
}

void SmartRecoveryServer::doneCallback(const std_msgs::msg::Bool::SharedPtr msg)
{
  if (msg->data) done_flag_ = true;
}

// --------------------------- main ---------------------------
#include <rclcpp/executors.hpp>
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SmartRecoveryServer>();

  // Multi-thread executor (스레드 수: 하드웨어 코어 수에 맞춰 자동)
  rclcpp::executors::MultiThreadedExecutor exec;
  exec.add_node(node);
  exec.spin();

  rclcpp::shutdown();
  return 0;
}
