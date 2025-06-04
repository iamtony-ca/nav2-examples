#include "robot_management/nav2_param_manager.hpp"

Nav2ParamManager::Nav2ParamManager(const rclcpp::Node::SharedPtr & node)
: node_(node) {}

bool Nav2ParamManager::set_param(const std::string & node_name,
                                  const std::string & param_name,
                                  const rclcpp::Parameter & value)
{
  std::string service_name = node_name + "/set_parameters";
  auto client = node_->create_client<rcl_interfaces::srv::SetParameters>(service_name);
  if (!client->wait_for_service(kServiceTimeout)) {
    RCLCPP_ERROR(node_->get_logger(), "set_parameters service not available for %s", node_name.c_str());
    return false;
  }

  auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
  request->parameters.push_back(value.to_parameter_msg());

  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to call set_parameters on %s", node_name.c_str());
    return false;
  }

  return true;
}

std::optional<rclcpp::Parameter> Nav2ParamManager::get_param(const std::string & node_name,
                                                             const std::string & param_name)
{
  std::string service_name = node_name + "/get_parameters";
  auto client = node_->create_client<rcl_interfaces::srv::GetParameters>(service_name);
  if (!client->wait_for_service(kServiceTimeout)) {
    RCLCPP_ERROR(node_->get_logger(), "get_parameters service not available for %s", node_name.c_str());
    return std::nullopt;
  }

  auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
  request->names.push_back(param_name);

  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node_, future) != rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_ERROR(node_->get_logger(), "Failed to call get_parameters on %s", node_name.c_str());
    return std::nullopt;
  }

  auto response = future.get();
  if (response->values.empty()) {
    RCLCPP_WARN(node_->get_logger(), "Parameter %s not found on %s", param_name.c_str(), node_name.c_str());
    return std::nullopt;
  }

  rcl_interfaces::msg::Parameter param_msg;
  param_msg.name = param_name;
  param_msg.value = response->values[0];

  return rclcpp::Parameter::from_parameter_msg(param_msg);
}
