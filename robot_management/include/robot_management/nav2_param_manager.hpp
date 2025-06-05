#include "robot_management/nav2_param_manager.hpp"

Nav2ParamManager::Nav2ParamManager(const rclcpp::Node::SharedPtr & node)
: node_(node) {}

rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr
Nav2ParamManager::get_set_client(const std::string & node_name)
{
  if (set_clients_.count(node_name)) {
    return set_clients_[node_name];
  }

  auto client = node_->create_client<rcl_interfaces::srv::SetParameters>(node_name + "/set_parameters");
  set_clients_[node_name] = client;
  return client;
}

rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr
Nav2ParamManager::get_get_client(const std::string & node_name)
{
  if (get_clients_.count(node_name)) {
    return get_clients_[node_name];
  }

  auto client = node_->create_client<rcl_interfaces::srv::GetParameters>(node_name + "/get_parameters");
  get_clients_[node_name] = client;
  return client;
}

void Nav2ParamManager::set_param(const std::string & node_name,
                                 const std::string & param_name,
                                 const rclcpp::Parameter & value)
{
  auto client = get_set_client(node_name);

  if (!client->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] set_parameters 서비스 없음", node_name.c_str());
    return;
  }

  auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
  request->parameters.push_back(value.to_parameter_msg());

  auto future_and_id = client->async_send_request(request);

  std::thread([this, node_name, param_name, result_future = std::move(future_and_id.future)]() mutable {
    if (result_future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      RCLCPP_ERROR(node_->get_logger(), " [%s] set_param 타임아웃", param_name.c_str());
      return;
    }

    try {
      auto response = result_future.get();
      for (const auto & result : response->results) {
        if (!result.successful) {
          RCLCPP_WARN(node_->get_logger(), " [%s] 설정 실패: %s", param_name.c_str(), result.reason.c_str());
        } else {
          RCLCPP_INFO(node_->get_logger(), "[%s] 성공적으로 설정됨", param_name.c_str());
        }
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(node_->get_logger(), " [%s] set_param 예외: %s", param_name.c_str(), e.what());
    }
  }).detach();
}

void Nav2ParamManager::get_param_async(const std::string & node_name,
                                       const std::string & param_name,
                                       std::function<void(std::optional<rclcpp::Parameter>)> callback)
{
  auto client = get_get_client(node_name);

  if (!client->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(node_->get_logger(), "[%s] get_parameters 서비스 없음", node_name.c_str());
    callback(std::nullopt);
    return;
  }

  auto request = std::make_shared<rcl_interfaces::srv::GetParameters::Request>();
  request->names.push_back(param_name);

  auto future_and_id = client->async_send_request(request);

  std::thread([this, node_name, param_name, result_future = std::move(future_and_id.future), callback]() mutable {
    if (result_future.wait_for(std::chrono::seconds(2)) != std::future_status::ready) {
      RCLCPP_ERROR(node_->get_logger(), " [%s] get_param 타임아웃", param_name.c_str());
      callback(std::nullopt);
      return;
    }

    try {
      auto response = result_future.get();
      if (response->values.empty()) {
        callback(std::nullopt);
        return;
      }

      rcl_interfaces::msg::Parameter param_msg;
      param_msg.name = param_name;
      param_msg.value = response->values[0];
      callback(rclcpp::Parameter::from_parameter_msg(param_msg));
    } catch (const std::exception & e) {
      RCLCPP_ERROR(node_->get_logger(), "[%s] get_param 예외: %s", param_name.c_str(), e.what());
      callback(std::nullopt);
    }
  }).detach();
}
