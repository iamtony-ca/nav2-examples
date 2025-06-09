#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <unordered_map>
#include <string>
#include <optional>
#include <functional>

class Nav2ParamManager {
public:
  explicit Nav2ParamManager(const rclcpp::Node::SharedPtr & node);

  void set_param(const std::string & node_name, const std::string & param_name, const rclcpp::Parameter & value);

  void get_param_async(const std::string & node_name,
                       const std::string & param_name,
                       std::function<void(std::optional<rclcpp::Parameter>)> callback);

  template<typename T>
  void set_param_typed(const std::string & node_name, const std::string & param_name, const T & value) {
    set_param(node_name, param_name, rclcpp::Parameter(param_name, value));
  }

private:
  rclcpp::Node::SharedPtr node_;
  std::unordered_map<std::string, rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr> set_clients_;
  std::unordered_map<std::string, rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr> get_clients_;

  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr get_set_client(const std::string & node_name);
  rclcpp::Client<rcl_interfaces::srv::GetParameters>::SharedPtr get_get_client(const std::string & node_name);
};
