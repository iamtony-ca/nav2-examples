#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <rcl_interfaces/srv/get_parameters.hpp>
#include <string>
#include <optional>

class Nav2ParamManager {
public:
  explicit Nav2ParamManager(const rclcpp::Node::SharedPtr & node);

  bool set_param(const std::string & node_name, const std::string & param_name, const rclcpp::Parameter & value);
  std::optional<rclcpp::Parameter> get_param(const std::string & node_name, const std::string & param_name);

  template<typename T>
  bool set_param_typed(const std::string & node_name, const std::string & param_name, const T & value) {
    return set_param(node_name, param_name, rclcpp::Parameter(param_name, value));
  }

private:
  rclcpp::Node::SharedPtr node_;
  const std::chrono::seconds kServiceTimeout{2};
};
