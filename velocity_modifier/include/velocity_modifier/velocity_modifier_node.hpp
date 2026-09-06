#ifndef VELOCITY_MODIFIER__VELOCITY_MODIFIER_NODE_HPP_
#define VELOCITY_MODIFIER__VELOCITY_MODIFIER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <robot_interfaces/msg/modifier_control.hpp>
#include <robot_interfaces/srv/set_velocity_modifier.hpp>
#include <std_msgs/msg/string.hpp>

#include <mutex> // <atomic> 대신 <mutex>를 포함
#include <limits>
#include <memory>
#include <cmath>  // for std::adb, std::copysign
#include <string>

namespace velocity_modifier
{

class VelocityModifierNode : public rclcpp::Node
{
public:
  using ModifierControl = robot_interfaces::msg::ModifierControl;
  using SetVelocityModifier = robot_interfaces::srv::SetVelocityModifier;
  using String = std_msgs::msg::String; // 타입 별칭 추가

  explicit VelocityModifierNode(const rclcpp::NodeOptions & options);

private:
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
  void controlCallback(const ModifierControl::SharedPtr msg);
  void recoveryModeCallback(const String::SharedPtr msg);

  /// 토픽과 서비스가 공유하는 실제 적용 로직.
  ///
  /// 두 경로가 각자 모드를 세팅하면 언젠가 갈라진다. 그래서 여기 한 곳에만 두고
  /// controlCallback(토픽)과 setControlService(서비스)가 모두 이 함수를 부른다.
  /// data_mutex_ 는 이 함수 안에서 잡는다(호출자는 잡지 않은 채로 들어와야 한다).
  ///
  /// @param req      요청 값
  /// @param applied  실제로 적용된 값 (제한이 걸리면 요청과 다를 수 있다)
  /// @param message  사람이 읽는 결과 설명
  /// @return         알 수 없는 command_type 이면 false (이때 모드는 바뀌지 않는다)
  bool applyControl(const ModifierControl & req, ModifierControl & applied,
                    std::string & message);

  void setControlService(
    const std::shared_ptr<SetVelocityModifier::Request> request,
    std::shared_ptr<SetVelocityModifier::Response> response);

  rclcpp::CallbackGroup::SharedPtr cb_group_cmd_vel_;
  rclcpp::CallbackGroup::SharedPtr cb_group_control_;
  rclcpp::CallbackGroup::SharedPtr cb_group_recovery_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<ModifierControl>::SharedPtr control_sub_;
  rclcpp::Subscription<String>::SharedPtr recovery_mode_sub_;
  rclcpp::Service<SetVelocityModifier>::SharedPtr control_service_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr adjusted_cmd_vel_pub_;

  // 데이터 보호를 위한 뮤텍스
  std::mutex data_mutex_;

  // 일반 멤버 변수
  double speed_limit_linear_ = std::numeric_limits<double>::max();
  double speed_limit_angular_ = std::numeric_limits<double>::max();
  double speed_scale_ = 1.0;

  // 새로운 모드를 위한 변수
  double ratio_limit_linear_ = std::numeric_limits<double>::max();
  double ratio_limit_angular_ = std::numeric_limits<double>::max();

  // 어떤 모드가 활성화되었는지 나타내는 Enum
  enum class SpeedMode {
    STANDARD_LIMIT,
    STANDARD_SCALE,
    RATIO_LIMIT_SCALE
  };
  SpeedMode current_mode_ = SpeedMode::STANDARD_LIMIT;


  double min_abs_linear_vel_ = 0.05;
  double min_abs_angular_vel_ = 0.05;
  // [추가] 회복 구간 저속 보정의 판단 기준. 모터 데드밴드는 로봇 몸체 속도가 아니라
  // 바퀴 속도의 성질이므로, 좌/우 바퀴 속도로 판단한다. min_abs_linear_vel_ /
  // min_abs_angular_vel_ 은 더 이상 이 판단에 쓰지 않는다(아래 cmdVelCallback 주석 참고).
  double min_abs_wheel_vel_ = 0.02;   // 실측 데드밴드 0.01 (그 이하면 안 움직임) + 2배 마진
  double wheel_separation_ = 0.526;   // roboteq.yaml 의 track_width
  
  // 비율 보정 시 적용될 상한선 
  double ratio_scaling_max_linear_vel_ = 0.35;
  double ratio_scaling_max_angular_vel_ = 0.25;

  bool recovery_mode_ = false;

  // === [Added] 주행 시작 속도 제한을 위한 변수 ===
  rclcpp::Time driving_start_time_;      // 주행 시작 시각 저장
  std::string last_robot_status_ = "";   // 상태 변화 감지용
  double startup_phase2_limit_ = 0.3;    // 1초 ~ 2.5초 사이의 속도 제한 값 (파라미터화)
  // [추가] 주행 초기 각속도 제한. 선속도 제한만으로는 제자리 선회를 못 막는다
  // (선속도가 0 이면 clamping 조건에 걸리지 않는다). 현장 v_angular_max 는 1.8 이다.
  double startup_angular_limit_ = 0.5;



};

}  // namespace velocity_modifier

#endif  // VELOCITY_MODIFIER__VELOCITY_MODIFIER_NODE_HPP_