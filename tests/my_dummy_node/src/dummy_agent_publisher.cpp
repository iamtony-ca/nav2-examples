/**
 * @file src/dummy_agent_publisher.cpp
 * @brief Implementation of the DummyAgentPublisher class.
 */

#include "my_dummy_node/dummy_agent_publisher.hpp"

using namespace std::chrono_literals;

namespace my_dummy_node
{

DummyAgentPublisher::DummyAgentPublisher(const rclcpp::NodeOptions & options)
: Node("dummy_agent_publisher_node", options), count_(0)
{
  // 1. 파라미터 선언 (기본값 설정)
  // launch 파일이나 params.yaml에서 덮어쓸 수 있습니다.
  this->declare_parameter("publish_rate", 10.0);
  this->declare_parameter("frame_id", "map");
  this->declare_parameter("agent_count", 3);
  this->declare_parameter("base_radius", 2.0);
  this->declare_parameter("radius_increment", 1.5);
  this->declare_parameter("base_speed", 0.5);

  // 2. 초기 파라미터 로드
  double publish_rate = this->get_parameter("publish_rate").as_double();

  // 3. Publisher 생성 (QoS: SensorData 권장 - 최신성 유지)
  auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
  publisher_ = this->create_publisher<multi_agent_msgs::msg::MultiAgentInfoArray>(
    "multi_agent_info", qos);

  // 4. Timer 생성
  // 주기를 파라미터에서 읽어와 설정 (Hz -> ms 변환)
  auto timer_period = std::chrono::duration<double, std::milli>(1000.0 / publish_rate);
  timer_ = this->create_wall_timer(
    timer_period, std::bind(&DummyAgentPublisher::timer_callback, this));

  RCLCPP_INFO(this->get_logger(), "Dummy Agent Publisher Initialized at %.1f Hz", publish_rate);
}

DummyAgentPublisher::~DummyAgentPublisher()
{
  RCLCPP_INFO(this->get_logger(), "Dummy Agent Publisher Shutting Down.");
}

void DummyAgentPublisher::timer_callback()
{
  // 파라미터 실시간 반영을 위해 콜백 내부에서 읽기 (Dynamic Parameter Tuning 가능)
  int agent_count = this->get_parameter("agent_count").as_int();
  double base_radius = this->get_parameter("base_radius").as_double();
  double radius_inc = this->get_parameter("radius_increment").as_double();
  double base_speed = this->get_parameter("base_speed").as_double();
  std::string frame_id = this->get_parameter("frame_id").as_string();

  auto msg = multi_agent_msgs::msg::MultiAgentInfoArray();
  msg.header.stamp = this->get_clock()->now();
  msg.header.frame_id = frame_id;

  // 설정된 개수만큼 에이전트 생성
  for (int i = 0; i < agent_count; ++i) {
    uint16_t id = static_cast<uint16_t>(i + 1);
    
    // 에이전트마다 반지름과 속도를 다르게 설정하여 패턴 생성
    double radius = base_radius + (i * radius_inc);
    
    // 짝수는 시계방향, 홀수는 반시계방향 등으로 변화를 줄 수 있음 (여기선 모두 정방향)
    double speed = base_speed; 
    
    // 타입 번갈아 가며 설정
    std::string type = (i % 2 == 0) ? "LIFT" : "TOW";

    msg.agents.push_back(create_dummy_agent(id, type, radius, speed));
  }

  publisher_->publish(msg);
  count_++;
}

multi_agent_msgs::msg::MultiAgentInfo DummyAgentPublisher::create_dummy_agent(
  uint16_t id, const std::string & type, double radius, double speed)
{
  multi_agent_msgs::msg::MultiAgentInfo agent;
  
  // 공통 헤더 정보
  agent.header.frame_id = this->get_parameter("frame_id").as_string();
  agent.header.stamp = this->get_clock()->now();

  // 1. 식별자
  agent.machine_id = id;
  agent.type_id = type;

  // 2. 위치 계산 (원형 이동)
  // count_는 타이머 호출 횟수, 0.1은 시간 스케일링 팩터
  double time_factor = count_ * 0.1; 
  double angle = time_factor * speed; // 각도 = 시간 * 각속도

  // 초기 위상(Phase)을 ID별로 다르게 주어 겹치지 않게 함
  angle += (id * (2.0 * M_PI / 3.0)); 

  double x = radius * std::cos(angle);
  double y = radius * std::sin(angle);
  double yaw = angle + (M_PI / 2.0); // 진행 방향 (접선)

  // Pose
  agent.current_pose.header = agent.header;
  agent.current_pose.pose.position.x = x;
  agent.current_pose.pose.position.y = y;
  agent.current_pose.pose.position.z = 0.0;
  
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  agent.current_pose.pose.orientation = tf2::toMsg(q);

  // Twist (로컬 기준 속도)
  agent.current_twist.linear.x = radius * speed; // v = r * omega
  agent.current_twist.angular.z = speed;

  // 3. 상태 (주기적 변경 시뮬레이션)
  // 100틱마다 상태 변경: MOVING <-> WAITING
  if ((count_ / 100) % 2 == 0) {
    agent.status.phase = multi_agent_msgs::msg::AgentStatus::STATUS_MOVING;
  } else {
    agent.status.phase = multi_agent_msgs::msg::AgentStatus::STATUS_WAITING_FOR_OBS;
    // 대기 중일 때는 속도 0 처리
    agent.current_twist.linear.x = 0.0;
    agent.current_twist.angular.z = 0.0;
  }

  // 4. Footprint (직사각형)
  agent.footprint.header = agent.header;
  std::vector<std::pair<float, float>> fp_pts = {
    {0.5f, 0.3f}, {0.5f, -0.3f}, {-0.5f, -0.3f}, {-0.5f, 0.3f}
  };
  for (const auto& pt : fp_pts) {
    geometry_msgs::msg::Point32 p;
    // 로컬 좌표 (base_link)
    p.x = pt.first; 
    p.y = pt.second;
    p.z = 0.0f;
    agent.footprint.polygon.points.push_back(p);
  }

  // 5. Truncated Path (미래 경로 10스텝 예측)
  agent.truncated_path.header = agent.header;
  if (agent.status.phase == multi_agent_msgs::msg::AgentStatus::STATUS_MOVING) {
    for (int i = 1; i <= 10; ++i) {
      geometry_msgs::msg::PoseStamped p;
      p.header = agent.header;
      
      double future_angle = angle + (i * 0.1 * speed);
      p.pose.position.x = radius * std::cos(future_angle);
      p.pose.position.y = radius * std::sin(future_angle);
      
      tf2::Quaternion q_future;
      q_future.setRPY(0, 0, future_angle + (M_PI / 2.0));
      p.pose.orientation = tf2::toMsg(q_future);
      
      agent.truncated_path.poses.push_back(p);
    }
  }

  // 6. 기타 정보 채우기
  agent.reroute = false;
  agent.current_waypoint = static_cast<uint8_t>((count_ / 50) % 255);
  agent.next_waypoint = static_cast<uint8_t>((agent.current_waypoint + 1) % 255);
  agent.mode = "AUTO";
  agent.area_id = 10;
  agent.occupancy = false;
  agent.transferring = true;
  agent.re_path_search = false;
  
  // 불확실성 (Uncertainty)
  agent.pos_std_m = 0.02f;
  agent.yaw_std_rad = 0.01f;

  return agent;
}

} // namespace my_dummy_node