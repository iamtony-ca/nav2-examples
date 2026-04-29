#include "amr_bt_nodes/trigger_receiver_and_parser.hpp" // 당신의 패키지 이름으로 변경

// 이 부분은 BT.CPP에서 플러그인 등록을 위해 필요합니다.
#include "behaviortree_cpp/bt_factory.h"

namespace amr_bt_nodes
{

// // providedPorts() 구현
// BT::PortsList TriggerReceiverAndParser::providedPorts()
// {
//     return {
//         // ROS2 노드 핸들을 입력으로 받습니다. Nav2 BT 환경에서 필요
//         BT::InputPort<rclcpp::Node::SharedPtr>("node", "Shared ROS2 Node handle from Nav2 Behavior Tree context"),
        
//         // --- 토픽 관련 포트 ---
//         BT::InputPort<std::string>("input_topic_name", "/navigation_command", "ROS2 input Topic Name for trigger messages (e.g., dynamic planner/controller change)"),
//         BT::InputPort<std::string>("output_topic_name", "/navigation_response", "ROS2 output Topic Name for response messages to trigger"),
        
//         // --- 서비스 관련 포트 ---
//         BT::InputPort<std::string>("service_name", "/set_navigation_mode", "ROS2 Service Name to call for specific mode changes"),
//         BT::InputPort<std::string>("service_trigger_key", "trigger_service_call", "Blackboard key: set to true to trigger service call (will be reset to false)"),
//         BT::InputPort<std::string>("service_mode_key", "service_request_mode", "Blackboard key: mode to send in service request (e.g., STOP, RECOVER)"),

//         // --- 공통 출력 포트 (Blackboard에 쓸 값) ---
//         BT::OutputPort<std::string>("output_planner_id", "Dynamically selected planner ID to be written to Blackboard"),
//         BT::OutputPort<std::string>("output_controller_id", "Dynamically selected controller ID to be written to Blackboard")
//     };
// }


// 생성자 구현
TriggerReceiverAndParser::TriggerReceiverAndParser(
    const std::string& name,
    const BT::NodeConfiguration& config)
    : BT::ActionNodeBase(name, config)
//   : BT::ActionNodeBase(name, config),
    // new_topic_msg_received_(false), // 토픽 메시지 수신 플래그 초기화
    // service_request_sent_(false)    // 서비스 요청 플래그 초기화
{
    // // 1. ROS2 노드 핸들 가져오기 (BT XML의 'node' 포트에서)
    // // Nav2의 BtActionNodeBase가 이 방식으로 ROS2 노드 포인터를 제공합니다.
    // if (!getInput("node", node_)) {
    //     throw BT::RuntimeError("[TriggerReceiverAndParser] Missing required input [node] (rclcpp::Node::SharedPtr)");
    // }
    
    // // 2. SharedExecutor 시작 (노드가 이미 add_node 되어 있다고 가정)
    // // 이 노드가 사용하는 ROS2 노드를 SharedExecutor에 추가하고 스핀 시작
    // SharedExecutor::start(node_); 
    // RCLCPP_INFO(node_->get_logger(), "[%s] ROS2 node reference obtained and SharedExecutor ensured.", name.c_str());

    // // 3. 입력 포트 값 읽기
    // // 토픽 관련 포트
    // if (!getInput("input_topic_name", input_topic_name_)) {
    //     throw BT::RuntimeError("[TriggerReceiverAndParser] Missing input [input_topic_name]");
    // }
    // if (!getInput("output_topic_name", output_topic_name_)) {
    //     throw BT::RuntimeError("[TriggerReceiverAndParser] Missing input [output_topic_name]");
    // }
    // // 서비스 관련 포트
    // if (!getInput("service_name", service_name_)) {
    //     throw BT::RuntimeError("[TriggerReceiverAndParser] Missing input [service_name]");
    // }
    // if (!getInput("service_trigger_key", service_trigger_key_)) { 
    //     throw BT::RuntimeError("[TriggerReceiverAndParser] Missing input [service_trigger_key]");
    // }
    // if (!getInput("service_mode_key", service_mode_key_)) { 
    //     throw BT::RuntimeError("[TriggerReceiverAndParser] Missing input [service_mode_key]");
    // }

    // // 출력 포트의 실제 키 이름을 가져옵니다.
    // output_planner_id_key_ = config().output_ports.at("output_planner_id");
    // output_controller_id_key_ = config().output_ports.at("output_controller_id");

    // // 4. 콜백 그룹 생성
    // callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // // 5. ROS2 토픽 구독자 생성
    // rclcpp::SubscriptionOptions sub_options;
    // sub_options.callback_group = callback_group_;
    // subscription_ = node_->create_subscription<robot_interfaces::msg::NavigationCommand>(
    //     input_topic_name_, 
    //     rclcpp::QoS(1).reliable().transient_local(), // QoS 설정 예시 (신뢰성, 최근 상태 유지)
    //     std::bind(&TriggerReceiverAndParser::triggerMessageCallback, this, std::placeholders::_1),
    //     sub_options);

    // // 6. ROS2 토픽 발행자 생성
    // publisher_ = node_->create_publisher<std_msgs::msg::String>(
    //     output_topic_name_, 
    //     rclcpp::QoS(1).reliable().transient_local()); 

    // // 7. ROS2 서비스 클라이언트 생성
    // service_client_ = node_->create_client<robot_interfaces::srv::SetNavigationMode>(
    //     service_name_,
    //     rclcpp::QoS(1).reliable().transient_local(), 
    //     callback_group_); 

    // RCLCPP_INFO(node_->get_logger(), "[%s] Initialized. Subscribing to: %s, Publishing to: %s, Service Client for: %s",
    //             name.c_str(), input_topic_name_.c_str(), output_topic_name_.c_str(), service_name_.c_str());
}

// 소멸자 구현
TriggerReceiverAndParser::~TriggerReceiverAndParser()
{
    // ROS2 객체는 shared_ptr로 관리되므로 노드 포인터가 소멸될 때 자동 정리됩니다.
    // RCLCPP_INFO(node_->get_logger(), "[%s] Destructor called.", name().c_str());
}



} // namespace amr_bt_nodes

// 플러그인 등록 매크로 (pluginlib 사용 시 필요)
// 이 함수는 BT::BehaviorTreeFactory가 플러그인 로드 시 자동으로 호출합니다.
// extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory &factory)
// {
//     factory.registerNodeType<amr_bt_nodes::TriggerReceiverAndParser>("TriggerReceiverAndParser");
// }