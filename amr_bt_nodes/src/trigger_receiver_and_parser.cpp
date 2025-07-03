#include "amr_bt_nodes/trigger_receiver_and_parser.hpp" // 당신의 패키지 이름으로 변경

// 이 부분은 BT.CPP에서 플러그인 등록을 위해 필요합니다.
#include "behaviortree_cpp/bt_factory.h"

namespace amr_bt_nodes
{

// providedPorts() 구현
BT::PortsList TriggerReceiverAndParser::providedPorts()
{
    return {
        // ROS2 노드 핸들을 입력으로 받습니다. Nav2 BT 환경에서 필요
        BT::InputPort<rclcpp::Node::SharedPtr>("node", "Shared ROS2 Node handle from Nav2 Behavior Tree context"),
        
        // --- 토픽 관련 포트 ---
        BT::InputPort<std::string>("input_topic_name", "/navigation_command", "ROS2 input Topic Name for trigger messages (e.g., dynamic planner/controller change)"),
        BT::InputPort<std::string>("output_topic_name", "/navigation_response", "ROS2 output Topic Name for response messages to trigger"),
        
        // --- 서비스 관련 포트 ---
        BT::InputPort<std::string>("service_name", "/set_navigation_mode", "ROS2 Service Name to call for specific mode changes"),
        BT::InputPort<std::string>("service_trigger_key", "trigger_service_call", "Blackboard key: set to true to trigger service call (will be reset to false)"),
        BT::InputPort<std::string>("service_mode_key", "service_request_mode", "Blackboard key: mode to send in service request (e.g., STOP, RECOVER)"),

        // --- 공통 출력 포트 (Blackboard에 쓸 값) ---
        BT::OutputPort<std::string>("output_planner_id", "Dynamically selected planner ID to be written to Blackboard"),
        BT::OutputPort<std::string>("output_controller_id", "Dynamically selected controller ID to be written to Blackboard")
    };
}


// 생성자 구현
TriggerReceiverAndParser::TriggerReceiverAndParser(
    const std::string& name,
    const BT::NodeConfiguration& config)
  : BT::ActionNodeBase(name, config),
    new_topic_msg_received_(false), // 토픽 메시지 수신 플래그 초기화
    service_request_sent_(false)    // 서비스 요청 플래그 초기화
{
    // 1. ROS2 노드 핸들 가져오기 (BT XML의 'node' 포트에서)
    // Nav2의 BtActionNodeBase가 이 방식으로 ROS2 노드 포인터를 제공합니다.
    if (!getInput("node", node_)) {
        throw BT::RuntimeError("[TriggerReceiverAndParser] Missing required input [node] (rclcpp::Node::SharedPtr)");
    }
    
    // 2. SharedExecutor 시작 (노드가 이미 add_node 되어 있다고 가정)
    // 이 노드가 사용하는 ROS2 노드를 SharedExecutor에 추가하고 스핀 시작
    SharedExecutor::start(node_); 
    RCLCPP_INFO(node_->get_logger(), "[%s] ROS2 node reference obtained and SharedExecutor ensured.", name.c_str());

    // 3. 입력 포트 값 읽기
    // 토픽 관련 포트
    if (!getInput("input_topic_name", input_topic_name_)) {
        throw BT::RuntimeError("[TriggerReceiverAndParser] Missing input [input_topic_name]");
    }
    if (!getInput("output_topic_name", output_topic_name_)) {
        throw BT::RuntimeError("[TriggerReceiverAndParser] Missing input [output_topic_name]");
    }
    // 서비스 관련 포트
    if (!getInput("service_name", service_name_)) {
        throw BT::RuntimeError("[TriggerReceiverAndParser] Missing input [service_name]");
    }
    if (!getInput("service_trigger_key", service_trigger_key_)) { 
        throw BT::RuntimeError("[TriggerReceiverAndParser] Missing input [service_trigger_key]");
    }
    if (!getInput("service_mode_key", service_mode_key_)) { 
        throw BT::RuntimeError("[TriggerReceiverAndParser] Missing input [service_mode_key]");
    }

    // 출력 포트의 실제 키 이름을 가져옵니다.
    output_planner_id_key_ = config().output_ports.at("output_planner_id");
    output_controller_id_key_ = config().output_ports.at("output_controller_id");

    // 4. 콜백 그룹 생성
    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    // 5. ROS2 토픽 구독자 생성
    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = callback_group_;
    subscription_ = node_->create_subscription<robot_interfaces::msg::NavigationCommand>(
        input_topic_name_, 
        rclcpp::QoS(1).reliable().transient_local(), // QoS 설정 예시 (신뢰성, 최근 상태 유지)
        std::bind(&TriggerReceiverAndParser::triggerMessageCallback, this, std::placeholders::_1),
        sub_options);

    // 6. ROS2 토픽 발행자 생성
    publisher_ = node_->create_publisher<std_msgs::msg::String>(
        output_topic_name_, 
        rclcpp::QoS(1).reliable().transient_local()); 

    // 7. ROS2 서비스 클라이언트 생성
    service_client_ = node_->create_client<robot_interfaces::srv::SetNavigationMode>(
        service_name_,
        rclcpp::QoS(1).reliable().transient_local(), 
        callback_group_); 

    RCLCPP_INFO(node_->get_logger(), "[%s] Initialized. Subscribing to: %s, Publishing to: %s, Service Client for: %s",
                name.c_str(), input_topic_name_.c_str(), output_topic_name_.c_str(), service_name_.c_str());
}

// 소멸자 구현
TriggerReceiverAndParser::~TriggerReceiverAndParser()
{
    // ROS2 객체는 shared_ptr로 관리되므로 노드 포인터가 소멸될 때 자동 정리됩니다.
    RCLCPP_INFO(node_->get_logger(), "[%s] Destructor called.", name().c_str());
}

// 트리거 메시지 콜백 함수
void TriggerReceiverAndParser::triggerMessageCallback(const robot_interfaces::msg::NavigationCommand::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(mutex_); // 뮤텍스로 공유 데이터 접근 보호
    last_received_topic_msg_ = msg;
    new_topic_msg_received_ = true;
    RCLCPP_INFO(node_->get_logger(), "[%s] Received new trigger message on topic %s.", name().c_str(), input_topic_name_.c_str());
}

// tick() 메서드 구현
BT::NodeStatus TriggerReceiverAndParser::tick()
{
    std::lock_guard<std::mutex> lock(mutex_); // 뮤텍스로 공유 데이터 접근 보호

    // --- 1. 토픽 메시지 처리 로직 ---
    if (new_topic_msg_received_) {
        new_topic_msg_received_ = false; // 플래그 리셋

        // 메시지 파싱 및 Blackboard 업데이트
        // 'robot_interfaces::msg::NavigationCommand' 메시지 구조에 따라 필드명을 조정하세요.
        std::string received_planner_id = last_received_topic_msg_->desired_planner_id; 
        std::string received_controller_id = last_received_topic_msg_->desired_controller_id; 

        // 메시지 필드가 비어있지 않은 경우에만 Blackboard를 업데이트하여 기존 값을 보존합니다.
        if (!received_planner_id.empty()) {
            setOutput(output_planner_id_key_, received_planner_id);
            RCLCPP_INFO(node_->get_logger(), "[%s] Blackboard: Set planner_id to '%s'. (from Topic)", name().c_str(), received_planner_id.c_str());
        } else {
            RCLCPP_DEBUG(node_->get_logger(), "[%s] Planner ID in topic message was empty. Keeping existing Blackboard value.", name().c_str());
        }

        if (!received_controller_id.empty()) {
            setOutput(output_controller_id_key_, received_controller_id);
            RCLCPP_INFO(node_->get_logger(), "[%s] Blackboard: Set controller_id to '%s'. (from Topic)", name().c_str(), received_controller_id.c_str());
        } else {
            RCLCPP_DEBUG(node_->get_logger(), "[%s] Controller ID in topic message was empty. Keeping existing Blackboard value.", name().c_str());
        }

        // 응답 토픽 발행
        std_msgs::msg::String response_msg;
        response_msg.data = "Topic Trigger processed. Planner: " + received_planner_id + ", Controller: " + received_controller_id + ". Timestamp: " + std::to_string(node_->now().seconds());
        publisher_->publish(response_msg);
        RCLCPP_INFO(node_->get_logger(), "[%s] Published response to %s.", name().c_str(), output_topic_name_.c_str());
        
        // 토픽 메시지 처리 후 성공 반환 (BT가 다음 틱에서 다시 트리거를 기다릴 수 있도록)
        return BT::NodeStatus::SUCCESS; 
    }

    // --- 2. 서비스 호출 처리 로직 ---
    // 서비스 호출 트리거 키가 Blackboard에 설정되었는지 확인
    BT::Optional<bool> service_trigger_val = getInput<bool>(service_trigger_key_);
    
    // Blackboard의 service_trigger_key가 true이고, 아직 서비스 요청을 보내지 않았다면
    if (service_trigger_val && *service_trigger_val == true && !service_request_sent_) { 
        // 서비스 서버가 가용한지 확인 (짧은 시간만 블록, 아니면 즉시 실패)
        if (!service_client_->wait_for_service(std::chrono::milliseconds(100))) { // 100ms 대기
            RCLCPP_ERROR(node_->get_logger(), "[%s] Service '%s' not available after 100ms. Returning FAILURE.", name().c_str(), service_name_.c_str());
            // 서비스 트리거 플래그를 false로 리셋하여 반복적인 실패 요청 방지
            setOutput(service_trigger_key_, false); 
            return BT::NodeStatus::FAILURE;
        }

        // 서비스 요청 생성 및 모드 설정
        auto request = std::make_shared<robot_interfaces::srv::SetNavigationMode::Request>();
        BT::Optional<std::string> service_mode_val = getInput<std::string>(service_mode_key_);
        if (service_mode_val) {
            request->mode = *service_mode_val; 
            RCLCPP_INFO(node_->get_logger(), "[%s] Sending service request to %s with mode: %s", name().c_str(), service_name_.c_str(), request->mode.c_str());
        } else {
            request->mode = "DEFAULT_MODE_NO_KEY"; // Blackboard 키가 없으면 기본 모드
            RCLCPP_WARN(node_->get_logger(), "[%s] No service mode specified via Blackboard key '%s'. Using default mode: %s", name().c_str(), service_mode_key_.c_str(), request->mode.c_str());
        }
        
        // 비동기 서비스 호출
        service_future_ = service_client_->async_send_request(request);
        service_request_sent_ = true; // 요청 상태 플래그 설정
        // 서비스 트리거 플래그를 false로 리셋하여 다음 틱에서 재요청 방지
        setOutput(service_trigger_key_, false); 
        RCLCPP_INFO(node_->get_logger(), "[%s] Service request sent. Waiting for response...", name().c_str());
        return BT::NodeStatus::RUNNING; // 요청을 보냈고 응답을 기다리는 중
    }

    // 서비스 응답 대기 중이라면 계속 RUNNING
    if (service_request_sent_) {
        // 응답이 준비되었는지 논블로킹 방식으로 확인
        if (service_future_.wait_for(std::chrono::milliseconds(0)) == std::future_status::ready) {
            auto response = service_future_.get();
            service_request_sent_ = false; // 요청 상태 리셋

            if (response->success) {
                RCLCPP_INFO(node_->get_logger(), "[%s] Service '%s' responded SUCCESS.", name().c_str(), service_name_.c_str());
                // 서비스 응답에서 플래너/컨트롤러 ID를 받을 수도 있다면 Blackboard에 업데이트 (예시)
                // if (!response->new_planner_id.empty()) { setOutput(output_planner_id_key_, response->new_planner_id); }
                return BT::NodeStatus::SUCCESS;
            } else {
                RCLCPP_ERROR(node_->get_logger(), "[%s] Service '%s' responded FAILURE: %s", name().c_str(), service_name_.c_str(), response->message.c_str());
                return BT::NodeStatus::FAILURE;
            }
        } else {
            // 서비스 응답을 아직 기다리는 중
            return BT::NodeStatus::RUNNING;
        }
    }

    // 토픽 메시지도 없고, 서비스 호출도 트리거되지 않았고, 서비스 응답도 기다리는 중이 아니라면
    // 다음 틱을 위해 RUNNING 상태 유지
    return BT::NodeStatus::RUNNING;
}

// halt() 메서드 구현
void TriggerReceiverAndParser::halt()
{
    std::lock_guard<std::mutex> lock(mutex_); // 뮤텍스로 공유 데이터 접근 보호
    new_topic_msg_received_ = false;
    last_received_topic_msg_ = nullptr;
    
    service_request_sent_ = false; // 서비스 요청 상태 리셋
    // 서비스 future도 리셋
    service_future_ = std::shared_future<std::shared_ptr<robot_interfaces::srv::SetNavigationMode::Response>>(); 
    
    setStatus(BT::NodeStatus::IDLE); // 액션 노드를 IDLE 상태로 리셋
    RCLCPP_INFO(node_->get_logger(), "[%s] Halted.", name().c_str());
}

} // namespace amr_bt_nodes

// 플러그인 등록 매크로 (pluginlib 사용 시 필요)
// 이 함수는 BT::BehaviorTreeFactory가 플러그인 로드 시 자동으로 호출합니다.
extern "C" void BT_RegisterNodesFromPlugin(BT::BehaviorTreeFactory &factory)
{
    factory.registerNodeType<amr_bt_nodes::TriggerReceiverAndParser>("TriggerReceiverAndParser");
}