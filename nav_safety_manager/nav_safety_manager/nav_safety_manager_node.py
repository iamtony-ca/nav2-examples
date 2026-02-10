#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.task import Future

# 메시지 및 서비스 임포트
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool  # 요청을 위해 SetBool 사용 (data=True/False)
from safety_plc_monitoring_msgs.msg import SafetyPlcMonitoring
from safety_plc_interfaces.srv import SetArea
# [중요] 사용자 정의 메시지 (실제 패키지명에 맞게 수정 필요)
from nav2_msgs.msg import CollisionDetectorState 
# 테스트를 위해 아래와 같이 가정하거나, 실제 환경에 맞게 주석 해제하세요.
# 여기서는 코드가 실행되도록 구조만 잡고, 실제 msg 타입은 주석 처리합니다.
# try:
#     from nav_msgs.msg import CollisionDetectorState # 예시
# except ImportError:
#     # 코드가 돌아가게 하기 위한 Mock Class (실제 사용시 삭제)
#     class CollisionDetectorState:
#         pass

class NavSafetyManagerNode(Node):

    def __init__(self):
        super().__init__('nav_safety_manager_node')

        # 설정 변수 (N초, M초)
        self.declare_parameter('plc_false_duration_sec', 30.0)
        self.declare_parameter('collision_clear_duration_sec', 3.0)
        self.declare_parameter('target_polygon_name', 'PolygonSafety')

        self.N_sec = self.get_parameter('plc_false_duration_sec').value
        self.M_sec = self.get_parameter('collision_clear_duration_sec').value
        self.target_polygon = self.get_parameter('target_polygon_name').value

        # 동시성 처리를 위한 그룹
        self.cb_group = ReentrantCallbackGroup()

        # 1. Subscribers
        self.sub_plc = self.create_subscription(
            SafetyPlcMonitoring, 
            '/ros2_safety_plc_status_data', 
            self.plc_callback, 
            10, 
            callback_group=self.cb_group
        )
        
        # 실제 환경에 맞는 타입으로 변경 필요
        # self.sub_col = self.create_subscription(
        #     CollisionDetectorState, 
        #     '/collision_detector_state', 
        #     self.collision_callback, 
        #     10, 
        #     callback_group=self.cb_group
        # )
        # *코드 에러 방지를 위해 임시로 Bool로 둡니다. 실제 적용 시 위 주석 해제*
        self.sub_col = self.create_subscription(
            CollisionDetectorState, # 실제로는 CollisionDetectorState
            '/collision_detector_state', 
            self.collision_callback, 
            10, 
            callback_group=self.cb_group
        )

        self.sub_nav_status = self.create_subscription(
            String, 
            '/robot_status', 
            self.nav_status_callback, 
            10, 
            callback_group=self.cb_group
        )


        # 2. Service Client
        self.cli = self.create_client(
            SetArea, 
            'set_area', 
            callback_group=self.cb_group
        )

        # 3. Main Logic Timer (10Hz)
        # 데이터 수신과 로직 처리를 분리하여 안전성 확보
        self.logic_timer = self.create_timer(
            0.1, 
            self.control_loop, 
            callback_group=self.cb_group
        )

        # 내부 상태 변수
        self.latest_plc_data = None
        self.latest_collision_msg = None
        self.latest_nav_status = None
        
        # 상태 머신 제어 변수
        self.current_phase = 0 
        # 0: Monitoring PLC (waiting for False)
        # 1: Waiting for PLC Reset (waiting for True)
        # 2: Monitoring Collision (waiting for Clear)

        # 시간 측정용 변수
        self.state_start_time = None
        self.service_future = None # 서비스 중복 호출 방지

        self.get_logger().info(f'Safety Node Started. PLC Wait: {self.N_sec}s, Col Clear: {self.M_sec}s')

    # =========================================
    # 1. Data Callbacks (데이터 갱신만 담당)
    # =========================================
    def plc_callback(self, msg):
        if (msg.protective_front is False) or (msg.protective_rear is False) :
            self.latest_plc_data = False
        else :
            self.latest_plc_data = True


    def collision_callback(self, msg):
        # CollisionDetectorState 메시지를 저장
        self.latest_collision_msg = msg

    def nav_status_callback(self, msg):
        self.get_logger().info(f'Received Nav Status: {msg.data}')
        self.latest_nav_status = msg.data

    # =========================================
    # 2. Main Control Loop (핵심 로직)
    # =========================================
    def control_loop(self):
        # 서비스 호출 중이면 로직 일시 중지 (순차 처리 보장)
        if self.service_future is not None:
            if self.service_future.done():
                self.handle_service_result(self.service_future)
                self.service_future = None
            else:
                return # 응답 대기 중


        # 네비게이션 상태에 따른 로직 분기
        if self.latest_nav_status is not None:
            if self.latest_nav_status in ['IDLE', 'SUCCEEDED', 'FAILED', 'CANCELED']:
                # 네비게이션이 비활성 상태일 때는 PLC 모니터링만 수행
                self.get_logger().info(f'Nav Status: {self.latest_nav_status}. Only monitoring PLC.')
                self.current_phase = 0 # PLC 모니터링 단계로 리셋
                self.state_start_time = None # 타이머 초기화
                return
        elif self.latest_nav_status is None:
            # 네비게이션 상태 정보가 없으면 로직 진행하지 않음
            self.get_logger().warn('Nav Status unknown. Waiting for status...')
            # return

        # -------------------------------------------------
        # Phase 0: PLC False 감지 (N초 유지)
        # -------------------------------------------------
        if self.current_phase == 0:
            if self.latest_plc_data is False:
                # 카운트 시작
                if self.state_start_time is None:
                    self.state_start_time = self.get_clock().now()
                    self.get_logger().info(f'[Phase 0] PLC is False. Timer started...')
                
                # 시간 체크
                elapsed = (self.get_clock().now() - self.state_start_time).nanoseconds / 1e9
                if elapsed >= self.N_sec:
                    self.get_logger().warn(f'[Phase 0] PLC False for {self.N_sec}s! Requesting Service TRUE.')
                    self.send_async_request(0) # Service Request True
                    self.current_phase = 1 # 다음 단계로
                    self.state_start_time = None # 타이머 초기화
            else:
                # True이거나 데이터 없으면 타이머 리셋
                if self.state_start_time is not None:
                    self.get_logger().info('[Phase 0] PLC returned to True (or init). Timer reset.')
                self.state_start_time = None

        # -------------------------------------------------
        # Phase 1: PLC가 True로 돌아왔는지 확인
        # -------------------------------------------------
        elif self.current_phase == 1:
            if self.latest_plc_data is True:
                self.get_logger().info('[Phase 1] PLC became True. Moving to Collision Monitoring.')
                self.current_phase = 2
                self.state_start_time = None

        # -------------------------------------------------
        # Phase 2: Collision Clear 감지 (M초 유지)
        # -------------------------------------------------
        elif self.current_phase == 2:
            is_safe = self.check_polygon_safety()

            if is_safe:
                # 안전 상태 유지 시간 측정
                if self.state_start_time is None:
                    self.state_start_time = self.get_clock().now()
                    self.get_logger().info(f'[Phase 2] Polygon Safe. Timer started...')
                
                elapsed = (self.get_clock().now() - self.state_start_time).nanoseconds / 1e9
                if elapsed >= self.M_sec:
                    self.get_logger().info(f'[Phase 2] Safety Cleared for {self.M_sec}s! Requesting Service FALSE.')
                    self.send_async_request(1) # Service Request False
                    self.current_phase = 0 # 처음으로 리셋
                    self.state_start_time = None
            else:
                # 위험 감지되면 타이머 리셋
                if self.state_start_time is not None:
                    self.get_logger().info('[Phase 2] Detection found! Timer reset.')
                self.state_start_time = None

    # =========================================
    # 3. Helper Functions
    # =========================================
    def check_polygon_safety(self) -> bool:
        """
        CollisionDetectorState 메시지를 파싱하여 특정 폴리곤의 안전 여부 반환
        return: True (Safe/False detection), False (Unsafe/True detection)
        """
        if self.latest_collision_msg is None:
            return False # 데이터 없으면 안전하지 않다고 간주

        # 실제 메시지 구조에 따른 파싱 로직
        # msg 구조: polygons=['A', 'B'], detections=[True, False]
        try:
            # *실제 사용 시 아래 주석 해제 및 msg 타입에 맞춰 수정*
            polygons = self.latest_collision_msg.polygons
            detections = self.latest_collision_msg.detections
            
            if self.target_polygon in polygons:
                idx = polygons.index(self.target_polygon)
                is_detected = detections[idx]
                return not is_detected # 감지가 안되어야(False) 안전(True)
            
            # [테스트용 더미 로직] 메시지 수신 시 무조건 안전하다고 가정
            # return True 

        except Exception as e:
            self.get_logger().error(f'Error parsing collision msg: {e}')
            return False
        
        return False

    def send_async_request(self, req_data: int):
        """
        비동기 서비스 요청
        """
        if not self.cli.wait_for_service(timeout_sec=3.5):
            self.get_logger().error('Service not available!')
            # 서비스가 없으면 상태를 롤백하거나 재시도 로직 필요 (여기선 생략)
            return

        req = SetArea.Request()
        req.input = req_data
        
        self.get_logger().info(f'Sending Service Request: {req_data}')
        self.service_future = self.cli.call_async(req)
        # 콜백은 control_loop에서 future.done()으로 확인합니다.

    def handle_service_result(self, future):
        """
        서비스 응답 처리
        """
        try:
            response = future.result()
            # self.get_logger().info(f'[Result] Service success: {response.success}, Msg: {response.message}')
            self.get_logger().info(f'[Result] Service success: {response.output}')
        except Exception as e:
            self.get_logger().error(f'[Result] Service call failed: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = NavSafetyManagerNode()
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()