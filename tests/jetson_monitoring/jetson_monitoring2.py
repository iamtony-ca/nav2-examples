import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import String
import json
import time

class ZedMonitor(Node):
    def __init__(self):
        super().__init__('zed_status_monitor')

        # 1. 설정
        # ZED Wrapper의 진단 메시지 이름 (보통 'zed_node: ZED Diagnostic' 형태임)
        # ros2 topic echo /diagnostics 로 확인 가능
        self.target_node_name = "zed_node" 
        
        # 2. Watchdog 설정 (Heartbeat)
        self.last_heartbeat_time = 0.0
        self.timeout_sec = 2.0  # 2초 동안 소식 없으면 사망 판정
        self.is_connected = False

        # 3. 데이터 저장 변수
        self.zed_stats = {
            "fps": 0.0,
            "temp_left": 0.0,
            "temp_right": 0.0,
            "camera_model": "Unknown",
            "sn": "Unknown"
        }

        # 4. Subscriber & Publisher
        # ZED가 발행하는 진단 정보를 수신
        self.sub_diag = self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.diag_callback,
            10
        )
        
        # 결과를 보기 좋게 JSON으로 발행
        self.pub_status = self.create_publisher(String, 'zed_monitor_status', 10)
        
        # 5. 주기적 감시 타이머 (1Hz)
        self.timer = self.create_timer(1.0, self.watchdog_callback)

        self.get_logger().info("ZED Monitor Started via /diagnostics")

    def diag_callback(self, msg):
        """
        /diagnostics 토픽이 들어올 때마다 실행.
        ZED 노드가 보내온 정보인지 확인하고 파싱함.
        """
        for status in msg.status:
            # ZED 노드에서 온 메시지인지 이름으로 필터링
            if self.target_node_name in status.name:
                self.last_heartbeat_time = time.time()
                self.is_connected = True
                
                # Key-Value 파싱
                # ZED Wrapper 버전에 따라 Key 이름이 다를 수 있으니 확인 필요
                # 보통: 'Input FPS', 'Left CMOS Temp', 'Right CMOS Temp' 등
                for kv in status.values:
                    if 'FPS' in kv.key: # Input FPS or Camera FPS
                        try: self.zed_stats['fps'] = float(kv.value)
                        except: pass
                    
                    elif 'Temp' in kv.key and 'Left' in kv.key:
                        try: self.zed_stats['temp_left'] = float(kv.value)
                        except: pass

                    elif 'Temp' in kv.key and 'Right' in kv.key:
                        try: self.zed_stats['temp_right'] = float(kv.value)
                        except: pass
                        
                    elif 'Model' in kv.key:
                        self.zed_stats['camera_model'] = kv.value
                        
                    elif 'Serial' in kv.key:
                        self.zed_stats['sn'] = kv.value

    def watchdog_callback(self):
        """
        주기적으로 실행되어 '마지막으로 메시지 받은 시간'을 체크함.
        """
        current_time = time.time()
        time_diff = current_time - self.last_heartbeat_time

        # 타임아웃 체크 (Heartbeat Logic)
        if time_diff > self.timeout_sec:
            self.is_connected = False
            status_msg = "DISCONNECTED / DEAD"
            # 연결 끊기면 수치 0으로 초기화 (선택사항)
            self.zed_stats['fps'] = 0.0
        else:
            status_msg = "ALIVE"

        # 최종 리포트 생성
        report = {
            "status": {
                "alive": self.is_connected,
                "msg": status_msg,
                "last_heartbeat_sec_ago": round(time_diff, 1)
            },
            "camera": {
                "fps": self.zed_stats['fps'],
                "temp": {
                    "left": self.zed_stats['temp_left'],
                    "right": self.zed_stats['temp_right']
                },
                "info": {
                    "model": self.zed_stats['camera_model'],
                    "sn": self.zed_stats['sn']
                }
            }
        }

        # JSON 발행
        msg = String()
        msg.data = json.dumps(report)
        self.pub_status.publish(msg)
        
        # 로그 (디버깅용)
        # if not self.is_connected:
        #     self.get_logger().warn("ZED Node is NOT responding!")

def main(args=None):
    rclpy.init(args=args)
    node = ZedMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
