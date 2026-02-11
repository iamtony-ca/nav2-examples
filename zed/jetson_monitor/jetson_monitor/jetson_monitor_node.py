import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import psutil
import subprocess
import time
from collections import deque
from jtop import jtop

class JetsonMonitorNode(Node):
    def __init__(self):
        super().__init__('jetson_monitor_node')
        
        # 1. 설정
        self.declare_parameter('target_ip', '192.168.0.1') # IPC 혹은 공유기 IP
        self.target_ip = self.get_parameter('target_ip').value
        self.interval = 0.1 # 1초 주기
        self.interval_network = 0.1
        self.net_metrics = 0.0
        self.ping = 0.0
        self.time_offset = 0.0


        # 2. 데이터 저장을 위한 큐 (최근 60초 데이터 보관 -> Avg/Max 계산용)
        # deque를 쓰면 maxlen 넘어가면 알아서 오래된 것 버림 (메모리 안전)
        self.history_len = 60
        self.cpu_q = deque(maxlen=self.history_len)
        self.gpu_q = deque(maxlen=self.history_len)
        
        # 3. 네트워크 속도 계산용 이전 상태
        self.prev_net = psutil.net_io_counters()
        self.prev_time = time.time()

        # 4. Jtop (GPU 모니터링) 안전하게 시작
        try:
            self.jetson = jtop()
            self.jetson.start()
            if not self.jetson.ok():
                self.get_logger().error("Jtop Service is NOT running. GPU stats will be empty.")
        except Exception as e:
            self.get_logger().error(f"Failed to start Jtop: {e}")
            self.jetson = None

        # 5. Timer 및 Publisher
        self.pub = self.create_publisher(String, 'system_status', 10)
        self.timer = self.create_timer(self.interval, self.update_stats)
        self.network_timer = self.create_timer(self.interval_network, self.update_network_stats)

        self.get_logger().info("Robust System Monitor Started.")

    def _get_network_metrics(self):
        """psutil을 이용한 네트워크 대역폭 계산 (가장 정확함)"""
        try:
            curr_net = psutil.net_io_counters()
            curr_time = time.time()
            dt = curr_time - self.prev_time
            
            if dt <= 0: dt = 1.0 # 0으로 나누기 방지

            sent_bps = (curr_net.bytes_sent - self.prev_net.bytes_sent) / dt
            recv_bps = (curr_net.bytes_recv - self.prev_net.bytes_recv) / dt

            self.prev_net = curr_net
            self.prev_time = curr_time

            return {
                "tx_mbps": round(sent_bps * 8 / 1_000_000, 2), # Mbps 변환
                "rx_mbps": round(recv_bps * 8 / 1_000_000, 2),
                "tx_total_mb": round(curr_net.bytes_sent / 1024 / 1024, 1),
                "rx_total_mb": round(curr_net.bytes_recv / 1024 / 1024, 1)
            }
        except Exception as e:
            self.get_logger().warn(f"Net Error: {e}")
            return {}

    def _get_latency_and_sync(self):
        """Subprocess를 사용하되 Timeout을 걸어 안전하게 측정"""
        ping_ms = -1.0
        sync_offset_ms = 0.0

        # 1. Network Latency (Ping)
        try:
            # 타임아웃 0.5초로 매우 짧게 설정하여 ROS 루프 지연 방지
            cmd = ['ping', '-c', '1', '-W', '1', self.target_ip]
            res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=0.5)
            if res.returncode == 0:
                # time=12.3 ms 파싱
                start = res.stdout.find('time=')
                if start != -1:
                    end = res.stdout.find(' ms', start)
                    ping_ms = float(res.stdout[start+5:end])
        except Exception:
            ping_ms = -1.0 # 실패 시 -1 표기

        # 2. Time Sync (Chrony)
        try:
            cmd = ['chronyc', 'tracking']
            res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=0.5)
            if res.returncode == 0:
                for line in res.stdout.splitlines():
                    if "Last offset" in line:
                        # "+0.000012 seconds" -> ms 변환
                        parts = line.split(':')
                        if len(parts) > 1:
                            seconds = float(parts[1].replace('seconds', '').strip())
                            sync_offset_ms = seconds * 1000.0
                        break
        except Exception:
            sync_offset_ms = 9999.9 # 실패 시 큰 값 혹은 식별값

        return ping_ms, sync_offset_ms

    def update_network_stats(self):
        # 3. Network & Sync
        self.net_metrics = self._get_network_metrics()
        self.ping, self.time_offset = self._get_latency_and_sync()



    def update_stats(self):
        # 1. CPU & RAM (psutil 사용 - 매우 안정적)
        cpu_cur = psutil.cpu_percent(interval=None) # interval=None은 non-blocking
        self.cpu_q.append(cpu_cur)
        
        mem = psutil.virtual_memory() # 전체 시스템 메모리
        
        # 2. GPU & Power (jtop 사용 - Jetson 특화)
        gpu_cur = 0
        gpu_temp = 0
        gpu_mem = 0
        power_w = 0
        
        if self.jetson and self.jetson.ok():
            try:
                # GPU 로드율
                gpu_cur = self.jetson.stats['GPU']
                self.gpu_q.append(gpu_cur)
                
                # 온도 및 메모리
                gpu_temp = self.jetson.stats['Temp gpu']
                cpu_temp = self.jetson.stats['Temp cpu']
                # jtop의 RAM['shared']가 GPU 사용량과 유사 (Unified Memory)
                gpu_mem = self.jetson.stats['RAM'] / 1024 # MB
                
                # fan
                fan_rpm = self.jetson.stats['Fan pwmfan0']

                # 전력
                power_w = self.jetson.stats['Power TOT'] / 1000 # Watt
            except KeyError:
                pass # 특정 필드가 없어도 죽지 않음

        # # 3. Network & Sync
        # self.net_metrics = self._get_network_metrics()
        # self.ping, self.time_offset = self._get_latency_and_sync()

        # 4. 통계 계산 (Safe division)
        cpu_avg = sum(self.cpu_q) / len(self.cpu_q) if self.cpu_q else 0
        cpu_max = max(self.cpu_q) if self.cpu_q else 0
        
        gpu_avg = sum(self.gpu_q) / len(self.gpu_q) if self.gpu_q else 0
        gpu_max = max(self.gpu_q) if self.gpu_q else 0

        # 5. 최종 메시지 생성
        status_data = {
            "timestamp": time.time(),
            "cpu": {
                "usage_percent": cpu_cur,
                "avg_60s": round(cpu_avg, 1),
                "max_60s": round(cpu_max, 1),
                "mem_usage_mb": round(mem.used / 1024 / 1024, 1),
                "mem_percent": mem.percent,
                "temp_cpu": int(cpu_temp)
            },
            "gpu": {
                "usage_percent": int(gpu_cur),
                "avg_60s": round(gpu_avg, 1),
                "max_60s": int(gpu_max),
                "temp_gpu": int(gpu_temp),
                "mem_usage_mb": int(gpu_mem)
            },
            "network": {
                "io": self.net_metrics,
                "latency_ping_ms": self.ping,
            },
            "system": {
                "pwmfan0": round(fan_rpm, 1),
                "time_offset_ms": round(self.time_offset, 4),
                "power_w": round(power_w, 2)
            }
        }

        # JSON Publish
        msg = String()
        msg.data = json.dumps(status_data)
        self.pub.publish(msg)

    def destroy_node(self):
        if self.jetson:
            self.jetson.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = JetsonMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    