import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import psutil
import subprocess
import time
from collections import deque
from jtop import jtop

class RobustMonitorV2(Node):
    def __init__(self):
        super().__init__('robust_system_monitor_v2')
        
        # 설정
        self.declare_parameter('target_ip', '192.168.0.1')
        self.target_ip = self.get_parameter('target_ip').value
        
        # 1. 큐 및 Jtop 초기화
        self.history_len = 60
        self.cpu_q = deque(maxlen=self.history_len)
        self.gpu_q = deque(maxlen=self.history_len)
        
        # Jtop 시작
        try:
            self.jetson = jtop()
            self.jetson.start()
        except Exception as e:
            self.get_logger().error(f"Jtop Start Fail: {e}")
            self.jetson = None

        # 2. 공유 변수 (Slow Timer가 업데이트하고, Fast Timer가 읽음)
        self.latest_ping = -1.0
        self.latest_sync_offset = 0.0
        self.latest_net_io = {}
        
        # 네트워크 계산용
        self.prev_net = psutil.net_io_counters()
        self.prev_time = time.time()

        # 3. 타이머 분리 (핵심!)
        
        # [Fast Timer] 0.1초 (10Hz) - CPU/GPU/Publish 담당
        self.create_timer(0.1, self.update_fast_stats)
        
        # [Slow Timer] 1.0초 (1Hz) - Ping/Network 담당 (블로킹 되어도 상관없는 주기)
        self.create_timer(1.0, self.update_slow_stats)

        self.get_logger().info("Monitor V2 Started: Fast(10Hz) & Slow(1Hz) Loops")

    def _get_latency_and_sync(self):
        """느린 작업들 (Ping, Chrony)"""
        ping_ms = -1.0
        sync_offset_ms = 0.0

        # Ping (-W 1 때문에 최대 1초 걸림)
        try:
            cmd = ['ping', '-c', '1', '-W', '1', self.target_ip]
            res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            if res.returncode == 0:
                start = res.stdout.find('time=')
                if start != -1:
                    end = res.stdout.find(' ms', start)
                    ping_ms = float(res.stdout[start+5:end])
        except Exception:
            pass

        # Chrony
        try:
            cmd = ['chronyc', 'tracking']
            res = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
            if res.returncode == 0:
                for line in res.stdout.splitlines():
                    if "Last offset" in line:
                        parts = line.split(':')
                        if len(parts) > 1:
                            seconds = float(parts[1].replace('seconds', '').strip())
                            sync_offset_ms = seconds * 1000.0
                        break
        except Exception:
            pass

        return ping_ms, sync_offset_ms

    def update_slow_stats(self):
        """1초마다 실행: 무거운 작업을 수행하고 공유 변수 업데이트"""
        # 1. Ping & Sync
        p, s = self._get_latency_and_sync()
        self.latest_ping = p
        self.latest_sync_offset = s

        # 2. Network Bandwidth (1초 간격 계산이 가장 정확함)
        try:
            curr_net = psutil.net_io_counters()
            curr_time = time.time()
            dt = curr_time - self.prev_time
            if dt <= 0: dt = 1.0

            sent_bps = (curr_net.bytes_sent - self.prev_net.bytes_sent) / dt
            recv_bps = (curr_net.bytes_recv - self.prev_net.bytes_recv) / dt

            self.prev_net = curr_net
            self.prev_time = curr_time

            self.latest_net_io = {
                "tx_mbps": round(sent_bps * 8 / 1_000_000, 2),
                "rx_mbps": round(recv_bps * 8 / 1_000_000, 2),
                "tx_total_mb": round(curr_net.bytes_sent / 1024 / 1024, 1),
                "rx_total_mb": round(curr_net.bytes_recv / 1024 / 1024, 1)
            }
        except Exception:
            pass

    def update_fast_stats(self):
        """0.1초마다 실행: 빠른 작업 수행 및 Publish"""
        # 1. CPU & RAM (Fast)
        cpu_cur = psutil.cpu_percent(interval=None) # Non-blocking
        self.cpu_q.append(cpu_cur)
        mem = psutil.virtual_memory()

        # 2. GPU & Power (Fast)
        gpu_cur = 0
        gpu_temp = 0
        gpu_mem = 0
        power_w = 0
        
        if self.jetson and self.jetson.ok():
            try:
                gpu_cur = self.jetson.stats['GPU']
                self.gpu_q.append(gpu_cur)
                gpu_temp = self.jetson.stats['Temp'].get('GPU', 0)
                gpu_mem = self.jetson.stats['RAM'].get('shared', 0) / 1024
                power_w = self.jetson.stats['Power'].get('avg', 0) / 1000
            except: pass

        # 통계 계산
        cpu_avg = sum(self.cpu_q) / len(self.cpu_q) if self.cpu_q else 0
        cpu_max = max(self.cpu_q) if self.cpu_q else 0
        gpu_avg = sum(self.gpu_q) / len(self.gpu_q) if self.gpu_q else 0
        gpu_max = max(self.gpu_q) if self.gpu_q else 0

        # 3. 메시지 생성 (느린 데이터는 최신값(latest)을 가져다 씀)
        status_data = {
            "timestamp": time.time(),
            "cpu": {
                "usage_percent": cpu_cur,
                "avg": round(cpu_avg, 1),
                "max": round(cpu_max, 1),
                "mem_usage_mb": round(mem.used / 1024 / 1024, 1),
            },
            "gpu": {
                "usage_percent": int(gpu_cur),
                "avg": round(gpu_avg, 1),
                "max": int(gpu_max),
                "temp_c": int(gpu_temp),
                "mem_usage_mb": int(gpu_mem)
            },
            "network": {
                "io": self.latest_net_io,       # Slow Loop에서 업데이트된 값
                "latency_ping_ms": self.latest_ping # Slow Loop에서 업데이트된 값
            },
            "system": {
                "time_offset_ms": round(self.latest_sync_offset, 4), # Slow Loop에서 업데이트된 값
                "power_w": round(power_w, 2)
            }
        }
        
        # Publish (10Hz)
        msg = String()
        msg.data = json.dumps(status_data)
        self.publisher_.publish(msg) # self.pub 대신 self.publisher_ 등 초기화 변수명에 맞게 사용

    def destroy_node(self):
        if self.jetson: self.jetson.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    # create_publisher 변수명만 주의해서 사용하면 됩니다. (위 코드에선 생략됨, init에 추가 필요)
    node = RobustMonitorV2()
    # publisher 초기화 누락 방지용 
    node.publisher_ = node.create_publisher(String, 'system_status', 10)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
