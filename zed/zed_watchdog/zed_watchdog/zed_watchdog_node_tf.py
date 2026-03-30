import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo, PointCloud2, Image
from tf2_msgs.msg import TFMessage  # [NEW] /tf 메시지 임포트

import subprocess
import time
import sys
import psutil  # [NEW] 프로세스 정밀 제어용
from enum import Enum

# ================= State Definition =================
class State(Enum):
    WAITING_FOR_TF = -1  # [NEW] /tf 데이터 수신 대기 상태 추가
    IDLE = 0
    LAUNCHING = 1
    STABILIZING = 2
    RUNNING = 3
    COOLDOWN = 4
    FATAL_ERROR = 5
# ================================================

class ZedWatchdog(Node):
    def __init__(self):
        super().__init__('zed_watchdog')

        # --- Parameters ---
        self.declare_parameter('launch_cmd', ["ros2", "launch", "zed_multi_camera", "zed_multi_camera.launch.py"])
        self.declare_parameter('check_topics', [
            "/zed_multi/zed_front/point_cloud/cloud_registered",
            "/zed_multi/zed_rear/point_cloud/cloud_registered",
            # "/zed_node_2/left/camera_info",
            "/zed_multi/zed_right/left/gray/rect/image"
        ])

        self.declare_parameter('cam_serials', '[1234,2345,3455]')

        # [NEW] /tf 대기 타임아웃 (분 단위 설정)
        self.declare_parameter('tf_timeout_mins', 5.0) 

        self.target_topics_msgs = [PointCloud2, PointCloud2, Image]

        self.declare_parameter('boot_timeout', 60.0)
        self.declare_parameter('stability_duration', 7.0)
        self.declare_parameter('msg_timeout', 3.0)
        self.declare_parameter('cooldown_sec', 10.0)
        self.declare_parameter('max_attempts', 5)

        self.launch_cmd = self.get_parameter('launch_cmd').value
        self.target_topics = self.get_parameter('check_topics').value
        self.boot_timeout = self.get_parameter('boot_timeout').value
        self.stability_duration = self.get_parameter('stability_duration').value
        self.msg_timeout = self.get_parameter('msg_timeout').value
        self.cooldown_sec = self.get_parameter('cooldown_sec').value
        self.max_attempts = self.get_parameter('max_attempts').value
        
        # [NEW] 분을 초로 변환
        self.tf_timeout_sec = self.get_parameter('tf_timeout_mins').value * 60.0 

        self.log_intervals = 10.0
        
        # --- Variables ---
        self.process = None  # gnome-terminal 프로세스 핸들
        self.state = State.IDLE
        self.attempt_count = 0
        self.state_start_time = 0.0
        self.last_log_time = 0.0
        
        self.topic_last_seen = {topic: 0.0 for topic in self.target_topics}
        self.subs = []
        
        self.tf_received = False  # [NEW] /tf 수신 여부 플래그

        # --- Subscribers ---
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        cnt = 0
        for topic in self.target_topics:
            self.create_subscription(
                self.target_topics_msgs[cnt], topic,
                lambda msg, t=topic: self.topic_callback(msg, t), qos
            )
            cnt += 1

        # [NEW] /tf Subscriber 추가
        self.create_subscription(TFMessage, '/tf', self.tf_callback, qos)

        # --- Timer ---
        self.create_timer(0.1, self.fsm_loop)
        
        # 시작 전 잔여 프로세스 정리 후 /tf 대기 상태로 진입
        self.force_kill_zed_processes() 
        
        # [MODIFIED] COOLDOWN 대신 WAITING_FOR_TF 로 시작
        self.transition_to(State.WAITING_FOR_TF)
        
        self.get_logger().info("🐶 ZED Watchdog Started! Waiting for /tf data...")

    def tf_callback(self, msg):
        """[NEW] /tf 데이터를 수신하면 플래그를 True로 변경"""
        if not self.tf_received:
            self.tf_received = True

    def topic_callback(self, msg, topic_name):
        if msg.header.stamp.sec > 0:
            self.topic_last_seen[topic_name] = time.time()

    def fsm_loop(self):
        now = time.time()

        # 1. 프로세스 생존 확인 (사용자가 창을 닫았는지 확인)
        if self.state in [State.LAUNCHING, State.STABILIZING, State.RUNNING]:
            if self.process is None or self.process.poll() is not None:
                self.get_logger().error("🚨 Terminal window closed unexpectedly!")
                self.transition_to(State.COOLDOWN)
                return

        # 2. State Machine Logic
        # [NEW] /tf 수신 대기 상태 처리
        if self.state == State.WAITING_FOR_TF:
            if self.tf_received:
                self.get_logger().info("✅ /tf data detected! Proceeding to launch ZED...")
                self.transition_to(State.COOLDOWN)
                # COOLDOWN 시간을 스킵하고 1초 뒤에 바로 실행되도록 조작 (선택사항)
                self.state_start_time = now - self.cooldown_sec + 1.0 
            
            elif now - self.state_start_time > self.tf_timeout_sec:
                self.get_logger().fatal(f"❌ /tf Timeout! No tf data received for {self.tf_timeout_sec / 60.0} mins.")
                self.transition_to(State.FATAL_ERROR)
                return

        elif self.state == State.LAUNCHING:
            if now - self.state_start_time > self.boot_timeout:
                self.get_logger().error(f"❌ Boot Timeout ({self.boot_timeout}s).")
                self.transition_to(State.COOLDOWN)
                return
            
            all_started = all(t > 0.0 for t in self.topic_last_seen.values())
            if all_started:
                self.get_logger().info("⚡ Signals detected. Checking stability...")
                self.transition_to(State.STABILIZING)

        elif self.state == State.STABILIZING:
            is_healthy, bad_topic = self.check_topic_health()
            if not is_healthy:
                self.get_logger().warn(f"⚠️ Unstable: {bad_topic} stalled.")
                self.transition_to(State.COOLDOWN)
                return
            
            if now - self.state_start_time >= self.stability_duration:
                self.get_logger().info(f"✅ System Stable. Entering RUNNING mode.")
                self.transition_to(State.RUNNING)

        elif self.state == State.RUNNING:
            is_healthy, bad_topic = self.check_topic_health()
            if not is_healthy:
                self.get_logger().error(f"🚨 Runtime Failure: {bad_topic} stopped.")
                self.transition_to(State.COOLDOWN)
                return
            
            if now - self.last_log_time > self.log_intervals:
                self.get_logger().info("🟢 System Healthy (Window Open).")
                self.last_log_time = now

        elif self.state == State.COOLDOWN:
            if now - self.state_start_time > self.cooldown_sec:
                self.start_launch_sequence()

        elif self.state == State.FATAL_ERROR:
            pass

    # ... (start_launch_sequence, transition_to 등 나머지 함수는 기존과 동일) ...
    def start_launch_sequence(self):
        if self.attempt_count >= self.max_attempts:
            self.get_logger().fatal("💥 Max attempts reached.")
            self.transition_to(State.FATAL_ERROR)
            return

        self.attempt_count += 1
        self.get_logger().info(f"🚀 [Attempt {self.attempt_count}] Cleaning up & Launching...")
        
        self.force_kill_zed_processes()
        self.topic_last_seen = {topic: 0.0 for topic in self.target_topics}
        
        try:
            cam_serials_val = self.get_parameter('cam_serials').value
            dynamic_args = [f"cam_serials:={cam_serials_val}"]
            
            full_cmd = ["gnome-terminal", "--wait", "--"] + self.launch_cmd + dynamic_args
            
            self.process = subprocess.Popen(full_cmd)
            self.transition_to(State.LAUNCHING)
        except Exception as e:
            self.get_logger().error(f"Failed to open terminal: {e}")
            self.transition_to(State.COOLDOWN)

    def transition_to(self, new_state):
        self.state = new_state
        self.state_start_time = time.time()
        
        if new_state == State.COOLDOWN:
            self.cleanup_and_close_window()
            self.get_logger().info(f"⏳ Cooldown {self.cooldown_sec}s...")

    def check_topic_health(self):
        now = time.time()
        for topic, last_seen in self.topic_last_seen.items():
            if now - last_seen > self.msg_timeout:
                return False, topic
        return True, None

    def force_kill_zed_processes(self):
        target_names = ["zed_left_main", "zed_right_main", "zed_rear_main", "zed_front_main"]
        killed_count = 0
        
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                proc_name = proc.info['name']
                cmdline = proc.info['cmdline'] or []
                cmd_str = " ".join(cmdline)

                if "gnome-terminal" in proc_name:
                    continue

                if proc.pid == 0 or proc.pid == self.process_id(): 
                    continue

                is_target = False
                
                if any(target in proc_name for target in target_names):
                    is_target = True
                
                if not is_target:
                    if "zed_multi_camera" in cmd_str and "launch.py" in cmd_str:
                        if not any(safe in cmd_str for safe in ["vim", "nano", "code", "grep"]):
                            is_target = True

                if is_target:
                    self.get_logger().warn(f"🔪 Killing process: {proc_name} (PID: {proc.info['pid']})")
                    proc.kill()
                    killed_count += 1
            
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                pass
        
        if killed_count > 0:
            time.sleep(1.0)

    def cleanup_and_close_window(self):
        self.get_logger().info("🧹 Cleaning up processes and closing window...")
        self.force_kill_zed_processes()

        if self.process:
            if self.process.poll() is None:
                self.get_logger().info("🛑 Terminating gnome-terminal window...")
                self.process.terminate()
                try:
                    self.process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    self.process.kill()
            self.process = None

    def process_id(self):
        import os
        return os.getpid()

    def destroy_node(self):
        self.cleanup_and_close_window()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ZedWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()