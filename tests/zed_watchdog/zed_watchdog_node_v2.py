import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo

import subprocess
import time
import sys
import psutil  # [NEW] 프로세스 정밀 제어용
from enum import Enum

# ================= State Definition =================
class State(Enum):
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
            "/zed_node_0/left/camera_info",
            "/zed_node_1/left/camera_info",
            "/zed_node_2/left/camera_info",
            "/zed_node_3/left/camera_info"
        ])
        self.declare_parameter('boot_timeout', 60.0)
        self.declare_parameter('stability_duration', 5.0)
        self.declare_parameter('msg_timeout', 1.0)
        self.declare_parameter('cooldown_sec', 10.0)
        self.declare_parameter('max_attempts', 3)

        self.launch_cmd = self.get_parameter('launch_cmd').value
        self.target_topics = self.get_parameter('check_topics').value
        self.boot_timeout = self.get_parameter('boot_timeout').value
        self.stability_duration = self.get_parameter('stability_duration').value
        self.msg_timeout = self.get_parameter('msg_timeout').value
        self.cooldown_sec = self.get_parameter('cooldown_sec').value
        self.max_attempts = self.get_parameter('max_attempts').value

        # --- Variables ---
        self.process = None  # gnome-terminal 프로세스 핸들
        self.state = State.IDLE
        self.attempt_count = 0
        self.state_start_time = 0.0
        self.last_log_time = 0.0
        
        self.topic_last_seen = {topic: 0.0 for topic in self.target_topics}
        self.subs = []

        # --- Subscribers ---
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        for topic in self.target_topics:
            self.create_subscription(
                CameraInfo, topic,
                lambda msg, t=topic: self.topic_callback(msg, t), qos
            )

        # --- Timer ---
        self.create_timer(0.1, self.fsm_loop)
        
        # 시작 전 혹시 모를 잔여 프로세스 정리 및 쿨다운 진입
        self.force_kill_zed_processes() 
        self.transition_to(State.COOLDOWN)
        self.state_start_time = time.time() - self.cooldown_sec + 1.0

        self.get_logger().info("🐶 ZED Watchdog with psutil Started!")

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
        if self.state == State.LAUNCHING:
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
            
            if now - self.last_log_time > 5.0:
                self.get_logger().info("🟢 System Healthy (Window Open).")
                self.last_log_time = now

        elif self.state == State.COOLDOWN:
            if now - self.state_start_time > self.cooldown_sec:
                self.start_launch_sequence()

        elif self.state == State.FATAL_ERROR:
            pass

    def start_launch_sequence(self):
        if self.attempt_count >= self.max_attempts:
            self.get_logger().fatal("💥 Max attempts reached.")
            self.transition_to(State.FATAL_ERROR)
            return

        self.attempt_count += 1
        self.get_logger().info(f"🚀 [Attempt {self.attempt_count}] Cleaning up & Launching...")
        
        # [핵심 요구사항] 실행 전 잔여 프로세스 검사 및 종료
        self.force_kill_zed_processes()

        self.topic_last_seen = {topic: 0.0 for topic in self.target_topics}
        
        try:
            # gnome-terminal --wait 실행
            full_cmd = ["gnome-terminal", "--wait", "--"] + self.launch_cmd
            self.process = subprocess.Popen(full_cmd)
            self.transition_to(State.LAUNCHING)
        except Exception as e:
            self.get_logger().error(f"Failed to open terminal: {e}")
            self.transition_to(State.COOLDOWN)

    def transition_to(self, new_state):
        self.state = new_state
        self.state_start_time = time.time()
        
        if new_state == State.COOLDOWN:
            # 상태 전환 시에도 확실한 정리를 수행
            self.cleanup_and_close_window()
            self.get_logger().info(f"⏳ Cooldown {self.cooldown_sec}s...")

    def check_topic_health(self):
        now = time.time()
        for topic, last_seen in self.topic_last_seen.items():
            if now - last_seen > self.msg_timeout:
                return False, topic
        return True, None

    def force_kill_zed_processes(self):
        """
        [핵심 요구사항] psutil을 사용하여 ZED 관련 모든 프로세스를 PID 기반으로 종료
        """
        target_names = ["zed_wrapper_node", "zed_multi_camera", "component_container"]
        killed_count = 0
        
        # 모든 실행 중인 프로세스 순회
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                # 1. 프로세스 이름 확인
                proc_name = proc.info['name']
                cmdline = proc.info['cmdline'] or []
                
                is_target = False
                
                # 실행 파일 이름 매칭
                if any(target in proc_name for target in target_names):
                    is_target = True
                
                # 커맨드라인 매칭 (launch.py 등 파이썬 프로세스 잡기 위함)
                if not is_target:
                    cmd_str = " ".join(cmdline)
                    if "zed_multi_camera" in cmd_str and "launch.py" in cmd_str:
                        is_target = True

                # [주의] watchdog 자신은 죽이면 안 됨
                if proc.pid == 0 or proc.pid == self.process_id(): 
                    continue

                if is_target:
                    self.get_logger().warn(f"🔪 Killing orphan process: {proc_name} (PID: {proc.info['pid']})")
                    proc.kill() # 강제 종료
                    killed_count += 1
            
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                pass
        
        if killed_count > 0:
            # 프로세스가 완전히 죽을 때까지 잠시 대기
            time.sleep(1.0)

    def cleanup_and_close_window(self):
        """터미널 창과 내부 프로세스를 모두 정리"""
        self.get_logger().info("🧹 Cleaning up processes and closing window...")

        # 1. 내부 ZED 노드들 먼저 psutil로 사살 (가장 중요)
        self.force_kill_zed_processes()

        # 2. 터미널 창(gnome-terminal) 핸들 종료
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
        """현재 Watchdog 노드의 PID 반환"""
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