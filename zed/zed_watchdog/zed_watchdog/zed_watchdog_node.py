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

class ZedWatchdogNode(Node):
    def __init__(self):
        super().__init__('zed_watchdog_node')

        # --- Parameters ---
        self.declare_parameter('launch_cmd', ["ros2", "launch", "zed_multi_camera", "zed_multi_camera_delay_separate_three_cams.launch.py"])
        self.declare_parameter('check_topics', [
            "/zed_multi/zed_front/point_cloud/cloud_registered",
            "/zed_multi/zed_rear/point_cloud/cloud_registered",
            # "/zed_node_2/left/camera_info",
            "/zed_multi/zed_right/left/gray/rect/image"
        ])

        self.declare_parameter('cam_serials', '[1234,2345,3455]')

        # [NEW] /tf 대기 타임아웃 (분 단위 설정)
        self.declare_parameter('tf_timeout_mins', 5.0) 

        self.declare_parameter('boot_timeout', 60.0)
        self.declare_parameter('stability_duration', 7.0)
        self.declare_parameter('msg_timeout', 3.0)
        self.declare_parameter('cooldown_sec', 10.0)
        self.declare_parameter('max_attempts', 5)
        # [WD4] RUNNING 상태가 이 시간(초) 동안 끊김 없이 유지되면 재시도 횟수를 0으로 초기화 (0 = 초기화 안 함)
        self.declare_parameter('attempt_reset_sec', 600.0)
        # [WD4] 재시도 횟수 초과(FATAL) 후 이 시간(초)마다 1회씩 다시 시도 (0 = 예전처럼 FATAL 에서 멈춤)
        self.declare_parameter('fatal_retry_sec', 300.0)

        self.launch_cmd = self.get_parameter('launch_cmd').value
        self.target_topics = self.get_parameter('check_topics').value
        self.boot_timeout = self.get_parameter('boot_timeout').value
        self.stability_duration = self.get_parameter('stability_duration').value
        self.msg_timeout = self.get_parameter('msg_timeout').value
        self.cooldown_sec = self.get_parameter('cooldown_sec').value
        self.max_attempts = self.get_parameter('max_attempts').value
        self.attempt_reset_sec = float(self.get_parameter('attempt_reset_sec').value)
        self.fatal_retry_sec = float(self.get_parameter('fatal_retry_sec').value)

        # [WD4] 감시 토픽의 메시지 타입을 토픽 이름으로 결정 (토픽 개수 제한 없음; 예전에는 3개 고정 목록)
        self.target_topics_msgs = [self.msg_type_for(t) for t in self.target_topics]

        # [NEW] 분을 초로 변환
        self.tf_timeout_sec = self.get_parameter('tf_timeout_mins').value * 60.0 

        self.log_intervals = 10.0
        
        # --- Variables ---
        self.process = None  # gnome-terminal 프로세스 핸들
        self.state = State.IDLE
        self.attempt_count = 0
        self.fatal_reason = None  # [WD4] 'max_attempts' 일 때만 fatal_retry_sec 마다 다시 시도
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
        self.get_logger().info(
            f"[WD4] watching {len(self.target_topics)} topics: "
            + ", ".join(f"{t} ({m.__name__})" for t, m in zip(self.target_topics, self.target_topics_msgs))
            + f"; max_attempts={self.max_attempts}, attempt_reset_sec={self.attempt_reset_sec:g}, "
              f"fatal_retry_sec={self.fatal_retry_sec:g}")

    @staticmethod
    def msg_type_for(topic):
        """[WD4] 토픽 이름 -> 메시지 타입. 알 수 없는 이름이면 시작하지 않고 오류를 냅니다."""
        name = topic.rstrip('/')
        last = name.rsplit('/', 1)[-1]
        if '/point_cloud/' in name or last == 'cloud_registered':
            return PointCloud2
        if last == 'camera_info':
            return CameraInfo
        if last == 'image' or last.startswith('image_'):
            return Image
        raise ValueError(f"check_topics: cannot tell the message type of '{topic}' "
                         "(expected .../point_cloud/..., .../camera_info or .../image)")

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
            self.get_logger().warn("self.state == State.WAITING_FOR_TF:...", throttle_duration_sec=3.0)
            if self.tf_received:
                self.get_logger().info("✅ /tf data detected! Proceeding to launch ZED...")
                self.transition_to(State.COOLDOWN)
                # COOLDOWN 시간을 스킵하고 1초 뒤에 바로 실행되도록 조작 (선택사항)
                self.state_start_time = now - self.cooldown_sec + 1.0 
            
            elif now - self.state_start_time > self.tf_timeout_sec:
                self.get_logger().fatal(f"❌ /tf Timeout! No tf data received for {self.tf_timeout_sec / 60.0} mins.")
                self.fatal_reason = 'tf_timeout'
                self.transition_to(State.FATAL_ERROR)
                return

        elif self.state == State.LAUNCHING:
            # self.get_logger().warn("self.state == State.LAUNCHING::...")
            
            if now - self.state_start_time > self.boot_timeout:
                self.get_logger().error(f"❌ Boot Timeout ({self.boot_timeout}s).")
                self.transition_to(State.COOLDOWN)
                return
            
            all_started = all(t > 0.0 for t in self.topic_last_seen.values())
            if all_started:
                self.get_logger().info("⚡ Signals detected. Checking stability...")
                self.transition_to(State.STABILIZING)

        elif self.state == State.STABILIZING:
            # self.get_logger().warn("self.state == State.STABILIZING:")

            is_healthy, bad_topic = self.check_topic_health()
            if not is_healthy:
                self.get_logger().warn(f"⚠️ Unstable: {bad_topic} stalled.")
                self.transition_to(State.COOLDOWN)
                return
            
            if now - self.state_start_time >= self.stability_duration:
                self.get_logger().info(f"✅ System Stable. Entering RUNNING mode.")
                self.transition_to(State.RUNNING)

        elif self.state == State.RUNNING:
            # self.get_logger().warn("self.state == State.RUNNING:")

            is_healthy, bad_topic = self.check_topic_health()
            if not is_healthy:
                self.get_logger().error(f"🚨 Runtime Failure: {bad_topic} stopped.")
                self.transition_to(State.COOLDOWN)
                return
            
            # [WD4] 일정 시간 안정적으로 동작하면 재시도 횟수 초기화
            if (self.attempt_reset_sec > 0 and self.attempt_count > 0
                    and now - self.state_start_time >= self.attempt_reset_sec):
                self.get_logger().info(
                    f"♻️ [WD4] Stable for {self.attempt_reset_sec:g}s: attempt counter reset "
                    f"({self.attempt_count} -> 0).")
                self.attempt_count = 0

            if now - self.last_log_time > self.log_intervals:
                self.get_logger().info("🟢 System Healthy (Window Open).")
                self.last_log_time = now

        elif self.state == State.COOLDOWN:
            # self.get_logger().warn("self.state == State.COOLDOWN:")

            if now - self.state_start_time > self.cooldown_sec:
                self.start_launch_sequence()

        elif self.state == State.FATAL_ERROR:
            # [WD4] 재시도 횟수 초과로 멈춘 경우 fatal_retry_sec 마다 1회씩 다시 시도
            if self.fatal_reason == 'max_attempts' and self.fatal_retry_sec > 0:
                if now - self.state_start_time >= self.fatal_retry_sec:
                    self.get_logger().warn(
                        f"🔁 [WD4] FATAL for {self.fatal_retry_sec:g}s: trying one more launch.")
                    self.fatal_reason = None
                    self.attempt_count = max(0, self.max_attempts - 1)
                    self.start_launch_sequence()
                else:
                    self.get_logger().warn(
                        f"💤 [WD4] FATAL (max attempts); next retry in "
                        f"{self.fatal_retry_sec - (now - self.state_start_time):.0f}s",
                        throttle_duration_sec=60.0)

    # ... (start_launch_sequence, transition_to 등 나머지 함수는 기존과 동일) ...
    def start_launch_sequence(self):
        if self.attempt_count >= self.max_attempts:
            self.get_logger().fatal("💥 Max attempts reached.")
            self.fatal_reason = 'max_attempts'
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
    node = ZedWatchdogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
