import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo

import subprocess
import time
import sys
import os
import psutil
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

        # --- Parameters Setting ---
        self.declare_parameter('launch_cmd', ["ros2", "launch", "zed_multi_camera", "zed_multi_camera.launch.py"])
        self.declare_parameter('check_topics', [
            "/zed_node_0/left/camera_info",
            "/zed_node_1/left/camera_info",
            "/zed_node_2/left/camera_info",
            "/zed_node_3/left/camera_info"
        ])
        self.declare_parameter('boot_timeout', 60.0)    # 부팅 허용 시간
        self.declare_parameter('stability_duration', 5.0) # 안정화 검사 시간
        self.declare_parameter('msg_timeout', 1.0)      # 데이터 끊김 허용 시간
        self.declare_parameter('cooldown_sec', 10.0)    # 재시작 대기 시간
        self.declare_parameter('max_attempts', 5)       # 최대 시도 횟수

        # 파라미터 로드
        self.launch_cmd = self.get_parameter('launch_cmd').value
        self.target_topics = self.get_parameter('check_topics').value
        self.boot_timeout = self.get_parameter('boot_timeout').value
        self.stability_duration = self.get_parameter('stability_duration').value
        self.msg_timeout = self.get_parameter('msg_timeout').value
        self.cooldown_sec = self.get_parameter('cooldown_sec').value
        self.max_attempts = self.get_parameter('max_attempts').value

        # --- Internal Variables ---
        self.process = None  # gnome-terminal subprocess 핸들
        self.state = State.IDLE
        self.attempt_count = 0
        self.state_start_time = 0.0
        self.last_log_time = 0.0
        self.my_pid = os.getpid()
        
        # 토픽별 마지막 수신 시간 저장용
        self.topic_last_seen = {topic: 0.0 for topic in self.target_topics}
        self.subs = []

        # --- Subscribers Setup ---
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

        # --- Timer Loop (10Hz) ---
        self.create_timer(0.1, self.fsm_loop)
        
        # [초기화] 시작 전 혹시 모를 좀비 프로세스 정리 후 쿨다운 진입
        self.force_kill_zed_processes() 
        self.transition_to(State.COOLDOWN)
        
        # 첫 실행은 쿨다운 없이 1초 뒤 바로 시작하도록 시간 조작
        self.state_start_time = time.time() - self.cooldown_sec + 1.0

        self.get_logger().info(f"🐶 ZED Watchdog Node Started! (PID: {self.my_pid})")

    def topic_callback(self, msg, topic_name):
        # 유효한 타임스탬프가 있는 경우에만 갱신
        if msg.header.stamp.sec > 0:
            self.topic_last_seen[topic_name] = time.time()

    def fsm_loop(self):
        now = time.time()

        # 1. 터미널 창 생존 확인 (사용자가 창을 강제로 닫았을 때 감지)
        if self.state in [State.LAUNCHING, State.STABILIZING, State.RUNNING]:
            if self.process is None or self.process.poll() is not None:
                self.get_logger().error("🚨 Monitor Window Closed Unexpectedly!")
                self.transition_to(State.COOLDOWN)
                return

        # 2. State Machine Logic
        if self.state == State.LAUNCHING:
            # 타임아웃 체크
            if now - self.state_start_time > self.boot_timeout:
                self.get_logger().error(f"❌ Boot Timeout ({self.boot_timeout}s).")
                self.transition_to(State.COOLDOWN)
                return
            
            # 모든 카메라 데이터 수신 확인
            all_started = all(t > 0.0 for t in self.topic_last_seen.values())
            if all_started:
                self.get_logger().info("⚡ Signals Detected. Starting Stability Check...")
                self.transition_to(State.STABILIZING)

        elif self.state == State.STABILIZING:
            # 데이터 끊김 확인
            is_healthy, bad_topic = self.check_topic_health()
            if not is_healthy:
                self.get_logger().warn(f"⚠️ Unstable during check: {bad_topic} stalled.")
                self.transition_to(State.COOLDOWN)
                return
            
            # 안정화 시간 달성
            if now - self.state_start_time >= self.stability_duration:
                self.get_logger().info(f"✅ System Stable. Entering RUNNING mode.")
                self.transition_to(State.RUNNING)

        elif self.state == State.RUNNING:
            # 런타임 감시
            is_healthy, bad_topic = self.check_topic_health()
            if not is_healthy:
                self.get_logger().error(f"🚨 Runtime Failure: {bad_topic} stopped.")
                self.transition_to(State.COOLDOWN)
                return
            
            # 5초마다 생존 로그
            if now - self.last_log_time > 5.0:
                self.get_logger().info("🟢 System Healthy (Monitoring active).")
                self.last_log_time = now

        elif self.state == State.COOLDOWN:
            # 대기 시간 후 재시작
            if now - self.state_start_time > self.cooldown_sec:
                self.start_launch_sequence()

        elif self.state == State.FATAL_ERROR:
            pass # 관리자 개입 필요

    def start_launch_sequence(self):
        if self.attempt_count >= self.max_attempts:
            self.get_logger().fatal("💥 Max attempts reached. Please check hardware connection.")
            self.transition_to(State.FATAL_ERROR)
            return

        self.attempt_count += 1
        self.get_logger().info(f"🚀 [Attempt {self.attempt_count}/{self.max_attempts}] Launching ZED Nodes...")
        
        # [Safety] 실행 전 좀비 프로세스 확실하게 정리
        self.force_kill_zed_processes()

        # 상태 초기화
        self.topic_last_seen = {topic: 0.0 for topic in self.target_topics}
        
        try:
            # gnome-terminal --wait 로 실행 (창이 닫힐 때까지 프로세스 유지)
            full_cmd = ["gnome-terminal", "--wait", "--"] + self.launch_cmd
            
            # shell=False로 실행해야 PID 추적이 용이함
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
        """MSG_TIMEOUT 이내에 데이터가 갱신되었는지 확인"""
        now = time.time()
        for topic, last_seen in self.topic_last_seen.items():
            if now - last_seen > self.msg_timeout:
                return False, topic
        return True, None

    def force_kill_zed_processes(self):
        """
        [안전 강화] ZED 관련 내부 프로세스만 골라서 종료.
        gnome-terminal 자체는 건드리지 않음 (다른 터미널 보호).
        """
        # 죽여야 할 핵심 프로세스 이름 키워드
        # component_container: ROS2 composable node container
        # robot_state_publisher: URDF 관련
        target_names = ["zed_wrapper_node", "zed_multi_camera", "component_container", "robot_state_publisher"]
        killed_count = 0
        
        for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
            try:
                # 이미 종료된 프로세스 접근 시 에러 방지
                if not proc.is_running():
                    continue

                proc_name = proc.info['name']
                cmdline = proc.info['cmdline'] or []
                cmd_str = " ".join(cmdline)

                # 1. 자기 자신 보호
                if proc.pid == self.my_pid or proc.pid == 0:
                    continue

                # 2. 터미널 창 보호 (gnome-terminal 프로세스는 직접 죽이지 않음)
                # 터미널은 self.process.terminate()로 관리함
                if "gnome-terminal" in proc_name:
                    continue

                is_target = False
                
                # A. 이름으로 매칭
                if any(target in proc_name for target in target_names):
                    is_target = True
                
                # B. 커맨드라인으로 매칭 (Python Launch 스크립트 등)
                if not is_target:
                    if "zed_multi_camera" in cmd_str and "launch.py" in cmd_str:
                        # 안전장치: 편집기(vim, code)나 grep 명령어는 제외
                        if not any(safe in cmd_str for safe in ["vim", "nano", "code", "grep"]):
                            is_target = True

                if is_target:
                    self.get_logger().warn(f"🔪 Killing orphan process: {proc_name} (PID: {proc.pid})")
                    proc.kill()
                    killed_count += 1
            
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                pass
        
        if killed_count > 0:
            time.sleep(1.0) # 프로세스 정리 대기

    def cleanup_and_close_window(self):
        """터미널 창과 내부 프로세스를 모두 안전하게 정리"""
        self.get_logger().info("🧹 Cleaning up processes...")

        # 1. 내용물(ZED 노드들) 먼저 정리
        self.force_kill_zed_processes()

        # 2. 껍데기(터미널 창) 닫기
        # 우리가 만든 그 subprocess 터미널만 정확히 닫음
        if self.process:
            if self.process.poll() is None:
                self.get_logger().info("🛑 Closing ZED Terminal Window...")
                self.process.terminate() # SIGTERM 전송 -> 창 닫힘
                try:
                    self.process.wait(timeout=2.0)
                except subprocess.TimeoutExpired:
                    self.process.kill() # 응답 없으면 강제 종료
            self.process = None

    def destroy_node(self):
        self.cleanup_and_close_window()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ZedWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Watchdog stopped by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()