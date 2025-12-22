# ... (기존 import 및 __init__ 등은 유지) ...

    # [수정됨] Nearest Point가 아니라, 앞서 있는 점(Look-ahead)을 찾음
    def get_lookahead_point(self, robot_pose, lookahead_dist=0.4):
        """
        로봇 위치에서 경로상 전방 lookahead_dist 만큼 떨어진 점을 반환
        """
        if not self.pruned_path or len(self.pruned_path.poses) < 2:
            return None

        path_arr = np.array([(p.pose.position.x, p.pose.position.y) for p in self.pruned_path.poses])
        robot_xy = np.array([robot_pose.position.x, robot_pose.position.y])
        
        # 1. 가장 가까운 점 인덱스 찾기
        dists = np.linalg.norm(path_arr - robot_xy, axis=1)
        min_idx = np.argmin(dists)
        
        # 2. 거기서부터 경로를 따라가며 lookahead 거리만큼 떨어진 점 찾기
        curr_dist = 0.0
        target_pt = path_arr[min_idx] # Default: nearest
        
        for i in range(min_idx, len(path_arr) - 1):
            p1 = path_arr[i]
            p2 = path_arr[i+1]
            segment_len = np.linalg.norm(p2 - p1)
            
            if curr_dist + segment_len >= lookahead_dist:
                # 이 세그먼트 위에 타겟이 있음 (보간)
                ratio = (lookahead_dist - curr_dist) / segment_len
                target_pt = p1 + (p2 - p1) * ratio
                return target_pt # [x, y] 반환
            
            curr_dist += segment_len
            target_pt = p2 # 끝까지 못 찾으면 마지막 점

        return target_pt

    # [수정됨] 제어 루프: CTE 대신 Look-ahead 각도 추종
    def control_loop(self):
        # 0. Safety Check
        if (self.get_clock().now() - self.latest_cmd_time).nanoseconds > 0.5 * 1e9:
            self.cmd_pub.publish(Twist())
            return

        final_cmd = Twist()
        final_cmd.linear = self.latest_cmd_vel.linear
        final_cmd.angular = self.latest_cmd_vel.angular

        # 후진 시 Bypass
        if final_cmd.linear.x < 0.0:
            self.cmd_pub.publish(final_cmd)
            self.is_correcting = False
            return

        # 1. 로봇 위치 (Map Frame) 구하기
        try:
            trans = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            robot_pose = Pose()
            robot_pose.position.x = trans.transform.translation.x
            robot_pose.position.y = trans.transform.translation.y
            robot_pose.position.z = trans.transform.translation.z
            robot_pose.orientation = trans.transform.rotation
            _, _, robot_yaw = tf_transformations.euler_from_quaternion(
                [robot_pose.orientation.x, robot_pose.orientation.y, robot_pose.orientation.z, robot_pose.orientation.w])
        except Exception:
            self.cmd_pub.publish(final_cmd)
            return

        if self.pruned_path is None: return

        path_len = self.get_path_length()

        # 2. Goal 진입 (2.5m 이내)
        if path_len < self.path_length_threshold:
            
            # --- Look-ahead Logic 적용 ---
            # 전방 0.4m 지점을 바라보게 함 (속도에 따라 가변적으로 해도 좋음)
            lookahead_pt = self.get_lookahead_point(robot_pose, lookahead_dist=0.4)
            
            if lookahead_pt is not None:
                # 타겟 점까지의 각도 계산
                dy = lookahead_pt[1] - robot_pose.position.y
                dx = lookahead_pt[0] - robot_pose.position.x
                target_yaw = math.atan2(dy, dx)
                
                # Yaw Error 계산
                yaw_error = normalize_angle(target_yaw - robot_yaw)
                
                # Y 오차 추정 (활성화 여부 판단용 - 근사치)
                # 현재 로봇이 타겟 벡터에서 얼마나 떨어져 있나 체크
                # 간단하게는 path 상의 nearest point와의 거리 사용
                path_arr = np.array([(p.pose.position.x, p.pose.position.y) for p in self.pruned_path.poses])
                robot_xy = np.array([robot_pose.position.x, robot_pose.position.y])
                min_dist = np.min(np.linalg.norm(path_arr - robot_xy, axis=1)) # 이것이 대략적인 y 오차
                
                # --- Hysteresis Logic ---
                if not self.is_correcting:
                    if min_dist > self.cte_enable_threshold: # 2.5cm
                        self.is_correcting = True
                else:
                    if min_dist < self.cte_disable_threshold: # 1.0cm
                        self.is_correcting = False

                # --- Control Execution ---
                if self.is_correcting:
                    # 1. 회전 제어 (Look-ahead point를 향해 돌기)
                    # P Gain을 적절히 조절 (너무 크면 흔들림)
                    final_cmd.angular.z = 2.0 * yaw_error 
                    
                    # 2. 최대 회전 속도 제한 (안전)
                    final_cmd.angular.z = max(min(final_cmd.angular.z, 0.6), -0.6)

                    # 3. Creep Speed (최소 속도 유지)
                    # 오차(yaw error)가 크면 속도를 줄이고, 방향이 맞으면 속도를 냄
                    if abs(yaw_error) > 0.2: # 각도가 많이 틀어졌으면 천천히
                        final_cmd.linear.x = self.min_creep_speed # 0.02
                    else:
                        # 각도가 대충 맞으면 조금 더 속도를 냄 (y축 수렴 가속화)
                        final_cmd.linear.x = max(final_cmd.linear.x, 0.05)

        else:
            self.is_correcting = False

        self.cmd_pub.publish(final_cmd)
