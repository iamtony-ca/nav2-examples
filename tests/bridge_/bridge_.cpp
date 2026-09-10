#include <unordered_map>




    // [로그] multi_agent_infos 발행 내용을 machine_id 별로 1초에 한 번만 남긴다.
    //
    // 이 함수는 이웃 로봇 패킷 하나당 한 번(초당 10~20회) 불린다. 그래서
    //  - RCLCPP_*_THROTTLE 을 출력 줄에만 씌우면 안 된다. 인자 평가만 건너뛸 뿐
    //    아래 문자열 조립은 계속 돌기 때문에, 블록 전체를 게이트한다.
    //  - THROTTLE 은 호출 지점 단위라 이웃이 여러 대면 한 대만 찍힌다.
    //    machine_id 를 키로 따로 재서 각 로봇이 1초마다 나오게 한다.
    //
    // map 은 이 함수에서만 만지고, 수신은 단일 스레드 executor(rclcpp::spin) 위에서
    // 도는 UDP 폴링 하나뿐이라 함수 지역 static 으로 충분하다.
    {
      static std::unordered_map<uint16_t, rclcpp::Time> last_agent_log;
      const auto now = this->get_clock()->now();
      auto it = last_agent_log.find(multi_agent_info.machine_id);
      if (it == last_agent_log.end() ||
          (now - it->second) >= rclcpp::Duration::from_seconds(1.0))
      {
        last_agent_log[multi_agent_info.machine_id] = now;

        const auto &poses = truncated_path.poses;
        std::ostringstream ss;
        ss << "truncated_path size=" << poses.size();
        if (!poses.empty())
        {
          const auto &f = poses.front().pose;
          const auto &b = poses.back().pose;
          ss << " first(" << f.position.x << ", " << f.position.y
             << ", " << tf2::getYaw(f.orientation) << ")"
             << " last("  << b.position.x << ", " << b.position.y
             << ", " << tf2::getYaw(b.orientation) << ")";
        }

        RCLCPP_INFO(this->get_logger(), "%s/%d/%s/%s | %s",
          multi_agent_info.header.frame_id.c_str(),
          multi_agent_info.machine_id, multi_agent_info.type_id.c_str(),
          multi_agent_info.mode.c_str(), ss.str().c_str());
      }
    }