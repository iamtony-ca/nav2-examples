네, `/plan` 토픽(`nav_msgs/msg/Path`)을 구독하여 전체 경로의 거리(Path Length)를 계산하는 독립적인 C++ 노드 예제 코드를 작성해 드릴게요.

Nav2 환경에서 개발 중이시니 두 가지 방법을 알려드리겠습니다.

1. **Standalone 방식:** Nav2 라이브러리 없이 순수 C++와 ROS 2 기본 메시지만 사용하는 방법 (가볍고 어디서든 사용 가능).
2. **Nav2 Util 방식:** 이미 Nav2 패키지를 쓰고 계시다면 `nav2_util`을 활용하는 한 줄 컷 방법.

---

### 방법 1: Standalone ROS 2 Node (순수 C++ 구현)

이 코드는 `/plan` 토픽이 들어오면 점과 점 사이의 유클리드 거리를 모두 합산하여 출력합니다.

**파일명:** `path_distance_calculator.cpp`

```cpp
#include <memory>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/path.hpp"

using std::placeholders::_1;

class PathDistanceCalculator : public rclcpp::Node
{
public:
  PathDistanceCalculator()
  : Node("path_distance_calculator")
  {
    // QoS 설정 (Transient Local: 늦게 구독해도 기존 메시지를 받음, Nav2 Path 특성)
    rclcpp::QoS qos(10);
    qos.transient_local();

    // /plan 토픽 구독
    subscription_ = this->create_subscription<nav_msgs::msg::Path>(
      "/plan", qos, std::bind(&PathDistanceCalculator::topic_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Waiting for /plan topic...");
  }

private:
  void topic_callback(const nav_msgs::msg::Path::SharedPtr msg) const
  {
    if (msg->poses.empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty path!");
      return;
    }

    double total_distance = 0.0;

    // 경로의 점들을 순회하며 연속된 두 점 사이의 거리를 누적
    for (size_t i = 0; i < msg->poses.size() - 1; ++i) {
      double dx = msg->poses[i+1].pose.position.x - msg->poses[i].pose.position.x;
      double dy = msg->poses[i+1].pose.position.y - msg->poses[i].pose.position.y;
      
      // 2D Navigation (z축 무시)
      total_distance += std::hypot(dx, dy); 
      
      // 만약 3D 드론 등이라면 아래 사용:
      // double dz = msg->poses[i+1].pose.position.z - msg->poses[i].pose.position.z;
      // total_distance += std::sqrt(dx*dx + dy*dy + dz*dz);
    }

    RCLCPP_INFO(this->get_logger(), 
      "Received Path with %zu points. Total Distance: %.3f meters", 
      msg->poses.size(), total_distance);
  }

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PathDistanceCalculator>());
  rclcpp::shutdown();
  return 0;
}

```

---

### 방법 2: Nav2 Util 활용 (가장 추천)

개발 중이신 플러그인(`StableStoppedGoalChecker`) 내부에서 사용하거나, Nav2 의존성이 있는 패키지라면 굳이 for문을 직접 돌릴 필요가 없습니다. `nav2_util`에 이미 최적화된 함수가 있습니다.

**필요 헤더:**

```cpp
#include "nav2_util/geometry_utils.hpp"

```

**사용 코드:**

```cpp
void topic_callback(const nav_msgs::msg::Path::SharedPtr msg) const
{
  // nav2_util 함수 한 줄로 끝!
  double total_distance = nav2_util::geometry_utils::calculate_path_length(*msg);
  
  RCLCPP_INFO(this->get_logger(), "Path Distance: %.3f m", total_distance);
}

```

### 💡 팁: 특정 인덱스부터 끝까지만 계산하고 싶다면?

앞서 Goal Checker에서 필요했던 기능(로봇 위치부터 끝까지)도 이 함수로 가능합니다.

```cpp
// path의 start_index 번째 점부터 끝점까지의 거리 계산
double remaining = nav2_util::geometry_utils::calculate_path_length(*msg, start_index);

```

### 요약

* 단순 테스트용 노드를 만드신다면 **방법 1**을 복사해서 빌드하세요.
* 기존 Nav2 플러그인 내부나 Nav2 관련 코드라면 **방법 2**를 쓰시는 게 가장 깔끔하고 정확합니다.