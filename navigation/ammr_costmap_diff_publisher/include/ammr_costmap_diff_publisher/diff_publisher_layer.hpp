// [10-10 W1-D] DiffPublisherLayer — global_costmap plugins 목록의 **마지막** 에 붙는 publisher 전용 layer.
//
//   - master 를 절대 바꾸지 않는다. updateBounds 는 bounds 를 넓히지 않는다. isClearable() = false.
//   - updateCosts(master, bounds) 에서 DiffEngine 으로 bounds 안만 비교 → 바뀐 tile 사각형만 CostmapUpdate 로 큐에 넣는다.
//   - publish 는 전용 thread 에서 costmap lock 밖에서 한다.
//   - get_costmap 서비스는 layer 가 만든 전용 callback group + 전용 executor thread 에서 돈다.
//   - [10-10 W1-D][리뷰] FULL publish 뒤 모든 reader ack 를 기다리고 (wait_for_all_acked) 그 뒤 catch-up 시각을 잰다.
//     손실을 알 수 없는 구독자(CostmapSubscriber, update reader KeepLast(10)) 를 위해 refresh·sweep 으로 스스로 메운다.
//
// 토픽 (costmap 노드 기준 상대 이름, <name> = layer 이름):
//   <name>/costmap_raw          nav2_msgs/Costmap        KeepLast(1)  transient_local reliable
//   <name>/costmap_raw_updates  nav2_msgs/CostmapUpdate  KeepLast(updates_qos_depth=10) transient_local reliable
//   <name>/get_costmap          nav2_msgs/srv/GetCostmap
// global_costmap 에서는 /global_costmap/diff_publisher/costmap_raw 등. nav2_costmap_2d::CostmapSubscriber 는
// costmap_topic 에 "_updates" 를 붙여 update 토픽을 구독하므로 costmap_topic = .../diff_publisher/costmap_raw 로 주면 된다.
#ifndef AMMR_COSTMAP_DIFF_PUBLISHER__DIFF_PUBLISHER_LAYER_HPP_
#define AMMR_COSTMAP_DIFF_PUBLISHER__DIFF_PUBLISHER_LAYER_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "ammr_costmap_diff_publisher/diff_engine.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_msgs/msg/costmap_update.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ammr_costmap_diff_publisher
{

class DiffPublisherLayer : public nav2_costmap_2d::Layer
{
public:
  DiffPublisherLayer() = default;
  ~DiffPublisherLayer() override;

  void onInitialize() override;
  void activate() override;
  void deactivate() override;
  // ClearEntireCostmap → Costmap2DROS::resetLayers 가 부른다. shadow 는 유지 (구독자가 가진 것을 나타내므로),
  // 다음 cycle 은 지도 전체를 비교하도록 표시만 한다.
  void reset() override;
  bool isClearable() override {return false;}
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void matchSize() override {}

  // 테스트용
  std::string fullTopic() const {return full_topic_;}
  std::string updatesTopic() const {return updates_topic_;}
  std::string serviceName() const {return service_name_;}
  DiffEngine * engine() {return engine_.get();}

private:
  void startPublisherThread();
  void stopPublisherThread();
  void publisherLoop();
  void publishOne(OutMsg & m);
  void logStats();
  void serviceCallback(
    const std::shared_ptr<rmw_request_id_t> header,
    const std::shared_ptr<nav2_msgs::srv::GetCostmap::Request> req,
    std::shared_ptr<nav2_msgs::srv::GetCostmap::Response> res);

  // params
  bool publish_full_on_new_subscriber_{true};
  double log_stats_period_sec_{10.0};
  int updates_qos_depth_{10};
  int publish_burst_msgs_{8};
  double publish_burst_pause_ms_{2.0};
  int burst_count_{0};   // publisher thread 전용
  // [10-10 W1-D][리뷰]
  double full_ack_timeout_sec_{10.0};
  int repair_burst_msgs_{4};
  double repair_burst_pause_ms_{50.0};
  int repair_count_{0};  // publisher thread 전용

  std::unique_ptr<DiffEngine> engine_;
  std::string full_topic_, updates_topic_, service_name_;

  rclcpp::Publisher<nav2_msgs::msg::Costmap>::SharedPtr full_pub_;
  rclcpp::Publisher<nav2_msgs::msg::CostmapUpdate>::SharedPtr update_pub_;
  rclcpp::Service<nav2_msgs::srv::GetCostmap>::SharedPtr service_;
  rclcpp::CallbackGroup::SharedPtr srv_cb_group_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr srv_executor_;
  std::thread srv_thread_;
  std::atomic<bool> srv_running_{true};

  std::thread pub_thread_;
  std::atomic<bool> pub_running_{false};
  std::atomic<bool> warned_filters_{false};
  size_t last_sub_count_{0};
  std::atomic<bool> new_match_pending_{false};
  bool matched_event_ok_{false};
  std::chrono::steady_clock::time_point last_stats_log_{};
  // 주기 통계 누적 (log 용)
  uint64_t total_fulls_{0};
};

}  // namespace ammr_costmap_diff_publisher

#endif  // AMMR_COSTMAP_DIFF_PUBLISHER__DIFF_PUBLISHER_LAYER_HPP_
