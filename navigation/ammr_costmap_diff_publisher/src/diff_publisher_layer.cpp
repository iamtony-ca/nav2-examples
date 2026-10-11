// [10-10 W1-D] DiffPublisherLayer 구현. 설계는 diff_publisher_layer.hpp / diff_engine.hpp 머리말 참고.
#include "ammr_costmap_diff_publisher/diff_publisher_layer.hpp"

#include <algorithm>
#include <cstdio>
#include <functional>
#include <utility>

#include "nav2_costmap_2d/layered_costmap.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace ammr_costmap_diff_publisher
{

using namespace std::chrono_literals;

DiffPublisherLayer::~DiffPublisherLayer()
{
  stopPublisherThread();
  srv_running_.store(false);
  if (srv_executor_) {
    srv_executor_->cancel();
  }
  if (srv_thread_.joinable()) {
    srv_thread_.join();
  }
  if (srv_executor_ && srv_cb_group_) {
    try {
      srv_executor_->remove_callback_group(srv_cb_group_);
    } catch (...) {
    }
  }
  service_.reset();
  srv_executor_.reset();
  full_pub_.reset();
  update_pub_.reset();
}

void DiffPublisherLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"[10-10 W1-D] DiffPublisherLayer: node lock 실패"};
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("tile_size", rclcpp::ParameterValue(64));
  declareParameter("max_rects", rclcpp::ParameterValue(256));
  declareParameter("max_queue_mb", rclcpp::ParameterValue(64.0));
  // [10-10 W1-D][리뷰] 50 → 10. Cyclone 의 reliable KeepLast writer 는 depth 와 상관없이 ack 안 된 sample 을 끝까지 보낸다
  //   (시험: writer depth 1, reader depth 1000, 300 개 연속 → 300 개 모두 수신). depth 는 늦게 붙은 구독자에게 다시 주는
  //   durable history 크기일 뿐이라 작을수록 메모리·옛 값 재생이 줄어든다.
  declareParameter("updates_qos_depth", rclcpp::ParameterValue(10));
  declareParameter("publish_full_on_new_subscriber", rclcpp::ParameterValue(true));
  declareParameter("log_stats_period_sec", rclcpp::ParameterValue(10.0));
  // [10-10 W1-D][리뷰] FULL ack 뒤 catch-up 단계들 (catchup_delay_sec 단일 값 대신)
  declareParameter("catchup_delays_sec", rclcpp::ParameterValue(std::vector<double>{2.0, 10.0}));
  declareParameter("full_ack_timeout_sec", rclcpp::ParameterValue(10.0));
  declareParameter("refresh_period_sec", rclcpp::ParameterValue(5.0));
  declareParameter("refresh_rounds", rclcpp::ParameterValue(2));
  declareParameter("refresh_max_rects", rclcpp::ParameterValue(32));
  declareParameter("sweep_period_sec", rclcpp::ParameterValue(300.0));
  declareParameter("sweep_tick_sec", rclcpp::ParameterValue(1.0));
  declareParameter("max_msg_mb", rclcpp::ParameterValue(2.0));
  declareParameter("max_update_cycle_mb", rclcpp::ParameterValue(16.0));
  declareParameter("copy_chunk_mb", rclcpp::ParameterValue(2.0));
  declareParameter("repair_burst_msgs", rclcpp::ParameterValue(4));
  declareParameter("repair_burst_pause_ms", rclcpp::ParameterValue(50.0));
  declareParameter("audit_period_cycles", rclcpp::ParameterValue(0));
  declareParameter("full_fallback_ratio", rclcpp::ParameterValue(0.5));
  // CostmapSubscriber(smoother) 의 update 구독은 reader KeepLast(10) 로 고정이다. 한 번에 수십 개를 몰아 보내면
  // reader 쪽에서 오래된 것이 소리 없이 덮인다 → burst 개마다 잠깐 쉰다 (0 이면 끔).
  declareParameter("publish_burst_msgs", rclcpp::ParameterValue(8));
  declareParameter("publish_burst_pause_ms", rclcpp::ParameterValue(2.0));

  int tile_size = 64, max_rects = 256, audit_period = 0, refresh_rounds = 2, refresh_max_rects = 32;
  double max_queue_mb = 64.0, full_fallback_ratio = 0.5;
  std::vector<double> catchup_delays{2.0, 10.0};
  double refresh_period = 5.0, sweep_period = 300.0, sweep_tick = 1.0;
  double max_msg_mb = 2.0, max_update_cycle_mb = 16.0, copy_chunk_mb = 2.0;
  node->get_parameter(getFullName("enabled"), enabled_);
  node->get_parameter(getFullName("tile_size"), tile_size);
  node->get_parameter(getFullName("max_rects"), max_rects);
  node->get_parameter(getFullName("max_queue_mb"), max_queue_mb);
  node->get_parameter(getFullName("updates_qos_depth"), updates_qos_depth_);
  node->get_parameter(getFullName("publish_full_on_new_subscriber"), publish_full_on_new_subscriber_);
  node->get_parameter(getFullName("log_stats_period_sec"), log_stats_period_sec_);
  node->get_parameter(getFullName("catchup_delays_sec"), catchup_delays);
  node->get_parameter(getFullName("full_ack_timeout_sec"), full_ack_timeout_sec_);
  node->get_parameter(getFullName("refresh_period_sec"), refresh_period);
  node->get_parameter(getFullName("refresh_rounds"), refresh_rounds);
  node->get_parameter(getFullName("refresh_max_rects"), refresh_max_rects);
  node->get_parameter(getFullName("sweep_period_sec"), sweep_period);
  node->get_parameter(getFullName("sweep_tick_sec"), sweep_tick);
  node->get_parameter(getFullName("max_msg_mb"), max_msg_mb);
  node->get_parameter(getFullName("max_update_cycle_mb"), max_update_cycle_mb);
  node->get_parameter(getFullName("copy_chunk_mb"), copy_chunk_mb);
  node->get_parameter(getFullName("repair_burst_msgs"), repair_burst_msgs_);
  node->get_parameter(getFullName("repair_burst_pause_ms"), repair_burst_pause_ms_);
  node->get_parameter(getFullName("audit_period_cycles"), audit_period);
  node->get_parameter(getFullName("full_fallback_ratio"), full_fallback_ratio);
  node->get_parameter(getFullName("publish_burst_msgs"), publish_burst_msgs_);
  node->get_parameter(getFullName("publish_burst_pause_ms"), publish_burst_pause_ms_);

  DiffEngine::Config cfg;
  cfg.tile_size = static_cast<uint32_t>(std::clamp(tile_size, 1, 4096));
  cfg.max_rects = static_cast<uint32_t>(std::clamp(max_rects, 1, 100000));
  cfg.max_queue_bytes = static_cast<size_t>(std::max(1.0, max_queue_mb) * 1024.0 * 1024.0);
  updates_qos_depth_ = std::max(1, updates_qos_depth_);
  cfg.history_depth = static_cast<size_t>(updates_qos_depth_);
  cfg.catchup_delays_sec = catchup_delays;
  cfg.refresh_period_sec = refresh_period;
  cfg.refresh_rounds = static_cast<uint32_t>(std::max(1, refresh_rounds));
  cfg.refresh_max_rects = static_cast<uint32_t>(std::clamp(refresh_max_rects, 1, 100000));
  cfg.sweep_period_sec = sweep_period;
  cfg.sweep_tick_sec = sweep_tick > 0.0 ? sweep_tick : 1.0;
  auto mb = [](double v) {return static_cast<size_t>(std::max(0.0, v) * 1024.0 * 1024.0);};
  cfg.max_msg_bytes = std::max<size_t>(4096, mb(max_msg_mb));
  cfg.max_update_cycle_bytes = mb(max_update_cycle_mb);
  cfg.copy_chunk_bytes = mb(copy_chunk_mb);
  cfg.audit_period_cycles = static_cast<uint32_t>(std::max(0, audit_period));
  cfg.full_fallback_ratio = full_fallback_ratio;
  cfg.frame_id = layered_costmap_->getGlobalFrameID();

  auto clock = clock_;
  engine_ = std::make_unique<DiffEngine>(
    cfg, [clock]() -> int64_t {return clock ? clock->now().nanoseconds() : 0;});

  // 토픽은 costmap 노드 기준 상대 이름 → /global_costmap/<name>/costmap_raw(_updates)
  full_topic_ = name_ + "/costmap_raw";
  updates_topic_ = full_topic_ + "_updates";   // CostmapSubscriber 가 기대하는 이름 = full + "_updates"
  service_name_ = name_ + "/get_costmap";

  // get_costmap: 전용 callback group (node 기본 executor 에 자동으로 붙지 않게 false) + 전용 executor thread.
  //   - node 기본 executor 는 clear_*_costmap 서비스(costmap lock 을 쥐고 resetLayers)·parameter 서비스를 돌린다.
  //   - Layer 에 넘어오는 callback_group_ 은 Costmap2DROS executor_thread_ 로 layer 구독(static map, scan, agent, tf)을 돌린다.
  //   2e8 셀 사본(~200 MB) 을 만드는 동안 그 둘을 막지 않으려고 따로 둔다. 서비스는 costmap lock 을 잡지 않는다 (shadow lock 만).
  srv_cb_group_ = node->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive, false);

  // LifecyclePublisher 가 아닌 일반 rclcpp::Publisher (activate 여부와 무관하게 publish 가능, 시점은 layer 가 정한다)
  // 새 구독자: PUBLICATION_MATCHED event (누적 수라 빠른 끊김→재연결도 잡는다). event 는 위 전용 group 에서 돈다.
  //   event 를 못 쓰는 rmw 면 publisher thread 의 구독 수 polling 만 쓴다.
  const auto full_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
  {
    rclcpp::PublisherOptions popts;
    popts.callback_group = srv_cb_group_;
    popts.event_callbacks.matched_callback = [this](rclcpp::MatchedInfo & info) {
        if (info.current_count_change > 0 || info.total_count_change > 0) {
          new_match_pending_.store(true);
        }
      };
    try {
      full_pub_ = rclcpp::create_publisher<nav2_msgs::msg::Costmap>(node, full_topic_, full_qos, popts);
      matched_event_ok_ = true;
    } catch (const std::exception & e) {
      RCLCPP_WARN(
        logger_, "[10-10 W1-D] %s: matched event 를 못 단다 (%s) — 구독 수 polling 만 쓴다",
        name_.c_str(), e.what());
      full_pub_ = rclcpp::create_publisher<nav2_msgs::msg::Costmap>(node, full_topic_, full_qos);
      matched_event_ok_ = false;
    }
  }
  update_pub_ = rclcpp::create_publisher<nav2_msgs::msg::CostmapUpdate>(
    node, updates_topic_,
    rclcpp::QoS(rclcpp::KeepLast(updates_qos_depth_)).transient_local().reliable());

  service_ = node->create_service<nav2_msgs::srv::GetCostmap>(
    service_name_,
    std::bind(
      &DiffPublisherLayer::serviceCallback, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3),
    rclcpp::ServicesQoS(), srv_cb_group_);
  srv_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  srv_executor_->add_callback_group(srv_cb_group_, node->get_node_base_interface());
  srv_thread_ = std::thread(
    [this, exec = srv_executor_]() {
      // spin() 은 cancel() 이 spin 시작 전에 오면 끝나지 않을 수 있어 spin_once 반복 + 플래그로 돈다
      // [10-10 W1-D][리뷰] (F5) 예외로 thread 가 조용히 끝나면 서비스·matched event 가 영영 안 돈다 → log 하고 계속.
      auto last_err = std::chrono::steady_clock::time_point{};
      while (srv_running_.load() && rclcpp::ok()) {
        try {
          exec->spin_once(100ms);
        } catch (const std::exception & e) {
          const auto now = std::chrono::steady_clock::now();
          if (now - last_err > 5s) {
            last_err = now;
            RCLCPP_ERROR(logger_, "[10-10 W1-D][리뷰] %s get_costmap executor 예외 (계속 돈다): %s", name_.c_str(), e.what());
          }
          std::this_thread::sleep_for(100ms);
        } catch (...) {
          std::this_thread::sleep_for(100ms);
        }
      }
    });

  if (layered_costmap_->isRolling()) {
    RCLCPP_WARN(
      logger_, "[10-10 W1-D] %s: rolling window costmap 에서는 매 cycle origin 이 바뀌어 FULL 만 보낸다. "
      "global(정적) costmap 용이다.", name_.c_str());
  }
  current_ = true;

  RCLCPP_INFO(
    logger_,
    "[10-10 W1-D] DiffPublisherLayer '%s' 초기화: enabled=%d full=%s updates=%s(depth %d) service=%s "
    "tile=%u max_rects=%u max_queue_mb=%.1f catchup=%zu stages(first %.1fs, last %.1fs) ack_timeout=%.1fs "
    "refresh=%.1fs x%u (%u rects) sweep=%.0fs/%.1fs max_msg=%.1fMB max_cycle=%.1fMB chunk=%.1fMB audit=%u "
    "fallback=%.2f new_sub_full=%d matched_event=%d burst=%d/%.1fms repair_burst=%d/%.1fms",
    name_.c_str(), enabled_, full_pub_->get_topic_name(), update_pub_->get_topic_name(),
    updates_qos_depth_, service_->get_service_name(), cfg.tile_size, cfg.max_rects, max_queue_mb,
    engine_->config().catchup_delays_sec.size(),
    engine_->config().catchup_delays_sec.empty() ? 0.0 : engine_->config().catchup_delays_sec.front(),
    engine_->config().catchup_delays_sec.empty() ? 0.0 : engine_->config().catchup_delays_sec.back(),
    full_ack_timeout_sec_, cfg.refresh_period_sec, cfg.refresh_rounds, cfg.refresh_max_rects,
    cfg.sweep_period_sec, cfg.sweep_tick_sec, cfg.max_msg_bytes / 1048576.0, cfg.max_update_cycle_bytes / 1048576.0,
    cfg.copy_chunk_bytes / 1048576.0, cfg.audit_period_cycles, cfg.full_fallback_ratio, publish_full_on_new_subscriber_,
    matched_event_ok_, publish_burst_msgs_, publish_burst_pause_ms_, repair_burst_msgs_, repair_burst_pause_ms_);
}

void DiffPublisherLayer::activate()
{
  if (!engine_) {
    return;
  }
  // 재활성화 (deactivate → activate) 면 구독자 상태를 모르므로 FULL 한 번. 첫 활성화는 첫 cycle 이 FULL 을 요청한다.
  if (engine_->geometry().valid()) {
    engine_->requestFull(FullReason::kActivate);
  }
  startPublisherThread();
}

void DiffPublisherLayer::deactivate()
{
  stopPublisherThread();
}

void DiffPublisherLayer::reset()
{
  // ClearEntireCostmap: Costmap2DROS::resetLayers 가 master 전체를 default 로 지운 뒤 각 layer 의 reset() 을 부른다.
  // shadow 는 그대로 (구독자가 가진 값). 다음 cycle 은 bounds 와 상관없이 지도 전체를 비교한다.
  if (engine_) {
    engine_->markResetPending();
  }
}

void DiffPublisherLayer::updateBounds(
  double, double, double, double *, double *, double *, double *)
{
  // bounds 를 넓히지 않는다 (다른 layer 가 정한 bounds 안만 비교).
}

void DiffPublisherLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid, int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_ || !engine_) {
    return;
  }
  if (&master_grid != layered_costmap_->getCostmap() && !warned_filters_.exchange(true)) {
    // filters 키가 있으면 plugin 은 primary_costmap_ 을 받고 filter 는 그 뒤에 combined 에 적용된다 → 최종값이 아니다.
    RCLCPP_WARN(
      logger_, "[10-10 W1-D] %s: updateCosts 의 master 가 최종 costmap 이 아니다 (costmap filters 사용 중). "
      "filter 결과는 보내지 못한다. filters 를 plugin 으로 두거나 이 layer 를 쓰지 말 것.", name_.c_str());
  }
  engine_->processCycle(master_grid, min_i, min_j, max_i, max_j);
}

void DiffPublisherLayer::startPublisherThread()
{
  if (pub_running_.exchange(true)) {
    return;
  }
  engine_->setStopping(false);
  last_stats_log_ = std::chrono::steady_clock::now();
  pub_thread_ = std::thread(&DiffPublisherLayer::publisherLoop, this);
}

void DiffPublisherLayer::stopPublisherThread()
{
  if (!pub_running_.exchange(false)) {
    return;
  }
  if (engine_) {
    engine_->setStopping(true);
  }
  if (pub_thread_.joinable()) {
    pub_thread_.join();
  }
}

void DiffPublisherLayer::publishOne(OutMsg & m)
{
  auto done = std::chrono::steady_clock::now();
  try {
    if (m.kind == OutMsg::Kind::kFull && m.full) {
      // 이 FULL 을 받는 구독자 수를 기준으로 삼는다 (그 뒤 늘어나면 새 구독자)
      last_sub_count_ = full_pub_->get_subscription_count();
      new_match_pending_.store(false);   // 지금 matched 된 구독자는 이 FULL 을 받는다
      full_pub_->publish(*m.full);
      // [10-10 W1-D][리뷰] (F1) 모든 reader 가 FULL 을 받을(ack) 때까지 기다린다. 그동안 update 는 큐에 쌓였다가 FULL 뒤에
      //   나가므로 reader 쪽에서 FULL 보다 먼저 도착하지 않는다. catch-up 시각도 여기서부터 잰다.
      const auto t_ack = std::chrono::steady_clock::now();
      bool acked = true;
      if (full_ack_timeout_sec_ > 0.0) {
        try {
          acked = full_pub_->wait_for_all_acked(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
              std::chrono::duration<double>(full_ack_timeout_sec_)));
        } catch (const std::exception & e) {
          RCLCPP_WARN(
            logger_, "[10-10 W1-D][리뷰] %s wait_for_all_acked 를 못 쓴다 (%s) — ack 대기 없이 catch-up 시각을 잰다",
            name_.c_str(), e.what());
          full_ack_timeout_sec_ = 0.0;
        }
      }
      done = std::chrono::steady_clock::now();
      const double ack_ms = std::chrono::duration<double, std::milli>(done - t_ack).count();
      engine_->noteFullAck(ack_ms, acked);
      ++total_fulls_;
      if (acked) {
        RCLCPP_INFO(
          logger_, "[10-10 W1-D] %s FULL 전송 (%s, %ux%u, %.1f MB, 구독 %zu, ack %.0f ms)", name_.c_str(),
          toString(m.reason), m.full->metadata.size_x, m.full->metadata.size_y,
          m.bytes / (1024.0 * 1024.0), last_sub_count_, ack_ms);
      } else {
        RCLCPP_WARN(
          logger_, "[10-10 W1-D][리뷰] %s FULL 전송 (%s, %.1f MB, 구독 %zu) — %.1f s 안에 모든 reader ack 를 못 받음. "
          "catch-up 은 지금부터 잰다 (늦은 reader 는 refresh/sweep 이 메운다)", name_.c_str(),
          toString(m.reason), m.bytes / (1024.0 * 1024.0), last_sub_count_, full_ack_timeout_sec_);
      }
    } else if (m.kind == OutMsg::Kind::kUpdate && m.update) {
      update_pub_->publish(*m.update);
    } else {
      return;
    }
  } catch (const std::exception & e) {
    // [10-10 W1-D][리뷰] (F10) 잃지 않는다: FULL 은 다시 요청, update 는 그 사각형을 repair 로 다시 보낸다.
    RCLCPP_WARN(
      logger_, "[10-10 W1-D][리뷰] %s %s publish 실패 (다시 보낸다): %s", name_.c_str(),
      m.kind == OutMsg::Kind::kFull ? "FULL" : "update", e.what());
    engine_->onPublishFailed(m);
    std::this_thread::sleep_for(200ms);
    return;
  }
  engine_->onPublished(m, done);
}

void DiffPublisherLayer::publisherLoop()
{
  auto last_sub_check = std::chrono::steady_clock::now();
  auto backoff = 200ms;
  auto last_err = std::chrono::steady_clock::time_point{};
  while (pub_running_.load() && rclcpp::ok()) {
    OutMsg m;
    bool got = false;
    try {
      got = engine_->waitPop(m, 100ms);
      backoff = 200ms;
    } catch (const std::exception & e) {
      // [10-10 W1-D][리뷰] (F6) FULL 버퍼(지도 크기) 할당 실패 등 — thread 를 살려 두고 잠시 뒤 다시 (FULL token 은 큐에 남아 있다)
      const auto now = std::chrono::steady_clock::now();
      if (now - last_err > 5s) {
        last_err = now;
        RCLCPP_ERROR(
          logger_, "[10-10 W1-D][리뷰] %s FULL/update 만들기 실패 (%s) — %ld ms 뒤 다시", name_.c_str(), e.what(),
          static_cast<long>(backoff.count()));
      }
      std::this_thread::sleep_for(backoff);
      backoff = std::min<std::chrono::milliseconds>(backoff * 2, 5000ms);
      continue;
    }
    if (got) {
      publishOne(m);
      if (m.kind == OutMsg::Kind::kUpdate && m.repair) {
        // repair 는 천천히 (reader KeepLast(10) 이 막혀 있어도 한 묶음은 다 들어가게)
        if (repair_burst_msgs_ > 0 && repair_burst_pause_ms_ > 0.0 && ++repair_count_ >= repair_burst_msgs_) {
          repair_count_ = 0;
          std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(repair_burst_pause_ms_));
        }
      } else if (m.kind == OutMsg::Kind::kUpdate && publish_burst_msgs_ > 0 && publish_burst_pause_ms_ > 0.0 &&
        ++burst_count_ >= publish_burst_msgs_)
      {
        burst_count_ = 0;
        std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(publish_burst_pause_ms_));
      }
    } else {
      burst_count_ = 0;
    }
    const auto now = std::chrono::steady_clock::now();
    if (now - last_sub_check >= 100ms) {
      last_sub_check = now;
      size_t n = 0;
      try {
        n = full_pub_->get_subscription_count();
        engine_->setHaveSubscribers(update_pub_->get_subscription_count() > 0);
      } catch (...) {
        n = last_sub_count_;
      }
      const bool matched = new_match_pending_.exchange(false);
      if ((n > last_sub_count_ || matched) && publish_full_on_new_subscriber_ &&
        engine_->geometry().valid() && !engine_->fullPending())
      {
        RCLCPP_INFO(
          logger_, "[10-10 W1-D] %s 새 구독자 (구독 수 %zu → %zu, matched event %d) — FULL 요청",
          name_.c_str(), last_sub_count_, n, matched);
        engine_->requestFull(FullReason::kNewSubscriber);
      }
      last_sub_count_ = n;
    }
    if (log_stats_period_sec_ > 0.0 &&
      now - last_stats_log_ >= std::chrono::duration<double>(log_stats_period_sec_))
    {
      last_stats_log_ = now;
      logStats();
    }
  }
}

void DiffPublisherLayer::logStats()
{
  const EngineStats s = engine_->takeStats();
  char buf[1400];
  std::snprintf(
    buf, sizeof(buf),
    "[10-10 W1-D] %s 통계 (%.0f s): cycles=%lu full_compare=%lu compared=%lu changed=%lu rects=%lu "
    "upd_sent=%lu(%.1f KB) fulls=%lu(%.1f MB, ack max %.0f ms, ack timeout %lu, abort %lu) dropped=%lu backlog=%lu "
    "large_diff=%lu catchup=%lu stages/%lu msgs(%.1f KB) refresh=%lu/%lu msgs(%.1f KB) sweep=%lu msgs(%.1f KB, laps %lu) "
    "retry=%lu pub_fail=%lu/%lu outside_bounds=%lu max_compare=%.2fms max_cycle=%.2fms max_shadow_hold=%.2fms "
    "queue=%zu(%.1f KB) repair_pending=%zu",
    name_.c_str(), log_stats_period_sec_,
    (unsigned long)s.cycles, (unsigned long)s.full_compare_cycles, (unsigned long)s.compared_cells,
    (unsigned long)s.changed_cells, (unsigned long)s.rects,
    (unsigned long)s.update_msgs_sent, s.update_bytes_sent / 1024.0,
    (unsigned long)s.fulls_sent, s.full_bytes_sent / (1024.0 * 1024.0), s.max_full_ack_ms,
    (unsigned long)s.full_ack_timeouts, (unsigned long)s.full_aborts,
    (unsigned long)s.dropped_updates, (unsigned long)s.backlog_trips, (unsigned long)s.large_diff_fulls,
    (unsigned long)s.catchups, (unsigned long)s.catchup_msgs, s.catchup_bytes / 1024.0,
    (unsigned long)s.refreshes, (unsigned long)s.refresh_msgs, s.refresh_bytes / 1024.0,
    (unsigned long)s.sweep_msgs, s.sweep_bytes / 1024.0, (unsigned long)s.sweeps_completed,
    (unsigned long)s.retry_msgs, (unsigned long)s.full_publish_failures, (unsigned long)s.update_publish_failures,
    (unsigned long)s.outside_bounds_cells, s.max_compare_ms, s.max_cycle_ms, s.max_shadow_hold_ms,
    engine_->queueLength(), engine_->queuedUpdateBytes() / 1024.0, engine_->repairPending());
  if (s.outside_bounds_cells > 0 || s.full_ack_timeouts > 0 || s.full_publish_failures > 0 ||
    s.update_publish_failures > 0)
  {
    // audit 에서 bounds 밖 변경이 보였다 = 어떤 layer 가 bounds 밖을 쓴다 (그 변경은 그 cycle 에 보냈다) / 전송 이상
    RCLCPP_WARN(logger_, "%s", buf);
  } else {
    RCLCPP_INFO(logger_, "%s", buf);
  }
}

void DiffPublisherLayer::serviceCallback(
  const std::shared_ptr<rmw_request_id_t>,
  const std::shared_ptr<nav2_msgs::srv::GetCostmap::Request>,
  std::shared_ptr<nav2_msgs::srv::GetCostmap::Response> res)
{
  // [10-10 W1-D][리뷰] (F5) 예외(지도 크기 응답 할당 실패 등)를 executor 밖으로 내보내지 않는다 → 빈 응답
  try {
    if (!engine_ || !engine_->snapshot(res->map)) {
      res->map = nav2_msgs::msg::Costmap();
      RCLCPP_WARN(logger_, "[10-10 W1-D] %s get_costmap: 아직 첫 cycle 전 — 빈 응답", name_.c_str());
    }
  } catch (const std::exception & e) {
    res->map = nav2_msgs::msg::Costmap();
    RCLCPP_ERROR(logger_, "[10-10 W1-D][리뷰] %s get_costmap 실패 (%s) — 빈 응답", name_.c_str(), e.what());
  }
}

}  // namespace ammr_costmap_diff_publisher

PLUGINLIB_EXPORT_CLASS(ammr_costmap_diff_publisher::DiffPublisherLayer, nav2_costmap_2d::Layer)
