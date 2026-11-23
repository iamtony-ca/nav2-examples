// nav_data_recorder: always-on, low-cost navigation recorder.
//
//   - Small topics: GenericSubscription, written serialized (no deserialization), optional rate gate.
//   - /tf: rate gate per child frame (several publishers share /tf; a single gate would starve the
//     slow ones). Also keeps the latest transforms for the event pose.
//   - Scans -> CompactScan, costmap_raw -> CompactGrid (deserialized only after the rate gate).
//   - rosbag2_cpp Writer, mcap + zstd_fast. The node splits files itself and re-writes latched
//     topics and a global costmap keyframe (get_costmap) at the start of every file.
//   - Events (state change, error, safety bit rising, mark) -> <root>/events/YYYYMMDD.jsonl.
//   - Action results: when a watched goal ends, its result is fetched with the action's get_result
//     service (GenericClient) and logged + written to /nav_data/action_result.
//   - Topics are subscribed when they appear in the graph; missing ones are reported once.
#include <fcntl.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <future>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <csignal>
#include <cmath>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "action_msgs/msg/goal_status.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "nav2_msgs/msg/costmap.hpp"
#include "nav2_msgs/srv/get_costmap.hpp"
#include "nav_data_recorder/config.hpp"
#include "nav_data_recorder/event_log.hpp"
#include "nav_data_recorder/field_reader.hpp"
#include "nav_data_recorder/grid_codec.hpp"
#include "nav_data_recorder/msg_definition.hpp"
#include "nav_data_recorder/scan_codec.hpp"
#include "nav_data_recorder/throttle.hpp"
#include "nav_data_recorder_msgs/msg/compact_grid.hpp"
#include "nav_data_recorder_msgs/msg/compact_scan.hpp"
#include "rclcpp/generic_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/serialization.hpp"
#include "rosbag2_cpp/bag_events.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_storage/storage_options.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Transform.h"
#include "tf2/utils.h"
#include "tf2_msgs/msg/tf_message.hpp"

namespace fs = std::filesystem;

namespace nav_data_recorder
{

static const char * kVersion = "0.1.0";
// Written at the start of every file: lets tools find latched topics without the QoS metadata.
static const char * kInfoTopic = "/nav_data/recorder_info";
// std_msgs/String JSON per finished goal of an action with `result:` in the event rule
static const char * kResultTopic = "/nav_data/action_result";
static const double kResultTimeoutS = 5.0;

static int64_t wallNs()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
}

static double steadyS()
{
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

static std::string expandHome(const std::string & p)
{
  if (!p.empty() && p[0] == '~') {
    const char * h = std::getenv("HOME");
    return std::string(h ? h : "") + p.substr(1);
  }
  return p;
}

static std::string bootIdShort()
{
  std::ifstream f("/proc/sys/kernel/random/boot_id");
  std::string id;
  std::getline(f, id);
  std::string hex;
  for (char c : id) {
    if (std::isxdigit(static_cast<unsigned char>(c))) {
      hex.push_back(c);
    }
    if (hex.size() == 4) {
      break;
    }
  }
  return hex.empty() ? "0000" : hex;
}

static std::string localStamp(const char * fmt)
{
  std::time_t t = std::time(nullptr);
  std::tm tm{};
  localtime_r(&t, &tm);
  char buf[32];
  std::strftime(buf, sizeof(buf), fmt, &tm);
  return buf;
}

// get_result client of one action, created when its first goal ends
struct ResultFetch
{
  std::string action;                  // e.g. /navigate_through_poses
  std::unique_ptr<FieldReader> req;    // <type>_GetResult_Request (buffer reused per request)
  std::unique_ptr<FieldReader> resp;   // <type>_GetResult_Response (field paths)
  rclcpp::GenericClient::SharedPtr client;
  bool failed{false};
};

struct EventWatch
{
  EventCfg cfg;
  std::unique_ptr<ResultFetch> fetch;
  std::map<std::string, std::optional<FieldValue>> last;
  std::map<std::string, int8_t> goal_status;   // action_status: goal uuid -> last status
  std::deque<std::string> goal_order;
};

struct Sub
{
  TopicCfg cfg;
  std::string type;
  std::string out_topic;
  std::string out_type;
  bool subscribed{false};
  bool failed{false};
  bool missing_reported{false};
  rclcpp::GenericSubscription::SharedPtr gsub;
  Throttle throttle;
  std::unordered_map<std::string, Throttle> tf_throttle;  // child frame -> gate
  std::unique_ptr<ScanEncoder> scan_enc;
  std::unique_ptr<GridEncoder> grid_enc;
  // latched: last message per publisher (/tf_static has several latched publishers)
  std::map<std::string, std::shared_ptr<const rclcpp::SerializedMessage>> latched_last;
  std::vector<EventWatch *> watches;
  std::unique_ptr<FieldReader> reader;
  uint64_t rx{0};
  uint64_t written{0};
};

class RecorderNode : public rclcpp::Node
{
public:
  explicit RecorderNode(const rclcpp::NodeOptions & opts)
  : rclcpp::Node("nav_data_recorder", opts)
  {
    root_ = expandHome(declare_parameter<std::string>("root_dir", "~/nav_data"));
    const std::string topics_file = declare_parameter<std::string>("topics_file", "");
    robot_id_ = declare_parameter<std::string>("robot_id", "robot");
    session_name_ = declare_parameter<std::string>("session_name", "");
    const double split_s = declare_parameter<double>("split_period_s", 600.0);
    const int64_t cache = declare_parameter<int64_t>("max_cache_size", 1024 * 1024);
    const std::string preset = declare_parameter<std::string>("storage_preset", "zstd_fast");
    // mcap writer options overlaid on the preset (chunkSize ...); "" = config/mcap_writer.yaml
    std::string storage_cfg = declare_parameter<std::string>("storage_config_uri", "");
    const double discovery_s = declare_parameter<double>("discovery_period_s", 2.0);
    missing_timeout_s_ = declare_parameter<double>("missing_timeout_s", 30.0);
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    const std::string ws_git = declare_parameter<std::string>("ws_git_rev", "");
    const std::string map_name = declare_parameter<std::string>("map_name", "");
    const std::string cfg_sha = declare_parameter<std::string>("config_sha256", "");
    // force the current file, its directory and today's event log to disk this often, so a power
    // cut loses at most this long plus one mcap chunk (0 = leave it to the OS, up to ~30 s)
    const double fsync_s = declare_parameter<double>("fsync_period_s", 5.0);

    if (topics_file.empty()) {
      throw std::runtime_error("parameter topics_file is required");
    }
    cfg_ = loadConfig(expandHome(topics_file));

    // --- bag location: rec/YYYYMMDD/HHMMSS_<boot> (one recorder start = one rosbag2 dir) ---
    boot_ = bootIdShort();
    if (session_name_.empty()) {
      session_name_ = localStamp("%Y%m%d_%H%M%S") + "_" + boot_;
    }
    const std::string day = session_name_.substr(0, 8);
    const std::string base = session_name_.substr(9);  // HHMMSS_boot
    fs::create_directories(root_ + "/rec/" + day);
    bag_dir_ = root_ + "/rec/" + day + "/" + base;
    if (fs::exists(bag_dir_)) {  // recorder restarted within the same second
      bag_dir_ += "_" + std::to_string(::getpid());
    }
    current_file_ = bag_dir_ + "/" + fs::path(bag_dir_).filename().string() + "_0.mcap";

    events_ = std::make_unique<EventLog>(root_ + "/events");

    rosbag2_cpp::bag_events::WriterEventCallbacks cbs;
    cbs.write_split_callback = [this](rosbag2_cpp::bag_events::BagSplitInfo & info) {
        std::lock_guard<std::mutex> lk(sync_mu_);
        current_file_ = info.opened_file;
      };
    writer_.add_event_callbacks(cbs);

    rosbag2_storage::StorageOptions so;
    so.uri = bag_dir_;
    so.storage_id = "mcap";
    so.max_cache_size = static_cast<uint64_t>(cache);
    so.storage_preset_profile = preset;
    if (storage_cfg.empty()) {
      try {
        const std::string def = ament_index_cpp::get_package_share_directory("nav_data_recorder") +
          "/config/mcap_writer.yaml";
        if (fs::exists(def)) {
          storage_cfg = def;
        }
      } catch (const std::exception &) {
      }
    }
    so.storage_config_uri = expandHome(storage_cfg);
    so.custom_data = {
      {"robot_id", robot_id_}, {"boot_id", boot_}, {"session", session_name_},
      {"map_name", map_name}, {"ws_git_rev", ws_git}, {"config_sha256", cfg_sha},
      {"recorder_version", kVersion}};
    writer_.open(so, rosbag2_cpp::ConverterOptions{"cdr", "cdr"});

    // --- subscriptions model ---
    for (const auto & tc : cfg_.topics) {
      auto s = std::make_unique<Sub>();
      s->cfg = tc;
      s->throttle = Throttle(tc.rate);
      subs_[tc.topic] = std::move(s);
    }
    for (const auto & ec : cfg_.events) {
      auto w = std::make_unique<EventWatch>();
      w->cfg = ec;
      auto it = subs_.find(ec.topic);
      if (it == subs_.end()) {  // event-only topic
        auto s = std::make_unique<Sub>();
        s->cfg.topic = ec.topic;
        s->cfg.record = false;
        it = subs_.emplace(ec.topic, std::move(s)).first;
      }
      it->second->watches.push_back(w.get());
      watches_.push_back(std::move(w));
    }

    if (cfg_.global_keyframe.enabled) {
      key_client_ = create_client<nav2_msgs::srv::GetCostmap>(cfg_.global_keyframe.service);
      createWriterTopic(
        cfg_.global_keyframe.output, "nav2_msgs/msg/Costmap",
        rclcpp::QoS(1).reliable().transient_local());
      key_pending_ = true;
    }

    createWriterTopic(kInfoTopic, "std_msgs/msg/String", rclcpp::QoS(1).reliable().transient_local());
    for (const auto & w : watches_) {
      if (!w->cfg.result_type.empty()) {
        createWriterTopic(kResultTopic, "std_msgs/msg/String", rclcpp::QoS(10).reliable());
        result_timer_ = create_wall_timer(std::chrono::milliseconds(200), [this]() {pollResults();});
        break;
      }
    }
    writeInfo();

    start_s_ = steadyS();
    logEvent("recorder", "", "\"start\"", {
        {"bag", jsonEscape(relative(bag_dir_))}, {"version", jsonEscape(kVersion)},
        {"robot_id", jsonEscape(robot_id_)}});
    RCLCPP_INFO(get_logger(), "recording to %s (%zu topics, %zu event rules)",
      bag_dir_.c_str(), cfg_.topics.size(), cfg_.events.size());

    // eager subscriptions (type given in config, e.g. /nav_data/mark)
    for (auto & kv : subs_) {
      if (!kv.second->cfg.type.empty()) {
        subscribe(*kv.second, kv.second->cfg.type, defaultQos(kv.second->cfg));
      }
    }

    discovery_timer_ = create_wall_timer(
      std::chrono::duration<double>(discovery_s), [this]() {discover(); serviceKeyframe();});
    if (split_s > 0) {
      split_timer_ = create_wall_timer(
        std::chrono::duration<double>(split_s), [this]() {split();});
    }
    discover();
    if (fsync_s > 0) {
      sync_thread_ = std::thread([this, fsync_s]() {syncLoop(fsync_s);});
    }
  }

  ~RecorderNode() override
  {
    {
      std::lock_guard<std::mutex> lk(sync_mu_);
      sync_stop_ = true;
    }
    sync_cv_.notify_all();
    if (sync_thread_.joinable()) {
      sync_thread_.join();
    }
    logEvent("recorder", "", "\"stop\"", {});
  }

private:
  // ------------------------------------------------------------------ helpers
  std::string relative(const std::string & p) const
  {
    std::error_code ec;
    auto r = fs::relative(p, root_, ec);
    return ec ? p : r.string();
  }

  void logEvent(
    const std::string & kind, const std::string & topic, const std::string & value,
    const std::map<std::string, std::string> & extra, const std::string & prev = "")
  {
    Event ev;
    ev.kind = kind;
    ev.topic = topic;
    ev.value = value;
    ev.prev = prev;
    ev.extra = extra;
    events_->write(wallNs(), ev, pose(), relative(current_file_));
  }

  void createWriterTopic(const std::string & name, const std::string & type, const rclcpp::QoS & qos)
  {
    if (created_topics_.count(name)) {
      return;
    }
    rosbag2_storage::TopicMetadata md;
    md.name = name;
    md.type = type;
    md.serialization_format = "cdr";
    md.offered_qos_profiles = {qos};
    // own schema text: rosbag2 leaves GoalStatusArray empty (see msg_definition.hpp)
    if (const auto def = ros2msgDefinition(type)) {
      rosbag2_storage::MessageDefinition d;
      d.topic_type = type;
      d.encoding = "ros2msg";
      d.encoded_message_definition = *def;
      writer_.create_topic(md, d);
    } else {
      writer_.create_topic(md);
    }
    created_topics_.insert(name);
  }

  void writeSerialized(
    Sub & s, const std::shared_ptr<const rclcpp::SerializedMessage> & msg, const rclcpp::Time & t)
  {
    try {
      writer_.write(msg, s.out_topic, s.out_type, t);
      ++s.written;
    } catch (const std::exception & e) {
      writeError(e.what());
    }
  }

  template<class MsgT>
  void writeTyped(Sub & s, const MsgT & msg, const rclcpp::Time & t)
  {
    try {
      writer_.write(msg, s.out_topic, t);
      ++s.written;
    } catch (const std::exception & e) {
      writeError(e.what());
    }
  }

  void writeError(const char * what)
  {
    const double now = steadyS();
    if (now - last_write_error_s_ > 60.0) {
      last_write_error_s_ = now;
      RCLCPP_ERROR(get_logger(), "write failed: %s", what);
      logEvent("recorder", "", "\"write_error\"", {{"error", jsonEscape(what)}});
    }
  }

  rclcpp::QoS defaultQos(const TopicCfg & c) const
  {
    rclcpp::QoS q(c.qos.depth.value_or(10));
    q.reliable();
    if (c.latched) {
      q.transient_local();
    }
    if (c.qos.reliability && *c.qos.reliability == "best_effort") {
      q.best_effort();
    }
    if (c.qos.durability) {
      if (*c.qos.durability == "transient_local") {
        q.transient_local();
      } else {
        q.durability_volatile();
      }
    }
    return q;
  }

  // match the publishers like ros2 bag record does: reliable only if every publisher is reliable,
  // transient_local only if every publisher is transient_local. Config overrides win.
  rclcpp::QoS adaptQos(const TopicCfg & c, const std::vector<rclcpp::TopicEndpointInfo> & pubs) const
  {
    bool all_reliable = true;
    bool all_tl = !pubs.empty();
    for (const auto & p : pubs) {
      if (p.qos_profile().reliability() != rclcpp::ReliabilityPolicy::Reliable) {
        all_reliable = false;
      }
      if (p.qos_profile().durability() != rclcpp::DurabilityPolicy::TransientLocal) {
        all_tl = false;
      }
    }
    // depth 10 also for latched topics: /tf_static has several latched publishers that all
    // deliver at once on subscription; depth 1 would keep only one of them.
    rclcpp::QoS q(c.qos.depth.value_or(10));
    if (all_reliable) {q.reliable();} else {q.best_effort();}
    if (all_tl) {q.transient_local();} else {q.durability_volatile();}
    if (c.qos.reliability) {
      if (*c.qos.reliability == "best_effort") {q.best_effort();} else {q.reliable();}
    }
    if (c.qos.durability) {
      if (*c.qos.durability == "transient_local") {q.transient_local();} else {
        q.durability_volatile();
      }
    }
    return q;
  }

  // ------------------------------------------------------------------ discovery
  void discover()
  {
    std::map<std::string, std::vector<std::string>> graph;
    try {
      graph = get_topic_names_and_types();
    } catch (const std::exception &) {
      return;
    }
    const bool timeout = (steadyS() - start_s_) > missing_timeout_s_;
    for (auto & kv : subs_) {
      Sub & s = *kv.second;
      if (s.subscribed || s.failed) {
        continue;
      }
      auto pubs = get_publishers_info_by_topic(s.cfg.topic);
      auto it = graph.find(s.cfg.topic);
      if (it != graph.end() && !it->second.empty() && !pubs.empty()) {
        subscribe(s, it->second.front(), adaptQos(s.cfg, pubs));
        if (s.missing_reported) {
          logEvent("recorder", s.cfg.topic, "\"topic_appeared\"", {});
        }
      } else if (timeout && !s.missing_reported) {
        s.missing_reported = true;
        RCLCPP_WARN(get_logger(), "topic %s not seen after %.0f s", s.cfg.topic.c_str(),
          missing_timeout_s_);
        logEvent("recorder", s.cfg.topic, "\"missing_topic\"", {});
      }
    }
  }

  void subscribe(Sub & s, const std::string & type, const rclcpp::QoS & qos)
  {
    s.type = type;
    const TopicCfg & c = s.cfg;
    if (c.mode == Mode::Scan && type != "sensor_msgs/msg/LaserScan") {
      failType(s, "scan entry is not sensor_msgs/msg/LaserScan");
      return;
    }
    if (c.mode == Mode::Grid && type != "nav2_msgs/msg/Costmap") {
      failType(s, "grid entry is not nav2_msgs/msg/Costmap");
      return;
    }
    if (c.mode == Mode::Tf && type != "tf2_msgs/msg/TFMessage") {
      failType(s, "tf entry is not tf2_msgs/msg/TFMessage");
      return;
    }
    try {
      for (auto * w : s.watches) {
        if (w->cfg.when != "action_status" && !s.reader) {
          s.reader = std::make_unique<FieldReader>(type);
        }
      }
      if (c.mode == Mode::Scan) {
        s.out_topic = c.output;
        s.out_type = "nav_data_recorder_msgs/msg/CompactScan";
        s.scan_enc = std::make_unique<ScanEncoder>(static_cast<uint16_t>(c.decimation), c.keyframe_s);
      } else if (c.mode == Mode::Grid) {
        s.out_topic = c.output;
        s.out_type = "nav_data_recorder_msgs/msg/CompactGrid";
        s.grid_enc = std::make_unique<GridEncoder>(c.keyframe_s);
      } else {
        s.out_topic = c.topic;
        s.out_type = type;
      }
      if (c.record) {
        rclcpp::QoS offered = qos;
        if (c.latched) {
          offered.transient_local();  // so ros2 bag play re-publishes it latched
        }
        createWriterTopic(s.out_topic, s.out_type, offered);
      }
      Sub * sp = &s;
      s.gsub = create_generic_subscription(
        c.topic, type, qos,
        [this, sp](std::shared_ptr<const rclcpp::SerializedMessage> msg,
        const rclcpp::MessageInfo & info) {onMessage(*sp, msg, info);});
      s.subscribed = true;
      RCLCPP_INFO(get_logger(), "subscribed %s [%s] %s%s", c.topic.c_str(), type.c_str(),
        qos.reliability() == rclcpp::ReliabilityPolicy::Reliable ? "reliable" : "best_effort",
        qos.durability() == rclcpp::DurabilityPolicy::TransientLocal ? " transient_local" : "");
    } catch (const std::exception & e) {
      failType(s, e.what());
    }
  }

  void failType(Sub & s, const std::string & why)
  {
    s.failed = true;
    RCLCPP_ERROR(get_logger(), "cannot record %s [%s]: %s", s.cfg.topic.c_str(), s.type.c_str(),
      why.c_str());
    logEvent("recorder", s.cfg.topic, "\"type_load_failed\"", {{"error", jsonEscape(why)},
        {"type", jsonEscape(s.type)}});
  }

  // ------------------------------------------------------------------ message path
  void onMessage(
    Sub & s, const std::shared_ptr<const rclcpp::SerializedMessage> & msg,
    const rclcpp::MessageInfo & info)
  {
    ++s.rx;
    const rclcpp::Time now = this->now();
    if (!s.watches.empty()) {
      runWatches(s, *msg);
    }
    if (s.cfg.topic == "/tf_static") {
      cacheStaticTf(*msg);
    }
    if (!s.cfg.record) {
      return;
    }
    switch (s.cfg.mode) {
      case Mode::Pass:
        if (s.cfg.latched) {
          const auto & gid = info.get_rmw_message_info().publisher_gid;
          s.latched_last[std::string(reinterpret_cast<const char *>(gid.data), sizeof(gid.data))] = msg;
          if (s.latched_last.size() > 64) {
            s.latched_last.erase(s.latched_last.begin());
          }
        }
        if (s.throttle.accept(now.nanoseconds())) {
          writeSerialized(s, msg, now);
        }
        break;
      case Mode::Tf:
        onTf(s, msg, now);
        break;
      case Mode::Scan:
        if (s.throttle.accept(now.nanoseconds())) {
          sensor_msgs::msg::LaserScan scan;
          scan_ser_.deserialize_message(msg.get(), &scan);
          nav_data_recorder_msgs::msg::CompactScan out;
          s.scan_enc->encode(scan, s.cfg.topic, steadyS(), out);
          writeTyped(s, out, now);
        }
        break;
      case Mode::Grid:
        if (s.throttle.accept(now.nanoseconds())) {
          nav2_msgs::msg::Costmap cm;
          grid_ser_.deserialize_message(msg.get(), &cm);
          nav_data_recorder_msgs::msg::CompactGrid out;
          s.grid_enc->encode(cm, s.cfg.topic, steadyS(), out);
          writeTyped(s, out, now);
        }
        break;
    }
  }

  // /tf_static completes the map -> base chain for the event pose (e.g. base_footprint -> base_link)
  void cacheStaticTf(const rclcpp::SerializedMessage & msg)
  {
    tf2_msgs::msg::TFMessage tf;
    try {
      tf_ser_.deserialize_message(&msg, &tf);
    } catch (const std::exception &) {
      return;
    }
    for (const auto & t : tf.transforms) {
      tf2::Transform x;
      x.setOrigin({t.transform.translation.x, t.transform.translation.y, t.transform.translation.z});
      x.setRotation({t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z,
          t.transform.rotation.w});
      tf_cache_[t.child_frame_id] = {t.header.frame_id, x};
    }
  }

  void onTf(Sub & s, const std::shared_ptr<const rclcpp::SerializedMessage> & msg, const rclcpp::Time & now)
  {
    tf2_msgs::msg::TFMessage tf;
    try {
      tf_ser_.deserialize_message(msg.get(), &tf);
    } catch (const std::exception &) {
      return;
    }
    bool keep = !s.throttle.enabled();
    for (const auto & t : tf.transforms) {
      tf2::Transform x;
      x.setOrigin({t.transform.translation.x, t.transform.translation.y, t.transform.translation.z});
      x.setRotation({t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z,
          t.transform.rotation.w});
      tf_cache_[t.child_frame_id] = {t.header.frame_id, x};
      if (s.throttle.enabled()) {
        auto it = s.tf_throttle.find(t.child_frame_id);
        if (it == s.tf_throttle.end()) {
          it = s.tf_throttle.emplace(t.child_frame_id, Throttle(s.cfg.rate)).first;
        }
        // evaluate every child so each gate advances on its own
        if (it->second.accept(now.nanoseconds())) {
          keep = true;
        }
      }
    }
    if (keep) {
      writeSerialized(s, msg, now);
    }
  }

  std::optional<std::array<double, 3>> pose() const
  {
    tf2::Transform acc;
    acc.setIdentity();
    std::string f = base_frame_;
    for (int hop = 0; hop < 8; ++hop) {
      if (f == map_frame_) {
        return std::array<double, 3>{acc.getOrigin().x(), acc.getOrigin().y(),
          tf2::getYaw(acc.getRotation())};
      }
      auto it = tf_cache_.find(f);
      if (it == tf_cache_.end()) {
        return std::nullopt;
      }
      acc = it->second.second * acc;
      f = it->second.first;
    }
    return std::nullopt;
  }

  // ------------------------------------------------------------------ events
  void runWatches(Sub & s, const rclcpp::SerializedMessage & msg)
  {
    bool loaded = false;
    for (EventWatch * w : s.watches) {
      if (w->cfg.when == "action_status") {
        actionWatch(s, *w, msg);
        continue;
      }
      if (!s.reader) {
        continue;
      }
      if (!loaded) {
        if (!s.reader->load(msg)) {
          return;
        }
        loaded = true;
      }
      for (const auto & spec : w->cfg.fields) {
        // "!name": active-low bit (e.g. estop_ok is true while OK);
        // the rule sees the inverted value, the log shows the raw one
        const bool inv = !spec.empty() && spec[0] == '!';
        const std::string field = inv ? spec.substr(1) : spec;
        auto raw = s.reader->get(field);
        if (!raw) {
          continue;
        }
        auto v = raw;
        if (inv && v->kind == FieldValue::Kind::Bool) {
          v->b = !v->b;
        }
        auto & last = w->last[field];
        const std::string topic_field = w->cfg.fields.size() > 1 || field != "data" ?
          s.cfg.topic + "." + field : s.cfg.topic;
        bool fire = false;
        if (w->cfg.when == "any") {
          fire = true;
        } else if (w->cfg.when == "change") {
          fire = !last || *last != *v;
        } else if (w->cfg.when == "nonzero") {
          fire = v->isNonZero() && (!last || *last != *v);
        } else if (w->cfg.when == "rising") {
          fire = v->isNonZero() && (!last || !last->isNonZero());
        }
        if (fire) {
          std::string prev;
          if (last) {
            FieldValue lr = *last;
            if (inv && lr.kind == FieldValue::Kind::Bool) {
              lr.b = !lr.b;
            }
            prev = lr.toJson();
          }
          logEvent(w->cfg.kind, topic_field, raw->toJson(), {}, prev);
        }
        last = v;
      }
    }
  }

  void actionWatch(Sub & s, EventWatch & w, const rclcpp::SerializedMessage & msg)
  {
    action_msgs::msg::GoalStatusArray arr;
    try {
      status_ser_.deserialize_message(&msg, &arr);
    } catch (const std::exception &) {
      return;
    }
    static const char * names[] = {"UNKNOWN", "ACCEPTED", "EXECUTING", "CANCELING", "SUCCEEDED",
      "CANCELED", "ABORTED"};
    for (const auto & st : arr.status_list) {
      char id[40];
      const auto & u = st.goal_info.goal_id.uuid;
      std::snprintf(id, sizeof(id), "%02x%02x%02x%02x-%02x%02x", u[0], u[1], u[2], u[3], u[4], u[5]);
      auto it = w.goal_status.find(id);
      if (it != w.goal_status.end() && it->second == st.status) {
        continue;
      }
      const std::string prev = it == w.goal_status.end() ? "" :
        jsonEscape(names[std::min<int>(std::max<int>(it->second, 0), 6)]);
      w.goal_status[id] = st.status;
      if (it == w.goal_status.end()) {
        w.goal_order.push_back(id);
        if (w.goal_order.size() > 256) {
          w.goal_status.erase(w.goal_order.front());
          w.goal_order.pop_front();
        }
      }
      const int code = std::min<int>(std::max<int>(st.status, 0), 6);
      // with fail_field the result decides: some action servers end every non-success as
      // ABORTED (also a reroute or an operator stop) and put the real reason in a result field
      const std::string kind = st.status == action_msgs::msg::GoalStatus::STATUS_ABORTED &&
        w.cfg.fail_field.empty() ? "error" : w.cfg.kind;
      logEvent(kind, s.cfg.topic, jsonEscape(names[code]), {{"goal", jsonEscape(id)}}, prev);
      // only goals seen running: a status array also lists goals that ended before the recorder
      // started (kept for the server's result timeout, 15 min)
      if (!w.cfg.result_type.empty() && !prev.empty() &&
        code >= action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)
      {
        requestResult(w, u, id, names[code]);
      }
    }
  }

  // ------------------------------------------------------------------ action results
  struct PendingResult
  {
    EventWatch * w;
    std::string id;
    std::string status;
    std::shared_future<std::shared_ptr<void>> future;
    int64_t request_id;
    double sent_s;
  };

  void requestResult(
    EventWatch & w, const std::array<uint8_t, 16> & uuid, const std::string & id,
    const char * status)
  {
    if (!w.fetch) {
      w.fetch = std::make_unique<ResultFetch>();
      auto & f = *w.fetch;
      const auto pos = w.cfg.topic.rfind("/_action/status");
      f.action = pos == std::string::npos ? w.cfg.topic : w.cfg.topic.substr(0, pos);
      try {
        f.req = std::make_unique<FieldReader>(w.cfg.result_type + "_GetResult_Request");
        f.resp = std::make_unique<FieldReader>(w.cfg.result_type + "_GetResult_Response");
        f.client = create_generic_client(
          f.action + "/_action/get_result", w.cfg.result_type + "_GetResult");
      } catch (const std::exception & e) {
        f.failed = true;
        RCLCPP_ERROR(get_logger(), "%s: cannot fetch results (%s): %s", f.action.c_str(),
          w.cfg.result_type.c_str(), e.what());
        logEvent("recorder", f.action, "\"type_load_failed\"", {
            {"type", jsonEscape(w.cfg.result_type)}, {"error", jsonEscape(e.what())}});
      }
    }
    auto & f = *w.fetch;
    if (f.failed) {
      return;
    }
    if (!f.client->service_is_ready()) {
      RCLCPP_DEBUG(get_logger(), "%s get_result not available", f.action.c_str());
      return;
    }
    if (!f.req->setBytes("goal_id.uuid", uuid.data(), uuid.size())) {
      f.failed = true;
      RCLCPP_ERROR(get_logger(), "%s: request has no goal_id.uuid", f.action.c_str());
      return;
    }
    auto fr = f.client->async_send_request(f.req->data());
    pending_results_.push_back({&w, id, status, fr.future.share(), fr.request_id, steadyS()});
  }

  void pollResults()
  {
    const double now = steadyS();
    for (auto it = pending_results_.begin(); it != pending_results_.end(); ) {
      if (it->future.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        onResult(*it, it->future.get());
        it = pending_results_.erase(it);
      } else if (now - it->sent_s > kResultTimeoutS) {
        it->w->fetch->client->remove_pending_request(it->request_id);
        RCLCPP_WARN(get_logger(), "%s get_result timed out (goal %s)",
          it->w->fetch->action.c_str(), it->id.c_str());
        it = pending_results_.erase(it);
      } else {
        ++it;
      }
    }
  }

  void onResult(const PendingResult & p, const std::shared_ptr<void> & resp)
  {
    const EventCfg & c = p.w->cfg;
    const ResultFetch & f = *p.w->fetch;
    std::string fields;
    // without fail_field the goal status decides (ABORTED = failure)
    bool fail = c.fail_field.empty() && p.status == "ABORTED";
    for (const auto & path : c.result_fields) {
      const auto v = f.resp->getFrom(resp.get(), path);
      const auto dot = path.rfind('.');
      const std::string key = dot == std::string::npos ? path : path.substr(dot + 1);
      fields += (fields.empty() ? "" : ",") + jsonEscape(key) + ":" + (v ? v->toJson() : "null");
      if (v && path == c.fail_field) {
        const int64_t x = v->kind == FieldValue::Kind::UInt ? static_cast<int64_t>(v->u) :
          v->kind == FieldValue::Kind::Bool ? static_cast<int64_t>(v->b) : v->i;
        fail = std::find(c.fail_values.begin(), c.fail_values.end(), x) != c.fail_values.end();
      }
    }
    logEvent(fail ? "error" : c.kind, f.action + "/result", "{" + fields + "}",
      {{"goal", jsonEscape(p.id)}, {"status", jsonEscape(p.status)}});
    std_msgs::msg::String m;
    m.data = "{\"action\":" + jsonEscape(f.action) + ",\"goal\":" + jsonEscape(p.id) +
      ",\"status\":" + jsonEscape(p.status) + ",\"fail\":" + (fail ? "true" : "false") +
      (fields.empty() ? "" : ",") + fields + "}";
    try {
      writer_.write(m, kResultTopic, this->now());
    } catch (const std::exception & e) {
      writeError(e.what());
    }
  }

  // ------------------------------------------------------------------ split / keyframes
  void writeInfo()
  {
    std::string latched, compact;
    for (const auto & c : cfg_.topics) {
      if (c.latched && c.record) {
        latched += (latched.empty() ? "" : ",") + jsonEscape(c.topic);
      }
      if ((c.mode == Mode::Scan || c.mode == Mode::Grid) && c.record) {
        compact += (compact.empty() ? "" : ",") + jsonEscape(c.output) + ":" + jsonEscape(c.topic);
      }
    }
    std_msgs::msg::String m;
    m.data = "{\"version\":" + jsonEscape(kVersion) + ",\"robot_id\":" + jsonEscape(robot_id_) +
      ",\"session\":" + jsonEscape(session_name_) + ",\"latched\":[" + latched + "]" +
      ",\"compact\":{" + compact + "}" +
      ",\"global_keyframe\":" + jsonEscape(cfg_.global_keyframe.enabled ? cfg_.global_keyframe.output : "") +
      ",\"global_diff\":" + jsonEscape(globalDiffTopic()) +
      ",\"global_occupancy\":" + jsonEscape(cfg_.global_keyframe.occupancy_output) + "}";
    try {
      writer_.write(m, kInfoTopic, this->now());
    } catch (const std::exception & e) {
      writeError(e.what());
    }
  }

  // full topic of the diff stream that the keyframe belongs to: <ns>/<layer>/get_costmap -> .../costmap_raw
  std::string globalDiffTopic() const
  {
    const std::string & svc = cfg_.global_keyframe.service;
    const auto pos = svc.rfind("/get_costmap");
    return pos == std::string::npos ? "" : svc.substr(0, pos) + "/costmap_raw";
  }

  static void fsyncPath(const std::string & path, bool dir = false)
  {
    const int fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC | (dir ? O_DIRECTORY : 0));
    if (fd >= 0) {
      ::fsync(fd);
      ::close(fd);
    }
  }

  // Own thread: an fsync on a slow disk must never hold up the subscriptions.
  void syncLoop(double period_s)
  {
    const auto period = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::duration<double>(period_s));
    std::unique_lock<std::mutex> lk(sync_mu_);
    while (!sync_cv_.wait_for(lk, period, [this]() {return sync_stop_;})) {
      const std::string file = current_file_, dir = bag_dir_;
      lk.unlock();
      fsyncPath(file);
      fsyncPath(dir, true);
      fsyncPath(root_ + "/events/" + localDate(wallNs()) + ".jsonl");
      lk.lock();
    }
  }

  void split()
  {
    const std::string closed = current_file_;
    try {
      writer_.split_bagfile();
    } catch (const std::exception & e) {
      writeError(e.what());
      return;
    }
    writeInfo();
    const rclcpp::Time now = this->now();
    uint64_t rx = 0, wr = 0;
    for (auto & kv : subs_) {
      Sub & s = *kv.second;
      rx += s.rx;
      wr += s.written;
      if (s.cfg.latched && s.cfg.record) {
        for (const auto & kv2 : s.latched_last) {
          writeSerialized(s, kv2.second, now);
        }
      }
      if (s.scan_enc) {s.scan_enc->forceKeyframe();}
      if (s.grid_enc) {s.grid_enc->forceKeyframe();}
    }
    if (key_client_) {
      key_pending_ = true;
      serviceKeyframe();
    }
    RCLCPP_INFO(get_logger(), "split -> %s (rx %lu, written %lu)", current_file_.c_str(),
      static_cast<unsigned long>(rx), static_cast<unsigned long>(wr));
    logEvent("recorder", "", "\"split\"", {{"closed", jsonEscape(relative(closed))},
        {"rx", std::to_string(rx)}, {"written", std::to_string(wr)}});
  }

  void serviceKeyframe()
  {
    if (!key_client_) {
      return;
    }
    const double now = steadyS();
    if (key_inflight_ && now - key_sent_s_ > cfg_.global_keyframe.timeout_s) {
      key_client_->prune_pending_requests();
      key_inflight_ = false;
      RCLCPP_WARN(get_logger(), "get_costmap timed out");
    }
    if (!key_pending_ || key_inflight_ || !key_client_->service_is_ready()) {
      return;
    }
    auto req = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
    key_inflight_ = true;
    key_sent_s_ = now;
    key_client_->async_send_request(
      req, [this](rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedFuture f) {
        key_inflight_ = false;
        auto resp = f.get();
        if (!resp || resp->map.metadata.size_x == 0 || resp->map.metadata.size_y == 0) {
          return;  // not ready yet; retried on the next discovery tick
        }
        try {
          writer_.write(resp->map, cfg_.global_keyframe.output, this->now());
          key_pending_ = false;
        } catch (const std::exception & e) {
          writeError(e.what());
        }
      });
  }

  // ------------------------------------------------------------------ state
  std::string root_, bag_dir_, current_file_, robot_id_, session_name_, boot_;
  std::string map_frame_, base_frame_;
  double missing_timeout_s_{30.0};
  double start_s_{0.0};
  double last_write_error_s_{-1e9};
  RecorderConfig cfg_;
  rosbag2_cpp::Writer writer_;
  std::set<std::string> created_topics_;
  std::map<std::string, std::unique_ptr<Sub>> subs_;
  std::vector<std::unique_ptr<EventWatch>> watches_;
  std::unique_ptr<EventLog> events_;
  std::mutex sync_mu_;                 // current_file_ is read by the fsync thread
  std::condition_variable sync_cv_;
  bool sync_stop_{false};
  std::thread sync_thread_;
  std::unordered_map<std::string, std::pair<std::string, tf2::Transform>> tf_cache_;
  rclcpp::Serialization<sensor_msgs::msg::LaserScan> scan_ser_;
  rclcpp::Serialization<nav2_msgs::msg::Costmap> grid_ser_;
  rclcpp::Serialization<tf2_msgs::msg::TFMessage> tf_ser_;
  rclcpp::Serialization<action_msgs::msg::GoalStatusArray> status_ser_;
  rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr key_client_;
  bool key_pending_{false};
  bool key_inflight_{false};
  double key_sent_s_{0.0};
  std::vector<PendingResult> pending_results_;
  rclcpp::TimerBase::SharedPtr discovery_timer_, split_timer_, result_timer_;
};

}  // namespace nav_data_recorder

// The start script runs us in a terminal (e.g. gnome-terminal). When the terminal closes at
// shutdown we get SIGHUP: stop like Ctrl+C so the current file is closed with its index and
// metadata.yaml, instead of dying with the last chunk unwritten. raise() is async-signal-safe and
// runs rclcpp's own SIGINT handler.
extern "C" void navDataOnHangup(int)
{
  std::raise(SIGINT);
}

int main(int argc, char ** argv)
{
  // stdout may be a pipe to ros2 launch; if launch dies first, a log line must not kill the
  // recorder (SIGPIPE) before it has closed the bag
  std::signal(SIGPIPE, SIG_IGN);
  rclcpp::init(argc, argv);             // installs the SIGINT / SIGTERM handlers
  std::signal(SIGHUP, navDataOnHangup);
  int rc = 0;
  try {
    auto node = std::make_shared<nav_data_recorder::RecorderNode>(rclcpp::NodeOptions());
    rclcpp::spin(node);
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("nav_data_recorder"), "%s", e.what());
    rc = 1;
  }
  rclcpp::shutdown();
  return rc;
}
