// topics.yaml model. See config/topics.yaml for the format.
#ifndef NAV_DATA_RECORDER__CONFIG_HPP_
#define NAV_DATA_RECORDER__CONFIG_HPP_

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace nav_data_recorder
{

enum class Mode {Pass, Tf, Scan, Grid};

struct QosOverride
{
  std::optional<std::string> reliability;  // reliable | best_effort
  std::optional<std::string> durability;   // volatile | transient_local
  std::optional<int> depth;
};

struct TopicCfg
{
  std::string topic;
  Mode mode{Mode::Pass};
  double rate{0.0};          // 0 = keep all
  bool latched{false};       // cache last message, re-write at every file start
  bool record{true};         // false = subscribe only for events
  std::string type;          // optional: subscribe at start without waiting for discovery
  QosOverride qos;
  // scan
  int decimation{1};
  // scan, grid
  double keyframe_s{0.0};    // 0 -> default (scan 1 s, grid 10 s)
  std::string output;        // recorded topic name for scan/grid; default /nav_data/compact<topic>
};

struct EventCfg
{
  std::string topic;
  std::string kind{"state"};          // state | error | safety | mark
  std::vector<std::string> fields;    // field paths; empty for action status
  std::string when{"change"};         // change | nonzero | rising | any | action_status
  // action_status only: when a goal ends, fetch its result with the action's get_result service
  // (results are not on any topic). Logged as an event and written to /nav_data/action_result.
  std::string result_type;                 // e.g. my_msgs/action/MyTask
  std::vector<std::string> result_fields;  // paths in <type>_GetResult_Response, e.g. result.result_code
  std::string fail_field;                  // one of result_fields
  std::vector<int64_t> fail_values;        // fail_field values that make the event kind "error"
};

struct GlobalKeyCfg
{
  bool enabled{false};
  std::string service;
  std::string output{"/nav_data/global_costmap_key"};
  double timeout_s{5.0};
  // OccupancyGrid topic nav_data_export writes the rebuilt costmap to ("" = <stream ns>/costmap)
  std::string occupancy_output;
};

struct RecorderConfig
{
  std::vector<TopicCfg> topics;
  std::vector<EventCfg> events;
  GlobalKeyCfg global_keyframe;
};

// throws std::runtime_error with a readable message on bad input
RecorderConfig loadConfig(const std::string & path);

// "/foo" for "foo"; leaves absolute names alone
std::string absTopic(const std::string & t);

}  // namespace nav_data_recorder

#endif  // NAV_DATA_RECORDER__CONFIG_HPP_
