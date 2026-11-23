// JSONL event log: <root>/events/YYYYMMDD.jsonl (local date), one object per line, flushed per line.
#ifndef NAV_DATA_RECORDER__EVENT_LOG_HPP_
#define NAV_DATA_RECORDER__EVENT_LOG_HPP_

#include <array>
#include <cstdio>
#include <map>
#include <optional>
#include <string>

namespace nav_data_recorder
{

std::string jsonEscape(const std::string & s);
// "2026-11-20T15:02:11.482+01:00" for a wall-clock time in ns since epoch
std::string isoLocal(int64_t wall_ns);
// "20261120" local date
std::string localDate(int64_t wall_ns);

struct Event
{
  std::string kind;   // state | error | safety | mark | recorder
  std::string topic;
  std::string value;  // already JSON-encoded value (string with quotes, number, bool, object)
  std::string prev;   // JSON-encoded or empty -> omitted
  std::map<std::string, std::string> extra;  // key -> JSON-encoded value
};

class EventLog
{
public:
  explicit EventLog(std::string events_dir);
  ~EventLog();

  // pose: map-frame x, y, yaw if known; file: current mcap relative to root
  void write(
    int64_t wall_ns, const Event & ev, const std::optional<std::array<double, 3>> & pose,
    const std::string & file);

private:
  void openFor(const std::string & date);

  std::string dir_;
  std::string date_;
  std::FILE * fp_{nullptr};
};

}  // namespace nav_data_recorder

#endif  // NAV_DATA_RECORDER__EVENT_LOG_HPP_
