#include "nav_data_recorder/event_log.hpp"

#include <ctime>
#include <filesystem>
#include <sstream>

namespace nav_data_recorder
{

std::string jsonEscape(const std::string & s)
{
  std::string o;
  o.reserve(s.size() + 2);
  o.push_back('"');
  for (unsigned char c : s) {
    switch (c) {
      case '"': o += "\\\""; break;
      case '\\': o += "\\\\"; break;
      case '\n': o += "\\n"; break;
      case '\r': o += "\\r"; break;
      case '\t': o += "\\t"; break;
      default:
        if (c < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x", c);
          o += buf;
        } else {
          o.push_back(static_cast<char>(c));  // UTF-8 passes through
        }
    }
  }
  o.push_back('"');
  return o;
}

static std::tm localTm(int64_t wall_ns)
{
  std::time_t t = static_cast<std::time_t>(wall_ns / 1000000000LL);
  std::tm tm{};
  localtime_r(&t, &tm);
  return tm;
}

std::string isoLocal(int64_t wall_ns)
{
  std::tm tm = localTm(wall_ns);
  char date[32];
  std::strftime(date, sizeof(date), "%Y-%m-%dT%H:%M:%S", &tm);
  const int ms = static_cast<int>((wall_ns / 1000000LL) % 1000);
  const long off = tm.tm_gmtoff;
  char buf[64];
  std::snprintf(
    buf, sizeof(buf), "%s.%03d%c%02ld:%02ld", date, ms, off < 0 ? '-' : '+',
    std::labs(off) / 3600, (std::labs(off) % 3600) / 60);
  return buf;
}

std::string localDate(int64_t wall_ns)
{
  std::tm tm = localTm(wall_ns);
  char buf[16];
  std::strftime(buf, sizeof(buf), "%Y%m%d", &tm);
  return buf;
}

EventLog::EventLog(std::string events_dir)
: dir_(std::move(events_dir))
{
  std::filesystem::create_directories(dir_);
}

EventLog::~EventLog()
{
  if (fp_) {
    std::fclose(fp_);
  }
}

void EventLog::openFor(const std::string & date)
{
  if (fp_ && date == date_) {
    return;
  }
  if (fp_) {
    std::fclose(fp_);
    fp_ = nullptr;
  }
  date_ = date;
  std::error_code ec;
  std::filesystem::create_directories(dir_, ec);
  fp_ = std::fopen((dir_ + "/" + date + ".jsonl").c_str(), "a");
}

void EventLog::write(
  int64_t wall_ns, const Event & ev, const std::optional<std::array<double, 3>> & pose,
  const std::string & file)
{
  openFor(localDate(wall_ns));
  if (!fp_) {
    return;
  }
  std::ostringstream o;
  o << "{\"t\":" << jsonEscape(isoLocal(wall_ns)) << ",\"kind\":" << jsonEscape(ev.kind);
  if (!ev.topic.empty()) {
    o << ",\"topic\":" << jsonEscape(ev.topic);
  }
  if (!ev.value.empty()) {
    o << ",\"value\":" << ev.value;
  }
  if (!ev.prev.empty()) {
    o << ",\"prev\":" << ev.prev;
  }
  for (const auto & kv : ev.extra) {
    o << "," << jsonEscape(kv.first) << ":" << kv.second;
  }
  if (pose) {
    char buf[96];
    std::snprintf(buf, sizeof(buf), "[%.2f,%.2f,%.2f]", (*pose)[0], (*pose)[1], (*pose)[2]);
    o << ",\"pose\":" << buf;
  }
  if (!file.empty()) {
    o << ",\"file\":" << jsonEscape(file);
  }
  o << "}\n";
  const std::string line = o.str();
  std::fwrite(line.data(), 1, line.size(), fp_);
  std::fflush(fp_);
}

}  // namespace nav_data_recorder
