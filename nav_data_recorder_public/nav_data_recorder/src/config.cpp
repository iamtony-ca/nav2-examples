#include "nav_data_recorder/config.hpp"

#include <set>
#include <stdexcept>

#include "yaml-cpp/yaml.h"

namespace nav_data_recorder
{

std::string absTopic(const std::string & t)
{
  if (t.empty() || t[0] == '/') {
    return t;
  }
  return "/" + t;
}

static QosOverride parseQos(const YAML::Node & n)
{
  QosOverride q;
  if (!n) {
    return q;
  }
  if (n["reliability"]) {
    q.reliability = n["reliability"].as<std::string>();
    if (*q.reliability != "reliable" && *q.reliability != "best_effort") {
      throw std::runtime_error("qos.reliability must be reliable or best_effort");
    }
  }
  if (n["durability"]) {
    q.durability = n["durability"].as<std::string>();
    if (*q.durability != "volatile" && *q.durability != "transient_local") {
      throw std::runtime_error("qos.durability must be volatile or transient_local");
    }
  }
  if (n["depth"]) {
    q.depth = n["depth"].as<int>();
  }
  return q;
}

static TopicCfg parseTopic(const YAML::Node & n, Mode mode)
{
  if (!n["topic"]) {
    throw std::runtime_error("entry without 'topic'");
  }
  TopicCfg c;
  c.topic = absTopic(n["topic"].as<std::string>());
  c.mode = mode;
  if (n["tf"] && n["tf"].as<bool>()) {
    c.mode = Mode::Tf;
  }
  c.rate = n["rate"] ? n["rate"].as<double>() : 0.0;
  c.latched = n["latched"] ? n["latched"].as<bool>() : false;
  c.record = n["record"] ? n["record"].as<bool>() : true;
  c.type = n["type"] ? n["type"].as<std::string>() : "";
  c.qos = parseQos(n["qos"]);
  c.decimation = n["decimation"] ? n["decimation"].as<int>() : 1;
  if (c.decimation < 1 || c.decimation > 1000) {
    throw std::runtime_error(c.topic + ": decimation must be 1..1000");
  }
  c.keyframe_s = n["keyframe_s"] ? n["keyframe_s"].as<double>() :
    (mode == Mode::Scan ? 1.0 : mode == Mode::Grid ? 10.0 : 0.0);
  c.output = n["output"] ? n["output"].as<std::string>() : "";
  if ((mode == Mode::Scan || mode == Mode::Grid) && c.output.empty()) {
    c.output = "/nav_data/compact" + c.topic;
  }
  return c;
}

RecorderConfig loadConfig(const std::string & path)
{
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const std::exception & e) {
    throw std::runtime_error("cannot read " + path + ": " + e.what());
  }
  RecorderConfig cfg;
  std::set<std::string> seen;
  auto add = [&](const YAML::Node & list, Mode mode) {
      if (!list) {
        return;
      }
      if (!list.IsSequence()) {
        throw std::runtime_error("section must be a list");
      }
      for (const auto & n : list) {
        TopicCfg c = parseTopic(n, mode);
        if (!seen.insert(c.topic).second) {
          throw std::runtime_error("topic listed twice: " + c.topic);
        }
        cfg.topics.push_back(c);
      }
    };
  try {
    add(root["passthrough"], Mode::Pass);
    add(root["scan"], Mode::Scan);
    add(root["grid"], Mode::Grid);

    if (const auto gk = root["global_keyframe"]) {
      cfg.global_keyframe.enabled = gk["enabled"] ? gk["enabled"].as<bool>() : true;
      cfg.global_keyframe.service = absTopic(gk["service"].as<std::string>());
      if (gk["output"]) {
        cfg.global_keyframe.output = absTopic(gk["output"].as<std::string>());
      }
      if (gk["timeout_s"]) {
        cfg.global_keyframe.timeout_s = gk["timeout_s"].as<double>();
      }
      if (gk["occupancy_output"]) {
        cfg.global_keyframe.occupancy_output = absTopic(gk["occupancy_output"].as<std::string>());
      }
    }

    if (const auto ev = root["events"]) {
      for (const auto & n : ev) {
        EventCfg e;
        e.topic = absTopic(n["topic"].as<std::string>());
        e.kind = n["kind"] ? n["kind"].as<std::string>() : "state";
        e.when = n["when"] ? n["when"].as<std::string>() : "change";
        if (n["field"]) {
          e.fields.push_back(n["field"].as<std::string>());
        }
        if (n["fields"]) {
          for (const auto & f : n["fields"]) {
            e.fields.push_back(f.as<std::string>());
          }
        }
        static const std::set<std::string> whens{"change", "nonzero", "rising", "any",
          "action_status"};
        if (!whens.count(e.when)) {
          throw std::runtime_error(e.topic + ": unknown when '" + e.when + "'");
        }
        if (const auto r = n["result"]) {
          if (e.when != "action_status") {
            throw std::runtime_error(e.topic + ": result needs when: action_status");
          }
          e.result_type = r["type"].as<std::string>();
          if (r["fields"]) {
            for (const auto & f : r["fields"]) {
              e.result_fields.push_back(f.as<std::string>());
            }
          }
          if (r["fail_field"]) {
            e.fail_field = r["fail_field"].as<std::string>();
            for (const auto & v : r["fail_values"]) {
              e.fail_values.push_back(v.as<int64_t>());
            }
          }
          if (e.result_type.find("/action/") == std::string::npos) {
            throw std::runtime_error(e.topic + ": result type must be <pkg>/action/<Name>");
          }
        }
        if (e.when != "action_status" && e.fields.empty()) {
          throw std::runtime_error(e.topic + ": event needs field or fields");
        }
        cfg.events.push_back(e);
      }
    }
  } catch (const YAML::Exception & e) {
    throw std::runtime_error(path + ": " + e.what());
  }
  return cfg;
}

}  // namespace nav_data_recorder
