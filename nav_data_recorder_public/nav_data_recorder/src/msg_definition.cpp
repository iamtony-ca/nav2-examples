#include "nav_data_recorder/msg_definition.hpp"

#include <fstream>
#include <set>
#include <sstream>

#include "ament_index_cpp/get_package_share_directory.hpp"

namespace nav_data_recorder
{

static const std::set<std::string> kBuiltins{
  "bool", "byte", "char", "float32", "float64", "int8", "uint8", "int16", "uint16", "int32",
  "uint32", "int64", "uint64", "string", "wstring"};

std::optional<std::string> readInstalledMsg(const std::string & pkg, const std::string & name)
{
  try {
    std::ifstream f(ament_index_cpp::get_package_share_directory(pkg) + "/msg/" + name + ".msg");
    if (!f) {
      return std::nullopt;
    }
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
  } catch (const std::exception &) {
    return std::nullopt;
  }
}

std::vector<std::string> msgDependencies(const std::string & text, const std::string & pkg)
{
  std::vector<std::string> out;
  std::istringstream in(text);
  std::string line;
  while (std::getline(in, line)) {
    const auto hash = line.find('#');
    if (hash != std::string::npos) {
      line.resize(hash);
    }
    std::istringstream ls(line);
    std::string type, name;
    if (!(ls >> type >> name)) {
      continue;
    }
    if (name.find('=') != std::string::npos) {
      continue;  // constant "TYPE NAME=VALUE"
    }
    std::string rest;
    if (ls >> rest && rest[0] == '=') {
      continue;  // constant "TYPE NAME = VALUE"
    }
    const auto br = type.find('[');
    if (br != std::string::npos) {
      type.resize(br);
    }
    const auto le = type.find("<=");  // bounded string
    if (le != std::string::npos) {
      type.resize(le);
    }
    if (kBuiltins.count(type)) {
      continue;
    }
    if (type == "Header") {
      out.push_back("std_msgs/Header");
    } else if (type.find('/') == std::string::npos) {
      out.push_back(pkg + "/" + type);
    } else {
      // "pkg/Name" or "pkg/msg/Name"
      const auto a = type.find('/');
      const auto b = type.rfind('/');
      out.push_back(type.substr(0, a) + "/" + type.substr(b + 1));
    }
  }
  return out;
}

std::optional<std::string> ros2msgDefinition(const std::string & type, const MsgFileReader & reader)
{
  const auto a = type.find('/');
  const auto b = type.rfind('/');
  if (a == std::string::npos || type.substr(a + 1, b - a - 1) != "msg") {
    return std::nullopt;
  }
  const std::string pkg = type.substr(0, a);
  const auto root = reader(pkg, type.substr(b + 1));
  if (!root) {
    return std::nullopt;
  }
  std::string out = *root;
  std::set<std::string> seen;
  // depth first, in field order (same order rosbag2 uses)
  std::vector<std::string> stack;
  auto push = [&stack](std::vector<std::string> deps) {
      for (auto it = deps.rbegin(); it != deps.rend(); ++it) {
        stack.push_back(*it);
      }
    };
  push(msgDependencies(*root, pkg));
  while (!stack.empty()) {
    const std::string dep = stack.back();
    stack.pop_back();
    if (!seen.insert(dep).second) {
      continue;
    }
    const auto slash = dep.find('/');
    const std::string dpkg = dep.substr(0, slash);
    const auto text = reader(dpkg, dep.substr(slash + 1));
    if (!text) {
      return std::nullopt;
    }
    if (!out.empty() && out.back() != '\n') {
      out += "\n";
    }
    out += std::string(80, '=') + "\nMSG: " + dep + "\n" + *text;
    push(msgDependencies(*text, dpkg));
  }
  return out;
}

}  // namespace nav_data_recorder
