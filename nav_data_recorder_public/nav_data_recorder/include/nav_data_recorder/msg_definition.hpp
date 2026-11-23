// ros2msg message definition text for the mcap schema, built from the installed .msg files.
// rosbag2 0.26.9 (Jazzy) leaves the schema of action_msgs/msg/GoalStatusArray empty: its
// LocalMessageDefinitionSource treats that type as an action type and finds nothing. Lichtblick
// then cannot decode /<action>/_action/status. Unqualified nested names ("GoalStatus[]") are
// qualified with the enclosing package.
#ifndef NAV_DATA_RECORDER__MSG_DEFINITION_HPP_
#define NAV_DATA_RECORDER__MSG_DEFINITION_HPP_

#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace nav_data_recorder
{

// "pkg/msg/Name" -> .msg text, or nullopt if not found
using MsgFileReader = std::function<std::optional<std::string>(const std::string & pkg,
    const std::string & name)>;

// reads <share of pkg>/msg/<name>.msg via the ament index
std::optional<std::string> readInstalledMsg(const std::string & pkg, const std::string & name);

// root text followed by "=====...\nMSG: pkg/Name\n<text>" for every nested type (depth first,
// each once), like rosbag2's ros2msg encoding. nullopt if any .msg file is missing.
std::optional<std::string> ros2msgDefinition(
  const std::string & type, const MsgFileReader & reader = readInstalledMsg);

// field types referenced by a .msg text, qualified as "pkg/Name" (constants and builtins skipped)
std::vector<std::string> msgDependencies(const std::string & text, const std::string & pkg);

}  // namespace nav_data_recorder

#endif  // NAV_DATA_RECORDER__MSG_DEFINITION_HPP_
