#include <gtest/gtest.h>

#include <map>
#include <string>

#include "nav_data_recorder/msg_definition.hpp"

using nav_data_recorder::msgDependencies;
using nav_data_recorder::ros2msgDefinition;

TEST(MsgDefinition, Dependencies)
{
  const std::string text =
    "# comment\n"
    "int8 STATUS_UNKNOWN   = 0\n"
    "uint8 X=3\n"
    "GoalStatus[] status_list  # unqualified array\n"
    "builtin_interfaces/Time stamp\n"
    "geometry_msgs/msg/Pose pose\n"
    "string<=10 name\n"
    "float64[36] covariance\n"
    "Header header\n";
  const auto d = msgDependencies(text, "action_msgs");
  ASSERT_EQ(d.size(), 4u);
  EXPECT_EQ(d[0], "action_msgs/GoalStatus");
  EXPECT_EQ(d[1], "builtin_interfaces/Time");
  EXPECT_EQ(d[2], "geometry_msgs/Pose");
  EXPECT_EQ(d[3], "std_msgs/Header");
}

TEST(MsgDefinition, NestedOnceDepthFirst)
{
  std::map<std::string, std::string> files{
    {"a/Root", "B[] bs\nc/C c\n"},
    {"a/B", "c/C c\nint32 x\n"},
    {"c/C", "float64 v\n"}};
  auto rd = [&files](const std::string & p, const std::string & n) -> std::optional<std::string> {
      auto it = files.find(p + "/" + n);
      if (it == files.end()) {return std::nullopt;}
      return it->second;
    };
  const auto def = ros2msgDefinition("a/msg/Root", rd);
  ASSERT_TRUE(def);
  const std::string sep(80, '=');
  EXPECT_EQ(*def, "B[] bs\nc/C c\n" + sep + "\nMSG: a/B\nc/C c\nint32 x\n" + sep +
    "\nMSG: c/C\nfloat64 v\n");
  EXPECT_FALSE(ros2msgDefinition("a/msg/Missing", rd));
  EXPECT_FALSE(ros2msgDefinition("a/srv/Root", rd));
}

TEST(MsgDefinition, InstalledGoalStatusArray)
{
  const auto def = ros2msgDefinition("action_msgs/msg/GoalStatusArray");
  ASSERT_TRUE(def);
  EXPECT_NE(def->find("MSG: action_msgs/GoalStatus\n"), std::string::npos);
  EXPECT_NE(def->find("MSG: action_msgs/GoalInfo\n"), std::string::npos);
  EXPECT_NE(def->find("MSG: unique_identifier_msgs/UUID\n"), std::string::npos);
  EXPECT_NE(def->find("MSG: builtin_interfaces/Time\n"), std::string::npos);
}
