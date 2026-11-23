// nav_data_fix_schema <in.mcap> <out_bag_dir>
// Copies an mcap file and writes the schema text of every topic from the installed .msg files.
// rosbag2 0.26.9 leaves the schema of action_msgs/msg/GoalStatusArray empty (it treats that type as
// an action type), so Lichtblick cannot show action status topics. nav_data_export calls this on
// its output; rosbag2_py has no create_topic overload that takes a definition.
#include <iostream>
#include <memory>
#include <string>

#include "nav_data_recorder/msg_definition.hpp"
#include "rosbag2_cpp/reader.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "rosbag2_storage/storage_options.hpp"

int main(int argc, char ** argv)
{
  if (argc != 3) {
    std::cerr << "usage: nav_data_fix_schema <in.mcap> <out_bag_dir>\n";
    return 2;
  }
  try {
    rosbag2_cpp::Reader reader;
    rosbag2_storage::StorageOptions in;
    in.uri = argv[1];
    in.storage_id = "mcap";
    reader.open(in, rosbag2_cpp::ConverterOptions{"cdr", "cdr"});

    rosbag2_cpp::Writer writer;
    rosbag2_storage::StorageOptions out;
    out.uri = argv[2];
    out.storage_id = "mcap";
    out.storage_preset_profile = "zstd_fast";
    writer.open(out, rosbag2_cpp::ConverterOptions{"cdr", "cdr"});

    int fixed = 0;
    for (const auto & md : reader.get_all_topics_and_types()) {
      if (const auto def = nav_data_recorder::ros2msgDefinition(md.type)) {
        rosbag2_storage::MessageDefinition d;
        d.topic_type = md.type;
        d.encoding = "ros2msg";
        d.encoded_message_definition = *def;
        writer.create_topic(md, d);
        ++fixed;
      } else {
        writer.create_topic(md);
      }
    }
    size_t n = 0;
    while (reader.has_next()) {
      writer.write(reader.read_next());
      ++n;
    }
    writer.close();
    std::cout << "copied " << n << " messages, " << fixed << " schemas from .msg files\n";
  } catch (const std::exception & e) {
    std::cerr << "nav_data_fix_schema: " << e.what() << "\n";
    return 1;
  }
  return 0;
}
