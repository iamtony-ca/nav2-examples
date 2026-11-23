// Reads scalar/string fields of any message type at run time (rosidl introspection), so event rules
// can watch custom messages (std_msgs/String, a safety PLC status, a diagnostics message ...)
// without build deps.
#ifndef NAV_DATA_RECORDER__FIELD_READER_HPP_
#define NAV_DATA_RECORDER__FIELD_READER_HPP_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/serialized_message.hpp"
#include "rcpputils/shared_library.hpp"
#include "rosidl_runtime_c/message_type_support_struct.h"

namespace nav_data_recorder
{

struct FieldValue
{
  enum class Kind {Bool, Int, UInt, Double, String} kind{Kind::Int};
  bool b{false};
  int64_t i{0};
  uint64_t u{0};
  double d{0.0};
  std::string s;

  std::string toJson() const;
  // numeric view; strings are "nonzero" when not empty
  bool isNonZero() const;
  bool operator==(const FieldValue & o) const;
  bool operator!=(const FieldValue & o) const {return !(*this == o);}
};

class FieldReader
{
public:
  // throws std::runtime_error if the type cannot be loaded
  explicit FieldReader(const std::string & type);
  ~FieldReader();

  // deserializes once; returns false on failure
  bool load(const rclcpp::SerializedMessage & msg);
  // "a.b.c" into nested messages; arrays are not supported
  std::optional<FieldValue> get(const std::string & path) const;
  // same, on another in-memory message of this type (e.g. a GenericClient response)
  std::optional<FieldValue> getFrom(const void * msg, const std::string & path) const;
  // writes a fixed-size uint8/octet array field (e.g. goal_id.uuid) of the internal buffer
  bool setBytes(const std::string & path, const uint8_t * data, size_t n);
  // the internal message buffer (initialized), e.g. to send as a GenericClient request
  void * data() {return buf_;}

private:
  struct Resolved
  {
    size_t offset{0};
    uint8_t type_id{0};
    size_t array_size{0};  // fixed-size array length (0 = not an array)
    bool ok{false};
  };
  Resolved resolve(const std::string & path, bool allow_fixed_array = false) const;

  std::shared_ptr<rcpputils::SharedLibrary> ts_lib_;
  std::shared_ptr<rcpputils::SharedLibrary> intro_lib_;
  const rosidl_message_type_support_t * ts_{nullptr};
  const void * members_{nullptr};  // MessageMembers
  void * buf_{nullptr};
  bool loaded_{false};
};

}  // namespace nav_data_recorder

#endif  // NAV_DATA_RECORDER__FIELD_READER_HPP_
