#include "nav_data_recorder/field_reader.hpp"

#include <cstring>
#include <sstream>
#include <stdexcept>

#include "nav_data_recorder/event_log.hpp"
#include "rclcpp/typesupport_helpers.hpp"
#include "rmw/rmw.h"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

namespace nav_data_recorder
{

namespace rti = rosidl_typesupport_introspection_cpp;

std::string FieldValue::toJson() const
{
  switch (kind) {
    case Kind::Bool: return b ? "true" : "false";
    case Kind::Int: return std::to_string(i);
    case Kind::UInt: return std::to_string(u);
    case Kind::Double: {
        std::ostringstream o;
        o.precision(6);
        o << d;
        return o.str();
      }
    case Kind::String: return jsonEscape(s);
  }
  return "null";
}

bool FieldValue::isNonZero() const
{
  switch (kind) {
    case Kind::Bool: return b;
    case Kind::Int: return i != 0;
    case Kind::UInt: return u != 0;
    case Kind::Double: return d != 0.0;
    case Kind::String: return !s.empty();
  }
  return false;
}

bool FieldValue::operator==(const FieldValue & o) const
{
  if (kind != o.kind) {
    return false;
  }
  switch (kind) {
    case Kind::Bool: return b == o.b;
    case Kind::Int: return i == o.i;
    case Kind::UInt: return u == o.u;
    case Kind::Double: return d == o.d;
    case Kind::String: return s == o.s;
  }
  return false;
}

static const rti::MessageMembers * membersOf(const rosidl_message_type_support_t * h)
{
  const rosidl_message_type_support_t * ih = h->func ?
    h->func(h, rti::typesupport_identifier) : h;
  if (!ih) {
    ih = h;
  }
  return static_cast<const rti::MessageMembers *>(ih->data);
}

FieldReader::FieldReader(const std::string & type)
{
  ts_lib_ = rclcpp::get_typesupport_library(type, "rosidl_typesupport_cpp");
  ts_ = rclcpp::get_message_typesupport_handle(type, "rosidl_typesupport_cpp", *ts_lib_);
  intro_lib_ = rclcpp::get_typesupport_library(type, "rosidl_typesupport_introspection_cpp");
  const auto * ih = rclcpp::get_message_typesupport_handle(
    type, "rosidl_typesupport_introspection_cpp", *intro_lib_);
  members_ = membersOf(ih);
  const auto * m = static_cast<const rti::MessageMembers *>(members_);
  buf_ = ::operator new(m->size_of_, std::align_val_t(alignof(std::max_align_t)));
  m->init_function(buf_, rosidl_runtime_cpp::MessageInitialization::ALL);
}

FieldReader::~FieldReader()
{
  if (buf_) {
    const auto * m = static_cast<const rti::MessageMembers *>(members_);
    m->fini_function(buf_);
    ::operator delete(buf_, std::align_val_t(alignof(std::max_align_t)));
  }
}

bool FieldReader::load(const rclcpp::SerializedMessage & msg)
{
  const auto ret = rmw_deserialize(&msg.get_rcl_serialized_message(), ts_, buf_);
  loaded_ = (ret == RMW_RET_OK);
  return loaded_;
}

FieldReader::Resolved FieldReader::resolve(const std::string & path, bool allow_fixed_array) const
{
  Resolved r;
  const auto * m = static_cast<const rti::MessageMembers *>(members_);
  size_t base = 0;
  size_t start = 0;
  while (true) {
    const size_t dot = path.find('.', start);
    const std::string name = path.substr(start, dot == std::string::npos ? std::string::npos : dot - start);
    const rti::MessageMember * found = nullptr;
    for (uint32_t i = 0; i < m->member_count_; ++i) {
      if (name == m->members_[i].name_) {
        found = &m->members_[i];
        break;
      }
    }
    const bool last = dot == std::string::npos;
    const bool fixed_array = found && found->is_array_ && found->array_size_ > 0 &&
      !found->is_upper_bound_;
    if (!found || (found->is_array_ && !(last && allow_fixed_array && fixed_array))) {
      return r;
    }
    base += found->offset_;
    if (last) {
      r.offset = base;
      r.type_id = found->type_id_;
      r.array_size = found->is_array_ ? found->array_size_ : 0;
      r.ok = found->type_id_ != rti::ROS_TYPE_MESSAGE;
      return r;
    }
    if (found->type_id_ != rti::ROS_TYPE_MESSAGE) {
      return r;
    }
    m = membersOf(found->members_);
    start = dot + 1;
  }
}

bool FieldReader::setBytes(const std::string & path, const uint8_t * data, size_t n)
{
  const Resolved r = resolve(path, true);
  if (!r.ok || r.array_size != n ||
    (r.type_id != rti::ROS_TYPE_UINT8 && r.type_id != rti::ROS_TYPE_OCTET))
  {
    return false;
  }
  // a fixed-size array of bytes is a std::array<uint8_t, N>: contiguous at the member offset
  std::memcpy(static_cast<uint8_t *>(buf_) + r.offset, data, n);
  return true;
}

std::optional<FieldValue> FieldReader::get(const std::string & path) const
{
  if (!loaded_) {
    return std::nullopt;
  }
  return getFrom(buf_, path);
}

std::optional<FieldValue> FieldReader::getFrom(const void * msg, const std::string & path) const
{
  const Resolved r = resolve(path);
  if (!r.ok || !msg) {
    return std::nullopt;
  }
  const uint8_t * p = static_cast<const uint8_t *>(msg) + r.offset;
  FieldValue v;
  auto rd = [p](auto & out) {std::memcpy(&out, p, sizeof(out));};
  switch (r.type_id) {
    case rti::ROS_TYPE_BOOLEAN: {bool x; rd(x); v.kind = FieldValue::Kind::Bool; v.b = x; break;}
    case rti::ROS_TYPE_OCTET:
    case rti::ROS_TYPE_UINT8:
    case rti::ROS_TYPE_CHAR: {uint8_t x; rd(x); v.kind = FieldValue::Kind::UInt; v.u = x; break;}
    case rti::ROS_TYPE_INT8: {int8_t x; rd(x); v.kind = FieldValue::Kind::Int; v.i = x; break;}
    case rti::ROS_TYPE_UINT16: {uint16_t x; rd(x); v.kind = FieldValue::Kind::UInt; v.u = x; break;}
    case rti::ROS_TYPE_INT16: {int16_t x; rd(x); v.kind = FieldValue::Kind::Int; v.i = x; break;}
    case rti::ROS_TYPE_UINT32: {uint32_t x; rd(x); v.kind = FieldValue::Kind::UInt; v.u = x; break;}
    case rti::ROS_TYPE_INT32: {int32_t x; rd(x); v.kind = FieldValue::Kind::Int; v.i = x; break;}
    case rti::ROS_TYPE_UINT64: {uint64_t x; rd(x); v.kind = FieldValue::Kind::UInt; v.u = x; break;}
    case rti::ROS_TYPE_INT64: {int64_t x; rd(x); v.kind = FieldValue::Kind::Int; v.i = x; break;}
    case rti::ROS_TYPE_FLOAT: {float x; rd(x); v.kind = FieldValue::Kind::Double; v.d = x; break;}
    case rti::ROS_TYPE_DOUBLE: {double x; rd(x); v.kind = FieldValue::Kind::Double; v.d = x; break;}
    case rti::ROS_TYPE_STRING: {
        v.kind = FieldValue::Kind::String;
        v.s = *reinterpret_cast<const std::string *>(p);
        break;
      }
    default:
      return std::nullopt;
  }
  return v;
}

}  // namespace nav_data_recorder
