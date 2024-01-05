// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from realsense2_camera_msgs:srv/DeviceInfo.idl
// generated code does not contain a copyright notice

#ifndef REALSENSE2_CAMERA_MSGS__SRV__DETAIL__DEVICE_INFO__TRAITS_HPP_
#define REALSENSE2_CAMERA_MSGS__SRV__DETAIL__DEVICE_INFO__TRAITS_HPP_

#include "realsense2_camera_msgs/srv/detail/device_info__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const realsense2_camera_msgs::srv::DeviceInfo_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const realsense2_camera_msgs::srv::DeviceInfo_Request & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<realsense2_camera_msgs::srv::DeviceInfo_Request>()
{
  return "realsense2_camera_msgs::srv::DeviceInfo_Request";
}

template<>
inline const char * name<realsense2_camera_msgs::srv::DeviceInfo_Request>()
{
  return "realsense2_camera_msgs/srv/DeviceInfo_Request";
}

template<>
struct has_fixed_size<realsense2_camera_msgs::srv::DeviceInfo_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<realsense2_camera_msgs::srv::DeviceInfo_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<realsense2_camera_msgs::srv::DeviceInfo_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

inline void to_yaml(
  const realsense2_camera_msgs::srv::DeviceInfo_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: device_name
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "device_name: ";
    value_to_yaml(msg.device_name, out);
    out << "\n";
  }

  // member: serial_number
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "serial_number: ";
    value_to_yaml(msg.serial_number, out);
    out << "\n";
  }

  // member: firmware_version
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "firmware_version: ";
    value_to_yaml(msg.firmware_version, out);
    out << "\n";
  }

  // member: usb_type_descriptor
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "usb_type_descriptor: ";
    value_to_yaml(msg.usb_type_descriptor, out);
    out << "\n";
  }

  // member: firmware_update_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "firmware_update_id: ";
    value_to_yaml(msg.firmware_update_id, out);
    out << "\n";
  }

  // member: sensors
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "sensors: ";
    value_to_yaml(msg.sensors, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const realsense2_camera_msgs::srv::DeviceInfo_Response & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<realsense2_camera_msgs::srv::DeviceInfo_Response>()
{
  return "realsense2_camera_msgs::srv::DeviceInfo_Response";
}

template<>
inline const char * name<realsense2_camera_msgs::srv::DeviceInfo_Response>()
{
  return "realsense2_camera_msgs/srv/DeviceInfo_Response";
}

template<>
struct has_fixed_size<realsense2_camera_msgs::srv::DeviceInfo_Response>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<realsense2_camera_msgs::srv::DeviceInfo_Response>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<realsense2_camera_msgs::srv::DeviceInfo_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<realsense2_camera_msgs::srv::DeviceInfo>()
{
  return "realsense2_camera_msgs::srv::DeviceInfo";
}

template<>
inline const char * name<realsense2_camera_msgs::srv::DeviceInfo>()
{
  return "realsense2_camera_msgs/srv/DeviceInfo";
}

template<>
struct has_fixed_size<realsense2_camera_msgs::srv::DeviceInfo>
  : std::integral_constant<
    bool,
    has_fixed_size<realsense2_camera_msgs::srv::DeviceInfo_Request>::value &&
    has_fixed_size<realsense2_camera_msgs::srv::DeviceInfo_Response>::value
  >
{
};

template<>
struct has_bounded_size<realsense2_camera_msgs::srv::DeviceInfo>
  : std::integral_constant<
    bool,
    has_bounded_size<realsense2_camera_msgs::srv::DeviceInfo_Request>::value &&
    has_bounded_size<realsense2_camera_msgs::srv::DeviceInfo_Response>::value
  >
{
};

template<>
struct is_service<realsense2_camera_msgs::srv::DeviceInfo>
  : std::true_type
{
};

template<>
struct is_service_request<realsense2_camera_msgs::srv::DeviceInfo_Request>
  : std::true_type
{
};

template<>
struct is_service_response<realsense2_camera_msgs::srv::DeviceInfo_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // REALSENSE2_CAMERA_MSGS__SRV__DETAIL__DEVICE_INFO__TRAITS_HPP_
