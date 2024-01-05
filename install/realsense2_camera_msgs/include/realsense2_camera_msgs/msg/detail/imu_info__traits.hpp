// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from realsense2_camera_msgs:msg/IMUInfo.idl
// generated code does not contain a copyright notice

#ifndef REALSENSE2_CAMERA_MSGS__MSG__DETAIL__IMU_INFO__TRAITS_HPP_
#define REALSENSE2_CAMERA_MSGS__MSG__DETAIL__IMU_INFO__TRAITS_HPP_

#include "realsense2_camera_msgs/msg/detail/imu_info__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const realsense2_camera_msgs::msg::IMUInfo & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_yaml(msg.header, out, indentation + 2);
  }

  // member: data
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.data.size() == 0) {
      out << "data: []\n";
    } else {
      out << "data:\n";
      for (auto item : msg.data) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: noise_variances
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.noise_variances.size() == 0) {
      out << "noise_variances: []\n";
    } else {
      out << "noise_variances:\n";
      for (auto item : msg.noise_variances) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: bias_variances
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.bias_variances.size() == 0) {
      out << "bias_variances: []\n";
    } else {
      out << "bias_variances:\n";
      for (auto item : msg.bias_variances) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const realsense2_camera_msgs::msg::IMUInfo & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<realsense2_camera_msgs::msg::IMUInfo>()
{
  return "realsense2_camera_msgs::msg::IMUInfo";
}

template<>
inline const char * name<realsense2_camera_msgs::msg::IMUInfo>()
{
  return "realsense2_camera_msgs/msg/IMUInfo";
}

template<>
struct has_fixed_size<realsense2_camera_msgs::msg::IMUInfo>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<realsense2_camera_msgs::msg::IMUInfo>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<realsense2_camera_msgs::msg::IMUInfo>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // REALSENSE2_CAMERA_MSGS__MSG__DETAIL__IMU_INFO__TRAITS_HPP_
