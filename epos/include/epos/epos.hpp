#pragma once

#include <Definitions.h>

#include <memory>

#include "epos_interface/srv/clear_fault.hpp"
#include "epos_interface/srv/set_enable.hpp"
#include "epos_interface/srv/set_operation_mode.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/int32.hpp"

namespace epos {

using ClearFault = epos_interface::srv::ClearFault;
using SetEnable = epos_interface::srv::SetEnable;
using SetOperationMode = epos_interface::srv::SetOperationMode;

struct Device {
  std::string device_name;
  std::string protocol;
  std::string interface;
  std::string port_name;

  void* key_handle;
};

struct Motor {
  void* key_handle;
  uint16_t id;
  int8_t vel_notation;

  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr
      move_with_velocity_sub;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr move_to_position_sub;

  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr velocity_pub;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr position_pub;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr current_pub;

  void handle_move_with_velocity(
      const std_msgs::msg::Float32::SharedPtr velocity);
  void handle_move_to_position(const std_msgs::msg::Int32::SharedPtr position);
};

class Epos : public rclcpp::Node {
 private:
 public:
  Epos();
  ~Epos();

 private:
  void handle_pub_data();
  void handle_clear_fault(const ClearFault::Request::SharedPtr request,
                          ClearFault::Response::SharedPtr response);
  void handle_set_enable(const SetEnable::Request::SharedPtr request,
                         SetEnable::Response::SharedPtr response);
  void handle_set_operation_mode(
      const SetOperationMode::Request::SharedPtr request,
      SetOperationMode::Response::SharedPtr response);

  rclcpp::Service<ClearFault>::SharedPtr clear_fault_service;
  rclcpp::Service<SetEnable>::SharedPtr set_enable_service;
  rclcpp::Service<SetOperationMode>::SharedPtr set_operation_mode_service;

  rclcpp::TimerBase::SharedPtr timer;

  Device device;
  Device sub_device;
  std::map<int64_t, Motor> motors;
};

}  // namespace epos