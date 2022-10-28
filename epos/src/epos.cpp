#include "epos/epos.hpp"

using namespace epos;
using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;

Epos::Epos() : Node("epos") {
  const rclcpp::Logger logger = this->get_logger();
  this->declare_parameter("device_name", "EPOS4");
  this->declare_parameter("interface", "USB");
  this->declare_parameter("portname", "USB0");
  this->declare_parameter("protocol", "MAXON SERIAL V2");

  device.device_name = this->get_parameter("device_name").as_string();
  device.interface = this->get_parameter("interface").as_string();
  device.port_name = this->get_parameter("portname").as_string();
  device.protocol = this->get_parameter("protocol").as_string();

  // open device
  uint error_code = 0;

  device.key_handle = VCS_OpenDevice(
      (char*)device.device_name.c_str(), (char*)device.protocol.c_str(),
      (char*)device.interface.c_str(), (char*)device.port_name.c_str(),
      &error_code);

  if (device.key_handle) {
    RCLCPP_INFO(logger, "Successed to open Device | device:%s | port: %s",
                device.device_name.c_str(), device.port_name.c_str());
    uint baud_rate = 0;
    uint time_out = 0;
    if (VCS_GetProtocolStackSettings(device.key_handle, &baud_rate, &time_out,
                                     &error_code)) {
      time_out += 100;
      VCS_SetProtocolStackSettings(device.key_handle, baud_rate, time_out,
                                   &error_code);
    }
  } else {
    RCLCPP_ERROR(logger, "Failed to open device! | device:%s | port: %s",
                 device.device_name.c_str(), device.port_name.c_str());
  }

  // open sub device
  this->declare_parameter("use_sub_device", false);
  if (this->get_parameter("use_sub_device").as_bool()) {
    this->declare_parameter("sub_device_name", "EPOS4");
    this->declare_parameter("sub_device_protocol", "CANopen");

    sub_device.device_name = this->get_parameter("sub_device_name").as_string();
    sub_device.protocol =
        this->get_parameter("sub_device_protocol").as_string();

    sub_device.key_handle = VCS_OpenSubDevice(
        device.key_handle, (char*)sub_device.device_name.c_str(),
        (char*)sub_device.protocol.c_str(), &error_code);

    if (sub_device.key_handle) {
      RCLCPP_INFO(logger,
                  "Successed to open sub device! | device:%s | protocol: %s",
                  sub_device.device_name.c_str(), sub_device.protocol.c_str());
    } else {
      RCLCPP_ERROR(logger,
                   "Failed to open sub device! | device:%s | protocol: %s",
                   sub_device.device_name.c_str(), sub_device.protocol.c_str());
    }
  }

  // motors
  this->declare_parameter("motor_id", std::vector<int64_t>{{}});
  this->declare_parameter("motor_id_on_sub", std::vector<int64_t>{{}});

  const auto id_array = this->get_parameter("motor_id").as_integer_array();
  for (const auto& motor_id : id_array) {
    RCLCPP_INFO_STREAM(logger, "id_array" << motor_id);
    Motor motor;
    motor.key_handle = device.key_handle;
    motor.id = motor_id;
    motors[motor_id] = motor;
  }

  const auto id_array_sub =
      this->get_parameter("motor_id_on_sub").as_integer_array();
  for (const auto& motor_id : id_array_sub) {
    RCLCPP_INFO_STREAM(logger, "id_array_on_sub" << motor_id);
    Motor motor;
    motor.key_handle = sub_device.key_handle;
    motor.id = motor_id;
    motors[motor_id] = motor;
  }

  this->declare_parameter("auto_enable", true);
  const bool auto_enable = this->get_parameter("auto_enable").as_bool();
  for (auto& [id, motor] : motors) {
    RCLCPP_INFO_STREAM(logger, "ids" << id);
    uint8_t dimension;
    char notation;
    VCS_GetVelocityUnits(motor.key_handle, id, &dimension, &notation,
                         &error_code);
    motor.vel_notation = notation;

    const std::string motor_namespace = "~/motor" + std::to_string(id);

    // sub
    motor.move_with_velocity_sub =
        this->create_subscription<std_msgs::msg::Float32>(
            motor_namespace + "/move_with_velocity", 1,
            std::bind(&Motor::handle_move_with_velocity, &motor, _1));
    motor.move_to_position_sub =
        this->create_subscription<std_msgs::msg::Int32>(
            motor_namespace + "/move_to_position", 1,
            std::bind(&Motor::handle_move_to_position, &motor, _1));

    // pub
    motor.velocity_pub = this->create_publisher<std_msgs::msg::Float32>(
        motor_namespace + "/velocity", 1);
    motor.position_pub = this->create_publisher<std_msgs::msg::Int32>(
        motor_namespace + "/position", 1);
    motor.current_pub = this->create_publisher<std_msgs::msg::Int32>(
        motor_namespace + "/current", 1);

    // enable
    if (auto_enable) {
      VCS_ClearFault(motor.key_handle, id, &error_code);
      VCS_SetEnableState(motor.key_handle, id, &error_code);
    }

    unsigned short state = 4;
    VCS_GetState(motor.key_handle, id, &state, &error_code);
    RCLCPP_INFO_STREAM(logger, "motor | id:" << id);
    switch (state) {
      case 0:
        RCLCPP_INFO(logger, "STATUS: Disabled");
        break;

      case 1:
        RCLCPP_INFO(logger, "STATUS: Enabled");
        break;

      case 2:
        RCLCPP_INFO(logger, "STATUS: quickstop");
        break;

      case 3:
        RCLCPP_WARN(logger, "STATUS: FAULT");
        break;

      default:
        RCLCPP_ERROR(logger, "motor not found");
        break;
    }
  }

  // pub timer
  this->declare_parameter("dataout_freq", 10);
  const auto dataout_freq = this->get_parameter("dataout_freq").as_int();
  timer =
      this->create_wall_timer(std::chrono::milliseconds(1000 / dataout_freq),
                              std::bind(&Epos::handle_pub_data, this));

  // service
  clear_fault_service = this->create_service<ClearFault>(
      "~/clear_fault", std::bind(&Epos::handle_clear_fault, this, _1, _2));
  set_enable_service = this->create_service<SetEnable>(
      "~/set_enable", std::bind(&Epos::handle_set_enable, this, _1, _2));
  set_operation_mode_service = this->create_service<SetOperationMode>(
      "~/set_operation_mode",
      std::bind(&Epos::handle_set_operation_mode, this, _1, _2));

  RCLCPP_INFO(logger, "EPOS initialization completed!");
}

Epos::~Epos() {
  uint error_code;
  VCS_CloseAllSubDevices(device.key_handle, &error_code);
  RCLCPP_INFO(this->get_logger(), "All sub devices closed");
  VCS_CloseAllDevices(&error_code);
  RCLCPP_INFO(this->get_logger(), "All devices closed");
}

void Epos::handle_pub_data() {
  for (const auto& [id, motor] : motors) {
    const auto& keyHandle = motor.key_handle;
    uint error_code;

    int velocity;
    VCS_GetVelocityIs(keyHandle, id, &velocity, &error_code);
    std_msgs::msg::Float32 velocity_rpm;
    velocity_rpm.data = velocity * pow(10.0f, motor.vel_notation);

    std_msgs::msg::Int32 position;
    VCS_GetPositionIs(keyHandle, id, &position.data, &error_code);

    std_msgs::msg::Int32 current;
    VCS_GetCurrentIsEx(keyHandle, id, &current.data, &error_code);

    motor.velocity_pub->publish(velocity_rpm);
    motor.position_pub->publish(position);
    motor.current_pub->publish(current);
  }
}

void Epos::handle_clear_fault(const ClearFault::Request::SharedPtr request,
                              ClearFault::Response::SharedPtr response) {
  uint error_code;
  for (const auto id : request->ids) {
    VCS_ClearFault(motors[id].key_handle, id, &error_code);
    RCLCPP_INFO(this->get_logger(), "cleared fault motor id:%2d", id);
  }

  response->set__successed(true);
}

void Epos::handle_set_enable(const SetEnable::Request::SharedPtr request,
                             SetEnable::Response::SharedPtr response) {
  uint error_code;
  for (const auto id : request->ids) {
    VCS_SetEnableState(motors[id].key_handle, id, &error_code);
    RCLCPP_INFO(this->get_logger(), "set enable motor id:%2d", id);
  }

  response->set__successed(true);
}

void Epos::handle_set_operation_mode(
    const SetOperationMode::Request::SharedPtr request,
    SetOperationMode::Response::SharedPtr response) {
  uint error_code;
  const auto id = request->id;
  VCS_SetOperationMode(motors[id].key_handle, request->mode, id, &error_code);

  response->set__successed(true);
}

void Motor::handle_move_with_velocity(
    const std_msgs::msg::Float32::SharedPtr velocity) {
  uint error_code;

  const long velocity_command = velocity->data * pow(10, -this->vel_notation);

  VCS_MoveWithVelocity(key_handle, id, velocity_command, &error_code);
}

void Motor::handle_move_to_position(
    const std_msgs::msg::Int32::SharedPtr position) {
  uint error_code;

  VCS_MoveToPosition(key_handle, id, position->data, 1, 1, &error_code);
}
