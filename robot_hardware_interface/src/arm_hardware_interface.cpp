#include "robot_hardware_interface/arm_hardware_interface.hpp"
#include "rclcpp/rclcpp.hpp"
namespace arm_hardware{
hardware_interface::CallbackReturn ArmHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
{
    if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // Lưu info URDF
    info_ = info;

    for (auto &param : info_.hardware_parameters) {
        RCLCPP_INFO(rclcpp::get_logger("ArmHardwareInterface"),
              "Param: %s = %s", param.first.c_str(), param.second.c_str());
        }


    // Cấu hình ID động cơ và port
    joint1_motor_id_ = std::stoi(info_.hardware_parameters["joint1_motor_id"]);
    joint2_motor_id_ = std::stoi(info_.hardware_parameters["joint2_motor_id"]);
    port_ =   info_.hardware_parameters["dynamixel_port"] ;    //  "/dev/ttyUSB0";         //"/dev/ttyACM0"
    driver_ = std::make_shared<XL330Driver>(port_);
    return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn ArmHardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (driver_->init() != 0)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ArmHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
//   // Reset giá trị state/command
  hw_positions_[0] = 0.0;
  hw_positions_[1] = 0.0;

  hw_commands_[0] = 0.0;
  hw_commands_[1] = 0.0;
//(void) previous_state;

  // Bật torque và chọn chế độ position
  driver_->activateWithPositionMode(joint1_motor_id_);
  driver_->activateWithPositionMode(joint2_motor_id_);

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn ArmHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  driver_->deactivate(joint1_motor_id_);
  driver_->deactivate(joint2_motor_id_);
  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::return_type ArmHardwareInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period)
{
  (void)time;
  (void)period;
  double joint1_pos = driver_->getPositionRadian(joint1_motor_id_);
  double joint2_pos = driver_->getPositionRadian(joint2_motor_id_);

  hw_positions_[0] = joint1_pos;
  hw_positions_[1] = joint2_pos;

  RCLCPP_INFO(rclcpp::get_logger("ArmHardwareInterface: "), "STATE joint 1: %lf, joint2: %lf",   hw_positions_[0], hw_positions_[1]);

  

  return hardware_interface::return_type::OK;
}


hardware_interface::return_type ArmHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    //(void)time; (void)period;

  double joint1_cmd  = hw_commands_[0];
  double joint2_cmd = hw_commands_[1];
  driver_->setTargetPositionRadian(joint1_motor_id_, joint1_cmd);
  driver_->setTargetPositionRadian(joint2_motor_id_, joint2_cmd);
  RCLCPP_INFO(rclcpp::get_logger("ArmHardwareInterface: "), "WRITE: joint1_cmd=%.3f rad | joint2_cmd=%.3f rad", joint1_cmd, joint2_cmd );

  return hardware_interface::return_type::OK;
}




std::vector<hardware_interface::StateInterface> ArmHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back("arm_joint1", "position", &hw_positions_[0]);

  state_interfaces.emplace_back("arm_joint2", "position", &hw_positions_[1]);

//   // In ra thông tin debug
//   for (auto &iface : state_interfaces) {
//     RCLCPP_INFO(
//       rclcpp::get_logger("MobileBaseHardwareInterface"),
//       "STATE INTERFACE: joint='%s', interface='%s'",
//       iface.get_name().c_str(),
//       iface.get_interface_name().c_str()
//     );
//   }

  return state_interfaces;
}
// -------------------------
// export_command_interfaces()
// -------------------------
std::vector<hardware_interface::CommandInterface> ArmHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back("arm_joint1", "position", &hw_commands_[0]);
  command_interfaces.emplace_back("arm_joint2", "position", &hw_commands_[1]);

  // In ra thông tin debug
//   for (auto &iface : command_interfaces) {
//     RCLCPP_INFO(
//       rclcpp::get_logger("MobileBaseHardwareInterface"),
//       "COMMAND INTERFACE: joint='%s', interface='%s'",
//       iface.get_name().c_str(),
//       iface.get_interface_name().c_str()
//     );
//   }

  return command_interfaces;
}

} // namespace arm_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(arm_hardware::ArmHardwareInterface, hardware_interface::SystemInterface)
