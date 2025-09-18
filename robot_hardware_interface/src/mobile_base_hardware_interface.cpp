#include "robot_hardware_interface/mobile_base_hardware_interface.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <chrono>
#include <vector>
#include "rclcpp/rclcpp.hpp"


namespace mobile_base_hardware
{

// -------------------------
// on_init()
// -------------------------
hardware_interface::CallbackReturn
MobileBaseHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Lưu info URDF
  info_ = info;

  // Cấu hình ID động cơ và port
  left_motor_id_ = std::stoi(info_.hardware_parameters["left_motor_id"]);
  right_motor_id_ = std::stoi(info_.hardware_parameters["right_motor_id"]);
  port_ =   info_.hardware_parameters["dynamixel_port"] ;    //  "/dev/ttyUSB0";         //"/dev/ttyACM0"
  driver_ = std::make_shared<XL330Driver>(port_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

// -------------------------
// on_configure()
// -------------------------
hardware_interface::CallbackReturn
MobileBaseHardwareInterface::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (driver_->init() != 0)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

//   for (const auto &[name, descr] : joint_command_interfaces_)
//   {
//     RCLCPP_INFO(get_logger(), "COMMAND INTERFACE NAME: ");
//     RCLCPP_INFO(get_logger(), name.c_str());

    
//   }

//   for(const auto &[name, descr] : joint_state_interfaces_)
//   {
//     RCLCPP_INFO(get_logger(), "STATE INTERFACE NAME: ");
//     RCLCPP_INFO(get_logger(), name.c_str());
    
//   }

  return hardware_interface::CallbackReturn::SUCCESS;
}

// -------------------------
// on_activate()
// -------------------------
hardware_interface::CallbackReturn
MobileBaseHardwareInterface::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reset giá trị state/command
  hw_positions_[0] = 0.0;
  hw_positions_[1] = 0.0;
  hw_velocities_[0] = 0.0;
  hw_velocities_[1] = 0.0;
  hw_commands_[0] = 0.0;
  hw_commands_[1] = 0.0;

  // Bật torque và chọn chế độ velocity
  driver_->activateWithVelocityMode(left_motor_id_);
  driver_->activateWithVelocityMode(right_motor_id_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

// -------------------------
// on_deactivate()
// -------------------------
hardware_interface::CallbackReturn
MobileBaseHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  driver_->deactivate(left_motor_id_);
  driver_->deactivate(right_motor_id_);
  return hardware_interface::CallbackReturn::SUCCESS;
}


// -------------------------
// read()
// -------------------------
hardware_interface::return_type
MobileBaseHardwareInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period)
{
    (void)time;
  double left_vel = driver_->getVelocityRadianPerSec(left_motor_id_);
  double right_vel = -1.0*driver_->getVelocityRadianPerSec(right_motor_id_);
  if(abs(left_vel) < 0.03)
  {
    left_vel = 0.0;
  }
  if(abs(right_vel) < 0.03)
  {
    right_vel = 0.0;
  }

  hw_velocities_[0] = left_vel;
  hw_velocities_[1] = right_vel;

  // Tích phân để cập nhật position
  hw_positions_[0] += left_vel * period.seconds();
  hw_positions_[1] += right_vel * period.seconds();

  RCLCPP_INFO(rclcpp::get_logger("MobileBaseHardwareInterface"), "READ: left_pos=%.3f rad, right_pos=%.3f rad, right_vel=%.3f rad/s, left_vel=%.3f rad/s",
                                hw_positions_[0], hw_positions_[1],hw_velocities_[0], hw_velocities_[1] );

  return hardware_interface::return_type::OK;
}

// -------------------------
// write()
// -------------------------
hardware_interface::return_type
MobileBaseHardwareInterface::write(const rclcpp::Time &, const rclcpp::Duration &)
{
    //(void)time; (void)period;

   double left_cmd  = hw_commands_[0];
  double right_cmd = hw_commands_[1];
  driver_->setTargetVelocityRadianPerSec(left_motor_id_, left_cmd);
  driver_->setTargetVelocityRadianPerSec(right_motor_id_, -1.0*right_cmd);
  RCLCPP_INFO(rclcpp::get_logger("MobileBaseHardwareInterface"), "WRITE: left_cmd=%.3f rad/s | right_cmd=%.3f rad/s", left_cmd, right_cmd );



  return hardware_interface::return_type::OK;
}


// -------------------------
// export_state_interfaces()
// -------------------------
std::vector<hardware_interface::StateInterface>
MobileBaseHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back("base_left_wheel_joint", hardware_interface::HW_IF_POSITION, &hw_positions_[0]);
  state_interfaces.emplace_back("base_left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_velocities_[0]);

  state_interfaces.emplace_back("base_right_wheel_joint", hardware_interface::HW_IF_POSITION, &hw_positions_[1]);
  state_interfaces.emplace_back("base_right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_velocities_[1]);

  // In ra thông tin debug
  for (auto &iface : state_interfaces) {
    RCLCPP_INFO(
      rclcpp::get_logger("MobileBaseHardwareInterface"),
      "STATE INTERFACE: joint='%s', interface='%s'",
      iface.get_name().c_str(),
      iface.get_interface_name().c_str()
    );
  }

  return state_interfaces;
}
// -------------------------
// export_command_interfaces()
// -------------------------
std::vector<hardware_interface::CommandInterface>
MobileBaseHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back("base_left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_commands_[0]);
  command_interfaces.emplace_back("base_right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &hw_commands_[1]);

  // In ra thông tin debug
  for (auto &iface : command_interfaces) {
    RCLCPP_INFO(
      rclcpp::get_logger("MobileBaseHardwareInterface"),
      "COMMAND INTERFACE: joint='%s', interface='%s'",
      iface.get_name().c_str(),
      iface.get_interface_name().c_str()
    );
  }

  return command_interfaces;
}



} // namespace mobile_base_hardware

// Export plugin (phải là SystemInterface, không phải StateInterface)
PLUGINLIB_EXPORT_CLASS(mobile_base_hardware::MobileBaseHardwareInterface, hardware_interface::SystemInterface)

