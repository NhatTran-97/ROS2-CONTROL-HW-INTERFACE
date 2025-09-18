#ifndef ARM_HARDWARE_INTERFACE_HPP
#define ARM_HARDWARE_INTERFACE_HPP

#include "hardware_interface/system_interface.hpp"
#include "robot_hardware_interface/xl330_driver.hpp"

namespace arm_hardware{
        class ArmHardwareInterface : public hardware_interface::SystemInterface
    {
        public: 

        // Lifecycle node override
        hardware_interface::CallbackReturn 
            on_configure(const rclcpp_lifecycle::State & previous_state) override;
        hardware_interface::CallbackReturn 
            on_activate(const rclcpp_lifecycle::State & previous_state) override;
        hardware_interface::CallbackReturn 
            on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

        // SystemInterface override
        hardware_interface::CallbackReturn
            on_init(const hardware_interface::HardwareInfo &info) override;
        
        hardware_interface::return_type
            read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
        
        hardware_interface::return_type
            write(const rclcpp::Time &time, const rclcpp::Duration &period) override;

        // Bắt buộc override trong Humble
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
            
        private:
            std::shared_ptr<XL330Driver> driver_;
            int joint1_motor_id_;
            int joint2_motor_id_;
            std::string port_;


            // Bộ nhớ lưu state/command
            double hw_positions_[2] = {0.0, 0.0};   // [0]=left, [1]=right
            double hw_commands_[2]   = {0.0, 0.0};


    }; // class ArmHardwareInterface

} // namespace arm_hardware


#endif 