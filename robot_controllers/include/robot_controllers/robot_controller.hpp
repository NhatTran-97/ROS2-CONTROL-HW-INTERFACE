#ifndef ROBOT_CONTROLLER_HPP
#define ROBOT_CONTROLLER_HPP

#include "controller_interface/controller_interface.hpp"
#include "example_interfaces/msg/float64_multi_array.hpp"

using FloatArray = example_interfaces::msg::Float64MultiArray;

namespace robot_controller 
{
    class RobotController: public controller_interface::ControllerInterface
    {
        public: 
            RobotController();
            controller_interface::InterfaceConfiguration command_interface_configuration() const override;
            controller_interface::InterfaceConfiguration state_interface_configuration() const override;

            controller_interface::CallbackReturn on_init() override;
            controller_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
            controller_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;


            controller_interface::return_type update(const rclcpp::Time & time, const rclcpp::Duration & period) override;


        protected:
        std::vector<std::string> joint_names_;
        std::string interface_name_;
        double coefficient_;

        std::vector<double> appCommand_;
        rclcpp::Subscription<FloatArray>::SharedPtr command_subscriber_;

    }; // class robot_controller

}// namespace robot_controller

#endif