

#include "motor_interfaces/motor_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace dc_motor_firmware
{
    MotorInterface::MotorInterface()
    {

    }

    MotorInterface::~MotorInterface()
    {
        if(arduino_.IsOpen())
        {
            try
            {
                arduino_.Close();
            }
            catch(...)
            {
               // std::cerr << e.what() << '\n';
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("Motor Interface"), "Something went wrong while closing the connection with port" << port_);

            }
            
        }
    }

    // CallbackReturn MotorInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
    // {
    //     (void)hardware_info;
    //     CallbackReturn result =  hardware_interface::SystemInterface::on_init(hardware_info);
    //     if(result != CallbackReturn::SUCCESS)
    //     {
    //         return result;
    //     }
    //     try
    //     {
    //         port_ = info_.hardware_parameters.at("port");
    //     }
    //     catch(const std::out_of_range & e)
    //     {
    //         RCLCPP_FATAL_STREAM(rclcpp::get_logger("Motor Interface"), "No Serial Port Provided! Aborting" << port_);
    //         return CallbackReturn::FAILURE;
    //     }

    //     velocity_commands_.reserve(info_.joints.size());
    //     position_states_.reserve(info_.joints.size());
    //     velocity_commands_.reserve(info_.joints.size());
    //     last_run_ = rclcpp::Clock().now();
    //     return CallbackReturn::SUCCESS;
        
        

    // }

CallbackReturn MotorInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
{
    // Gọi hàm on_init của lớp cha để xử lý cơ bản
    CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
    if (result != CallbackReturn::SUCCESS)
    {
        return result;
    }

    // Lấy thông tin cổng UART từ URDF/xacro
    try
    {
        port_ = info_.hardware_parameters.at("port");
    }
    catch (const std::out_of_range &e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("MotorInterface"), "No serial port provided in hardware parameters!");
        return CallbackReturn::FAILURE;
    }

    // Resize vector thay vì reserve!
    size_t num_joints = info_.joints.size();
    velocity_commands_.resize(num_joints, 0.0);
    position_states_.resize(num_joints, 0.0);
    velocity_states_.resize(num_joints, 0.0);  // Nếu bạn có thêm velocity state

    last_run_ = rclcpp::Clock().now();

    RCLCPP_INFO(rclcpp::get_logger("MotorInterface"), "Initialized MotorInterface with %ld joints.", num_joints);

    return CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> MotorInterface::export_state_interfaces()
{
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for(size_t i = 0; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, 
            hardware_interface::HW_IF_POSITION, &position_states_[i]));

            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, 
            hardware_interface::HW_IF_VELOCITY, &velocity_states_[i]));

        }
        return state_interfaces;

}

std::vector<hardware_interface::CommandInterface> MotorInterface::export_command_interfaces()
{
        std::vector<hardware_interface::CommandInterface> command_interfaces;
    //    command_interfaces.emplace_back("wheel_left_joint", hardware_interface::HW_IF_VELOCITY, &velocity_commands_[0]);
     //   command_interfaces.emplace_back("wheel_right_joint", hardware_interface::HW_IF_VELOCITY, &velocity_commands_[1]);
        for(size_t i = 0; i < info_.joints.size(); i++)
        {
           // RCLCPP_INFO(rclcpp::get_logger("MotorInterface"), "Exporting command interface for joint: %s", info_.joints[i].name.c_str());
            RCLCPP_INFO(rclcpp::get_logger("MotorInterface"), "Exporting command interface for joint: %s", info_.joints[i].name.c_str()
        );
            command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name,
            hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]));

        }
        return command_interfaces;
}


CallbackReturn MotorInterface::on_activate(const rclcpp_lifecycle::State &previous_state) 
{
        (void)previous_state;
        RCLCPP_INFO(rclcpp::get_logger("MotorInterface"), "Starting robot hardware...");

        velocity_commands_ = {0.0, 0.0};
        position_states_ = {0.0, 0.0};
        velocity_commands_ = {0.0, 0.0};
        
        try
        {
            arduino_.Open(port_);
            arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        }
        catch(...)
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("Motor Interface"), "something went wrong while interacting with port " << port_);
            return CallbackReturn::FAILURE;

        }
        RCLCPP_INFO(rclcpp::get_logger("MotorInterface"), "hardware started, ready to take commands");
        return CallbackReturn::SUCCESS;

}
CallbackReturn MotorInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state) 
{
        (void)previous_state;

        RCLCPP_INFO(rclcpp::get_logger("MotorInterface"), "Stopping robot hardware...");
        if(arduino_.IsOpen())
        {
            try
            {
                arduino_.Close();
            }
            catch(...)
            {
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("Motor Interface"), "something went wrong while interacting with port " << port_);
                return CallbackReturn::FAILURE;
            }
        }
        return CallbackReturn::SUCCESS; 

}

hardware_interface::return_type MotorInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period)
{
        /*
        Responsible for reading the current velocity 
        Format: rp2.57 = right positive 2.57 rad/s,  ln5.71 = left negative 5.71 rad/s -> velocity of the left and right 
        wheels (rad/s)
        */
        (void)time;
        (void)period;
        if(arduino_.IsDataAvailable()) // read message from Arduino
        {
            auto dt = (rclcpp::Clock().now() - last_run_).seconds();
            std::string message;
            arduino_.ReadLine(message);
            std::stringstream ss(message);
            std::string res;
            int multiplier = 1;  // direction 
            while(std::getline(ss, res, ','))
            {
                multiplier = res.at(1) == 'p' ? 1: -1;
                if(res.at(0) == 'r')
                {
                    velocity_states_.at(0) = multiplier * std::stod(res.substr(2, res.size())); // rad/s
                    position_states_.at(0) += velocity_states_.at(0) * dt;  // rad
                }
                else if(res.at(0) == 'l')
                {
                    velocity_states_.at(1) = multiplier * std::stod(res.substr(2, res.size()));  // rad/s
                    position_states_.at(1) += velocity_states_.at(1) * dt;  // rad
                }
            }

        }
        last_run_ = rclcpp::Clock().now();
        return hardware_interface::return_type::OK;


}
hardware_interface::return_type MotorInterface::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
        /*
        Allows us to send velocity commands to the Arduino, and so to the motors
        */

        (void)time;
        (void)period;

        //RCLCPP_INFO(rclcpp::get_logger("MotorInterface"), "Received commands -> left: %.2f, right: %.2f", velocity_commands_.at(1),  velocity_commands_.at(0) );
        
        std::stringstream message_stream;
        char right_wheel_sign = velocity_commands_.at(0) >= 0 ? 'p' : 'n';
        char left_wheel_sign = velocity_commands_.at(1) >= 0 ? 'p' : 'n';

        // RCLCPP_DEBUG_STREAM(rclcpp::get_logger("MotorInterface"),
        //             "Right velocity cmd: " << velocity_commands_.at(0)
        //             << " | Left velocity cmd: " << velocity_commands_.at(1));



        std::string compensate_zero_right = (std::abs(velocity_commands_.at(0)) < 10.0) ? "0" : "";
        std::string compensate_zero_left  = (std::abs(velocity_commands_.at(1)) < 10.0) ? "0" : "";

        // std::string compensate_zero_right = "";
        // std::string compensate_zero_left = "";
        // if(std::abs(velocity_commands_.at(0)) < 10.0)
        // {
        //     compensate_zero_right = "0";
        // }
        // else
        // {
        //     compensate_zero_right = "";
        // }

        // if(std::abs(velocity_commands_.at(1)) < 10.0)
        // {
        //     compensate_zero_left = "0";
        // }
        // else
        // {
        //     compensate_zero_left = "";

        // }

    
        message_stream << std::fixed << std::setprecision(2) << "r" << right_wheel_sign << compensate_zero_right << std::abs(velocity_commands_.at(0)) <<
                ",l" << left_wheel_sign << compensate_zero_left << std::abs(velocity_commands_.at(1)) << ",";
        
        std::string msg = message_stream.str();

        // RCLCPP_INFO_STREAM(rclcpp::get_logger("MotorInterface"),
        //                "Sending to Arduino: " << msg << " (port: " << port_ << ")");



        
        try
        {
            arduino_.Write(msg);
        }
        catch(...)
        {
            RCLCPP_ERROR_STREAM(rclcpp::get_logger("MotorInterface"),
                                        "Error sending message: " << msg << " on port " << port_);

            return hardware_interface::return_type::ERROR;
        }
        return hardware_interface::return_type::OK;
}
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(dc_motor_firmware::MotorInterface, hardware_interface::SystemInterface);