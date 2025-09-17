#include "robot_hardware_interface/mobile_base_hardware_interface.hpp"
#include <chrono>


namespace mobile_base_hardware{


            /*
        Phần khởi tạo hardware interface. Được gọi khi hệ thống load plugin ROS2 control
        info chứa thông tin phần cứng lấy từ urdf, hoặc phần cứng vật lý
        */
        hardware_interface::CallbackReturn
            MobileBaseHardwareInterface::on_init(const hardware_interface::HardwareInfo &info)
            {
                /*
                "info" stores information about hardware defined in a robot's urdf
                */
                if(hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
                {
                    return hardware_interface::CallbackReturn::ERROR;
                }
                info_ = info;
                left_motor_id_ = 10;
                right_motor_id_ =  20;
                port_ = "/dev/ttyACM0";
                driver_ = std::make_shared<XL330Driver>(port_);
                return hardware_interface::CallbackReturn::SUCCESS; // trả về SUCCESS nếu khởi tạo thành công

            }
        
            // Lifecycle node override
        hardware_interface::CallbackReturn 
        /*
        Được gọi khi controller manager chuyển hardware sang trạng thái configured
        Thường dùng để chuẩn bị tài nguyên (load tham số, check hardware)
        */
            MobileBaseHardwareInterface::on_configure(const rclcpp_lifecycle::State & previous_state)
            {
                (void)previous_state;
                // Initialize the hardware
                if(driver_ -> init() !=0)
                {
                    return hardware_interface::CallbackReturn::ERROR;
                }
                return hardware_interface::CallbackReturn::SUCCESS;

                

            }
            /*
            Được gọi khi chuyển từ configured -> active
            Thường được dùng để bật đọng cơ, reset encoder, zero position
            */
        hardware_interface::CallbackReturn
            MobileBaseHardwareInterface::on_activate(const rclcpp_lifecycle::State & previous_state)
            {
                (void) previous_state;
                set_state("base_left_wheel_joint/velocity", 0.0);
                set_state("base_right_wheel_joint/velocity", 0.0);
                set_state("base_left_wheel_joint/position", 0.0);
                set_state("base_right_wheel_joint/position", 0.0);
                driver_->activateWithVelocityMode(left_motor_id_);
                driver_->activateWithVelocityMode(right_motor_id_);
                return hardware_interface::CallbackReturn::SUCCESS;


            }
            /*
            Được gọi khi controller dừng, từ active -> inactive 
            Thường dùng để tắt động cơ, disable torque
            */
        hardware_interface::CallbackReturn 
            MobileBaseHardwareInterface::on_deactivate(const rclcpp_lifecycle::State & previous_state)
            {
                (void)previous_state;
                driver_ ->deactivate(left_motor_id_);
                driver_ ->deactivate(right_motor_id_);
                return hardware_interface::CallbackReturn::SUCCESS;


            }

        // SystemInterface override


            /*
            Hàm này ROS2 Control sẽ gọi định kỳ để đọc trạng thái từ hardware, vd đọc encoder position, velocity từ XL330
            Sau đó publish lên joint_states

            In the context of "hardware interface"
            -> state interfaces (position, velocity, effort)
            -> command interface  (position, velocity, effort)

            StateInterface: những gì phần cứng báo về (position, velocity, effort).
            CommandInterface: những gì controller gửi xuống (position, velocity, effort).

            */
        
        hardware_interface::return_type
            MobileBaseHardwareInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period) // update states from the hardware 
            {
                (void)time; 
                double left_vel = driver_->getVelocityRadianPerSec(left_motor_id_);
                double right_vel = driver_->getVelocityRadianPerSec(right_motor_id_);

                set_state("base_left_wheel_joint/velocity", left_vel); // This method origin from the systemInterface
                set_state("base_right_wheel_joint/velocity", right_vel); 
                
                set_state("base_left_wheel_joint/position", get_state("base_left_wheel_joint/position") + left_vel * period.seconds());
                set_state("base_right_wheel_joint/position", get_state("base_right_wheel_joint/position") + right_vel * period.seconds());

                return hardware_interface::return_type::OK;

            }
        
        /*
        Hàm này được ROS2 Control gọi định kỳ để gửi lệnh xuống hardware. vd lấy command từ controller (cmd_vel -> wheel speed) rồi gửi xuống
        XL330 
        */
        hardware_interface::return_type
            MobileBaseHardwareInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period) // send command to hardware
            {
                (void)time;
                (void)period;
                driver_->setTargetVelocityRadianPerSec(left_motor_id_, get_command("base_left_wheel_joint/velocity"));
                driver_->setTargetVelocityRadianPerSec(right_motor_id_, get_command("base_right_wheel_joint/velocity"));
                

            }



} // namespace mobile_base_hardware

#include "pluginlib/class_list_macros.hpp"

// PLUGINLIB_EXPORT_CLASS(child class, parent class)
PLUGINLIB_EXPORT_CLASS(mobile_base_hardware::MobileBaseHardwareInterface, hardware_interface::StateInterface)






/*
Đoạn code này là bộ khung Hardware Interface cho mobile robot
* Kết nối ROS2 Control <--> Dynamixel XL330 driver
* Các hàm (on_configure, on_activate, on_deactivate) là để quản lý vòng đời
* on_init khởi tạo driver, gán ID motor và port
* read và write sẽ là nơi đọc encoder và gửi command xuống động cơ

    Trình tự khi load 1 hardware interface plugin (MobileBaseHardwareInterface):
    1. on_init() -> Được gọi khi plugin được tạo bởi pluginlib và controller_manager load. -> Đây là bước khởi tạo ban đầu, gán biến
    , mở driver, đọc thông tin từ URDF (info). Luôn được gọi trước cả configure/activate
    2. on_configure()  --> Khi gọi lệnh ros2 lifecycle set <node> configure -> Dùng để chuẩn bị hardware cho bước active (load tham số, kiểm tra kết nối, ...)
    3.  on_activate() --> Khi controller được start, hoặc bạn gọi ros2 lifecycle set <node> activate. Lúc này hardware được kích hoạt cho bước chạy thực tế (enable torque, reset encoder,)
    4. on_deactivate() -> Khi controller stop (hoặc lifecycle chuyển về inactive)

    Trường hợp phần cứng đã active, ROS2 control bắt đầu chạy control loop 
    Trong vòng lặp này
    1. read():
    * được gọi đầu tiên mỗi chu kỳ control loop
    * Nhiệm vụ: đọc trạng thái từ phần cứng (encoder, vận tốc, mô-ment)
    * Sau đó ROS2 Control cập nhật các state_interface (vd position, velocity) để các controller có dữ liệu mới
    2. Controller update():
    * Controller (vd DiffDriveController) dùng dữ liệu state để tính toán lệnh điều khiển (command)
    * vd từ cmd_vel tính ra tốc độ bánh trái/phải
    3. write()
    * Được gọi sau khi controller tính toán xong
    * Nhiệm vụ : gửi lệnh xuống phần cứng (vd tốc độ bánh trái/ phải)
*/