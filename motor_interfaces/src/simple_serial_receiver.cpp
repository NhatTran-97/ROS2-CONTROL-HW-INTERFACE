#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <libserial/SerialPort.h>
#include <chrono>

using namespace std::chrono_literals;

class SimpleSerialReceiver : public rclcpp::Node
{
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    std::string port_;
    LibSerial::SerialPort arduino_;
    rclcpp::TimerBase::SharedPtr timer_;
    
public:
    SimpleSerialReceiver()  : Node("simple_serial_receiver")
    {
        declare_parameter<std::string>("port", "/dev/ttyUSB*");
        port_ = get_parameter("port").as_string();
        arduino_.Open(port_);
        arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
        pub_ = create_publisher<std_msgs::msg::String>("serial_receiver", 10);
        timer_ = create_wall_timer(0.01s, std::bind(&SimpleSerialReceiver::timerCallback, this));


    }
    ~SimpleSerialReceiver()
    {
        arduino_.Close();
    }

    void timerCallback()
    {
        auto message = std_msgs::msg::String();
        if(rclcpp::ok() && arduino_.IsDataAvailable())
        {
            arduino_.ReadLine(message.data);
        }
        pub_->publish(message);

    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleSerialReceiver>());
    rclcpp::shutdown();
    return 0;

}


