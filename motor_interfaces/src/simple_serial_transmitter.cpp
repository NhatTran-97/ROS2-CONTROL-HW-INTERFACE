#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>
#include <libserial/SerialPort.h>

using std::placeholders::_1;

class SimpleSerialTransmitter : public rclcpp::Node 
{
public:
    SimpleSerialTransmitter() : Node("simple_serial_transmitter")
    {
        declare_parameter<std::string>("port", "/dev/ttyUSB0");
        port_ = get_parameter("port").as_string();
        arduino_.Open(port_);
        arduino_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    
        sub_ = create_subscription<std_msgs::msg::String>(
            "serial_transmitter", 10, std::bind(&SimpleSerialTransmitter::msgCallback, this, _1));
        
    }
    ~SimpleSerialTransmitter()
    {
        arduino_.Close();
    }
private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    LibSerial::SerialPort arduino_;
    std::string port_;

    void msgCallback(const std_msgs::msg::String &msg)  
    {
        arduino_.Write(msg.data);
    }


};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleSerialTransmitter>());
    rclcpp::shutdown();
    return 0;

}