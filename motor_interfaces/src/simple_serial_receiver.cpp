#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <libserial/SerialPort.h>
#include <chrono>
#include <regex>

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
        
        if(rclcpp::ok() && arduino_.IsDataAvailable())
        {
            auto message = std_msgs::msg::String();
            arduino_.ReadLine(message.data);
            // Xử lý loại bỏ kí tự \r\n thừa
            // message.data.erase(std::remove(message.data.begin(), message.data.end(), '\r'), message.data.end());
            // message.data.erase(std::remove(message.data.begin(), message.data.end(), '\n'), message.data.end());
            if(!message.data.empty())
            {
                pub_->publish(message);

            }
            

      
        }
        

    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleSerialReceiver>());
    rclcpp::shutdown();
    return 0;

}


