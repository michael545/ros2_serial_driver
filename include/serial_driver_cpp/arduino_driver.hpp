#ifndef ARDUINO_DRIVER_HPP_
#define ARDUINO_DRIVER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "serial/serial.h" // the lib header
#include <string>

class ArduinoDriver : public rclcpp::Node {
public:
    ArduinoDriver();

private:
    void read_serial_data();
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    // ROS2 trigger the read callback at a fixed rate
    rclcpp::TimerBase::SharedPtr timer_;

    serial::Serial serial_port_;
    
    std::string port_name_;
    int baud_rate_;
};

#endif 