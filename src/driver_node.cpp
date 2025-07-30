#include "serial_driver_cpp/arduino_driver.hpp"
#include <chrono>
#include <memory>

ArduinoDriver::ArduinoDriver() : Node("arduino_driver_node") {

    this->declare_parameter<std::string>("port", "/dev/ttyACM0"); //templeate parameters allow for easy config
    this->declare_parameter<int>("baud_rate", 115200);
    this->get_parameter("port", port_name_);
    this->get_parameter("baud_rate", baud_rate_);

    RCLCPP_INFO(this->get_logger(), "Connecting to port '%s' at %d baud.", port_name_.c_str(), baud_rate_);

    try {
        serial_port_.setPort(port_name_);
        serial_port_.setBaudrate(baud_rate_);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000); // 1-second timeout
        serial_port_.setTimeout(timeout);
        serial_port_.open();
    } catch (const serial::IOException& e) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
        // If the port fails to open, shut down the node gracefully.
        rclcpp::shutdown();
        return;
    }

    if (serial_port_.isOpen()) {
        RCLCPP_INFO(this->get_logger(), "Serial port opened successfully.");
    }

    // Create a publisher for std_msgs::msg::String messages on the "arduino_data" topic.
    publisher_ = this->create_publisher<std_msgs::msg::String>("arduino_data", 10);
    
    // Create a wall timer that calls the read_serial_data method every 200ms (5Hz).
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200), std::bind(&ArduinoDriver::read_serial_data, this));
}

void ArduinoDriver::read_serial_data() {
    // Check if there is data available to read from the serial port.
    if (serial_port_.available()) {
        std_msgs::msg::String msg;
        // Read one line of data, terminated by the newline character.
        msg.data = serial_port_.readline();
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", msg.data.c_str());
        // Publish the received data on the ROS2 topic.
        publisher_->publish(msg);
    }
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    // Create an instance of the ArduinoDriver and spin it to process callbacks.
    rclcpp::spin(std::make_shared<ArduinoDriver>());
    rclcpp::shutdown();
    return 0;
}