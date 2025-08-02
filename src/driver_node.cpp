#include "serial_driver_cpp/arduino_driver.hpp"
#include <chrono>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

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
    } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to open serial port.");
        rclcpp::shutdown();
        return;
    }

    sensor_publisher_ = this->create_publisher<serial_driver_cpp::msg::SensorData>("sensor_data", 10);
    command_subscriber_ = this->create_subscription<std_msgs::msg::String>(
        "serial_command", 10, std::bind(&ArduinoDriver::command_callback, this, std::placeholders::_1));
    
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200), 
        std::bind(&ArduinoDriver::read_serial_data, this));
}

void ArduinoDriver::command_callback(const std_msgs::msg::String::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Writing to serial port: '%s'", msg->data.c_str());
    serial_port_.write(msg->data + "\n");
}

void ArduinoDriver::read_serial_data() {
    if (serial_port_.available()) {
        std::string line = serial_port_.readline();
        RCLCPP_INFO(this->get_logger(), "Read from serial: '%s'", line.c_str());

        auto message = serial_driver_cpp::msg::SensorData();
        message.header.stamp = this->now();
        message.header.frame_id = "sensor_data";

        // Basic parsing for "temp:XX.X,hum:YY.Y"
        std::stringstream ss(line);
        std::string segment;
        std::vector<std::string> seglist;

        while(std::getline(ss, segment, ','))
        {
           seglist.push_back(segment);
        }

        if (seglist.size() == 2) {
            try {
                size_t temp_pos = seglist[0].find(":");
                if (temp_pos != std::string::npos) {
                    message.temperature = std::stof(seglist[0].substr(temp_pos + 1));
                }

                size_t hum_pos = seglist[1].find(":");
                if (hum_pos != std::string::npos) {
                    message.humidity = std::stof(seglist[1].substr(hum_pos + 1));
                }
                sensor_publisher_->publish(message);
            } catch (const std::invalid_argument& ia) {
                RCLCPP_WARN(this->get_logger(), "Could not parse serial data: '%s'", line.c_str());
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Received malformed data: '%s'", line.c_str());
        }
    }
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    // Create an instance of the ArduinoDriver and spin it to process callbacks.
    rclcpp::spin(std::make_shared<ArduinoDriver>());
    rclcpp::shutdown();
    return 0;
}