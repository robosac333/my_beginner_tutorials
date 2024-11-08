/**
 * @file MinimalPublisher.cpp
 * @brief Implementation of the MinimalPublisher class for a ROS2 node that publishes messages to a topic.
 * @version 0.1
 * @date 2024-11-06
 *
 * @author
 * Sachin Jadhav (sjd3333@umd.edu)
 *
 * @copyright
 * Copyright (c) 2024 Sachin Jadhav
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/**
 * @class MinimalPublisher
 * @brief A simple publisher node in ROS2 that periodically publishes a String message.
 *
 * This class creates a ROS2 node that publishes a "Hello, world!" message to a topic named "topic" every 500 milliseconds.
 */
class MinimalPublisher : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Minimal Publisher node.
   *
   * The constructor initializes the node, sets up a publisher for the "topic" topic,
   * and starts a timer to periodically invoke the publishing callback.
   */
  MinimalPublisher() : Node("minimal_publisher"), count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

 private:
  /**
   * @brief Timer callback function to publish messages.
   *
   * This function is called periodically by the timer. It creates a String message
   * with a count and publishes it to the topic. The count increments with each message.
   */
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  /// Timer object that triggers the callback function periodically.
  rclcpp::TimerBase::SharedPtr timer_;
  /// Publisher object for sending messages to the topic.
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  /// Counter for the number of messages published.
  size_t count_;
};

/**
 * @brief Main function for the MinimalPublisher node.
 *
 * Initializes the ROS2 system, creates the MinimalPublisher node, and starts
 * spinning to process the timer callbacks.
 *
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return int Exit status of the program.
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
