/**
 * @file MinimalPublisher.cpp
 * @author Sachin Jadhav (sjd3333@umd.edu)
 * @brief Implementation of MinimalPublisher class
 * @version 0.1
 * @date 2024-11-06
 *
 * @copyright Copyright (c) 2024 Sachin Jadhav
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

#include "beginner_tutorials/publisher_member_function.hpp"

using namespace std::chrono_literals;

MinimalPublisher::MinimalPublisher()
    : Node("minimal_publisher"), count_(0) {
  // Declare parameters
  this->declare_parameter("frequency", 2.0);  // Default 2 Hz
  double freq = this->get_parameter("frequency").as_double();
  auto timer_period = std::chrono::duration<double>(1.0 / freq);

  // Debug level message
  RCLCPP_DEBUG(this->get_logger(), "Initializing publisher node");

  // Initialize the message templates
  current_message_index_ = 0;
  message_templates_ = {"Hello, world!", "Greetings!", "Hi there!", "Welcome!"};

  // Warn if message templates vector is small
  if (message_templates_.size() < 3) {
  RCLCPP_WARN(
    this->get_logger(),
    "Low number of message templates available");
  }

  // Create publisher
  publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
  if (!publisher_) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create publisher");
  }

  // Create timer with parameter-defined frequency
  timer_ = this->create_wall_timer(timer_period,
    std::bind(&MinimalPublisher::timer_callback, this));

  // Create service
  service_ = this->create_service<std_srvs::srv::Trigger>(
    "change_message",
    std::bind(&MinimalPublisher::message_change_callback, this,
              std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(this->get_logger(),
    "Publisher initialized with frequency: %.2f Hz", freq);
}

void MinimalPublisher::timer_callback() {
  auto message = std_msgs::msg::String();
  message.data = message_templates_[current_message_index_] + " " +
                std::to_string(count_++);

  // Debug message for each publication
  RCLCPP_DEBUG(this->get_logger(),
    "Preparing to publish message number: %zu", count_);

  // Warning for high count
  if (count_ % 100 == 0) {
    RCLCPP_WARN(this->get_logger(),
      "Message count reached multiple of 100: %zu", count_);
  }

  // Error simulation for demonstration
  if (count_ % 150 == 0) {
    RCLCPP_ERROR(this->get_logger(),
      "Simulated error at message count: %zu", count_);
  }

  // Fatal error simulation
  if (count_ % 300 == 0) {
    RCLCPP_FATAL(this->get_logger(),
      "Simulated fatal error at count: %zu", count_);
  }

  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);
}

void MinimalPublisher::message_change_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
    current_message_index_ = (current_message_index_ + 1)
        % message_templates_.size();

  RCLCPP_INFO(this->get_logger(),
    "Changing to message template: %s",
    message_templates_[current_message_index_].c_str());

  response->success = true;
  response->message = "Changed to message template: " +
                     message_templates_[current_message_index_];
}

/**
 * @brief Main function that initializes and runs the MinimalPublisher node
 * @param argc Number of command line arguments
 * @param argv Array of command line arguments
 * @return int Return status
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
