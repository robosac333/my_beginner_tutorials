/**
 * @file MinimalSubscriber.cpp
 * @author Sachin Jadhav (sjd3333@umd.edu)
 * @brief Implementation of MinimalSubscriber class
 * @version 0.1
 * @date 2024-11-06
 *
 * @copyright Copyright (c) 2024 Sachin Jadhav
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

#include "beginner_tutorials/subscriber_member_function.hpp"

using std::placeholders::_1;

MinimalSubscriber::MinimalSubscriber()
    : Node("minimal_subscriber"), count_(0) {
  // Debug level message for initialization
  RCLCPP_DEBUG(this->get_logger(), "Initializing subscriber node");

  // Create subscription with appropriate callback
  subscription_ = this->create_subscription<std_msgs::msg::String>(
    "topic",
    10,  // QoS depth
    std::bind(&MinimalSubscriber::topic_callback, this, _1));

  // Check if subscription creation was successful
  if (!subscription_) {
    RCLCPP_ERROR(this->get_logger(), "Failed to create subscription");
  }

  // Log successful initialization
  RCLCPP_INFO(this->get_logger(), "Subscriber initialized successfully");
}

void MinimalSubscriber::topic_callback(const std_msgs::msg::String& msg) {
  // Increment message counter
  count_++;

  // Debug level logging for message reception
  RCLCPP_DEBUG(this->get_logger(),
    "Received message number: %zu", count_);

  // Info level logging for message content
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());

  // Warning log for every 50th message
  if (count_ % 50 == 0) {
    RCLCPP_WARN(this->get_logger(),
      "Received 50 messages. Current count: %zu", count_);
  }

  // Error simulation for demonstration
  if (count_ % 175 == 0) {
    RCLCPP_ERROR(this->get_logger(),
      "Simulated error at message count: %zu", count_);
  }
}

/**
 * @brief Main function that initializes and runs the MinimalSubscriber node
 * @param argc Number of command line arguments
 * @param argv Array of command line arguments
 * @return int Return status
 */
int main(int argc, char* argv[]) {
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Create and spin the subscriber node
  auto node = std::make_shared<MinimalSubscriber>();
  rclcpp::spin(node);

  // Clean shutdown
  rclcpp::shutdown();
  return 0;
}
