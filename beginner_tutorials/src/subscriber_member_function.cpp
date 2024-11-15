/**
 * @file MinimalSubscriber.cpp
 * @brief Implementation of the MinimalSubscriber class for a ROS2 node that subscribes to a topic.
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

#include "beginner_tutorials/subscriber_member_function.hpp"
#include <functional>
#include <memory>

using std::placeholders::_1;

/**
 * @brief Constructor for MinimalSubscriber class
 * @details Initializes the ROS2 node and creates a subscription to the 'topic' topic
 */
MinimalSubscriber::MinimalSubscriber()
    : Node("minimal_subscriber"),
      count_(0) {
  subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic",
      10,
      std::bind(&MinimalSubscriber::topic_callback, this, _1));
}

/**
 * @brief Callback function for processing received messages
 * @param msg The received string message
 * @details Processes incoming messages and demonstrates different logging levels
 *          Keeps track of message count and provides various status updates
 */
void MinimalSubscriber::topic_callback(const std_msgs::msg::String& msg) {
  // Log message at different severity levels for demonstration
  RCLCPP_DEBUG(this->get_logger(),
               "Debug: Message #%zu received",
               count_);
  RCLCPP_INFO(this->get_logger(),
              "I heard: '%s'",
              msg.data.c_str());

  // Example of different logging levels
  if (count_ % 5 == 0) {
    RCLCPP_WARN(this->get_logger(),
                "Received %zu messages so far",
                count_);
  }

  if (msg.data.empty()) {
    RCLCPP_ERROR(this->get_logger(),
                 "Received empty message!");
  }

  count_++;
}

/**
 * @brief Main function for the minimal subscriber node
 * @param argc Number of command line arguments
 * @param argv Array of command line arguments
 * @return 0 on successful execution, non-zero otherwise
 * @details Initializes ROS2 system, creates the minimal subscriber node,
 *          and runs the node until shutdown
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
