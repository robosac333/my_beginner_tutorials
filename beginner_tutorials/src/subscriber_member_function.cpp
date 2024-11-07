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

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

/**
 * @class MinimalSubscriber
 * @brief A simple subscriber node in ROS2 that listens to a String message topic.
 *
 * This class creates a ROS2 node that subscribes to a topic named "topic" and logs the received messages.
 */
class MinimalSubscriber : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Minimal Subscriber node.
   *
   * The constructor initializes the node and sets up a subscription to the "topic" topic.
   */
  MinimalSubscriber() : Node("minimal_subscriber") {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

 private:
  /**
   * @brief Callback function to process received messages.
   *
   * This function is triggered each time a new message is received on the subscribed topic.
   * It logs the content of the message.
   *
   * @param msg The message received from the topic, of type std_msgs::msg::String.
   */
  void topic_callback(const std_msgs::msg::String& msg) const {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
  }

  /// Subscription object for receiving messages from the topic.
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

/**
 * @brief Main function for the MinimalSubscriber node.
 *
 * Initializes the ROS2 system, creates the MinimalSubscriber node, and starts
 * spinning to process callbacks.
 *
 * @param argc Number of command-line arguments.
 * @param argv Array of command-line arguments.
 * @return int Exit status of the program.
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
