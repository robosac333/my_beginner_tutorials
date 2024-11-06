/**
 * @file MinimalSubscriber.hpp
 * @author Sachin Jadhav (sjd3333@umd.edu)
 * @brief Header file for MinimalSubscriber class that subscribes to string messages
 * @version 0.1
 * @date 2024-11-06
 *
 * @copyright Copyright (c) 2024 Sachin Jadhav
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

#ifndef MINIMAL_SUBSCRIBER_HPP_
#define MINIMAL_SUBSCRIBER_HPP_

#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

/**
 * @brief A class that subscribes to string messages and processes them
 * @details This class inherits from rclcpp::Node and implements a subscriber
 *          that receives and processes string messages. It includes various
 *          logging levels for demonstration and monitoring purposes.
 */
class MinimalSubscriber : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Minimal Subscriber object
   * @details Initializes the subscriber node and sets up the subscription
   *          with appropriate callback and quality of service settings.
   */
  MinimalSubscriber();

 private:
  /**
   * @brief Callback function that processes received messages
   * @param msg The received string message
   * @details This function is called whenever a new message is received.
   *          It processes the message and includes various logging levels
   *          for monitoring and demonstration purposes.
   */
  void topic_callback(const std_msgs::msg::String& msg);

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;  ///< Subscriber for string messages
  size_t count_;  ///< Counter for number of messages received
};

#endif  // MINIMAL_SUBSCRIBER_HPP_
