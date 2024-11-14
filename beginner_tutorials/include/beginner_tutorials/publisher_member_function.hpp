/**
 * @file MinimalPublisher.hpp
 * @author Sachin Jadhav (sjd3333@umd.edu)
 * @brief Header file for MinimalPublisher class that publishes string messages and provides a message change service
 * @version 0.1
 * @date 2024-11-06
 *
 * @copyright Copyright (c) 2024 Sachin Jadhav
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

#ifndef MINIMAL_PUBLISHER_HPP_
#define MINIMAL_PUBLISHER_HPP_

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

/**
 * @brief A class that publishes string messages and provides a service to change the message template
 * @details This class inherits from rclcpp::Node and implements a publisher that sends string
 *          messages periodically. It also provides a service to change the message template.
 */
class MinimalPublisher : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new Minimal Publisher object
   * @details Initializes the publisher node with a default frequency of 2Hz and sets up
   *          the publisher, timer, and service.
   */
  MinimalPublisher();

 private:
  /**
   * @brief Callback function for the timer that triggers message publishing
   * @details This function is called periodically by the timer to publish messages.
   *          It also includes various logging messages at different severity levels
   *          for demonstration purposes.
   */
  void timer_callback();

  /**
   * @brief Callback function for the message change service
   * @param request The service request (unused)
   * @param response The service response containing success status and message
   * @details This function handles service calls to change the message template.
   *          It cycles through the available message templates and updates the response.
   */
  void message_change_callback(
      const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response);

  rclcpp::TimerBase::SharedPtr timer_;  ///< Timer for periodic message publishing
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;  ///< Publisher for string messages
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr service_;  ///< Service for changing message template
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;  ///< TF2 broadcaster
  size_t count_;  ///< Counter for number of messages published
  std::vector<std::string> message_templates_;  ///< Vector of available message templates
  size_t current_message_index_;  ///< Index of current message template
};

#endif  // MINIMAL_PUBLISHER_HPP_
