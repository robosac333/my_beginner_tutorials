// For publisher_member_function.cpp:
// Copyright 2024 Sachin Jadhav
/**
 * @file MinimalPublisher.cpp
 * @brief Implementation of the MinimalPublisher class for a ROS2 node that
 * publishes messages and broadcasts transforms.
 * @version 0.1
 * @date 2024-11-15
 *
 * @author
 * Sachin Jadhav (sjd3333@umd.edu)
 *
 * @copyright
 * Copyright (c) 2024 Sachin Jadhav
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 */

#include "beginner_tutorials/publisher_member_function.hpp"

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "tf2/LinearMath/Quaternion.h"

using namespace std::chrono_literals;

/**
 * @brief Constructor for MinimalPublisher class
 * @details Initializes the ROS2 node, publisher, TF broadcaster, service, and
 * timer Sets up message templates and initializes counters
 */
MinimalPublisher::MinimalPublisher()
    : Node("minimal_publisher"), count_(0), current_message_index_(0) {
  // Initialize the publisher
  publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

  // Initialize message templates
  message_templates_ = {"Hello, world!", "Welcome to ROS2!",
                        "Broadcasting transform..."};

  // Initialize the TF broadcaster
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  // Initialize the service
  service_ = this->create_service<std_srvs::srv::Trigger>(
      "change_message",
      std::bind(&MinimalPublisher::message_change_callback, this,
                std::placeholders::_1, std::placeholders::_2));

  // Create a timer with 500ms callback
  timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
}

/**
 * @brief Timer callback function for periodic publishing and transform
 * broadcasting
 * @details Publishes messages based on templates and broadcasts time-varying
 * transforms Creates oscillating motion in translation and rotation
 */
void MinimalPublisher::timer_callback() {
  auto message = std_msgs::msg::String();
  message.data = message_templates_[current_message_index_] + " " +
                 std::to_string(count_++);
  RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
  publisher_->publish(message);

  // Create and publish the transform
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = "world";
  t.child_frame_id = "talk";

  // Create a time-varying translation
  double time_sec = this->get_clock()->now().seconds();
  t.transform.translation.x =
      2.0 * std::sin(time_sec);  // Oscillating X position
  t.transform.translation.y =
      1.0 * std::cos(time_sec);     // Oscillating Y position
  t.transform.translation.z = 0.5;  // Constant Z position

  // Create a time-varying rotation
  tf2::Quaternion q;
  q.setRPY(0.0,        // Roll (constant)
           0.0,        // Pitch (constant)
           time_sec);  // Yaw (varying with time)
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();
  t.transform.rotation.w = q.w();

  // Send the transform
  try {
    tf_broadcaster_->sendTransform(t);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to send transform: %s", e.what());
  }
}

/**
 * @brief Service callback for changing the current message template
 * @param request Empty trigger request
 * @param response Service response containing success status and message
 * @details Cycles through available message templates and updates the response
 */
void MinimalPublisher::message_change_callback(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
  current_message_index_ =
      (current_message_index_ + 1) % message_templates_.size();
  response->success = true;
  response->message =
      "Changed to message template " + std::to_string(current_message_index_);
}

#ifndef BUILDING_LIBRARY
/**
 * @brief Main function for the minimal publisher node
 * @param argc Number of command line arguments
 * @param argv Array of command line arguments
 * @return 0 on successful execution, non-zero otherwise
 * @details Initializes ROS2 system, creates the minimal publisher node,
 *          and runs the node until shutdown
 */
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
#endif
