// src/subscriber_member_function.cpp
// Copyright 2024 Sachin Jadhav

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node {
public:
  MinimalSubscriber() : Node("minimal_subscriber"), count_(0) {
    RCLCPP_DEBUG(this->get_logger(), "Initializing subscriber node");

    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic",
      10,
      std::bind(&MinimalSubscriber::topic_callback, this, _1));

    if (!subscription_) {
      RCLCPP_ERROR(this->get_logger(), "Failed to create subscription");
    }

    RCLCPP_INFO(this->get_logger(), "Subscriber initialized successfully");
  }

private:
  void topic_callback(const std_msgs::msg::String& msg) {
    count_++;

    // Debug message for each received message
    RCLCPP_DEBUG(this->get_logger(),
      "Received message number: %zu", count_);

    // Regular info logging
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());

    // Warning for every 50th message
    if (count_ % 50 == 0) {
      RCLCPP_WARN(this->get_logger(),
        "Received 50 messages. Current count: %zu", count_);
    }

    // Error simulation
    if (count_ % 175 == 0) {
      RCLCPP_ERROR(this->get_logger(),
        "Simulated error at message count: %zu", count_);
    }
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  size_t count_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MinimalSubscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
