#include <catch_ros2/catch_ros2.hpp>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "beginner_tutorials/publisher_member_function.hpp"

using namespace std::chrono_literals;
using std_msgs::msg::String;

class TestFixture {
public:
  TestFixture() {
    // Create the test node
    test_node_ = std::make_shared<rclcpp::Node>("test_listener");

    // Initialize message received flag
    message_received_ = false;
    received_message_ = "";

    // Create subscriber
    subscription_ = test_node_->create_subscription<String>(
      "topic", 10,
      [this](const String::SharedPtr msg) {
        message_received_ = true;
        received_message_ = msg->data;
        RCLCPP_INFO(test_node_->get_logger(), "Received message: '%s'", msg->data.c_str());
      });
  }

  bool waitForMessage(const std::chrono::seconds timeout = 5s) {
    auto start = std::chrono::steady_clock::now();
    rclcpp::Rate rate(10);  // 10 Hz checking rate

    while (rclcpp::ok() && !message_received_) {
      rclcpp::spin_some(test_node_);
      rate.sleep();

      auto now = std::chrono::steady_clock::now();
      if (now - start > timeout) {
        RCLCPP_ERROR(test_node_->get_logger(), "Timeout waiting for message");
        return false;
      }
    }
    return message_received_;
  }

  std::string getLastMessage() const {
    return received_message_;
  }

  rclcpp::Node::SharedPtr test_node_;

private:
  rclcpp::Subscription<String>::SharedPtr subscription_;
  bool message_received_;
  std::string received_message_;
};

TEST_CASE("Test MinimalPublisher Message Publishing", "[publisher]") {
  // Do not initialize ROS2 here, catch_ros2 handles it

  RCLCPP_INFO(rclcpp::get_logger("test_case"), "Starting publisher test");

  // Create fixture
  TestFixture fixture;

  SECTION("Verify message publishing") {
    // Wait for and verify message
    REQUIRE(fixture.waitForMessage());

    // Get the received message and check its content
    std::string received_message = fixture.getLastMessage();
    CHECK(!received_message.empty());
    RCLCPP_INFO(fixture.test_node_->get_logger(), "Test completed successfully");
  }

  // Do not shutdown ROS2 here, catch_ros2 handles it
}
