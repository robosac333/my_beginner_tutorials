/**
 * @file publisher_test.cpp
 * @brief Test suite for ROS2 minimal publisher functionality
 * @details Contains test fixtures and cases to verify publisher behavior using catch_ros2
 */

#include <catch_ros2/catch_ros2.hpp>
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include "beginner_tutorials/publisher_member_function.hpp"

using namespace std::chrono_literals;
using std_msgs::msg::String;

/**
 * @brief Test fixture class for ROS2 publisher testing
 * @details Provides setup for subscriber and message reception verification
 */
class TestFixture {
public:
  /**
   * @brief Constructor that initializes the test node and subscriber
   */
  TestFixture() {
    // Create the test node
    test_node_ = std::make_shared<rclcpp::Node>("test_listener");

    // Initialize message received flag
    message_received_ = false;
    received_message_ = "";

    // Create subscriber
    subscription_ = test_node_->create_subscription<String>(
        "topic",
        10,
        [this](const String::SharedPtr msg) {
          message_received_ = true;
          received_message_ = msg->data;
          RCLCPP_INFO(test_node_->get_logger(),
                      "Received message: '%s'",
                      msg->data.c_str());
        });
  }

  /**
   * @brief Waits for a message to be received within specified timeout
   * @param timeout Maximum duration to wait for message
   * @return true if message was received, false if timeout occurred
   */
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

  /**
   * @brief Retrieves the last received message
   * @return String containing the last received message
   */
  std::string getLastMessage() const { return received_message_; }

  /** @brief Shared pointer to the test node */
  rclcpp::Node::SharedPtr test_node_;

private:
  /** @brief Subscription handle for the test topic */
  rclcpp::Subscription<String>::SharedPtr subscription_;
  /** @brief Flag indicating whether a message has been received */
  bool message_received_;
  /** @brief Storage for the last received message */
  std::string received_message_;
};

/**
 * @brief Test case to verify minimal publisher functionality
 * @details Verifies that the publisher can successfully publish messages
 *          and that they can be received by a subscriber
 */
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
