#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/loaned_command_interface.hpp"
#include "hardware_interface/loaned_state_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "unitree_g1_ros2_control/forward_position_controller.hpp"

namespace unitree_g1_ros2_control {
namespace {

class ForwardPositionControllerTest : public ::testing::Test {
protected:
    void SetUp() override {
        if (!rclcpp::ok()) {
            int argc = 0;
            char** argv = nullptr;
            rclcpp::init(argc, argv);
        }
    }

    void TearDown() override {
        if (rclcpp::ok()) {
            rclcpp::shutdown();
        }
    }
};

TEST_F(ForwardPositionControllerTest, forwards_finite_position_commands_unchanged) {
    double command_position = std::nan("");
    double state_position = 0.0;
    hardware_interface::CommandInterface command_interface(
        "joint", hardware_interface::HW_IF_POSITION, &command_position);
    hardware_interface::StateInterface state_interface(
        "joint", hardware_interface::HW_IF_POSITION, &state_position);

    ForwardPositionController controller;
    ASSERT_EQ(controller.init("test_forward_position_controller"),
              controller_interface::return_type::OK);
    ASSERT_TRUE(controller.get_node()->set_parameter(
        rclcpp::Parameter("joints", std::vector<std::string>{"joint"})).successful);
    ASSERT_EQ(controller.configure().label(), "inactive");

    std::vector<hardware_interface::LoanedCommandInterface> command_interfaces;
    command_interfaces.emplace_back(command_interface);
    std::vector<hardware_interface::LoanedStateInterface> state_interfaces;
    state_interfaces.emplace_back(state_interface);
    controller.assign_interfaces(
        std::move(command_interfaces), std::move(state_interfaces));
    ASSERT_EQ(controller.activate().label(), "active");
    ASSERT_DOUBLE_EQ(command_position, 0.0);

    auto publisher_node = std::make_shared<rclcpp::Node>("fpc_test_publisher");
    auto publisher = publisher_node->create_publisher<std_msgs::msg::Float64MultiArray>(
        "/test_forward_position_controller/commands",
        rclcpp::QoS(rclcpp::KeepLast(1)).best_effort());
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(controller.get_node());
    executor.add_node(publisher_node);

    const auto discovery_deadline = std::chrono::steady_clock::now() +
                                    std::chrono::seconds(2);
    while (publisher->get_subscription_count() == 0 &&
           std::chrono::steady_clock::now() < discovery_deadline) {
        executor.spin_some();
        std::this_thread::yield();
    }
    ASSERT_EQ(publisher->get_subscription_count(), 1U);

    const auto publish_and_expect = [&](double target, double expected) {
        std_msgs::msg::Float64MultiArray message;
        message.data = {target};
        publisher->publish(message);
        const auto command_deadline = std::chrono::steady_clock::now() +
                                      std::chrono::seconds(2);
        while (command_position != expected &&
               std::chrono::steady_clock::now() < command_deadline) {
            executor.spin_some();
            controller.update();
            std::this_thread::yield();
        }
        EXPECT_DOUBLE_EQ(command_position, expected);
    };

    publish_and_expect(0.08, 0.08);
    publish_and_expect(-0.08, -0.08);
    publish_and_expect(0.2, 0.2);

    state_position = -0.1;
    ASSERT_EQ(controller.deactivate().label(), "inactive");
    ASSERT_EQ(controller.activate().label(), "active");
    EXPECT_DOUBLE_EQ(command_position, -0.1);

    EXPECT_EQ(controller.deactivate().label(), "inactive");
    controller.release_interfaces();
}

}  // namespace
}  // namespace unitree_g1_ros2_control