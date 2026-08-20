#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "hardware_interface/component_parser.hpp"
#include "rclcpp/rclcpp.hpp"
#include "unitree_g1_ros2_control/g1_topic_system.hpp"
#include "yaml-cpp/yaml.h"

namespace unitree_g1_ros2_control {
namespace {

const std::string kGainFile = std::string(CONFIG_DIR) + "/default_31dof_param.yaml";

/// Joint order read back from the file the plugin itself loads, so a rename in
/// one place fails here rather than at bring-up.
std::vector<std::string> shipped_joints() {
    const YAML::Node document = YAML::LoadFile(kGainFile);
    return document["/**"]["ros__parameters"]["joints"].as<std::vector<std::string>>();
}

std::string interfaces(const char* tag, const std::vector<std::string>& names) {
    std::string block;
    for (const auto& name : names) block += std::string("<") + tag + " name=\"" + name + "\"/>";
    return block;
}

/// The `<ros2_control>` block the xacro macro emits.
std::string control_urdf() {
    std::string urdf =
        "<?xml version=\"1.0\"?><robot name=\"g1\">"
        "<ros2_control name=\"g1_whole_body_system\" type=\"system\">"
        "<hardware><plugin>unitree_g1_ros2_control/G1TopicSystem</plugin>"
        "<param name=\"gain_file\">" + kGainFile + "</param></hardware>";
    for (const auto& joint : shipped_joints()) {
        urdf += "<joint name=\"" + joint + "\">";
        urdf += interfaces("command_interface", {"position"});
        urdf += interfaces("state_interface", {"position", "velocity", "effort"});
        urdf += "</joint>";
    }
    for (const char* name : {"left_ft_sensor", "right_ft_sensor"}) {
        urdf += std::string("<sensor name=\"") + name + "\">";
        urdf += interfaces(
            "state_interface",
            {"force.x", "force.y", "force.z", "torque.x", "torque.y", "torque.z"});
        urdf += "</sensor>";
    }
    urdf += "<sensor name=\"pelvis_imu\">";
    urdf += interfaces(
        "state_interface",
        {"orientation.x", "orientation.y", "orientation.z", "orientation.w",
         "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
         "linear_acceleration.x", "linear_acceleration.y", "linear_acceleration.z"});
    urdf += "</sensor></ros2_control></robot>";
    return urdf;
}

hardware_interface::CallbackReturn initialise(const std::string& urdf) {
    G1TopicSystem system;
    return system.on_init(hardware_interface::parse_control_resources_from_urdf(urdf).front());
}

/// The joint and sensor layout is agreed between the xacro macro and
/// `configure_interfaces`, and a mismatch is silent from the outside: the
/// component refuses to initialise, controller_manager still comes up but with
/// no resources, and every spawner just times out after thirty seconds. The
/// only trace is one ERROR line in ~/.ros/log/ros2_control_node_*.log.
TEST(G1TopicSystemInterfaces, accepts_the_shipped_layout) {
    EXPECT_EQ(initialise(control_urdf()), hardware_interface::CallbackReturn::SUCCESS);
}

}  // namespace
}  // namespace unitree_g1_ros2_control

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    const int result = RUN_ALL_TESTS();
    rclcpp::shutdown();
    return result;
}
