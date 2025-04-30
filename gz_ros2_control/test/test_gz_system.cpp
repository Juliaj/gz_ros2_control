// Copyright 2025 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "test_gz_system.hpp"

#include <algorithm>
#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <control_toolbox/pid.hpp>
#include <gtest/gtest.h>
#include <hardware_interface/component_parser.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/resource_manager.hpp>
#include <rclcpp/rclcpp.hpp>

#include "gz_ros2_control/gz_system.hpp"
#include "gz_ros2_control/joint_pid_helper.hpp"

namespace gz_ros2_control
{
// Test class for the GzSystem
class TestGzSystem : public ::testing::Test
{
public:
  TestGzSystem()
  {
    // Create a test node (ROS is already initialized in main)
    node_ = std::make_shared<rclcpp::Node>("test_gz_system_node");
  }

  ~TestGzSystem()
  {
    // Just reset the node, don't shutdown ROS (done in main)
    if (node_) {
      node_.reset();
    }
  }

  // Helper method to create a component info with parameters
  hardware_interface::ComponentInfo create_component_info(
    const std::string & name,
    const std::vector<std::string> & command_interfaces,
    const std::vector<std::string> & state_interfaces,
    const std::unordered_map<std::string, std::string> & parameters = {})
  {
    hardware_interface::ComponentInfo component_info;
    component_info.name = name;

    for (const auto & interface : command_interfaces) {
      hardware_interface::InterfaceInfo interface_info;
      interface_info.name = interface;
      component_info.command_interfaces.push_back(interface_info);
    }

    for (const auto & interface : state_interfaces) {
      hardware_interface::InterfaceInfo interface_info;
      interface_info.name = interface;
      component_info.state_interfaces.push_back(interface_info);
    }

    component_info.parameters = parameters;

    return component_info;
  }

  // Shared ROS node for parameter handling
  rclcpp::Node::SharedPtr node_;
};

// Test for PID parameter parsing with defaults
TEST_F(TestGzSystem, pid_parameters_defaults)
{
  // Create PID helper
  JointPosVelPidHelper pid_helper;

  // Create hardware info for velocity controlled joint (based on XML template)
  auto velocity_joint_info = create_component_info(
    "rear_left_wheel_joint",
    {"velocity"},
    {"velocity", "position"},
    std::unordered_map<std::string, std::string>{{"p_vel", "1000.0"}});

  // Test vectors for storing parameters
  std::vector<rclcpp::Parameter> parameters;
  control_toolbox::Pid pid;

  // Default values for initialization
  double dummy_p_pos = 10.0;  // Arbitrary default
  double max_velocity = 10.0;
  double max_effort = 100.0;

  // Configure velocity PID
  pid_helper.configure_velocity_pid(
    "rear_left_wheel_joint",
    velocity_joint_info,
    parameters,
    pid,
    dummy_p_pos,
    max_velocity,
    max_effort
  );

  // Verify the PID parameters were properly set
  double p, i, d, i_max, i_min;
  bool antiwindup;
  pid.get_gains(p, i, d, i_max, i_min, antiwindup);

  EXPECT_DOUBLE_EQ(p, 1000.0);  // From our XML p_vel
  EXPECT_DOUBLE_EQ(i, dummy_p_pos / 1000.0);  // Default
  EXPECT_DOUBLE_EQ(d, 0.0);  // Default

  // Create hardware info for position controlled joint (based on XML template)
  auto position_joint_info = create_component_info(
    "left_wheel_steering_joint",
    {"position"},
    {"position"},
    std::unordered_map<std::string, std::string>{{"p_pos", "1000.0"}});

  // Configure position PID
  pid_helper.configure_position_pid(
    "left_wheel_steering_joint",
    position_joint_info,
    parameters,
    pid,
    dummy_p_pos,
    max_velocity
  );

  // Verify the PID parameters were properly set
  pid.get_gains(p, i, d, i_max, i_min, antiwindup);

  EXPECT_DOUBLE_EQ(p, 1000.0);  // From our XML p_pos
  EXPECT_DOUBLE_EQ(i, 0.0);  // Default
  EXPECT_DOUBLE_EQ(d, 1000.0 / 100.0);  // Default based on p_pos
}

// Test for PID parameter parsing with explicit values
TEST_F(TestGzSystem, pid_parameters_explicit)
{
  // Create PID helper
  JointPosVelPidHelper pid_helper;

  // Create hardware info for velocity controlled joint with full parameters
  auto velocity_joint_info = create_component_info(
    "rear_left_wheel_joint",
    {"velocity"},
    {"velocity", "position"},
    std::unordered_map<std::string, std::string>{
      {"p_vel", "1000.0"},
      {"i_vel", "10.0"},
      {"d_vel", "5.0"},
      {"i_vel_max", "50.0"},
      {"i_vel_min", "-50.0"},
      {"cmd_vel_max", "5.0"},
      {"cmd_vel_min", "-5.0"},
      {"cmd_vel_forward_gain", "0.5"}
    });

  // Test vectors for storing parameters
  std::vector<rclcpp::Parameter> parameters;
  control_toolbox::Pid pid;

  // Default values for initialization
  double dummy_p_pos = 10.0;  // Arbitrary default
  double max_velocity = 10.0;
  double max_effort = 100.0;

  // Configure velocity PID
  pid_helper.configure_velocity_pid(
    "rear_left_wheel_joint",
    velocity_joint_info,
    parameters,
    pid,
    dummy_p_pos,
    max_velocity,
    max_effort
  );

  // Verify the PID parameters were properly set
  double p, i, d, i_max, i_min;
  bool antiwindup;
  pid.get_gains(p, i, d, i_max, i_min, antiwindup);

  EXPECT_DOUBLE_EQ(p, 1000.0);
  EXPECT_DOUBLE_EQ(i, 10.0);
  EXPECT_DOUBLE_EQ(d, 5.0);
  EXPECT_DOUBLE_EQ(i_max, 50.0);
  EXPECT_DOUBLE_EQ(i_min, -50.0);
  // Note: cmd_max, cmd_min, cmd_offset aren't directly supported in control_toolbox::Pid

  // Create hardware info for position controlled joint with full parameters
  auto position_joint_info = create_component_info(
    "left_wheel_steering_joint",
    {"position"},
    {"position"},
    std::unordered_map<std::string, std::string>{
      {"p_pos", "1000.0"},
      {"i_pos", "20.0"},
      {"d_pos", "10.0"},
      {"i_pos_max", "100.0"},
      {"i_pos_min", "-100.0"},
      {"cmd_pos_max", "2.0"},
      {"cmd_pos_min", "-2.0"},
      {"cmd_pos_forward_gain", "0.2"}
    });

  // Reset PID object and parameters
  pid = control_toolbox::Pid();
  parameters.clear();

  // Configure position PID
  pid_helper.configure_position_pid(
    "left_wheel_steering_joint",
    position_joint_info,
    parameters,
    pid,
    dummy_p_pos,
    max_velocity
  );

  // Verify the PID parameters were properly set
  pid.get_gains(p, i, d, i_max, i_min, antiwindup);

  EXPECT_DOUBLE_EQ(p, 1000.0);
  EXPECT_DOUBLE_EQ(i, 20.0);
  EXPECT_DOUBLE_EQ(d, 10.0);
  EXPECT_DOUBLE_EQ(i_max, 100.0);
  EXPECT_DOUBLE_EQ(i_min, -100.0);
  // Note: cmd_max, cmd_min, cmd_offset aren't directly supported in control_toolbox::Pid
}

// Test the complete ackermann drive XML configuration
TEST_F(TestGzSystem, TestAckermannDriveConfiguration)
{
  // XML string based on the provided template
  const std::string xml_string =
    R"(
    <ros2_control name="GazeboSystem" type="system">
      <hardware>
        <plugin>gz_ros2_control/GazeboSimSystem</plugin>
      </hardware>
      <joint name="rear_left_wheel_joint">
        <command_interface name="velocity" />
        <state_interface name="velocity" />
        <state_interface name="position" />
        <param name="p_vel">1000.0</param>
      </joint>
      <joint name="rear_right_wheel_joint">
        <command_interface name="velocity" />
        <state_interface name="velocity" />
        <state_interface name="position" />
        <param name="p_vel">1000.0</param>
      </joint>
      <joint name="left_wheel_steering_joint">
        <command_interface name="position" />
        <state_interface name="position" />
        <param name="p_pos">1000.0</param>
      </joint>
      <joint name="right_wheel_steering_joint">
        <command_interface name="position" />
        <state_interface name="position" />
        <param name="p_pos">1000.0</param>
      </joint>
    </ros2_control>
  )";

  // Parse the XML string to hardware info
  const auto hardware_info = hardware_interface::parse_control_resources_from_urdf(xml_string);

  // Verify 4 joints were parsed
  ASSERT_EQ(hardware_info.size(), 1u);
  ASSERT_EQ(hardware_info[0].joints.size(), 4u);

  // Create PID helper for testing parsed values
  JointPosVelPidHelper pid_helper;
  std::vector<rclcpp::Parameter> parameters;
  control_toolbox::Pid pid;

  // Test first velocity joint (rear_left_wheel_joint)
  auto & rear_left_joint = hardware_info[0].joints[0];
  EXPECT_EQ(rear_left_joint.name, "rear_left_wheel_joint");
  EXPECT_EQ(rear_left_joint.command_interfaces.size(), 1u);
  EXPECT_EQ(rear_left_joint.command_interfaces[0].name, "velocity");
  EXPECT_EQ(rear_left_joint.parameters.size(), 1u);
  EXPECT_EQ(rear_left_joint.parameters.at("p_vel"), "1000.0");

  // Configure and test PID parameters for rear_left_wheel_joint
  pid_helper.configure_velocity_pid(
    rear_left_joint.name,
    rear_left_joint,
    parameters,
    pid,
    10.0,   // dummy_p_pos
    10.0,   // max_velocity
    100.0   // max_effort
  );
  double p, i, d, i_max, i_min;
  bool antiwindup;
  pid.get_gains(p, i, d, i_max, i_min, antiwindup);
  EXPECT_DOUBLE_EQ(p, 1000.0);

  // Test first position joint (left_wheel_steering_joint)
  auto & left_steering_joint = hardware_info[0].joints[2];
  EXPECT_EQ(left_steering_joint.name, "left_wheel_steering_joint");
  EXPECT_EQ(left_steering_joint.command_interfaces.size(), 1u);
  EXPECT_EQ(left_steering_joint.command_interfaces[0].name, "position");
  EXPECT_EQ(left_steering_joint.parameters.size(), 1u);
  EXPECT_EQ(left_steering_joint.parameters.at("p_pos"), "1000.0");

  // Configure and test PID parameters for left_wheel_steering_joint
  pid = control_toolbox::Pid();  // Reset PID
  pid_helper.configure_position_pid(
    left_steering_joint.name,
    left_steering_joint,
    parameters,
    pid,
    10.0,   // dummy_p_pos
    10.0    // max_velocity
  );
  pid.get_gains(p, i, d, i_max, i_min, antiwindup);
  EXPECT_DOUBLE_EQ(p, 1000.0);
}

// Test interaction with global gz_ros_control parameters
TEST_F(TestGzSystem, TestGlobalParameters)
{
  // Set up global parameters as they would appear in a ROS parameter file
  node_->declare_parameter("hold_joints", false);
  node_->declare_parameter("position_proportional_gain", 0.5);

  // Verify parameters were correctly set
  EXPECT_FALSE(node_->get_parameter("hold_joints").as_bool());
  EXPECT_DOUBLE_EQ(node_->get_parameter("position_proportional_gain").as_double(), 0.5);

  // Create joint info with minimal parameters
  auto joint_info = create_component_info(
    "test_joint",
    {"position"},
    {"position"},
    std::unordered_map<std::string, std::string>{}  // No PID parameters specified
  );

  // Create PID helper and test vectors
  JointPosVelPidHelper pid_helper;
  std::vector<rclcpp::Parameter> parameters;
  control_toolbox::Pid pid;

  // Default values - the position_proportional_gain would be used if no p_pos is specified
  double dummy_p_pos = 10.0;  // This would be calculated from joint limits
  double max_velocity = 5.0;

  // Configure position PID - with no p_pos in joint params, should use defaults
  pid_helper.configure_position_pid(
    "test_joint",
    joint_info,
    parameters,
    pid,
    dummy_p_pos,  // This is what would be used without position_proportional_gain
    max_velocity
  );

  // Without global parameter influence, PGain would be dummy_p_pos
  double p, i, d, i_max, i_min;
  bool antiwindup;
  pid.get_gains(p, i, d, i_max, i_min, antiwindup);
  EXPECT_DOUBLE_EQ(p, dummy_p_pos);

  // Test the force calculation with global parameter values
  rclcpp::Duration period = rclcpp::Duration::from_seconds(0.1);

  // Calculate with a position error of 1.0
  double target_force = JointPosVelPidHelper::calculate_position_target_force(
    pid,
    pid,  // Using same PID for position and velocity for simplicity
    2.0,  // current position
    1.0,  // target position
    0.0,  // current velocity
    -10.0,  // lower limit
    10.0,  // upper limit
    max_velocity,
    false,  // no cascade
    period
  );

  // Expected force calculation (with default values)
  // For direct mode with p_gain = dummy_p_pos:
  // target_force = vel_p_gain * position_error = dummy_p_pos * (2.0 - 1.0) = dummy_p_pos * 1.0
  EXPECT_NEAR(target_force, dummy_p_pos, 0.01);

  // Now let's test with hold_joints parameter influence
  // Re-create test joint info with velocity command interface
  auto velocity_joint_info = create_component_info(
    "velocity_joint",
    {"velocity"},
    {"velocity", "position"},
    std::unordered_map<std::string, std::string>{}  // No PID parameters specified
  );

  // When hold_joints is false, force should be 0 when no control modes are active
  // This would be tested in the actual system, but we can verify the parameters are correctly read
  bool hold_joints = node_->get_parameter("hold_joints").as_bool();
  EXPECT_FALSE(hold_joints);
}

// Test for parameter precedence (joint-specific vs global)
TEST_F(TestGzSystem, TestParameterPrecedence)
{
  // Set global parameters
  node_->declare_parameter("position_proportional_gain", 0.5);

  // Create joint info with explicit p_pos parameter (should override global)
  auto joint_info = create_component_info(
    "test_joint",
    {"position"},
    {"position"},
    std::unordered_map<std::string, std::string>{{"p_pos", "20.0"}}  // Explicitly set p_pos
  );

  // Create PID helper and test vectors
  JointPosVelPidHelper pid_helper;
  std::vector<rclcpp::Parameter> parameters;
  control_toolbox::Pid pid;

  // Configure position PID
  pid_helper.configure_position_pid(
    "test_joint",
    joint_info,
    parameters,
    pid,
    10.0,  // dummy_p_pos (would be used if no p_pos or global param)
    5.0    // max_velocity
  );

  // Joint-specific parameter should take precedence
  double p, i, d, i_max, i_min;
  bool antiwindup;
  pid.get_gains(p, i, d, i_max, i_min, antiwindup);
  EXPECT_DOUBLE_EQ(p, 20.0);  // Should use joint's p_pos, not global

  // Now test with a joint that doesn't specify p_pos
  auto default_joint_info = create_component_info(
    "default_joint",
    {"position"},
    {"position"},
    std::unordered_map<std::string, std::string>{}  // No PID parameters
  );

  // Reset PID
  pid = control_toolbox::Pid();

  // When joint doesn't specify p_pos, the global parameter or fallback should be used
  pid_helper.configure_position_pid(
    "default_joint",
    default_joint_info,
    parameters,
    pid,
    10.0,  // dummy_p_pos
    5.0    // max_velocity
  );

  // Should fall back to dummy_p_pos since we don't have the global param
  // in the actual system implementation
  pid.get_gains(p, i, d, i_max, i_min, antiwindup);
  EXPECT_DOUBLE_EQ(p, 10.0);
}

// Test XML with combined global and joint-specific parameters
TEST_F(TestGzSystem, TestCombinedParameters)
{
  // Set global parameters
  node_->declare_parameter("hold_joints", true);
  node_->declare_parameter("position_proportional_gain", 0.5);

  // XML with some joints specifying parameters and others not
  const std::string xml_string =
    R"(
    <ros2_control name="GazeboSystem" type="system">
      <hardware>
        <plugin>gz_ros2_control/GazeboSimSystem</plugin>
      </hardware>
      <joint name="specific_joint">
        <command_interface name="position" />
        <state_interface name="position" />
        <param name="p_pos">1000.0</param>
        <param name="i_pos">10.0</param>
      </joint>
      <joint name="default_joint">
        <command_interface name="position" />
        <state_interface name="position" />
        <!-- No PID parameters specified, should use defaults/globals -->
      </joint>
      <joint name="velocity_joint">
        <command_interface name="velocity" />
        <state_interface name="velocity" />
        <param name="p_vel">500.0</param>
      </joint>
    </ros2_control>
  )";

  // Parse the XML string to hardware info
  const auto hardware_info = hardware_interface::parse_control_resources_from_urdf(xml_string);

  // Verify 3 joints were parsed
  ASSERT_EQ(hardware_info.size(), 1u);
  ASSERT_EQ(hardware_info[0].joints.size(), 3u);

  // Create PID helper for testing
  JointPosVelPidHelper pid_helper;
  std::vector<rclcpp::Parameter> parameters;
  control_toolbox::Pid pid;

  // Test joint with specific parameters
  auto & specific_joint = hardware_info[0].joints[0];
  EXPECT_EQ(specific_joint.name, "specific_joint");
  EXPECT_EQ(specific_joint.parameters.at("p_pos"), "1000.0");

  pid_helper.configure_position_pid(
    specific_joint.name,
    specific_joint,
    parameters,
    pid,
    10.0,   // dummy_p_pos (would be used if no p_pos)
    5.0     // max_velocity
  );

  // Should use joint-specific value
  double p, i, d, i_max, i_min;
  bool antiwindup;
  pid.get_gains(p, i, d, i_max, i_min, antiwindup);
  EXPECT_DOUBLE_EQ(p, 1000.0);
  EXPECT_DOUBLE_EQ(i, 10.0);  // From joint param

  // Test joint without specific parameters
  auto & default_joint = hardware_info[0].joints[1];
  EXPECT_EQ(default_joint.name, "default_joint");
  EXPECT_TRUE(default_joint.parameters.empty());

  // Reset PID
  pid = control_toolbox::Pid();

  pid_helper.configure_position_pid(
    default_joint.name,
    default_joint,
    parameters,
    pid,
    10.0,   // dummy_p_pos
    5.0     // max_velocity
  );

  // Without a direct link to the global parameter system,
  // the test will use the dummy_p_pos
  pid.get_gains(p, i, d, i_max, i_min, antiwindup);
  EXPECT_DOUBLE_EQ(p, 10.0);  // Will use dummy_p_pos
}

}  // namespace gz_ros2_control

int main(int argc, char ** argv)
{
  ::testing::InitGoogleTest(&argc, argv);

  // Initialize ROS
  rclcpp::init(argc, argv);

  // Run tests
  int result = RUN_ALL_TESTS();

  // Shutdown ROS
  rclcpp::shutdown();

  return result;
}
