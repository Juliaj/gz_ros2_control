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

#ifndef GZ_ROS2_CONTROL__TEST_GZ_SYSTEM_HPP_
#define GZ_ROS2_CONTROL__TEST_GZ_SYSTEM_HPP_

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>
#include <map>

#include "hardware_interface/component_parser.hpp"
#include <rclcpp/rclcpp.hpp>

// Test fixture for GazeboSimSystem tests
class TestGzSystem : public ::testing::Test
{
protected:
  void SetUp() override
  {
    // Initialize ROS context for tests
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    node_ = std::make_shared<rclcpp::Node>("test_gz_system");
  }

  void TearDown() override
  {
    node_.reset();
  }

  std::shared_ptr<rclcpp::Node> node_;
};

// Helper function to create a hardware info component from XML parameters
hardware_interface::ComponentInfo create_component_info(
  const std::string & name,
  const std::vector<std::string> & command_interfaces,
  const std::vector<std::string> & state_interfaces,
  const std::map<std::string, std::string> & parameters)
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

  // Copy parameters to the unordered_map
  for (const auto & param : parameters) {
    component_info.parameters[param.first] = param.second;
  }

  return component_info;
}

#endif  // GZ_ROS2_CONTROL__TEST_GZ_SYSTEM_HPP_
