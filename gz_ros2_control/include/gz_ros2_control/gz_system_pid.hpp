// Copyright 2025 ros2_control development team
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

#ifndef GZ_ROS2_CONTROL__GZ_SYSTEM_PID_HPP_
#define GZ_ROS2_CONTROL__GZ_SYSTEM_PID_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>
#include <chrono>

#include <hardware_interface/hardware_info.hpp>
#include <rclcpp/rclcpp.hpp>
#include <gz/math/PID.hh>

namespace gz_ros2_control
{

/// \brief Helper class for PID parameter configuration
class PidConfigHelper
{
public:
  /// \brief Constructor
  PidConfigHelper() = default;

  /// \brief Initialize a position PID controller with parameters
  /// \param joint_name Name of the joint
  /// \param joint_info Hardware info for the joint
  /// \param parameters Parameter vector to store params
  /// \param pid PID controller to initialize
  /// \param initial_p_pos Default P gain if not specified
  /// \param max_velocity Maximum velocity for limits
  void configure_position_pid(
    const std::string & joint_name,
    const hardware_interface::ComponentInfo & joint_info,
    std::vector<rclcpp::Parameter> & parameters,
    gz::math::PID & pid,
    double initial_p_pos,
    double max_velocity);

  /// \brief Initialize a velocity PID controller with parameters
  /// \param joint_name Name of the joint
  /// \param joint_info Hardware info for the joint
  /// \param parameters Parameter vector to store params
  /// \param pid PID controller to initialize
  /// \param initial_p_pos Default P gain if not specified
  /// \param max_velocity Maximum velocity for limits
  /// \param max_effort Maximum effort for limits
  void configure_velocity_pid(
    const std::string & joint_name,
    const hardware_interface::ComponentInfo & joint_info,
    std::vector<rclcpp::Parameter> & parameters,
    gz::math::PID & pid,
    double initial_p_pos,
    double max_velocity,
    double max_effort);

  /// \brief Configure a PID controller's parameters from existing values
  /// \param pid PID controller to configure
  /// \param p P gain
  /// \param i I gain
  /// \param d D gain
  /// \param i_max Maximum integral term
  /// \param i_min Minimum integral term
  /// \param cmd_max Maximum command output
  /// \param cmd_min Minimum command output
  /// \param cmd_offset Command offset
  static void configure_pid(
    gz::math::PID & pid,
    double p, double i, double d,
    double i_max, double i_min,
    double cmd_max, double cmd_min,
    double cmd_offset);
    
  /// \brief Calculate target force for velocity control
  /// \param pid The velocity PID controller
  /// \param current_velocity Current joint velocity 
  /// \param target_velocity Target velocity (command)
  /// \param max_velocity Maximum allowable velocity
  /// \param period Control period duration
  /// \return Calculated force/torque command
  static double calculate_velocity_target_force(
    gz::math::PID & pid,
    double current_velocity,
    double target_velocity,
    double max_velocity,
    const rclcpp::Duration & period);
    
  /// \brief Calculate target force for position control
  /// \param pos_pid The position PID controller
  /// \param vel_pid The velocity PID controller
  /// \param current_position Current joint position
  /// \param target_position Target position (command)
  /// \param current_velocity Current joint velocity
  /// \param lower_limit Lower joint position limit
  /// \param upper_limit Upper joint position limit
  /// \param max_velocity Maximum allowable velocity
  /// \param use_cascade_control Whether to use cascade control (position->velocity->effort)
  /// \param period Control period duration
  /// \return Calculated force/torque command
  static double calculate_position_target_force(
    gz::math::PID & pos_pid,
    gz::math::PID & vel_pid,
    double current_position,
    double target_position,
    double current_velocity,
    double lower_limit,
    double upper_limit,
    double max_velocity,
    bool use_cascade_control,
    const rclcpp::Duration & period);
    
private:
  /// \brief Helper to get parameter value with default fallback
  static double get_param(
    const hardware_interface::ComponentInfo & joint_info,
    const std::string & param_name,
    double default_value);

  /// \brief Helper to add joint gain parameters to parameter vector
  static void add_joint_gain_parameter(
    std::vector<rclcpp::Parameter> & parameters,
    const std::string & joint_name,
    const std::string & param_suffix,
    double value);
};

}  // namespace gz_ros2_control

#endif  // GZ_ROS2_CONTROL__GZ_SYSTEM_PID_HPP_ 