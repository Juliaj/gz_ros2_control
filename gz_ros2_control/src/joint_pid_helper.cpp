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

#include "gz_ros2_control/joint_pid_helper.hpp"
#include <algorithm>  // For std::min and std::max

namespace gz_ros2_control
{

// Define a custom clamp function since older compilers may not have std::clamp
template<typename T>
T clamp(const T & value, const T & low, const T & high)
{
  return std::max(low, std::min(value, high));
}

double JointPosVelPidHelper::get_param(
  const hardware_interface::ComponentInfo & joint_info,
  const std::string & param_name,
  double default_value)
{
  return (joint_info.parameters.find(param_name) == joint_info.parameters.end()) ?
         default_value :
         std::stod(joint_info.parameters.at(param_name));
}

void JointPosVelPidHelper::add_joint_gain_parameter(
  std::vector<rclcpp::Parameter> & parameters,
  const std::string & joint_name,
  const std::string & param_suffix,
  double value)
{
  parameters.push_back(rclcpp::Parameter{"gains." + joint_name + "." + param_suffix, value});
}

void JointPosVelPidHelper::configure_pid(
  control_toolbox::Pid & pid,
  double p, double i, double d,
  double i_max, double i_min,
  double cmd_max, double cmd_min,
  double cmd_offset)
{
  // Note: control_toolbox::Pid doesn't have a direct equivalent to cmd_max/cmd_min/cmd_offset
  // We can set gains and i_min/i_max, but output limiting would need to be done separately
  pid.set_gains(p, i, d, i_max, i_min);

  // The command offset and command limits need to be handled manually when computing commands
  // since control_toolbox::Pid doesn't have these built-in
}

double JointPosVelPidHelper::calculate_velocity_target_force(
  control_toolbox::Pid & pid,
  double current_velocity,
  double target_velocity,
  double max_velocity,
  const rclcpp::Duration & period)
{
  // Clamp the target velocity to limits
  double velocity_cmd_clamped = clamp(
    target_velocity,
    -1.0 * max_velocity,
    max_velocity);

  // Calculate velocity error (note sign flip from gz::math::PID to control_toolbox::Pid)
  // In control_toolbox, error = target - state
  double velocity_error = velocity_cmd_clamped - current_velocity;

  // Apply PID control to calculate target force
  double target_force = pid.compute_command(velocity_error, period);

  return target_force;
}

double JointPosVelPidHelper::calculate_position_target_force(
  control_toolbox::Pid & pos_pid,
  control_toolbox::Pid & vel_pid,
  double current_position,
  double target_position,
  double current_velocity,
  double lower_limit,
  double upper_limit,
  double max_velocity,
  bool use_cascade_control,
  const rclcpp::Duration & period)
{
  // Clamp the target position to joint limits
  double position_cmd_clamped = clamp(
    target_position,
    lower_limit,
    upper_limit);

  // Calculate position error (flipped sign from original implementation)
  // In control_toolbox, error = target - state
  double position_error = position_cmd_clamped - current_position;

  // Apply sign and limit to position error
  double position_error_sign = copysign(1.0, position_error);
  double position_error_abs_clamped = clamp(
    std::abs(position_error),
    0.0,
    std::abs(upper_limit - lower_limit));
  position_error = position_error_sign * position_error_abs_clamped;

  double target_force = 0.0;

  // Determine control approach based on cascade control flag
  if (use_cascade_control) {
    // Calculate target velocity from position error (cascade control)
    double target_vel = pos_pid.compute_command(position_error, period);

    // Clamp target velocity
    double target_vel_clamped = clamp(target_vel, -1.0 * max_velocity, max_velocity);

    // Calculate velocity error (for inner loop)
    double velocity_error = target_vel_clamped - current_velocity;

    // Use velocity error for the inner loop
    target_force = vel_pid.compute_command(velocity_error, period);
  } else {
    // Direct position error for single-loop control
    target_force = vel_pid.compute_command(position_error, period);
  }

  // Round for numerical stability
  return round(target_force * 10000.0) / 10000.0;
}

void JointPosVelPidHelper::configure_position_pid(
  const std::string & joint_name,
  const hardware_interface::ComponentInfo & joint_info,
  std::vector<rclcpp::Parameter> & parameters,
  control_toolbox::Pid & pid,
  double initial_p_pos,
  double max_velocity)
{
  // Get position PID parameters
  double p_gain_pos = get_param(joint_info, "p_pos", initial_p_pos);
  double i_gain_pos = get_param(joint_info, "i_pos", 0.0);
  double d_gain_pos = get_param(joint_info, "d_pos", initial_p_pos / 100.0);
  double i_pos_max = get_param(joint_info, "i_pos_max", 0.0);
  double i_pos_min = get_param(joint_info, "i_pos_min", 0.0);

  // These parameters are for command limiting, which control_toolbox::Pid doesn't directly support
  // We'll store them in parameters but handle limits separately
  double cmd_pos_max = get_param(joint_info, "cmd_pos_max", max_velocity);
  double cmd_pos_min = get_param(joint_info, "cmd_pos_min", -1.0 * max_velocity);
  double cmd_pos_forward_gain = get_param(joint_info, "cmd_pos_forward_gain", 0.0);

  // Add position parameters to parameter vector
  add_joint_gain_parameter(parameters, joint_name, "p_pos", p_gain_pos);
  add_joint_gain_parameter(parameters, joint_name, "i_pos", i_gain_pos);
  add_joint_gain_parameter(parameters, joint_name, "d_pos", d_gain_pos);
  add_joint_gain_parameter(parameters, joint_name, "i_pos_max", i_pos_max);
  add_joint_gain_parameter(parameters, joint_name, "i_pos_min", i_pos_min);
  add_joint_gain_parameter(parameters, joint_name, "cmd_pos_max", cmd_pos_max);
  add_joint_gain_parameter(parameters, joint_name, "cmd_pos_min", cmd_pos_min);
  add_joint_gain_parameter(parameters, joint_name, "cmd_pos_forward_gain", cmd_pos_forward_gain);

  // Initialize the PID controller
  pid.initialize(p_gain_pos, i_gain_pos, d_gain_pos, i_pos_max, i_pos_min);
}

void JointPosVelPidHelper::configure_velocity_pid(
  const std::string & joint_name,
  const hardware_interface::ComponentInfo & joint_info,
  std::vector<rclcpp::Parameter> & parameters,
  control_toolbox::Pid & pid,
  double initial_p_pos,
  double max_velocity,
  double max_effort)
{
  // Get velocity PID parameters
  double p_gain_vel = get_param(joint_info, "p_vel", initial_p_pos / 100.0);
  double i_gain_vel = get_param(joint_info, "i_vel", initial_p_pos / 1000.0);
  double d_gain_vel = get_param(joint_info, "d_vel", 0.0);
  double i_vel_max = get_param(joint_info, "i_vel_max", max_effort / 2.0);
  double i_vel_min = get_param(joint_info, "i_vel_min", -1.0 * max_effort / 2.0);

  // These parameters are for command limiting, which control_toolbox::Pid doesn't directly support
  // We'll store them in parameters but handle limits separately
  double cmd_vel_max = get_param(joint_info, "cmd_vel_max", max_velocity);
  double cmd_vel_min = get_param(joint_info, "cmd_vel_min", -1.0 * max_velocity);
  double cmd_vel_forward_gain = get_param(joint_info, "cmd_vel_forward_gain", 0.0);

  // Add velocity parameters to parameter vector
  add_joint_gain_parameter(parameters, joint_name, "p_vel", p_gain_vel);
  add_joint_gain_parameter(parameters, joint_name, "i_vel", i_gain_vel);
  add_joint_gain_parameter(parameters, joint_name, "d_vel", d_gain_vel);
  add_joint_gain_parameter(parameters, joint_name, "i_vel_max", i_vel_max);
  add_joint_gain_parameter(parameters, joint_name, "i_vel_min", i_vel_min);
  add_joint_gain_parameter(parameters, joint_name, "cmd_vel_max", cmd_vel_max);
  add_joint_gain_parameter(parameters, joint_name, "cmd_vel_min", cmd_vel_min);
  add_joint_gain_parameter(parameters, joint_name, "cmd_vel_forward_gain", cmd_vel_forward_gain);

  // Initialize the PID controller
  pid.initialize(p_gain_vel, i_gain_vel, d_gain_vel, i_vel_max, i_vel_min);
}

}  // namespace gz_ros2_control 