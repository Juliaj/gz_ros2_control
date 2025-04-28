// Copyright 2021 Open Source Robotics Foundation, Inc.
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

#include "gz_ros2_control/gz_system.hpp"

#include <gz/msgs/imu.pb.h>

#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gz/physics/Geometry.hh>
#include <gz/sim/components/AngularVelocity.hh>
#include <gz/sim/components/Imu.hh>
#include <gz/sim/components/JointAxis.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/sim/components/JointTransmittedWrench.hh>
#include <gz/sim/components/JointType.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/JointVelocity.hh>
#include <gz/sim/components/JointVelocityReset.hh>
#include <gz/sim/components/LinearAcceleration.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Sensor.hh>
#include <gz/transport/Node.hh>
#define GZ_TRANSPORT_NAMESPACE gz::transport::
#define GZ_MSGS_NAMESPACE gz::msgs::

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/lexical_casts.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>


// pid pos related
#include <gz/math/PID.hh>
#include <normApi.h>

struct jointData
{
  /// \brief Joint's names.
  std::string name;

  /// \brief Joint's type.
  sdf::JointType joint_type;

  /// \brief Joint's axis.
  sdf::JointAxis joint_axis;

  /// \brief Current joint position
  double joint_position;

  /// \brief Current joint velocity
  double joint_velocity;

  /// \brief Current joint effort
  double joint_effort;

  /// \brief Current cmd joint position
  double joint_position_cmd;

  /// \brief Current cmd joint velocity
  double joint_velocity_cmd;

  /// \brief Current cmd joint effort
  double joint_effort_cmd;

  /// \brief flag if joint is actuated (has command interfaces) or passive
  bool is_actuated;

  /// \brief handles to the joints from within Gazebo
  sim::Entity sim_joint;

  /// \brief Control method defined in the URDF for each joint.
  gz_ros2_control::GazeboSimSystemInterface::ControlMethod joint_control_method;

  /// \brief PID for position control
  gz::math::PID pid_pos;

  /// \brief PID for velocity control
  gz::math::PID pid_vel;
};

class ImuData
{
public:
  /// \brief imu's name.
  std::string name{};

  /// \brief imu's topic name.
  std::string topicName{};

  /// \brief handles to the imu from within Gazebo
  sim::Entity sim_imu_sensors_ = sim::kNullEntity;

  /// \brief An array per IMU with 4 orientation, 3 angular velocity and 3 linear acceleration
  std::array<double, 10> imu_sensor_data_;

  /// \brief callback to get the IMU topic values
  void OnIMU(const GZ_MSGS_NAMESPACE IMU & _msg);
};

void ImuData::OnIMU(const GZ_MSGS_NAMESPACE IMU & _msg)
{
  this->imu_sensor_data_[0] = _msg.orientation().x();
  this->imu_sensor_data_[1] = _msg.orientation().y();
  this->imu_sensor_data_[2] = _msg.orientation().z();
  this->imu_sensor_data_[3] = _msg.orientation().w();
  this->imu_sensor_data_[4] = _msg.angular_velocity().x();
  this->imu_sensor_data_[5] = _msg.angular_velocity().y();
  this->imu_sensor_data_[6] = _msg.angular_velocity().z();
  this->imu_sensor_data_[7] = _msg.linear_acceleration().x();
  this->imu_sensor_data_[8] = _msg.linear_acceleration().y();
  this->imu_sensor_data_[9] = _msg.linear_acceleration().z();
}

class gz_ros2_control::GazeboSimSystemPrivate
{
public:
  GazeboSimSystemPrivate() = default;

  ~GazeboSimSystemPrivate() = default;
  /// \brief Degrees od freedom.
  size_t n_dof_;

  /// \brief last time the write method was called.
  rclcpp::Time last_update_sim_time_ros_;

  /// \brief vector with the joint's names.
  std::vector<struct jointData> joints_;

  /// \brief vector with the imus .
  std::vector<std::shared_ptr<ImuData>> imus_;

  /// \brief state interfaces that will be exported to the Resource Manager
  std::vector<hardware_interface::StateInterface> state_interfaces_;

  /// \brief command interfaces that will be exported to the Resource Manager
  std::vector<hardware_interface::CommandInterface> command_interfaces_;

  /// \brief Entity component manager, ECM shouldn't be accessed outside those
  /// methods, otherwise the app will crash
  sim::EntityComponentManager * ecm;

  /// \brief controller update rate
  unsigned int update_rate;

  /// \brief Gazebo communication node.
  GZ_TRANSPORT_NAMESPACE Node node;

  /// \brief Gain which converts position error to a velocity command
  double position_proportional_gain_;

  // Should hold the joints if no control_mode is active
  bool hold_joints_ = true;
};

namespace gz_ros2_control
{

bool GazeboSimSystem::initSim(
  rclcpp::Node::SharedPtr & model_nh,
  std::map<std::string, sim::Entity> & enableJoints,
  const hardware_interface::HardwareInfo & hardware_info,
  sim::EntityComponentManager & _ecm,
  unsigned int update_rate)
{
  this->dataPtr = std::make_unique<GazeboSimSystemPrivate>();
  this->dataPtr->last_update_sim_time_ros_ = rclcpp::Time();

  // nh_ node handle for the ros2 control system
  this->nh_ = model_nh;
  this->dataPtr->ecm = &_ecm;
  this->dataPtr->n_dof_ = hardware_info.joints.size();

  this->dataPtr->update_rate = update_rate;

  try {
    this->dataPtr->hold_joints_ = this->nh_->get_parameter("hold_joints").as_bool();
  } catch (rclcpp::exceptions::ParameterUninitializedException & ex) {
    RCLCPP_ERROR(
      this->nh_->get_logger(),
      "Parameter 'hold_joints' not initialized, with error %s", ex.what());
    RCLCPP_WARN_STREAM(
      this->nh_->get_logger(), "Using default value: " << this->dataPtr->hold_joints_);
  } catch (rclcpp::exceptions::ParameterNotDeclaredException & ex) {
    RCLCPP_ERROR(
      this->nh_->get_logger(),
      "Parameter 'hold_joints' not declared, with error %s", ex.what());
    RCLCPP_WARN_STREAM(
      this->nh_->get_logger(), "Using default value: " << this->dataPtr->hold_joints_);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(
      this->nh_->get_logger(),
      "Parameter 'hold_joints' has wrong type: %s", ex.what());
    RCLCPP_WARN_STREAM(
      this->nh_->get_logger(), "Using default value: " << this->dataPtr->hold_joints_);
  }
  RCLCPP_DEBUG_STREAM(
    this->nh_->get_logger(), "hold_joints (system): " << this->dataPtr->hold_joints_ << std::endl);


  RCLCPP_DEBUG(this->nh_->get_logger(), "n_dof_ %lu", this->dataPtr->n_dof_);

  this->dataPtr->joints_.resize(this->dataPtr->n_dof_);

  try {
    this->dataPtr->position_proportional_gain_ =
      this->nh_->get_parameter("position_proportional_gain").as_double();
  } catch (rclcpp::exceptions::ParameterUninitializedException & ex) {
    RCLCPP_ERROR(
      this->nh_->get_logger(),
      "Parameter 'position_proportional_gain' not initialized, with error %s", ex.what());
    RCLCPP_WARN_STREAM(
      this->nh_->get_logger(),
      "Using default value: " << this->dataPtr->position_proportional_gain_);
  } catch (rclcpp::exceptions::ParameterNotDeclaredException & ex) {
    RCLCPP_ERROR(
      this->nh_->get_logger(),
      "Parameter 'position_proportional_gain' not declared, with error %s", ex.what());
    RCLCPP_WARN_STREAM(
      this->nh_->get_logger(),
      "Using default value: " << this->dataPtr->position_proportional_gain_);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(
      this->nh_->get_logger(),
      "Parameter 'position_proportional_gain' has wrong type: %s", ex.what());
    RCLCPP_WARN_STREAM(
      this->nh_->get_logger(),
      "Using default value: " << this->dataPtr->position_proportional_gain_);
  }

  RCLCPP_INFO_STREAM(
    this->nh_->get_logger(),
    "The position_proportional_gain has been set to: " <<
      this->dataPtr->position_proportional_gain_);

  if (this->dataPtr->n_dof_ == 0) {
    RCLCPP_ERROR_STREAM(this->nh_->get_logger(), "There is no joint available");
    return false;
  }

  //TODO(juliajia): check whether this is correct
  std::vector<std::string> joint_names;
  this->param_node_ =
    rclcpp::Node::make_shared(
    hardware_info.name,
    rclcpp::NodeOptions().allow_undeclared_parameters(true));
  std::vector<rclcpp::Parameter> param_vec;

  for (unsigned int j = 0; j < this->dataPtr->n_dof_; j++) {
    auto & joint_info = hardware_info.joints[j];
    std::string joint_name = this->dataPtr->joints_[j].name = joint_info.name;

    auto it_joint = enableJoints.find(joint_name);
    if (it_joint == enableJoints.end()) {
      RCLCPP_WARN_STREAM(
        this->nh_->get_logger(), "Skipping joint in the URDF named '" << joint_name <<
          "' which is not in the gazebo model.");
      continue;
    }

    sim::Entity simjoint = enableJoints[joint_name];
    this->dataPtr->joints_[j].sim_joint = simjoint;
    this->dataPtr->joints_[j].joint_type = _ecm.Component<sim::components::JointType>(
      simjoint)->Data();
    this->dataPtr->joints_[j].joint_axis = _ecm.Component<sim::components::JointAxis>(
      simjoint)->Data();

    // Create joint position component if one doesn't exist
    if (!_ecm.EntityHasComponentType(
        simjoint,
        sim::components::JointPosition().TypeId()))
    {
      _ecm.CreateComponent(simjoint, sim::components::JointPosition());
    }

    // Create joint velocity component if one doesn't exist
    if (!_ecm.EntityHasComponentType(
        simjoint,
        sim::components::JointVelocity().TypeId()))
    {
      _ecm.CreateComponent(simjoint, sim::components::JointVelocity());
    }

    // Create joint transmitted wrench component if one doesn't exist
    if (!_ecm.EntityHasComponentType(
        simjoint,
        sim::components::JointTransmittedWrench().TypeId()))
    {
      _ecm.CreateComponent(simjoint, sim::components::JointTransmittedWrench());
    }

    const auto * jointAxis =
      this->dataPtr->ecm->Component<gz::sim::components::JointAxis>(
      this->dataPtr->joints_[
        j].sim_joint);

    bool use_cascade_control =
      (hardware_info.joints[j].parameters.find("use_cascade_control") ==
      hardware_info.joints[j].parameters.end()) ?
      false :
      [&]() {
      if (hardware_info.joints[j].parameters.at("use_cascade_control") == "true" ||
        hardware_info.joints[j].parameters.at("use_cascade_control") == "True")
      {
        return true;
      } else {
        return false;
      }
    } ();

    param_vec.push_back(
      rclcpp::Parameter{"mode." + joint_name + ".use_cascade_control",
        use_cascade_control});

    double upper = jointAxis->Data().Upper();
    double lower = jointAxis->Data().Lower();
    double max_velocity = jointAxis->Data().MaxVelocity();
    double max_effort = jointAxis->Data().Effort();

    double dummy_guess_p_pos = 10 * max_velocity / abs(upper - lower);

    // PID parameters
    double p_gain_pos =
      (hardware_info.joints[j].parameters.find(
        "p_pos") == hardware_info.joints[j].parameters.end()) ?
      dummy_guess_p_pos :
      stod(hardware_info.joints[j].parameters.at("p_pos"));
    double i_gain_pos =
      (hardware_info.joints[j].parameters.find(
        "i_pos") == hardware_info.joints[j].parameters.end()) ?
      0.0 :
      stod(hardware_info.joints[j].parameters.at("i_pos"));
    double d_gain_pos =
      (hardware_info.joints[j].parameters.find(
        "d_pos") == hardware_info.joints[j].parameters.end()) ?
      dummy_guess_p_pos / 100.0 :
      stod(hardware_info.joints[j].parameters.at("d_pos"));
    // set integral max and min component to 50 percent of the max effort
    double i_pos_max =
      (hardware_info.joints[j].parameters.find("i_pos_max") ==
      hardware_info.joints[j].parameters.end()) ?
      0.0 :
      stod(hardware_info.joints[j].parameters.at("i_pos_max"));
    double i_pos_min =
      (hardware_info.joints[j].parameters.find("i_pos_min") ==
      hardware_info.joints[j].parameters.end()) ?
      0.0 :
      stod(hardware_info.joints[j].parameters.at("i_pos_min"));
    double cmd_pos_max =
      (hardware_info.joints[j].parameters.find("cmd_pos_max") ==
      hardware_info.joints[j].parameters.end()) ?
      max_velocity :
      stod(hardware_info.joints[j].parameters.at("cmd_pos_max"));
    double cmd_pos_min =
      (hardware_info.joints[j].parameters.find("cmd_pos_min") ==
      hardware_info.joints[j].parameters.end()) ?
      -1.0 * max_velocity :
      stod(hardware_info.joints[j].parameters.at("cmd_pos_min"));
    double cmd_pos_forward_gain =
      (hardware_info.joints[j].parameters.find("cmd_pos_forward_gain") ==
      hardware_info.joints[j].parameters.end()) ?
      0.0 :
      stod(hardware_info.joints[j].parameters.at("cmd_pos_forward_gain"));

    param_vec.push_back(rclcpp::Parameter{"gains." + joint_name + ".p_pos", p_gain_pos});
    param_vec.push_back(rclcpp::Parameter{"gains." + joint_name + ".i_pos", i_gain_pos});
    param_vec.push_back(rclcpp::Parameter{"gains." + joint_name + ".d_pos", d_gain_pos});
    param_vec.push_back(rclcpp::Parameter{"gains." + joint_name + ".i_pos_max", i_pos_max});
    param_vec.push_back(rclcpp::Parameter{"gains." + joint_name + ".i_pos_min", i_pos_min});
    param_vec.push_back(rclcpp::Parameter{"gains." + joint_name + ".cmd_pos_max", cmd_pos_max});
    param_vec.push_back(rclcpp::Parameter{"gains." + joint_name + ".cmd_pos_min", cmd_pos_min});
    param_vec.push_back(rclcpp::Parameter{"gains." + joint_name + ".cmd_pos_forward_gain", cmd_pos_forward_gain});

    this->dataPtr->joints_[j].pid_pos.Init(
      p_gain_pos, i_gain_pos, d_gain_pos, i_pos_max, i_pos_min, cmd_pos_max,
      cmd_pos_min, cmd_pos_forward_gain);

    double p_gain_vel =
      (hardware_info.joints[j].parameters.find(
        "p_vel") == hardware_info.joints[j].parameters.end()) ?
      dummy_guess_p_pos / 100.0 :
      stod(hardware_info.joints[j].parameters.at("p_vel"));
    double i_gain_vel =
      (hardware_info.joints[j].parameters.find(
        "i_vel") == hardware_info.joints[j].parameters.end()) ?
      dummy_guess_p_pos / 1000.0 :
      stod(hardware_info.joints[j].parameters.at("i_vel"));
    double d_gain_vel =
      (hardware_info.joints[j].parameters.find(
        "d_vel") == hardware_info.joints[j].parameters.end()) ?
      0.0 :
      stod(hardware_info.joints[j].parameters.at("d_vel"));
    // set integral max and min component to 50 percent of the max effort
    double i_vel_max =
      (hardware_info.joints[j].parameters.find("i_vel_max") ==
      hardware_info.joints[j].parameters.end()) ?
      max_effort / 2.0 :
      stod(hardware_info.joints[j].parameters.at("i_vel_max"));
    double i_vel_min =
      (hardware_info.joints[j].parameters.find("i_vel_min") ==
      hardware_info.joints[j].parameters.end()) ?
      -1.0 * max_effort / 2.0 :
      stod(hardware_info.joints[j].parameters.at("i_vel_min"));
    double cmd_vel_max =
      (hardware_info.joints[j].parameters.find("cmd_vel_max") ==
      hardware_info.joints[j].parameters.end()) ?
      max_velocity :
      stod(hardware_info.joints[j].parameters.at("cmd_vel_max"));
    double cmd_vel_min =
      (hardware_info.joints[j].parameters.find("cmd_vel_min") ==
      hardware_info.joints[j].parameters.end()) ?
      -1.0 * max_velocity :
      stod(hardware_info.joints[j].parameters.at("cmd_vel_min"));
    double cmd_vel_forward_gain =
      (hardware_info.joints[j].parameters.find("cmd_vel_forward_gain") ==
      hardware_info.joints[j].parameters.end()) ?
      0.0 :
      stod(hardware_info.joints[j].parameters.at("cmd_vel_forward_gain"));

    param_vec.push_back(rclcpp::Parameter{"gains." + joint_name + ".p_vel", p_gain_vel});
    param_vec.push_back(rclcpp::Parameter{"gains." + joint_name + ".i_vel", i_gain_vel});
    param_vec.push_back(rclcpp::Parameter{"gains." + joint_name + ".d_vel", d_gain_vel});
    param_vec.push_back(rclcpp::Parameter{"gains." + joint_name + ".i_vel_max", i_vel_max});
    param_vec.push_back(rclcpp::Parameter{"gains." + joint_name + ".i_vel_min", i_vel_min});
    param_vec.push_back(rclcpp::Parameter{"gains." + joint_name + ".cmd_vel_max", cmd_vel_max});
    param_vec.push_back(rclcpp::Parameter{"gains." + joint_name + ".cmd_vel_min", cmd_vel_min});
    param_vec.push_back(rclcpp::Parameter{"gains." + joint_name + ".cmd_vel_forward_gain", cmd_vel_forward_gain});

    this->dataPtr->joints_[j].pid_vel.Init(
      p_gain_vel, i_gain_vel, d_gain_vel, i_vel_max, i_vel_min, cmd_vel_max,
      cmd_vel_min, cmd_vel_forward_gain);

    // Accept this joint and continue configuration
    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Loading joint: " << joint_name);

    // check if joint is mimicked
    auto it = std::find_if(
      hardware_info.mimic_joints.begin(),
      hardware_info.mimic_joints.end(),
      [j](const hardware_interface::MimicJoint & mj) {
        return mj.joint_index == j;
      });

    if (it != hardware_info.mimic_joints.end()) {
      RCLCPP_INFO_STREAM(
        this->nh_->get_logger(),
        "Joint '" << joint_name << "'is mimicking joint '" <<
          this->dataPtr->joints_[it->mimicked_joint_index].name <<
          "' with multiplier: " << it->multiplier << " and offset: " << it->offset);
    }

    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\tState:");

    auto get_initial_value =
      [this, joint_name](const hardware_interface::InterfaceInfo & interface_info) {
        double initial_value{0.0};
        if (!interface_info.initial_value.empty()) {
          try {
            initial_value = hardware_interface::stod(interface_info.initial_value);
            RCLCPP_INFO(this->nh_->get_logger(), "\t\t\t found initial value: %f", initial_value);
          } catch (std::invalid_argument &) {
            RCLCPP_ERROR_STREAM(
              this->nh_->get_logger(),
              "Failed converting initial_value string to real number for the joint "
                << joint_name
                << " and state interface " << interface_info.name
                << ". Actual value of parameter: " << interface_info.initial_value
                << ". Initial value will be set to 0.0");
            throw std::invalid_argument("Failed converting initial_value string");
          }
        }
        return initial_value;
      };

    double initial_position = std::numeric_limits<double>::quiet_NaN();
    double initial_velocity = std::numeric_limits<double>::quiet_NaN();
    double initial_effort = std::numeric_limits<double>::quiet_NaN();

    // register the state handles
    for (unsigned int i = 0; i < joint_info.state_interfaces.size(); ++i) {
      if (joint_info.state_interfaces[i].name == "position") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t position");
        this->dataPtr->state_interfaces_.emplace_back(
          joint_name,
          hardware_interface::HW_IF_POSITION,
          &this->dataPtr->joints_[j].joint_position);
        initial_position = get_initial_value(joint_info.state_interfaces[i]);
        this->dataPtr->joints_[j].joint_position = initial_position;
      }
      if (joint_info.state_interfaces[i].name == "velocity") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t velocity");
        this->dataPtr->state_interfaces_.emplace_back(
          joint_name,
          hardware_interface::HW_IF_VELOCITY,
          &this->dataPtr->joints_[j].joint_velocity);
        initial_velocity = get_initial_value(joint_info.state_interfaces[i]);
        this->dataPtr->joints_[j].joint_velocity = initial_velocity;
      }
      if (joint_info.state_interfaces[i].name == "effort") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t effort");
        this->dataPtr->state_interfaces_.emplace_back(
          joint_name,
          hardware_interface::HW_IF_EFFORT,
          &this->dataPtr->joints_[j].joint_effort);
        initial_effort = get_initial_value(joint_info.state_interfaces[i]);
        this->dataPtr->joints_[j].joint_effort = initial_effort;
      }
    }

    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\tCommand:");

    // register the command handles
    for (unsigned int i = 0; i < joint_info.command_interfaces.size(); ++i) {
      if (joint_info.command_interfaces[i].name == "position") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t position");
        this->dataPtr->command_interfaces_.emplace_back(
          joint_name,
          hardware_interface::HW_IF_POSITION,
          &this->dataPtr->joints_[j].joint_position_cmd);
        if (!std::isnan(initial_position)) {
          this->dataPtr->joints_[j].joint_position_cmd = initial_position;
        }
      } else if (joint_info.command_interfaces[i].name == "velocity") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t velocity");
        this->dataPtr->command_interfaces_.emplace_back(
          joint_name,
          hardware_interface::HW_IF_VELOCITY,
          &this->dataPtr->joints_[j].joint_velocity_cmd);
        if (!std::isnan(initial_velocity)) {
          this->dataPtr->joints_[j].joint_velocity_cmd = initial_velocity;
        }
      } else if (joint_info.command_interfaces[i].name == "effort") {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t effort");
        this->dataPtr->command_interfaces_.emplace_back(
          joint_name,
          hardware_interface::HW_IF_EFFORT,
          &this->dataPtr->joints_[j].joint_effort_cmd);
        if (!std::isnan(initial_effort)) {
          this->dataPtr->joints_[j].joint_effort_cmd = initial_effort;
        }
      }
      // independently of existence of command interface set initial value if defined
      if (!std::isnan(initial_position)) {
        this->dataPtr->joints_[j].joint_position = initial_position;
        this->dataPtr->ecm->CreateComponent(
          this->dataPtr->joints_[j].sim_joint,
          sim::components::JointPositionReset({initial_position}));
      }
      if (!std::isnan(initial_velocity)) {
        this->dataPtr->joints_[j].joint_velocity = initial_velocity;
        this->dataPtr->ecm->CreateComponent(
          this->dataPtr->joints_[j].sim_joint,
          sim::components::JointVelocityReset({initial_velocity}));
      }
    }

    // check if joint is actuated (has command interfaces) or passive
    this->dataPtr->joints_[j].is_actuated = (joint_info.command_interfaces.size() > 0);
    RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Joint " << joint_name << " is actuated: " << this->dataPtr->joints_[j].is_actuated);
  }

  // register the joint names parameter
  rclcpp::Parameter joint_names_parameter("joints", joint_names);
  if (!this->param_node_->has_parameter("joints")) {
    this->param_node_->set_parameter(joint_names_parameter);
  }
  for (const auto & p : param_vec) {
    if (!this->param_node_->has_parameter(p.get_name())) {
      this->param_node_->set_parameter(p);
    }
  }

  spin_thread_ = std::thread(
    [this]() {
      exec_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
      exec_->add_node(this->param_node_);

      while (rclcpp::ok() && !stop_spin_) {
        exec_->spin_once();
      }
      exec_->remove_node(this->param_node_);
      exec_.reset();
    });

  try {
    // Create the parameter listener and get the parameters
    param_listener_ = std::make_shared<ParamListener>(this->param_node_);
    params_ = param_listener_->get_params();
  } catch (const std::exception & e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return false;
  }

  // update the params
  param_vec.push_back(joint_names_parameter);
  param_listener_->update(param_vec);

  registerSensors(hardware_info);

  return true;
}

void GazeboSimSystem::registerSensors(
  const hardware_interface::HardwareInfo & hardware_info)
{
  // Collect gazebo sensor handles
  size_t n_sensors = hardware_info.sensors.size();
  std::vector<hardware_interface::ComponentInfo> sensor_components_;

  for (unsigned int j = 0; j < n_sensors; j++) {
    hardware_interface::ComponentInfo component = hardware_info.sensors[j];
    sensor_components_.push_back(component);
  }
  // This is split in two steps: Count the number and type of sensor and associate the interfaces
  // So we have resize only once the structures where the data will be stored, and we can safely
  // use pointers to the structures

  this->dataPtr->ecm->Each<sim::components::Imu,
    sim::components::Name>(
    [&](const sim::Entity & _entity,
    const sim::components::Imu *,
    const sim::components::Name * _name) -> bool
    {
      auto imuData = std::make_shared<ImuData>();
      RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Loading sensor: " << _name->Data());

      auto sensorTopicComp = this->dataPtr->ecm->Component<
        sim::components::SensorTopic>(_entity);
      if (sensorTopicComp) {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "Topic name: " << sensorTopicComp->Data());
      }

      RCLCPP_INFO_STREAM(
        this->nh_->get_logger(), "\tState:");
      imuData->name = _name->Data();
      imuData->sim_imu_sensors_ = _entity;

      hardware_interface::ComponentInfo component;
      for (auto & comp : sensor_components_) {
        if (comp.name == _name->Data()) {
          component = comp;
        }
      }

      static const std::map<std::string, size_t> interface_name_map = {
        {"orientation.x", 0},
        {"orientation.y", 1},
        {"orientation.z", 2},
        {"orientation.w", 3},
        {"angular_velocity.x", 4},
        {"angular_velocity.y", 5},
        {"angular_velocity.z", 6},
        {"linear_acceleration.x", 7},
        {"linear_acceleration.y", 8},
        {"linear_acceleration.z", 9},
      };

      for (const auto & state_interface : component.state_interfaces) {
        RCLCPP_INFO_STREAM(this->nh_->get_logger(), "\t\t " << state_interface.name);

        size_t data_index = interface_name_map.at(state_interface.name);
        this->dataPtr->state_interfaces_.emplace_back(
          imuData->name,
          state_interface.name,
          &imuData->imu_sensor_data_[data_index]);
      }
      this->dataPtr->imus_.push_back(imuData);
      return true;
    });
}

CallbackReturn
GazeboSimSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
  if (info.hardware_plugin_name.compare("gz_ros2_control/GazeboSimSystem") != 0) {
    RCLCPP_WARN(
      this->nh_->get_logger(),
      "The ign_ros2_control plugin got renamed to gz_ros2_control.\n"
      "Update the <ros2_control> tag and gazebo plugin to\n"
      "<hardware>\n"
      "  <plugin>gz_ros2_control/GazeboSimSystem</plugin>\n"
      "</hardware>\n"
      "<gazebo>\n"
      "  <plugin filename=\"gz_ros2_control-system\""
      "name=\"gz_ros2_control::GazeboSimROS2ControlPlugin\">\n"
      "    ...\n"
      "  </plugin>\n"
      "</gazebo>"
    );
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn GazeboSimSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(
    this->nh_->get_logger(), "System Successfully configured!");

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
GazeboSimSystem::export_state_interfaces()
{
  return std::move(this->dataPtr->state_interfaces_);
}

std::vector<hardware_interface::CommandInterface>
GazeboSimSystem::export_command_interfaces()
{
  return std::move(this->dataPtr->command_interfaces_);
}

CallbackReturn GazeboSimSystem::on_activate(const rclcpp_lifecycle::State & previous_state)
{
  return CallbackReturn::SUCCESS;
  return hardware_interface::SystemInterface::on_activate(previous_state);
}

CallbackReturn GazeboSimSystem::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
  //TODO(juliajia): check whether this is correct
  stop_spin_ = true;
  spin_thread_.join();
  return CallbackReturn::SUCCESS;
  return hardware_interface::SystemInterface::on_deactivate(previous_state);
}

hardware_interface::return_type GazeboSimSystem::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  for (unsigned int i = 0; i < this->dataPtr->joints_.size(); ++i) {
    if (this->dataPtr->joints_[i].sim_joint == sim::kNullEntity) {
      continue;
    }

    // Get the joint velocity
    const auto * jointVelocity =
      this->dataPtr->ecm->Component<sim::components::JointVelocity>(
      this->dataPtr->joints_[i].sim_joint);

    // Get the joint force via joint transmitted wrench
    const auto * jointWrench =
      this->dataPtr->ecm->Component<sim::components::JointTransmittedWrench>(
      this->dataPtr->joints_[i].sim_joint);

    // Get the joint position
    const auto * jointPositions =
      this->dataPtr->ecm->Component<sim::components::JointPosition>(
      this->dataPtr->joints_[i].sim_joint);

    this->dataPtr->joints_[i].joint_position = jointPositions->Data()[0];
    this->dataPtr->joints_[i].joint_velocity = jointVelocity->Data()[0];
    gz::physics::Vector3d force_or_torque;
    if (this->dataPtr->joints_[i].joint_type == sdf::JointType::PRISMATIC) {
      force_or_torque = {jointWrench->Data().force().x(),
        jointWrench->Data().force().y(), jointWrench->Data().force().z()};
    } else {  // REVOLUTE and CONTINUOUS
      force_or_torque = {jointWrench->Data().torque().x(),
        jointWrench->Data().torque().y(), jointWrench->Data().torque().z()};
    }
    // Calculate the scalar effort along the joint axis
    this->dataPtr->joints_[i].joint_effort = force_or_torque.dot(
      gz::physics::Vector3d{this->dataPtr->joints_[i].joint_axis.Xyz()[0],
        this->dataPtr->joints_[i].joint_axis.Xyz()[1],
        this->dataPtr->joints_[i].joint_axis.Xyz()[2]});
  }

  for (unsigned int i = 0; i < this->dataPtr->imus_.size(); ++i) {
    if (this->dataPtr->imus_[i]->topicName.empty()) {
      auto sensorTopicComp = this->dataPtr->ecm->Component<
        sim::components::SensorTopic>(this->dataPtr->imus_[i]->sim_imu_sensors_);
      if (sensorTopicComp) {
        this->dataPtr->imus_[i]->topicName = sensorTopicComp->Data();
        RCLCPP_INFO_STREAM(
          this->nh_->get_logger(), "IMU " << this->dataPtr->imus_[i]->name <<
            " has a topic name: " << sensorTopicComp->Data());

        this->dataPtr->node.Subscribe(
          this->dataPtr->imus_[i]->topicName, &ImuData::OnIMU,
          this->dataPtr->imus_[i].get());
      }
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type
GazeboSimSystem::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  for (unsigned int j = 0; j < this->dataPtr->joints_.size(); j++) {
    // Clear joint control method bits corresponding to stop interfaces
    for (const std::string & interface_name : stop_interfaces) {
      if (interface_name == (this->dataPtr->joints_[j].name + "/" +
        hardware_interface::HW_IF_POSITION))
      {
        this->dataPtr->joints_[j].joint_control_method &=
          static_cast<ControlMethod_>(VELOCITY & EFFORT);
      } else if (interface_name == (this->dataPtr->joints_[j].name + "/" + // NOLINT
        hardware_interface::HW_IF_VELOCITY))
      {
        this->dataPtr->joints_[j].joint_control_method &=
          static_cast<ControlMethod_>(POSITION & EFFORT);
      } else if (interface_name == (this->dataPtr->joints_[j].name + "/" + // NOLINT
        hardware_interface::HW_IF_EFFORT))
      {
        this->dataPtr->joints_[j].joint_control_method &=
          static_cast<ControlMethod_>(POSITION & VELOCITY);
      }
    }

    // Set joint control method bits corresponding to start interfaces
    for (const std::string & interface_name : start_interfaces) {
      if (interface_name == (this->dataPtr->joints_[j].name + "/" +
        hardware_interface::HW_IF_POSITION))
      {
        this->dataPtr->joints_[j].joint_control_method |= POSITION;
      } else if (interface_name == (this->dataPtr->joints_[j].name + "/" + // NOLINT
        hardware_interface::HW_IF_VELOCITY))
      {
        this->dataPtr->joints_[j].joint_control_method |= VELOCITY;
      } else if (interface_name == (this->dataPtr->joints_[j].name + "/" + // NOLINT
        hardware_interface::HW_IF_EFFORT))
      {
        this->dataPtr->joints_[j].joint_control_method |= EFFORT;
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type GazeboSimSystem::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & period)
{

  // refresh params
  param_listener_->refresh_dynamic_parameters();
  params_ = param_listener_->get_params();

  for (unsigned int i = 0; i < this->dataPtr->joints_.size(); ++i) {
    if (this->dataPtr->joints_[i].sim_joint == sim::kNullEntity) {
      continue;
    }

    // assuming every joint has axis
    const auto * jointAxis =
      this->dataPtr->ecm->Component<gz::sim::components::JointAxis>(
      this->dataPtr->joints_[
        i].sim_joint);

    // update PIDs
    this->dataPtr->joints_[i].pid_pos.SetPGain(
      params_.gains.joints_map[this->dataPtr->joints_[i].
      name].p_pos);
    this->dataPtr->joints_[i].pid_pos.SetIGain(
      params_.gains.joints_map[this->dataPtr->joints_[i].
      name].i_pos);
    this->dataPtr->joints_[i].pid_pos.SetDGain(
      params_.gains.joints_map[this->dataPtr->joints_[i].
      name].d_pos);
    this->dataPtr->joints_[i].pid_pos.SetIMax(
      params_.gains.joints_map[this->dataPtr->joints_[i].
      name].i_pos_max);
    this->dataPtr->joints_[i].pid_pos.SetIMin(
      params_.gains.joints_map[this->dataPtr->joints_[i].
      name].i_pos_min);
    this->dataPtr->joints_[i].pid_pos.SetCmdMax(
      params_.gains.joints_map[this->dataPtr->joints_[i].
      name].cmd_pos_max);
    this->dataPtr->joints_[i].pid_pos.SetCmdMin(
      params_.gains.joints_map[this->dataPtr->joints_[i].
      name].cmd_pos_min);
    this->dataPtr->joints_[i].pid_pos.SetCmdOffset(
      params_.gains.joints_map[this->dataPtr->joints_[i].name].cmd_pos_forward_gain);

    this->dataPtr->joints_[i].pid_vel.SetPGain(
      params_.gains.joints_map[this->dataPtr->joints_[i].
      name].p_vel);
    this->dataPtr->joints_[i].pid_vel.SetIGain(
      params_.gains.joints_map[this->dataPtr->joints_[i].
      name].i_vel);
    this->dataPtr->joints_[i].pid_vel.SetDGain(
      params_.gains.joints_map[this->dataPtr->joints_[i].
      name].d_vel);
    this->dataPtr->joints_[i].pid_vel.SetIMax(
      params_.gains.joints_map[this->dataPtr->joints_[i].
      name].i_vel_max);
    this->dataPtr->joints_[i].pid_vel.SetIMin(
      params_.gains.joints_map[this->dataPtr->joints_[i].
      name].i_vel_min);
    this->dataPtr->joints_[i].pid_vel.SetCmdMax(
      params_.gains.joints_map[this->dataPtr->joints_[i].
      name].cmd_vel_max);
    this->dataPtr->joints_[i].pid_vel.SetCmdMin(
      params_.gains.joints_map[this->dataPtr->joints_[i].
      name].cmd_vel_min);
    this->dataPtr->joints_[i].pid_vel.SetCmdOffset(
      params_.gains.joints_map[this->dataPtr->joints_[i].name].cmd_vel_forward_gain);

    if (this->dataPtr->joints_[i].joint_control_method & VELOCITY) {

      double velocity = this->dataPtr->joints_[i].joint_velocity;
      double velocity_cmd_clamped = std::clamp(
        this->dataPtr->joints_[i].joint_velocity_cmd,
        -1.0 * jointAxis->Data().MaxVelocity(), jointAxis->Data().MaxVelocity());

      double velocity_error = velocity - velocity_cmd_clamped;

      // calculate target force/torque - output of inner pid
      double target_force = this->dataPtr->joints_[i].pid_vel.Update(
        velocity_error,
        std::chrono::duration<double>(period.to_chrono<std::chrono::nanoseconds>()));

      // remember for potential effort state interface
      this->dataPtr->joints_[i].joint_effort_cmd = target_force;

      auto forceCmd = this->dataPtr->ecm->Component<sim::components::JointForceCmd>(
        this->dataPtr->joints_[i].sim_joint);

      if (forceCmd == nullptr) {
        this->dataPtr->ecm->CreateComponent(
          this->dataPtr->joints_[i].sim_joint,
          sim::components::JointForceCmd({target_force}));
      } else {
        *forceCmd = sim::components::JointForceCmd({target_force});
      }
    } else if (this->dataPtr->joints_[i].joint_control_method & POSITION) {
      // calculate error with clamped position command
      double position = this->dataPtr->joints_[i].joint_position;
      double position_cmd_clamped = std::clamp(
        this->dataPtr->joints_[i].joint_position_cmd, jointAxis->Data().Lower(),
        jointAxis->Data().Upper());

      double position_error = position - position_cmd_clamped;

      double position_error_sign = copysign(1.0, position_error);

      double position_error_abs_clamped =
        std::clamp(
        std::abs(position_error), 0.0,
        std::abs(jointAxis->Data().Upper() - jointAxis->Data().Lower()));

      // move forward with calculated position error
      position_error = position_error_sign * position_error_abs_clamped;

      double position_or_velocity_error = 0.0;

      // check if cascade control is used for this joint
      if (params_.mode.joints_map[this->dataPtr->joints_[i].name].use_cascade_control) {
        // calculate target velocity - output of outer pid - input to inner pid
        double target_vel = this->dataPtr->joints_[i].pid_pos.Update(
          position_error, std::chrono::duration<double>(
            period.to_chrono<std::chrono::nanoseconds>()));

        double velocity_error =
          this->dataPtr->joints_[i].joint_velocity -
          std::clamp(
          target_vel, -1.0 * jointAxis->Data().MaxVelocity(),
          jointAxis->Data().MaxVelocity());

        // prepare velocity error value for inner pid
        position_or_velocity_error = velocity_error;
      } else {
        // prepare velocity error value for inner pid
        position_or_velocity_error = position_error;
      }

      // calculate target force/torque - output of inner pid
      double target_force = this->dataPtr->joints_[i].pid_vel.Update(
        position_or_velocity_error,
        std::chrono::duration<double>(period.to_chrono<std::chrono::nanoseconds>()));

      // round the force
      target_force = round(target_force * 10000.0) / 10000.0;

      // remember for potential effort state interface
      this->dataPtr->joints_[i].joint_effort_cmd = target_force;

      auto forceCmd = this->dataPtr->ecm->Component<gz::sim::components::JointForceCmd>(
        this->dataPtr->joints_[i].sim_joint);

      if (forceCmd == nullptr) {
        this->dataPtr->ecm->CreateComponent(
          this->dataPtr->joints_[i].sim_joint,
          gz::sim::components::JointForceCmd({target_force}));
      } else {
        *forceCmd = gz::sim::components::JointForceCmd({target_force});
      }
    } else if (this->dataPtr->joints_[i].joint_control_method & EFFORT) {
      if (!this->dataPtr->ecm->Component<sim::components::JointForceCmd>(
          this->dataPtr->joints_[i].sim_joint))
      {
        this->dataPtr->ecm->CreateComponent(
          this->dataPtr->joints_[i].sim_joint,
          sim::components::JointForceCmd({0}));
      } else {
        const auto jointEffortCmd =
          this->dataPtr->ecm->Component<sim::components::JointForceCmd>(
          this->dataPtr->joints_[i].sim_joint);
        *jointEffortCmd = sim::components::JointForceCmd(
          {this->dataPtr->joints_[i].joint_effort_cmd});
      }
    } else if (this->dataPtr->joints_[i].is_actuated && this->dataPtr->hold_joints_) {
      // Fallback case is a velocity command of zero (only for actuated joints)
      double target_vel = 0.0;
      auto vel =
        this->dataPtr->ecm->Component<sim::components::JointVelocityCmd>(
        this->dataPtr->joints_[i].sim_joint);

      if (vel == nullptr) {
        this->dataPtr->ecm->CreateComponent(
          this->dataPtr->joints_[i].sim_joint,
          sim::components::JointVelocityCmd({target_vel}));
      } else if (!vel->Data().empty()) {
        vel->Data()[0] = target_vel;
      } else if (!vel->Data().empty()) {
        vel->Data()[0] = target_vel;
      }
    }
  }

  // set values of all mimic joints with respect to mimicked joint
  for (const auto & mimic_joint : this->info_.mimic_joints) {
    // Get the joint position
    double position_mimicked_joint =
      this->dataPtr->ecm->Component<sim::components::JointPosition>(
      this->dataPtr->joints_[mimic_joint.mimicked_joint_index].sim_joint)->Data()[0];

    double position_mimic_joint =
      this->dataPtr->ecm->Component<sim::components::JointPosition>(
      this->dataPtr->joints_[mimic_joint.joint_index].sim_joint)->Data()[0];

    double position_error =
      position_mimic_joint - position_mimicked_joint * mimic_joint.multiplier;

    double velocity_sp = (-1.0) * position_error * this->dataPtr->update_rate;

    auto vel =
      this->dataPtr->ecm->Component<sim::components::JointVelocityCmd>(
      this->dataPtr->joints_[mimic_joint.joint_index].sim_joint);

    if (vel == nullptr) {
      this->dataPtr->ecm->CreateComponent(
        this->dataPtr->joints_[mimic_joint.joint_index].sim_joint,
        sim::components::JointVelocityCmd({velocity_sp}));
    } else if (!vel->Data().empty()) {
      vel->Data()[0] = velocity_sp;
    }
  }

  return hardware_interface::return_type::OK;
}
}  // namespace gz_ros2_control

#include "pluginlib/class_list_macros.hpp"  // NOLINT
PLUGINLIB_EXPORT_CLASS(
  gz_ros2_control::GazeboSimSystem, gz_ros2_control::GazeboSimSystemInterface)
// for backward compatibility with Ignition Gazebo
PLUGINLIB_EXPORT_CLASS(
  ign_ros2_control::IgnitionSystem, gz_ros2_control::GazeboSimSystemInterface)
