// Copyright 2025 Justo Dario Valverde <justodariovalverde@gmail.com>
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

#include "coco_hardware/pca9685_system.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"



namespace coco_hardware
{

hardware_interface::CallbackReturn Pca9685SystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  try {
    pca = std::make_unique<PiPCA9685::PCA9685>("/dev/i2c-1", 0x40);  // Default I2C bus and PCA9685 address
    pca->set_pwm_freq(50.0);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(
      rclcpp::get_logger("Pca9685SystemHardware"),
      "Failed to initialize PCA9685: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }


  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_commands_.resize(info_.joints.size(), std::numeric_limits<float>::quiet_NaN());
  hw_states_.resize(info_.joints.size(), std::numeric_limits<float>::quiet_NaN());

  RCLCPP_INFO(
    rclcpp::get_logger("Pca9685SystemHardware"),
    "Initializing hardware with %zu joints", info_.joints.size());
  
  for (const auto& joint : info_.joints) {
    RCLCPP_INFO(
      rclcpp::get_logger("Pca9685SystemHardware"),
      "Joint: %s", joint.name.c_str());
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // Pca9685System has one command interface on each output
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Pca9685SystemHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("Pca9685SystemHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface> Pca9685SystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> Pca9685SystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  RCLCPP_INFO(
    rclcpp::get_logger("Pca9685SystemHardware"),
    "Exported %zu command interfaces", command_interfaces.size());
  
  for (const auto& interface : command_interfaces) {
    RCLCPP_INFO(
      rclcpp::get_logger("Pca9685SystemHardware"),
      "Command interface: %s", interface.get_name().c_str());
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn Pca9685SystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto i = 0u; i < hw_commands_.size(); i++)
  {
    if (std::isnan(hw_commands_[i]))
    {
      hw_commands_[i] = 0;
    }
    if (std::isnan(hw_states_[i]))
    {
      hw_states_[i] = 0;
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"), "Successfully activated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Pca9685SystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("Pca9685SystemHardware"), "Successfully deactivated!");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Pca9685SystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  return hardware_interface::return_type::OK;
}
void Pca9685SystemHardware::transform_angles(std::vector<double>& hw_commands){
  /*Since in your robot built you might need each leg to be rotated 60 degrees for example
   but all servo orientations may not be the same so here you can apply rectifications for each servo*/
  // In order of joint_names
  hw_commands[0] = hw_commands[0];  //lf_coax
  hw_commands[1] = M_PI - hw_commands[1];  //lf_femur
  hw_commands[2] = M_PI - hw_commands[2];  //lf_tibia
  hw_commands[3] = M_PI - hw_commands[3];  //rf_coax
  hw_commands[4] = hw_commands[4];  //rf_femur
  hw_commands[5] = hw_commands[5];  //rf_tibia
  hw_commands[6] = hw_commands[6];  //lb_coax
  hw_commands[7] = M_PI - hw_commands[7];  //lb_femur
  hw_commands[8] = M_PI - hw_commands[8];  //lb_tibia
  hw_commands[9] = M_PI - hw_commands[9];  //rb_coax
  hw_commands[10] = hw_commands[10];  //rb_femur
  hw_commands[11] = hw_commands[11];  //rb_tibia
}
double Pca9685SystemHardware::command_to_duty_cycle(double command){

    double min_rads = 0.0;
    double max_rads = M_PI;
    double min_us = 500;
    double max_us = 2500;
    //Gaitplanifier shouldnt write rads outside range but in case
    double clamped_command = std::clamp(command , min_rads, max_rads);
    return min_us + ((clamped_command - min_rads) * (max_us - min_us)) / (max_rads - min_rads);


}

hardware_interface::return_type Pca9685SystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  hw_states_ = hw_commands_;
  transform_angles(hw_commands_);
  for (auto i = 0u; i < hw_commands_.size(); i++)
  { 

    double duty_cycle = command_to_duty_cycle(hw_commands_[i] + servos_offsets_[i]);
    // RCLCPP_INFO(
    //     rclcpp::get_logger("Pca9685SystemHardware"),
    //     "Joint '%d' has command '%f', duty_cycle: '%f'.", i, hw_commands_[i], duty_cycle);

    pca->set_pwm_ms(i, duty_cycle/1000.0);

  }

  return hardware_interface::return_type::OK;
}

}  // namespace coco_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  coco_hardware::Pca9685SystemHardware, hardware_interface::SystemInterface)
