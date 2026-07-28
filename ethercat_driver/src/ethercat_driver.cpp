// Copyright 2022 ICUBE Laboratory, University of Strasbourg
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

#include "ethercat_driver/ethercat_driver.hpp"

#include <algorithm>
#include <cctype>
#include <exception>
#include <limits>
#include <string>
#include <unordered_map>
#include <utility>

#include "ethercat_driver/ethercat_ros2_control_xml_parser.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ethercat_driver
{

namespace
{

bool defaultCreateUserspaceMaster()
{
#ifdef ETHERCAT_INTERFACE_DPDK_USERMODE
  return true;
#else
  return false;
#endif
}

bool bool_from_string(const std::string & value)
{
  std::string lowered = value;
  std::transform(
    lowered.begin(), lowered.end(), lowered.begin(),
    [](unsigned char c) {return static_cast<char>(std::tolower(c));});

  if (lowered == "1" || lowered == "true" || lowered == "yes" || lowered == "on") {
    return true;
  }
  if (lowered == "0" || lowered == "false" || lowered == "no" || lowered == "off") {
    return false;
  }
  throw std::invalid_argument("invalid bool value: " + value);
}

bool bool_parameter_or_default(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & name,
  bool default_value)
{
  const auto it = parameters.find(name);
  if (it == parameters.end()) {
    return default_value;
  }
  return bool_from_string(it->second);
}

unsigned int uint_parameter_or_default(
  const std::unordered_map<std::string, std::string> & parameters,
  const std::string & name,
  unsigned int default_value)
{
  const auto it = parameters.find(name);
  if (it == parameters.end()) {
    return default_value;
  }
  return std::stoul(it->second);
}

ConfiguredEcModule make_configured_ec_module(
  std::unordered_map<std::string, std::string> parameters,
  std::vector<double> * input_values,
  std::vector<double> * output_values,
  std::string component_name,
  std::string module_type,
  size_t module_number)
{
  ConfiguredEcModule module;
  module.parameters = std::move(parameters);
  module.input_values = input_values;
  module.output_values = output_values;
  module.component_name = std::move(component_name);
  module.module_type = std::move(module_type);
  module.module_number = module_number;
  return module;
}

}  // namespace

bool configure_ethercat_bus_config(
  const std::unordered_map<std::string, std::string> & hardware_parameters,
  EthercatBusConfig & bus_config)
{
  unsigned int master_id = 666;
  // Get master id
  if (hardware_parameters.find("master_id") == hardware_parameters.end()) {
    // Master id was not provided, default to 0
    master_id = 0;
  } else {
    try {
      master_id = std::stoul(hardware_parameters.at("master_id"));
    } catch (std::exception & e) {
      RCLCPP_FATAL(
        rclcpp::get_logger("EthercatDriver"), "Invalid master id (%s)!", e.what());
      return false;
    }
  }
  bus_config.master_id = master_id;

  try {
    bus_config.userspace_node_id = uint_parameter_or_default(
      hardware_parameters, "userspace_node_id", 0);
    bus_config.create_userspace_master = bool_parameter_or_default(
      hardware_parameters,
      "create_userspace_master",
      defaultCreateUserspaceMaster());
    bus_config.wait_for_slaves = bool_parameter_or_default(
      hardware_parameters,
      "wait_for_slaves",
      bus_config.create_userspace_master);
    bus_config.wait_for_slave_count = uint_parameter_or_default(
      hardware_parameters,
      "expected_slave_count",
      bus_config.wait_for_slave_count);
    bus_config.wait_for_slave_count = uint_parameter_or_default(
      hardware_parameters,
      "wait_for_slave_count",
      bus_config.wait_for_slave_count);
    const auto dc_reference = hardware_parameters.find("dc_reference_position");
    if (dc_reference != hardware_parameters.end()) {
      bus_config.has_dc_reference = true;
      bus_config.dc_reference_alias = uint_parameter_or_default(
        hardware_parameters, "dc_reference_alias", 0);
      bus_config.dc_reference_position = uint_parameter_or_default(
        hardware_parameters, "dc_reference_position", 0);
      if (bus_config.dc_reference_alias > std::numeric_limits<uint16_t>::max() ||
        bus_config.dc_reference_position > std::numeric_limits<uint16_t>::max())
      {
        throw std::out_of_range("DC reference alias or position exceeds uint16 range");
      }
    }
  } catch (std::exception & e) {
    RCLCPP_FATAL(
      rclcpp::get_logger("EthercatDriver"), "Invalid EtherCAT master option (%s)!", e.what());
    return false;
  }

  // Get control frequency
  if (hardware_parameters.find("control_frequency") == hardware_parameters.end()) {
    // Control frequency was not provided, default to 100 Hz
    bus_config.control_frequency = 100.0;
  } else {
    try {
      bus_config.control_frequency = std::stod(hardware_parameters.at("control_frequency"));
    } catch (std::exception & e) {
      RCLCPP_FATAL(
        rclcpp::get_logger("EthercatDriver"), "Invalid control frequency (%s)!", e.what());
      return false;
    }
  }

  const auto transfer_config = hardware_parameters.find("transfer_config");
  const auto fsoe_config = hardware_parameters.find("fsoe_config");
  if (transfer_config != hardware_parameters.end() && fsoe_config != hardware_parameters.end()) {
    RCLCPP_FATAL(
      rclcpp::get_logger("EthercatDriver"),
      "Both transfer_config and fsoe_config parameters are provided! Please provide only one "
      "of them.");
    return false;
  }
  if (transfer_config != hardware_parameters.end() && transfer_config->second.empty()) {
    RCLCPP_FATAL(
      rclcpp::get_logger("EthercatDriver"), "Empty transfer_config or fsoe_config parameter!");
    return false;
  }
  if (fsoe_config != hardware_parameters.end() && fsoe_config->second.empty()) {
    RCLCPP_FATAL(
      rclcpp::get_logger("EthercatDriver"), "Empty transfer_config or fsoe_config parameter!");
    return false;
  }

  if (transfer_config != hardware_parameters.end()) {
    bus_config.transfer_config = transfer_config->second;
  }

  if (fsoe_config != hardware_parameters.end()) {
    bus_config.fsoe_config = fsoe_config->second;
  }

  return true;
}

CallbackReturn EthercatDriver::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // Set state vectors
  hw_joint_states_.resize(info_.joints.size());
  for (uint j = 0; j < info_.joints.size(); j++) {
    hw_joint_states_[j].resize(
      info_.joints[j].state_interfaces.size(),
      std::numeric_limits<double>::quiet_NaN());
  }
  hw_sensor_states_.resize(info_.sensors.size());
  for (uint s = 0; s < info_.sensors.size(); s++) {
    hw_sensor_states_[s].resize(
      info_.sensors[s].state_interfaces.size(),
      std::numeric_limits<double>::quiet_NaN());
  }
  hw_gpio_states_.resize(info_.gpios.size());
  for (uint g = 0; g < info_.gpios.size(); g++) {
    hw_gpio_states_[g].resize(
      info_.gpios[g].state_interfaces.size(),
      std::numeric_limits<double>::quiet_NaN());
  }

  // Set command vectors
  hw_joint_commands_.resize(info_.joints.size());
  for (uint j = 0; j < info_.joints.size(); j++) {
    hw_joint_commands_[j].resize(
      info_.joints[j].command_interfaces.size(),
      std::numeric_limits<double>::quiet_NaN());
  }
  hw_sensor_commands_.resize(info_.sensors.size());
  for (uint s = 0; s < info_.sensors.size(); s++) {
    hw_sensor_commands_[s].resize(
      info_.sensors[s].command_interfaces.size(),
      std::numeric_limits<double>::quiet_NaN());
  }
  hw_gpio_commands_.resize(info_.gpios.size());
  for (uint g = 0; g < info_.gpios.size(); g++) {
    hw_gpio_commands_[g].resize(
      info_.gpios[g].command_interfaces.size(),
      std::numeric_limits<double>::quiet_NaN());
  }

  std::vector<ConfiguredEcModule> configured_modules;

  // Setup slave modules defined per joints in the URDF
  for (uint j = 0; j < info_.joints.size(); j++) {
    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "joints");
    // check all joints for EC modules and load into ec_modules_
    auto module_params = getEcModuleParam(
      info_.original_xml, info_.joints[j].name, "joint");
    for (auto i = 0ul; i < module_params.size(); i++) {
      for (auto k = 0ul; k < info_.joints[j].state_interfaces.size(); k++) {
        module_params[i]["state_interface/" +
          info_.joints[j].state_interfaces[k].name] = std::to_string(k);
      }
      for (auto k = 0ul; k < info_.joints[j].command_interfaces.size(); k++) {
        module_params[i]["command_interface/" +
          info_.joints[j].command_interfaces[k].name] = std::to_string(k);
      }
      configured_modules.push_back(
        make_configured_ec_module(
          module_params[i],
          &hw_joint_states_[j],
          &hw_joint_commands_[j],
          info_.joints[j].name,
          "Joint",
          i + 1));
    }
  }

  // Setup slave modules defined per GPIOs in the URDF
  for (uint g = 0; g < info_.gpios.size(); g++) {
    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "gpios");
    // check all gpios for EC modules and load into ec_modules_
    auto module_params = getEcModuleParam(
      info_.original_xml, info_.gpios[g].name, "gpio");
    for (auto i = 0ul; i < module_params.size(); i++) {
      for (auto k = 0ul; k < info_.gpios[g].state_interfaces.size(); k++) {
        module_params[i]["state_interface/" +
          info_.gpios[g].state_interfaces[k].name] = std::to_string(k);
      }
      for (auto k = 0ul; k < info_.gpios[g].command_interfaces.size(); k++) {
        module_params[i]["command_interface/" +
          info_.gpios[g].command_interfaces[k].name] = std::to_string(k);
      }
      configured_modules.push_back(
        make_configured_ec_module(
          module_params[i],
          &hw_gpio_states_[g],
          &hw_gpio_commands_[g],
          info_.gpios[g].name,
          "GPIO",
          i + 1));
    }
  }

  // Setup slave modules defined per sensors in the URDF
  for (uint s = 0; s < info_.sensors.size(); s++) {
    RCLCPP_INFO(rclcpp::get_logger("EthercatDriver"), "sensors");
    // check all sensors for EC modules and load into ec_modules_
    auto module_params = getEcModuleParam(
      info_.original_xml, info_.sensors[s].name, "sensor");
    for (auto i = 0ul; i < module_params.size(); i++) {
      for (auto k = 0ul; k < info_.sensors[s].state_interfaces.size(); k++) {
        module_params[i]["state_interface/" +
          info_.sensors[s].state_interfaces[k].name] = std::to_string(k);
      }
      for (auto k = 0ul; k < info_.sensors[s].command_interfaces.size(); k++) {
        module_params[i]["command_interface/" +
          info_.sensors[s].command_interfaces[k].name] = std::to_string(k);
      }
      configured_modules.push_back(
        make_configured_ec_module(
          module_params[i],
          &hw_sensor_states_[s],
          &hw_sensor_commands_[s],
          info_.sensors[s].name,
          "Sensor",
          i + 1));
    }
  }

  EthercatBusConfig bus_config;
  if (!configure_ethercat_bus_config(info_.hardware_parameters, bus_config)) {
    return CallbackReturn::ERROR;
  }

  if (bus_config.wait_for_slaves && bus_config.wait_for_slave_count == 0) {
    std::vector<std::unordered_map<std::string, std::string>> module_parameters;
    module_parameters.reserve(configured_modules.size());
    for (const auto & configured_module : configured_modules) {
      module_parameters.push_back(configured_module.parameters);
    }
    bus_config.wait_for_slave_count = expectedSlaveWaitCount(module_parameters);
  }

  if (!bus_manager_.configureModules(bus_config, configured_modules)) {
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn EthercatDriver::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

CallbackReturn EthercatDriver::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  bus_manager_.cleanup();

  RCLCPP_INFO(
    rclcpp::get_logger("EthercatDriver"), "System successfully cleaned up!");

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
EthercatDriver::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  // export joint state interface
  for (uint j = 0; j < info_.joints.size(); j++) {
    for (uint i = 0; i < info_.joints[j].state_interfaces.size(); i++) {
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          info_.joints[j].name,
          info_.joints[j].state_interfaces[i].name,
          &hw_joint_states_[j][i]));
    }
  }
  // export sensor state interface
  for (uint s = 0; s < info_.sensors.size(); s++) {
    for (uint i = 0; i < info_.sensors[s].state_interfaces.size(); i++) {
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          info_.sensors[s].name,
          info_.sensors[s].state_interfaces[i].name,
          &hw_sensor_states_[s][i]));
    }
  }
  // export gpio state interface
  for (uint g = 0; g < info_.gpios.size(); g++) {
    for (uint i = 0; i < info_.gpios[g].state_interfaces.size(); i++) {
      state_interfaces.emplace_back(
        hardware_interface::StateInterface(
          info_.gpios[g].name,
          info_.gpios[g].state_interfaces[i].name,
          &hw_gpio_states_[g][i]));
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
EthercatDriver::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  // export joint command interface
  std::vector<double> test;
  for (uint j = 0; j < info_.joints.size(); j++) {
    for (uint i = 0; i < info_.joints[j].command_interfaces.size(); i++) {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          info_.joints[j].name,
          info_.joints[j].command_interfaces[i].name,
          &hw_joint_commands_[j][i]));
    }
  }
  // export sensor command interface
  for (uint s = 0; s < info_.sensors.size(); s++) {
    for (uint i = 0; i < info_.sensors[s].command_interfaces.size(); i++) {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          info_.sensors[s].name,
          info_.sensors[s].command_interfaces[i].name,
          &hw_sensor_commands_[s][i]));
    }
  }
  // export gpio command interface
  for (uint g = 0; g < info_.gpios.size(); g++) {
    for (uint i = 0; i < info_.gpios[g].command_interfaces.size(); i++) {
      command_interfaces.emplace_back(
        hardware_interface::CommandInterface(
          info_.gpios[g].name,
          info_.gpios[g].command_interfaces[i].name,
          &hw_gpio_commands_[g][i]));
    }
  }
  return command_interfaces;
}

CallbackReturn EthercatDriver::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!bus_manager_.activateBus()) {
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn EthercatDriver::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  bus_manager_.deactivateBus();

  return CallbackReturn::SUCCESS;
}

hardware_interface::return_type EthercatDriver::read(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  if (bus_manager_.read() == EthercatCycleResult::kError) {
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type EthercatDriver::write(
  const rclcpp::Time & /*time*/,
  const rclcpp::Duration & /*period*/)
{
  if (bus_manager_.write() == EthercatCycleResult::kError) {
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

}  // namespace ethercat_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  ethercat_driver::EthercatDriver, hardware_interface::SystemInterface)
