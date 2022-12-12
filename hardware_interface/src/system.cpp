// Copyright 2020 - 2021 ros2_control Development Team
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

#include "hardware_interface/system.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/lifecycle_state_names.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace hardware_interface
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

System::System(std::unique_ptr<SystemInterface> impl) : impl_(std::move(impl)) {}

const rclcpp_lifecycle::State & System::initialize(const HardwareInfo & system_info)
{
  if (impl_->get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN)
  {
    switch (impl_->on_init(system_info))
    {
      case CallbackReturn::SUCCESS:
        impl_->set_state(rclcpp_lifecycle::State(
          lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
          lifecycle_state_names::UNCONFIGURED));
        break;
      case CallbackReturn::FAILURE:
      case CallbackReturn::ERROR:
        impl_->set_state(rclcpp_lifecycle::State(
          lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED, lifecycle_state_names::FINALIZED));
        break;
    }
  }
  return impl_->get_state();
}

const rclcpp_lifecycle::State & System::configure()
{
  if (impl_->get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED)
  {
    switch (impl_->on_configure(impl_->get_state()))
    {
      case CallbackReturn::SUCCESS:
        impl_->set_state(rclcpp_lifecycle::State(
          lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, lifecycle_state_names::INACTIVE));
        break;
      case CallbackReturn::FAILURE:
        impl_->set_state(rclcpp_lifecycle::State(
          lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
          lifecycle_state_names::UNCONFIGURED));
        break;
      case CallbackReturn::ERROR:
        impl_->set_state(error());
        break;
    }
  }
  return impl_->get_state();
}

const rclcpp_lifecycle::State & System::cleanup()
{
  if (impl_->get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    switch (impl_->on_cleanup(impl_->get_state()))
    {
      case CallbackReturn::SUCCESS:
        impl_->set_state(rclcpp_lifecycle::State(
          lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
          lifecycle_state_names::UNCONFIGURED));
        break;
      case CallbackReturn::FAILURE:
      case CallbackReturn::ERROR:
        impl_->set_state(error());
        break;
    }
  }
  return impl_->get_state();
}

const rclcpp_lifecycle::State & System::shutdown()
{
  if (
    impl_->get_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN &&
    impl_->get_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED)
  {
    switch (impl_->on_shutdown(impl_->get_state()))
    {
      case CallbackReturn::SUCCESS:
        impl_->set_state(rclcpp_lifecycle::State(
          lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED, lifecycle_state_names::FINALIZED));
        break;
      case CallbackReturn::FAILURE:
      case CallbackReturn::ERROR:
        impl_->set_state(error());
        break;
    }
  }
  return impl_->get_state();
}

const rclcpp_lifecycle::State & System::activate()
{
  if (impl_->get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE)
  {
    switch (impl_->on_activate(impl_->get_state()))
    {
      case CallbackReturn::SUCCESS:
        impl_->set_state(rclcpp_lifecycle::State(
          lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, lifecycle_state_names::ACTIVE));
        break;
      case CallbackReturn::FAILURE:
        impl_->set_state(rclcpp_lifecycle::State(
          lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, lifecycle_state_names::INACTIVE));
        break;
      case CallbackReturn::ERROR:
        impl_->set_state(error());
        break;
    }
  }
  return impl_->get_state();
}

const rclcpp_lifecycle::State & System::deactivate()
{
  if (impl_->get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    switch (impl_->on_deactivate(impl_->get_state()))
    {
      case CallbackReturn::SUCCESS:
        impl_->set_state(rclcpp_lifecycle::State(
          lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE, lifecycle_state_names::INACTIVE));
        break;
      case CallbackReturn::FAILURE:
        impl_->set_state(rclcpp_lifecycle::State(
          lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE, lifecycle_state_names::ACTIVE));
        break;
      case CallbackReturn::ERROR:
        impl_->set_state(error());
        break;
    }
  }
  return impl_->get_state();
}

const rclcpp_lifecycle::State & System::error()
{
  if (impl_->get_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN)
  {
    switch (impl_->on_error(impl_->get_state()))
    {
      case CallbackReturn::SUCCESS:
        impl_->set_state(rclcpp_lifecycle::State(
          lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED,
          lifecycle_state_names::UNCONFIGURED));
        break;
      case CallbackReturn::FAILURE:
      case CallbackReturn::ERROR:
        impl_->set_state(rclcpp_lifecycle::State(
          lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED, lifecycle_state_names::FINALIZED));
        break;
    }
  }
  return impl_->get_state();
}

std::vector<std::string> System::export_state_interfaces()
{
  const auto state_interface_description = impl_->export_state_interfaces_descriptions();
  std::vector<std::string> state_interfaces;
  for (const auto & interface_description : state_interface_description)
  {
    StateInterface state_interface(interface_description);
    state_interfaces.push_back(impl_->add_state_interface(state_interface));
  }
  return state_interfaces;
}

std::vector<std::string> System::export_command_interfaces()
{
  const auto command_interface_description = impl_->export_command_interfaces_descriptions();
  std::vector<std::string> command_interfaces;
  for (const auto & interface_description : command_interface_description)
  {
    CommandInterface command_interface(interface_description);
    command_interfaces.push_back(impl_->add_command_interface(command_interface));
  }
  return command_interfaces;
}
// problem: functionality for claiming (creation) and release of loans
// need to be passed to implementation specific interface and HANDLED there
// not very clean, don't like this
LoanedStateInterface System::claim_state_interfaces(const std::string & state_interface)
{
  return impl_->claim_state_interface(state_interface);
}

LoanedCommandInterface System::claim_command_interfaces(const std::string & command_interface)
{
  return impl_->claim_command_interface(command_interface);
}

return_type System::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  return impl_->prepare_command_mode_switch(start_interfaces, stop_interfaces);
}

return_type System::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  return impl_->perform_command_mode_switch(start_interfaces, stop_interfaces);
}

std::string System::get_name() const { return impl_->get_name(); }

const rclcpp_lifecycle::State & System::get_state() const { return impl_->get_state(); }

return_type System::read(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // TODO(destogl): discuss what should be default return value, e.g., "NOT_EXECUTED"
  return_type result = return_type::ERROR;
  if (
    impl_->get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE ||
    impl_->get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    result = impl_->read(time, period);
    if (result == return_type::ERROR)
    {
      error();
    }
  }
  return result;
}

return_type System::write(const rclcpp::Time & time, const rclcpp::Duration & period)
{
  // TODO(destogl): discuss what should be default return value, e.g., "NOT_EXECUTED"
  return_type result = return_type::ERROR;
  if (
    impl_->get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE ||
    impl_->get_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
  {
    result = impl_->write(time, period);
    if (result == return_type::ERROR)
    {
      error();
    }
  }
  return result;
}

}  // namespace hardware_interface
