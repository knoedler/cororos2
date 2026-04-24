// Copyright (c) 2026, b-robotized Group
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

#include "pwm_hardware_interface/pwm_hardware_interface.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <utility>

#include <pluginlib/class_list_macros.hpp>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace
{
template <typename T>
T clamp_value(T value, T minimum, T maximum)
{
  return std::max(minimum, std::min(value, maximum));
}

std::string parse_non_empty_string(const std::string & value)
{
  if (value.empty())
  {
    throw std::invalid_argument("must not be empty");
  }
  return value;
}

int parse_int(const std::string & value) { return std::stoi(value); }

double parse_double(const std::string & value) { return std::stod(value); }

bool parse_flexible_bool(const std::string & value)
{
  if (
    value == "true" || value == "True" || value == "TRUE" || value == "1" || value == "yes" ||
    value == "on")
  {
    return true;
  }
  if (
    value == "false" || value == "False" || value == "FALSE" || value == "0" || value == "no" ||
    value == "off")
  {
    return false;
  }
  throw std::invalid_argument("must be a boolean value");
}

uint8_t parse_channel(const std::string & value)
{
  const int channel = std::stoi(value);
  if (channel < 0 || channel > 255)
  {
    throw std::out_of_range("must be within [0, 255]");
  }
  return static_cast<uint8_t>(channel);
}
}  // namespace

namespace pwm_hardware_interface
{

hardware_interface::CallbackReturn PwmHardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.joints.size() != 4)
  {
    RCLCPP_ERROR(
      get_logger(), "pwm_hardware_interface expects exactly 4 wheel joints, got %zu",
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto & hardware_parameters = info_.hardware_parameters;

  const auto parse_required_parameter = [this, &hardware_parameters](
                                          const char * key, const char * expected_description,
                                          auto parser, auto & output) -> bool
  {
    const auto iterator = hardware_parameters.find(key);
    if (iterator == hardware_parameters.end())
    {
      RCLCPP_ERROR(
        get_logger(), "Missing required hardware parameter '%s' for pwm_hardware_interface.", key);
      return false;
    }

    try
    {
      output = parser(iterator->second);
    }
    catch (const std::exception & error)
    {
      RCLCPP_ERROR(
        get_logger(), "Invalid hardware parameter '%s' value '%s': %s. Expected %s.", key,
        iterator->second.c_str(), error.what(), expected_description);
      return false;
    }

    return true;
  };

  const auto parse_optional_parameter = [this, &hardware_parameters](
                                          const char * key, const char * expected_description,
                                          auto parser, auto & output) -> bool
  {
    const auto iterator = hardware_parameters.find(key);
    if (iterator == hardware_parameters.end())
    {
      return true;
    }

    try
    {
      output = parser(iterator->second);
    }
    catch (const std::exception & error)
    {
      RCLCPP_ERROR(
        get_logger(), "Invalid hardware parameter '%s' value '%s': %s. Expected %s.", key,
        iterator->second.c_str(), error.what(), expected_description);
      return false;
    }

    return true;
  };

  const char * bool_description =
    "one of: true, false, True, False, TRUE, FALSE, 1, 0, yes, no, on, off";

  if (
    !parse_required_parameter(
      "device_path", "a non-empty string", parse_non_empty_string, device_path_) ||
    !parse_required_parameter("pwm_min", "an integer", parse_int, pwm_min_) ||
    !parse_required_parameter("pwm_neutral", "an integer", parse_int, pwm_neutral_) ||
    !parse_required_parameter("pwm_max", "an integer", parse_int, pwm_max_) ||
    !parse_required_parameter(
      "wheel_radius", "a floating-point number", parse_double, wheel_radius_) ||
    !parse_required_parameter(
      "max_wheel_speed_mps", "a floating-point number", parse_double, max_wheel_speed_mps_) ||
    !parse_optional_parameter(
      "min_active_pwm_delta", "a non-negative integer", parse_int, min_active_pwm_delta_) ||
    !parse_required_parameter("invert_left", bool_description, parse_flexible_bool, invert_left_) ||
    !parse_required_parameter(
      "invert_right", bool_description, parse_flexible_bool, invert_right_) ||
    !parse_required_parameter(
      "channel_fl", "an integer in the range [0, 255]", parse_channel, channels_[0]) ||
    !parse_required_parameter(
      "channel_fr", "an integer in the range [0, 255]", parse_channel, channels_[1]) ||
    !parse_required_parameter(
      "channel_rl", "an integer in the range [0, 255]", parse_channel, channels_[2]) ||
    !parse_required_parameter(
      "channel_rr", "an integer in the range [0, 255]", parse_channel, channels_[3]))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!(pwm_min_ < pwm_neutral_ && pwm_neutral_ < pwm_max_))
  {
    RCLCPP_ERROR(get_logger(), "Expected pwm_min < pwm_neutral < pwm_max.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  const int forward_span = pwm_max_ - pwm_neutral_;
  const int reverse_span = pwm_neutral_ - pwm_min_;
  if (
    min_active_pwm_delta_ < 0 || min_active_pwm_delta_ >= forward_span ||
    min_active_pwm_delta_ >= reverse_span)
  {
    RCLCPP_ERROR(
      get_logger(), "min_active_pwm_delta must be non-negative and smaller than both PWM spans.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (wheel_radius_ <= 0.0 || max_wheel_speed_mps_ <= 0.0)
  {
    RCLCPP_ERROR(get_logger(), "wheel_radius and max_wheel_speed_mps must be positive.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_commands_.assign(info_.joints.size(), 0.0);
  hw_positions_.assign(info_.joints.size(), 0.0);
  hw_velocities_.assign(info_.joints.size(), 0.0);
  last_pwm_.fill(static_cast<int16_t>(pwm_neutral_));

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> PwmHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  state_interfaces.reserve(info_.joints.size() * 2);

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> PwmHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  command_interfaces.reserve(info_.joints.size());

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]);
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn PwmHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
  std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);

  if (!start_backend())
  {
    RCLCPP_ERROR(get_logger(), "Failed to start PWM backend.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!write_neutral())
  {
    RCLCPP_ERROR(get_logger(), "Failed to write neutral PWM during configure.");
    stop_backend();
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_logger(), "Allie PWM hardware interface configured.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PwmHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
  std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);

  if (!backend_running_)
  {
    RCLCPP_ERROR(get_logger(), "PWM backend is not running.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!write_neutral())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_logger(), "Allie PWM hardware interface activated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PwmHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
  std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);

  if (backend_running_)
  {
    write_neutral();
  }

  RCLCPP_INFO(get_logger(), "Allie PWM hardware interface deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PwmHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (backend_running_)
  {
    write_neutral();
  }
  stop_backend();
  RCLCPP_INFO(get_logger(), "Allie PWM hardware interface cleaned up.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PwmHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (backend_running_)
  {
    write_neutral();
  }
  stop_backend();
  RCLCPP_INFO(get_logger(), "Allie PWM hardware interface shut down.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PwmHardwareInterface::on_error(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (backend_running_)
  {
    write_neutral();
  }
  stop_backend();
  RCLCPP_ERROR(get_logger(), "Allie PWM hardware interface entered error state.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type PwmHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  const double dt = period.seconds();

  for (size_t i = 0; i < hw_commands_.size(); ++i)
  {
    hw_velocities_[i] = hw_commands_[i];
    hw_positions_[i] += hw_velocities_[i] * dt;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type PwmHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!backend_running_ || maestro_fd_ < 0)
  {
    RCLCPP_ERROR(get_logger(), "PWM backend is not running.");
    return hardware_interface::return_type::ERROR;
  }

  const int16_t front_left_pwm = speed_to_pwm(hw_commands_[0], invert_left_);
  const int16_t front_right_pwm = speed_to_pwm(hw_commands_[1], invert_right_);
  const int16_t rear_left_pwm = speed_to_pwm(hw_commands_[2], invert_left_);
  const int16_t rear_right_pwm = speed_to_pwm(hw_commands_[3], invert_right_);

  last_pwm_ = {front_left_pwm, front_right_pwm, rear_left_pwm, rear_right_pwm};

  if (
    !write_maestro_target(channels_[0], static_cast<uint16_t>(front_left_pwm * 4)) ||
    !write_maestro_target(channels_[1], static_cast<uint16_t>(front_right_pwm * 4)) ||
    !write_maestro_target(channels_[2], static_cast<uint16_t>(rear_left_pwm * 4)) ||
    !write_maestro_target(channels_[3], static_cast<uint16_t>(rear_right_pwm * 4)))
  {
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

int16_t PwmHardwareInterface::speed_to_pwm(double wheel_velocity_rad_s, bool invert) const
{
  double wheel_speed_mps = wheel_velocity_rad_s * wheel_radius_;
  if (invert)
  {
    wheel_speed_mps = -wheel_speed_mps;
  }

  const double normalized = clamp_value(wheel_speed_mps / max_wheel_speed_mps_, -1.0, 1.0);
  if (std::abs(normalized) < 1e-6)
  {
    return static_cast<int16_t>(pwm_neutral_);
  }

  if (normalized >= 0.0)
  {
    const double span = static_cast<double>(pwm_max_ - pwm_neutral_);
    const double scaled_span =
      static_cast<double>(min_active_pwm_delta_) + normalized * (span - min_active_pwm_delta_);
    return static_cast<int16_t>(std::lround(pwm_neutral_ + scaled_span));
  }

  const double span = static_cast<double>(pwm_neutral_ - pwm_min_);
  const double scaled_span = static_cast<double>(min_active_pwm_delta_) +
                             std::abs(normalized) * (span - min_active_pwm_delta_);
  return static_cast<int16_t>(std::lround(pwm_neutral_ - scaled_span));
}

bool PwmHardwareInterface::start_backend()
{
  if (backend_running_)
  {
    return true;
  }

  if (!open_maestro())
  {
    return false;
  }

  if (!configure_maestro_serial())
  {
    close_maestro();
    return false;
  }

  backend_running_ = true;
  return true;
}

void PwmHardwareInterface::stop_backend()
{
  if (maestro_fd_ >= 0)
  {
    close_maestro();
  }
  backend_running_ = false;
}

bool PwmHardwareInterface::open_maestro()
{
  maestro_fd_ = open(device_path_.c_str(), O_RDWR | O_NOCTTY);
  if (maestro_fd_ < 0)
  {
    RCLCPP_ERROR(
      get_logger(), "Failed to open Maestro device '%s': %s", device_path_.c_str(),
      std::strerror(errno));
    return false;
  }

  return true;
}

void PwmHardwareInterface::close_maestro()
{
  if (maestro_fd_ >= 0)
  {
    close(maestro_fd_);
    maestro_fd_ = -1;
  }
}

bool PwmHardwareInterface::configure_maestro_serial()
{
  struct termios options;
  if (tcgetattr(maestro_fd_, &options) < 0)
  {
    RCLCPP_ERROR(get_logger(), "Failed to get serial attributes for Maestro.");
    return false;
  }

  options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
  options.c_oflag &= ~(ONLCR | OCRNL);
  options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);

  if (tcsetattr(maestro_fd_, TCSANOW, &options) < 0)
  {
    RCLCPP_ERROR(get_logger(), "Failed to set serial attributes for Maestro.");
    return false;
  }

  return true;
}

bool PwmHardwareInterface::write_maestro_target(uint8_t channel, uint16_t target)
{
  const uint8_t command[] = {
    0x84, channel, static_cast<uint8_t>(target & 0x7F), static_cast<uint8_t>((target >> 7) & 0x7F)};

  const auto bytes_written = ::write(maestro_fd_, command, sizeof(command));
  if (bytes_written != static_cast<ssize_t>(sizeof(command)))
  {
    RCLCPP_ERROR(get_logger(), "Failed to write Maestro target for channel %u.", channel);
    return false;
  }

  return true;
}

bool PwmHardwareInterface::write_neutral()
{
  const uint16_t neutral_target = static_cast<uint16_t>(pwm_neutral_ * 4);

  return write_maestro_target(channels_[0], neutral_target) &&
         write_maestro_target(channels_[1], neutral_target) &&
         write_maestro_target(channels_[2], neutral_target) &&
         write_maestro_target(channels_[3], neutral_target);
}

}  // namespace pwm_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  pwm_hardware_interface::PwmHardwareInterface, hardware_interface::SystemInterface)
