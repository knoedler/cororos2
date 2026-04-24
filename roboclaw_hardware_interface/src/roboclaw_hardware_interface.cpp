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

#include "roboclaw_hardware_interface/roboclaw_hardware_interface.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <pluginlib/class_list_macros.hpp>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace
{
bool has_interface(
  const std::vector<hardware_interface::InterfaceInfo> & interfaces, const std::string & name)
{
  return std::any_of(
    interfaces.begin(), interfaces.end(),
    [&name](const hardware_interface::InterfaceInfo & interface_info)
    { return interface_info.name == name; });
}

std::string format_double(double value)
{
  if (std::isfinite(value))
  {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3) << value;
    return stream.str();
  }
  return "nan";
}

const std::vector<std::pair<int, std::pair<uint8_t, const char *>>> & error_table()
{
  static const std::vector<std::pair<int, std::pair<uint8_t, const char *>>> table = {
    {0x0001, {diagnostic_msgs::msg::DiagnosticStatus::WARN, "M1 over current"}},
    {0x0002, {diagnostic_msgs::msg::DiagnosticStatus::WARN, "M2 over current"}},
    {0x0004, {diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Emergency Stop"}},
    {0x0008, {diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Temperature1"}},
    {0x0010, {diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Temperature2"}},
    {0x0020, {diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Main batt voltage high"}},
    {0x0040, {diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Logic batt voltage high"}},
    {0x0080, {diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Logic batt voltage low"}},
    {0x0100, {diagnostic_msgs::msg::DiagnosticStatus::WARN, "M1 driver fault"}},
    {0x0200, {diagnostic_msgs::msg::DiagnosticStatus::WARN, "M2 driver fault"}},
    {0x0400, {diagnostic_msgs::msg::DiagnosticStatus::WARN, "Main batt voltage high"}},
    {0x0800, {diagnostic_msgs::msg::DiagnosticStatus::WARN, "Main batt voltage low"}},
    {0x1000, {diagnostic_msgs::msg::DiagnosticStatus::WARN, "Temperature1"}},
    {0x2000, {diagnostic_msgs::msg::DiagnosticStatus::WARN, "Temperature2"}},
    {0x4000, {diagnostic_msgs::msg::DiagnosticStatus::OK, "M1 home"}},
    {0x8000, {diagnostic_msgs::msg::DiagnosticStatus::OK, "M2 home"}},
  };
  return table;
}

uint8_t diagnostic_level_from_error_word(int error_word)
{
  if (error_word < 0)
  {
    return diagnostic_msgs::msg::DiagnosticStatus::WARN;
  }

  uint8_t level = diagnostic_msgs::msg::DiagnosticStatus::OK;
  for (const auto & entry : error_table())
  {
    if ((error_word & entry.first) != 0)
    {
      level = std::max(level, entry.second.first);
    }
  }
  return level;
}

std::string diagnostic_message_from_error_word(int error_word)
{
  if (error_word < 0)
  {
    return "Roboclaw error register unavailable";
  }
  if (error_word == 0)
  {
    return "Normal";
  }

  std::ostringstream stream;
  bool first = true;
  for (const auto & entry : error_table())
  {
    if ((error_word & entry.first) == 0)
    {
      continue;
    }
    if (!first)
    {
      stream << "; ";
    }
    stream << entry.second.second;
    first = false;
  }

  if (first)
  {
    stream << "Roboclaw reported an unrecognized status code";
  }
  return stream.str();
}

diagnostic_msgs::msg::KeyValue make_key_value(const std::string & key, const std::string & value)
{
  diagnostic_msgs::msg::KeyValue item;
  item.key = key;
  item.value = value;
  return item;
}

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
}  // namespace

namespace roboclaw_hardware_interface
{

RoboclawHardwareInterface::~RoboclawHardwareInterface() { stop_backend(); }

hardware_interface::CallbackReturn RoboclawHardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!validate_joint_configuration())
  {
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
        get_logger(), "Missing required hardware parameter '%s' for roboclaw_hardware_interface.",
        key);
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

  if (
    !parse_required_parameter("device", "a non-empty string", parse_non_empty_string, device_) ||
    !parse_required_parameter("baud", "an integer", parse_int, baud_) ||
    !parse_required_parameter("address", "an integer", parse_int, address_) ||
    !parse_required_parameter("max_speed", "a floating-point number", parse_double, max_speed_) ||
    !parse_required_parameter(
      "ticks_at_max_speed", "a floating-point number", parse_double, ticks_at_max_speed_) ||
    !parse_required_parameter("acceleration", "an integer", parse_int, acceleration_) ||
    !parse_required_parameter(
      "ticks_per_meter", "a floating-point number", parse_double, ticks_per_meter_) ||
    !parse_required_parameter(
      "wheel_radius", "a floating-point number", parse_double, wheel_radius_) ||
    !parse_required_parameter(
      "status_interval_sec", "a floating-point number", parse_double, status_interval_sec_) ||
    !parse_required_parameter(
      "use_encoder", "one of: true, false, True, False, TRUE, FALSE, 1, 0, yes, no, on, off",
      parse_flexible_bool, use_encoder_))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (
    baud_ <= 0 || address_ < 0 || address_ > 255 || max_speed_ <= 0.0 ||
    ticks_at_max_speed_ <= 0.0 || acceleration_ < 0 || ticks_per_meter_ <= 0.0 ||
    wheel_radius_ <= 0.0 || status_interval_sec_ < 0.0)
  {
    RCLCPP_ERROR(get_logger(), "roboclaw_hardware_interface received invalid numeric parameters.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  reset_command_and_state_buffers();

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> RoboclawHardwareInterface::export_state_interfaces()
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

std::vector<hardware_interface::CommandInterface>
RoboclawHardwareInterface::export_command_interfaces()
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

hardware_interface::CallbackReturn RoboclawHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  reset_command_and_state_buffers();
  reset_status_publish_state();

  if (!start_backend())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!ensure_publishers())
  {
    stop_backend();
    release_publishers();
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
  std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);

  if (!poll_and_publish_status())
  {
    stop_backend();
    release_publishers();
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (use_encoder_ && !read_state_from_backend())
  {
    stop_backend();
    release_publishers();
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoboclawHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!backend_running_)
  {
    RCLCPP_ERROR(
      get_logger(), "Cannot activate Roboclaw hardware before successful configuration.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
  std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);

  if (!activate_backend())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  reset_status_publish_state();

  if (use_encoder_ && !read_state_from_backend())
  {
    std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
    std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);
    if (backend_running_ && !deactivate_backend())
    {
      RCLCPP_ERROR(get_logger(), "Failed to deactivate Roboclaw hardware after activate failure.");
    }
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!poll_and_publish_status())
  {
    std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
    std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);
    if (backend_running_ && !deactivate_backend())
    {
      RCLCPP_ERROR(get_logger(), "Failed to deactivate Roboclaw hardware after activate failure.");
    }
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoboclawHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
  std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);

  if (backend_running_ && !deactivate_backend())
  {
    RCLCPP_ERROR(get_logger(), "Failed to deactivate Roboclaw hardware safely.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoboclawHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (backend_running_)
  {
    std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
    std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);
    if (!deactivate_backend())
    {
      RCLCPP_ERROR(get_logger(), "Failed to deactivate Roboclaw hardware during cleanup.");
    }
  }

  stop_backend();
  reset_runtime_state();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoboclawHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (backend_running_)
  {
    std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
    std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);
    if (!deactivate_backend())
    {
      RCLCPP_ERROR(get_logger(), "Failed to deactivate Roboclaw hardware during shutdown.");
    }
  }

  stop_backend();
  reset_runtime_state();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn RoboclawHardwareInterface::on_error(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_ERROR(
    get_logger(), "Roboclaw hardware entering error handling from lifecycle state '%s'.",
    previous_state.label().c_str());

  if (backend_running_)
  {
    std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
    std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);
    if (!deactivate_backend())
    {
      RCLCPP_ERROR(get_logger(), "Failed to deactivate Roboclaw hardware during error handling.");
    }
  }

  stop_backend();
  reset_runtime_state();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type RoboclawHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!backend_running_)
  {
    RCLCPP_ERROR(get_logger(), "Roboclaw read called while communication is not configured.");
    return hardware_interface::return_type::ERROR;
  }

  if (!can_read_in_current_state())
  {
    RCLCPP_ERROR(
      get_logger(), "Roboclaw read called in lifecycle state '%s', which is not readable.",
      get_lifecycle_state().label().c_str());
    return hardware_interface::return_type::ERROR;
  }

  if (use_encoder_)
  {
    if (!read_state_from_backend())
    {
      return hardware_interface::return_type::ERROR;
    }
  }
  else
  {
    if (is_active_lifecycle_state())
    {
      const double dt = period.seconds();
      for (size_t i = 0; i < hw_commands_.size(); ++i)
      {
        hw_velocities_[i] = hw_commands_[i];
        hw_positions_[i] += hw_velocities_[i] * dt;
      }
    }
    else
    {
      std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);
    }
  }

  if (!poll_and_publish_status())
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Failed to read Roboclaw state from backend.");
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type RoboclawHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!backend_running_)
  {
    RCLCPP_ERROR(get_logger(), "Roboclaw write called while communication is not configured.");
    return hardware_interface::return_type::ERROR;
  }

  if (!is_active_lifecycle_state())
  {
    return hardware_interface::return_type::OK;
  }

  const double left_mps = hw_commands_[0] * wheel_radius_;
  const double right_mps = hw_commands_[1] * wheel_radius_;

  if (!drive_backend(left_mps, right_mps))
  {
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

bool RoboclawHardwareInterface::validate_joint_configuration() const
{
  if (info_.joints.size() != 2)
  {
    RCLCPP_ERROR(
      get_logger(), "roboclaw_hardware_interface expects exactly 2 track joints, got %zu.",
      info_.joints.size());
    return false;
  }

  for (const auto & joint : info_.joints)
  {
    if (
      joint.command_interfaces.size() != 1 ||
      joint.command_interfaces.front().name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_ERROR(
        get_logger(),
        "Joint '%s' must expose exactly one '%s' command interface for Roboclaw hardware.",
        joint.name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return false;
    }

    if (
      !has_interface(joint.state_interfaces, hardware_interface::HW_IF_POSITION) ||
      !has_interface(joint.state_interfaces, hardware_interface::HW_IF_VELOCITY))
    {
      RCLCPP_ERROR(
        get_logger(),
        "Joint '%s' must expose '%s' and '%s' state interfaces for Roboclaw hardware.",
        joint.name.c_str(), hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY);
      return false;
    }
  }

  return true;
}

bool RoboclawHardwareInterface::read_state_from_backend()
{
  int32_t right_raw_ticks = 0;
  int32_t left_raw_ticks = 0;
  if (!protocol_.read_encoders(right_raw_ticks, left_raw_ticks))
  {
    RCLCPP_ERROR(get_logger(), "%s", protocol_.last_error().c_str());
    return false;
  }

  const int32_t right_ticks = right_raw_ticks;
  const int32_t left_ticks = left_raw_ticks;
  const auto now = std::chrono::steady_clock::now();

  const double left_position_m = static_cast<double>(left_ticks) / ticks_per_meter_;
  const double right_position_m = static_cast<double>(right_ticks) / ticks_per_meter_;
  double left_velocity_mps = 0.0;
  double right_velocity_mps = 0.0;

  if (last_left_ticks_.has_value() && last_right_ticks_.has_value())
  {
    const double dt = std::chrono::duration<double>(now - last_encoder_read_time_).count();
    if (dt > 0.0)
    {
      left_velocity_mps =
        static_cast<double>(left_ticks - last_left_ticks_.value()) / ticks_per_meter_ / dt;
      right_velocity_mps =
        static_cast<double>(right_ticks - last_right_ticks_.value()) / ticks_per_meter_ / dt;
    }
  }

  last_left_ticks_ = left_ticks;
  last_right_ticks_ = right_ticks;
  last_encoder_read_time_ = now;

  hw_positions_[0] = left_position_m / wheel_radius_;
  hw_velocities_[0] = left_velocity_mps / wheel_radius_;
  hw_positions_[1] = right_position_m / wheel_radius_;
  hw_velocities_[1] = right_velocity_mps / wheel_radius_;

  return true;
}

void RoboclawHardwareInterface::release_publishers()
{
  battery_pub_.reset();
  logic_battery_voltage_pub_.reset();
  m1_current_pub_.reset();
  m2_current_pub_.reset();
  temp1_pub_.reset();
  temp2_pub_.reset();
  diagnostics_pub_.reset();
}

void RoboclawHardwareInterface::reset_command_and_state_buffers()
{
  hw_commands_.assign(info_.joints.size(), 0.0);
  hw_positions_.assign(info_.joints.size(), 0.0);
  hw_velocities_.assign(info_.joints.size(), 0.0);
}

void RoboclawHardwareInterface::reset_runtime_state()
{
  reset_command_and_state_buffers();
  release_publishers();
  reset_status_publish_state();
}

bool RoboclawHardwareInterface::is_active_lifecycle_state() const
{
  return get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
}

bool RoboclawHardwareInterface::can_read_in_current_state() const
{
  const auto lifecycle_state = get_lifecycle_state().id();
  return lifecycle_state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE ||
         lifecycle_state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
}

bool RoboclawHardwareInterface::start_backend()
{
  if (backend_running_)
  {
    return true;
  }

  if (!open_roboclaw())
  {
    RCLCPP_ERROR(get_logger(), "%s", protocol_.last_error().c_str());
    return false;
  }
  if (!configure_roboclaw_serial())
  {
    RCLCPP_ERROR(get_logger(), "%s", protocol_.last_error().c_str());
    close_roboclaw();
    return false;
  }
  if (!ping_roboclaw())
  {
    RCLCPP_ERROR(get_logger(), "%s", protocol_.last_error().c_str());
    close_roboclaw();
    return false;
  }

  backend_running_ = true;
  return true;
}

void RoboclawHardwareInterface::stop_backend()
{
  if (backend_running_)
  {
    stop_motors();
  }
  close_roboclaw();
  backend_running_ = false;
  last_left_ticks_.reset();
  last_right_ticks_.reset();
  last_encoder_read_time_ = {};
}

bool RoboclawHardwareInterface::open_roboclaw()
{
  return protocol_.connect(device_, baud_, address_);
}

void RoboclawHardwareInterface::close_roboclaw() { protocol_.disconnect(); }

bool RoboclawHardwareInterface::configure_roboclaw_serial() { return protocol_.is_connected(); }

bool RoboclawHardwareInterface::ping_roboclaw() { return protocol_.ping(); }

bool RoboclawHardwareInterface::activate_backend()
{
  if (!stop_motors())
  {
    return false;
  }

  if (use_encoder_ && !protocol_.reset_encoders())
  {
    RCLCPP_ERROR(get_logger(), "%s", protocol_.last_error().c_str());
    return false;
  }

  last_left_ticks_.reset();
  last_right_ticks_.reset();
  last_encoder_read_time_ = {};
  return true;
}

bool RoboclawHardwareInterface::deactivate_backend() { return stop_motors(); }

bool RoboclawHardwareInterface::drive_backend(double left_mps, double right_mps)
{
  left_mps = clamp_value(left_mps, -max_speed_, max_speed_);
  right_mps = clamp_value(right_mps, -max_speed_, max_speed_);

  if (use_encoder_)
  {
    auto m1 = static_cast<int32_t>(right_mps * ticks_per_meter_);
    auto m2 = static_cast<int32_t>(left_mps * ticks_per_meter_);
    if (m1 == 0 && m2 == 0)
    {
      return stop_motors();
    }
    if (!protocol_.drive_closed_loop(m1, m2))
    {
      RCLCPP_ERROR(get_logger(), "%s", protocol_.last_error().c_str());
      return false;
    }
    return true;
  }

  auto m1 = static_cast<int16_t>((right_mps / max_speed_) * ticks_at_max_speed_);
  auto m2 = static_cast<int16_t>((left_mps / max_speed_) * ticks_at_max_speed_);
  if (m1 == 0 && m2 == 0)
  {
    return stop_motors();
  }

  if (!protocol_.drive_open_loop(m1, m2, static_cast<uint32_t>(acceleration_)))
  {
    RCLCPP_ERROR(get_logger(), "%s", protocol_.last_error().c_str());
    return false;
  }
  return true;
}

bool RoboclawHardwareInterface::stop_motors()
{
  if (!protocol_.stop(use_encoder_))
  {
    RCLCPP_ERROR(get_logger(), "%s", protocol_.last_error().c_str());
    return false;
  }
  return true;
}

bool RoboclawHardwareInterface::read_status_from_backend(RoboclawTelemetry & telemetry)
{
  return protocol_.read_status(telemetry);
}

bool RoboclawHardwareInterface::ensure_publishers()
{
  if (
    battery_pub_ != nullptr && logic_battery_voltage_pub_ != nullptr &&
    m1_current_pub_ != nullptr && m2_current_pub_ != nullptr && temp1_pub_ != nullptr &&
    temp2_pub_ != nullptr && diagnostics_pub_ != nullptr)
  {
    return true;
  }

  const auto node = get_node();
  if (node == nullptr)
  {
    RCLCPP_ERROR(get_logger(), "Failed to access the default hardware component node.");
    return false;
  }

  battery_pub_ =
    node->create_publisher<sensor_msgs::msg::BatteryState>("roboclaw/battery_state", 10);
  logic_battery_voltage_pub_ =
    node->create_publisher<std_msgs::msg::Float32>("roboclaw/logic_battery_voltage", 10);
  m1_current_pub_ = node->create_publisher<std_msgs::msg::Float32>("roboclaw/m1_current", 10);
  m2_current_pub_ = node->create_publisher<std_msgs::msg::Float32>("roboclaw/m2_current", 10);
  temp1_pub_ = node->create_publisher<std_msgs::msg::Float32>("roboclaw/temp1", 10);
  temp2_pub_ = node->create_publisher<std_msgs::msg::Float32>("roboclaw/temp2", 10);
  diagnostics_pub_ =
    node->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 10);
  return true;
}

bool RoboclawHardwareInterface::poll_and_publish_status()
{
  if (!ensure_publishers())
  {
    return false;
  }

  const auto now = std::chrono::steady_clock::now();
  if (
    has_published_status_ &&
    std::chrono::duration<double>(now - last_status_publish_).count() < status_interval_sec_)
  {
    return true;
  }

  RoboclawTelemetry telemetry;
  if (!read_status_from_backend(telemetry))
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Failed to read Roboclaw status from backend.");
    return false;
  }

  publish_status(telemetry);
  last_status_publish_ = now;
  has_published_status_ = true;
  return true;
}

void RoboclawHardwareInterface::publish_status(const RoboclawTelemetry & telemetry)
{
  sensor_msgs::msg::BatteryState battery_msg;
  battery_msg.header.stamp = get_clock()->now();
  battery_msg.voltage = telemetry.main_battery_voltage;
  battery_msg.current = telemetry.m1_current + telemetry.m2_current;
  battery_msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
  battery_msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNKNOWN;
  battery_msg.power_supply_technology =
    sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
  battery_pub_->publish(battery_msg);

  std_msgs::msg::Float32 m1_current_msg;
  m1_current_msg.data = static_cast<float>(telemetry.m1_current);
  m1_current_pub_->publish(m1_current_msg);

  std_msgs::msg::Float32 m2_current_msg;
  m2_current_msg.data = static_cast<float>(telemetry.m2_current);
  m2_current_pub_->publish(m2_current_msg);

  std_msgs::msg::Float32 logic_battery_voltage_msg;
  logic_battery_voltage_msg.data = static_cast<float>(telemetry.logic_battery_voltage);
  logic_battery_voltage_pub_->publish(logic_battery_voltage_msg);

  std_msgs::msg::Float32 temp1_msg;
  temp1_msg.data = static_cast<float>(telemetry.temp1_c);
  temp1_pub_->publish(temp1_msg);

  std_msgs::msg::Float32 temp2_msg;
  temp2_msg.data = static_cast<float>(telemetry.temp2_c);
  temp2_pub_->publish(temp2_msg);

  diagnostic_msgs::msg::DiagnosticStatus status;
  status.name = "Roboclaw";
  status.hardware_id = "Roboclaw@" + device_ + "#" + std::to_string(address_);
  status.level = diagnostic_level_from_error_word(telemetry.error_word);
  status.message = diagnostic_message_from_error_word(telemetry.error_word);
  status.values.push_back(make_key_value("device", device_));
  status.values.push_back(make_key_value("address", std::to_string(address_)));
  status.values.push_back(
    make_key_value("main_battery_voltage_v", format_double(telemetry.main_battery_voltage)));
  status.values.push_back(
    make_key_value("logic_battery_voltage_v", format_double(telemetry.logic_battery_voltage)));
  status.values.push_back(make_key_value("m1_current_a", format_double(telemetry.m1_current)));
  status.values.push_back(make_key_value("m2_current_a", format_double(telemetry.m2_current)));
  status.values.push_back(make_key_value("temp1_c", format_double(telemetry.temp1_c)));
  status.values.push_back(make_key_value("temp2_c", format_double(telemetry.temp2_c)));
  {
    std::ostringstream stream;
    if (telemetry.error_word >= 0)
    {
      stream << "0x" << std::hex << std::uppercase << telemetry.error_word;
    }
    else
    {
      stream << "unavailable";
    }
    status.values.push_back(make_key_value("error_word", stream.str()));
  }

  diagnostic_msgs::msg::DiagnosticArray diagnostics;
  diagnostics.header.stamp = battery_msg.header.stamp;
  diagnostics.status.push_back(status);
  diagnostics_pub_->publish(diagnostics);

  if (telemetry.error_word != last_error_word_)
  {
    if (status.level >= diagnostic_msgs::msg::DiagnosticStatus::ERROR)
    {
      RCLCPP_ERROR(get_logger(), "Roboclaw status changed: %s", status.message.c_str());
    }
    else if (status.level == diagnostic_msgs::msg::DiagnosticStatus::WARN)
    {
      RCLCPP_WARN(get_logger(), "Roboclaw status changed: %s", status.message.c_str());
    }
    else
    {
      RCLCPP_INFO(get_logger(), "Roboclaw status changed: %s", status.message.c_str());
    }
    last_error_word_ = telemetry.error_word;
  }
}

void RoboclawHardwareInterface::reset_status_publish_state()
{
  has_published_status_ = false;
  last_status_publish_ = std::chrono::steady_clock::time_point{};
  last_error_word_ = -2;
}

}  // namespace roboclaw_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  roboclaw_hardware_interface::RoboclawHardwareInterface, hardware_interface::SystemInterface)
