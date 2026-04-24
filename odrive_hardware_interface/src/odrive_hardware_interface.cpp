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

#include "odrive_hardware_interface/odrive_hardware_interface.hpp"

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <cmath>
#include <csignal>
#include <sstream>
#include <stdexcept>
#include <string>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <pluginlib/class_list_macros.hpp>

#include "hardware_interface/types/hardware_interface_return_values.hpp"

namespace
{
constexpr double kTwoPi = 2.0 * M_PI;

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
}  // namespace

namespace odrive_hardware_interface
{

ODriveHardwareInterface::~ODriveHardwareInterface() { stop_backend(); }

hardware_interface::CallbackReturn ODriveHardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::signal(SIGPIPE, SIG_IGN);

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
        get_logger(), "Missing required hardware parameter '%s' for odrive_hardware_interface.",
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
    !parse_required_parameter(
      "python_executable", "a non-empty string", parse_non_empty_string, python_executable_) ||
    !parse_required_parameter(
      "front_serial_number", "a non-empty string", parse_non_empty_string, front_serial_number_) ||
    !parse_required_parameter(
      "rear_serial_number", "a non-empty string", parse_non_empty_string, rear_serial_number_) ||
    !parse_required_parameter("front_right_axis", "an integer", parse_int, front_right_axis_) ||
    !parse_required_parameter("rear_right_axis", "an integer", parse_int, rear_right_axis_) ||
    !parse_required_parameter(
      "connect_timeout", "a floating-point number", parse_double, connect_timeout_))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (
    (front_right_axis_ != 0 && front_right_axis_ != 1) ||
    (rear_right_axis_ != 0 && rear_right_axis_ != 1))
  {
    RCLCPP_ERROR(
      get_logger(),
      "odrive_hardware_interface parameters 'front_right_axis' and 'rear_right_axis' must be 0 or "
      "1.");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (connect_timeout_ <= 0.0)
  {
    RCLCPP_ERROR(
      get_logger(), "odrive_hardware_interface parameter 'connect_timeout' must be positive.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  reset_command_and_state_buffers();

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ODriveHardwareInterface::export_state_interfaces()
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
ODriveHardwareInterface::export_command_interfaces()
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

hardware_interface::CallbackReturn ODriveHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  reset_command_and_state_buffers();

  if (!start_backend())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
  std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);

  if (!read_state_from_backend())
  {
    stop_backend();
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ODriveHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!backend_running_)
  {
    RCLCPP_ERROR(get_logger(), "Cannot activate ODrive hardware before successful configuration.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
  std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);

  if (!expect_ok("ACTIVATE"))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!read_state_from_backend())
  {
    std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
    std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);
    if (backend_running_ && !expect_ok("DEACTIVATE"))
    {
      RCLCPP_ERROR(get_logger(), "Failed to deactivate ODrive hardware after activate failure.");
    }
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ODriveHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
  std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);

  if (backend_running_ && !expect_ok("DEACTIVATE"))
  {
    RCLCPP_ERROR(get_logger(), "Failed to deactivate ODrive hardware safely.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ODriveHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (backend_running_)
  {
    std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
    std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);
    if (!expect_ok("DEACTIVATE"))
    {
      RCLCPP_ERROR(get_logger(), "Failed to deactivate ODrive hardware during cleanup.");
    }
  }

  stop_backend();
  reset_runtime_state();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ODriveHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (backend_running_)
  {
    std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
    std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);
    if (!expect_ok("DEACTIVATE"))
    {
      RCLCPP_ERROR(get_logger(), "Failed to deactivate ODrive hardware during shutdown.");
    }
  }

  stop_backend();
  reset_runtime_state();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ODriveHardwareInterface::on_error(
  const rclcpp_lifecycle::State & previous_state)
{
  RCLCPP_ERROR(
    get_logger(), "ODrive hardware entering error handling from lifecycle state '%s'.",
    previous_state.label().c_str());

  if (backend_running_)
  {
    std::fill(hw_commands_.begin(), hw_commands_.end(), 0.0);
    std::fill(hw_velocities_.begin(), hw_velocities_.end(), 0.0);
    if (!expect_ok("DEACTIVATE"))
    {
      RCLCPP_ERROR(get_logger(), "Failed to deactivate ODrive hardware during error handling.");
    }
  }

  stop_backend();
  reset_runtime_state();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ODriveHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!backend_running_)
  {
    RCLCPP_ERROR(get_logger(), "ODrive read called while communication is not configured.");
    return hardware_interface::return_type::ERROR;
  }

  if (!can_read_in_current_state())
  {
    RCLCPP_ERROR(
      get_logger(), "ODrive read called in lifecycle state '%s', which is not readable.",
      get_lifecycle_state().label().c_str());
    return hardware_interface::return_type::ERROR;
  }

  return read_state_from_backend() ? hardware_interface::return_type::OK
                                   : hardware_interface::return_type::ERROR;
}

hardware_interface::return_type ODriveHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!backend_running_)
  {
    RCLCPP_ERROR(get_logger(), "ODrive write called while communication is not configured.");
    return hardware_interface::return_type::ERROR;
  }

  if (!is_active_lifecycle_state())
  {
    return hardware_interface::return_type::OK;
  }

  std::ostringstream command;
  command << "WRITE";
  for (double cmd_rad_s : hw_commands_)
  {
    command << ' ' << (cmd_rad_s / kTwoPi);
  }

  return expect_ok(command.str()) ? hardware_interface::return_type::OK
                                  : hardware_interface::return_type::ERROR;
}

bool ODriveHardwareInterface::validate_joint_configuration() const
{
  if (info_.joints.size() != 4)
  {
    RCLCPP_ERROR(
      get_logger(), "odrive_hardware_interface expects exactly 4 wheel joints, got %zu.",
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
        "Joint '%s' must expose exactly one '%s' command interface for ODrive hardware.",
        joint.name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return false;
    }

    bool has_position_state = false;
    bool has_velocity_state = false;
    for (const auto & state_interface : joint.state_interfaces)
    {
      if (state_interface.name == hardware_interface::HW_IF_POSITION)
      {
        has_position_state = true;
      }
      else if (state_interface.name == hardware_interface::HW_IF_VELOCITY)
      {
        has_velocity_state = true;
      }
    }

    if (joint.state_interfaces.size() != 2 || !has_position_state || !has_velocity_state)
    {
      RCLCPP_ERROR(
        get_logger(), "Joint '%s' must expose '%s' and '%s' state interfaces for ODrive hardware.",
        joint.name.c_str(), hardware_interface::HW_IF_POSITION, hardware_interface::HW_IF_VELOCITY);
      return false;
    }
  }

  return true;
}

void ODriveHardwareInterface::reset_command_and_state_buffers()
{
  hw_commands_.assign(info_.joints.size(), 0.0);
  hw_positions_.assign(info_.joints.size(), 0.0);
  hw_velocities_.assign(info_.joints.size(), 0.0);
}

void ODriveHardwareInterface::reset_runtime_state() { reset_command_and_state_buffers(); }

bool ODriveHardwareInterface::is_active_lifecycle_state() const
{
  return get_lifecycle_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
}

bool ODriveHardwareInterface::can_read_in_current_state() const
{
  const auto lifecycle_state = get_lifecycle_state().id();
  return lifecycle_state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE ||
         lifecycle_state == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
}

bool ODriveHardwareInterface::start_backend()
{
  if (backend_running_)
  {
    return true;
  }

  const auto share_dir = ament_index_cpp::get_package_share_directory("odrive_hardware_interface");
  const auto script_path = share_dir + "/python/odrive_backend.py";

  int child_stdin[2];
  int child_stdout[2];
  if (pipe(child_stdin) != 0 || pipe(child_stdout) != 0)
  {
    perror("pipe");
    return false;
  }

  backend_pid_ = fork();
  if (backend_pid_ < 0)
  {
    perror("fork");
    return false;
  }

  if (backend_pid_ == 0)
  {
    dup2(child_stdin[0], STDIN_FILENO);
    dup2(child_stdout[1], STDOUT_FILENO);
    close(child_stdin[1]);
    close(child_stdout[0]);

    execlp(
      python_executable_.c_str(), python_executable_.c_str(), "-u", script_path.c_str(),
      "--front-serial", front_serial_number_.c_str(), "--rear-serial", rear_serial_number_.c_str(),
      "--front-right-axis", std::to_string(front_right_axis_).c_str(), "--rear-right-axis",
      std::to_string(rear_right_axis_).c_str(), "--connect-timeout",
      std::to_string(connect_timeout_).c_str(), static_cast<char *>(nullptr));
    perror("execlp");
    _exit(127);
  }

  close(child_stdin[0]);
  close(child_stdout[1]);

  backend_in_ = fdopen(child_stdin[1], "w");
  backend_out_ = fdopen(child_stdout[0], "r");
  if (backend_in_ == nullptr || backend_out_ == nullptr)
  {
    perror("fdopen");
    stop_backend();
    return false;
  }

  backend_running_ = true;

  std::string response;
  if (!send_command("PING", response) || response != "PONG")
  {
    fprintf(stderr, "Failed to start ODrive backend, got response '%s'\n", response.c_str());
    stop_backend();
    return false;
  }

  return true;
}

void ODriveHardwareInterface::stop_backend()
{
  if (backend_running_)
  {
    int status = 0;
    const pid_t child_state = backend_pid_ > 0 ? waitpid(backend_pid_, &status, WNOHANG) : -1;
    if (child_state == 0)
    {
      std::string response;
      send_command("EXIT", response);
    }
  }

  if (backend_in_ != nullptr)
  {
    fclose(backend_in_);
    backend_in_ = nullptr;
  }
  if (backend_out_ != nullptr)
  {
    fclose(backend_out_);
    backend_out_ = nullptr;
  }
  if (backend_pid_ > 0)
  {
    int status = 0;
    waitpid(backend_pid_, &status, 0);
    backend_pid_ = -1;
  }
  backend_running_ = false;
}

bool ODriveHardwareInterface::send_command(const std::string & command, std::string & response)
{
  if (!backend_running_ || backend_in_ == nullptr || backend_out_ == nullptr)
  {
    return false;
  }

  if (fprintf(backend_in_, "%s\n", command.c_str()) < 0)
  {
    perror("fprintf");
    return false;
  }
  fflush(backend_in_);

  char buffer[1024];
  if (fgets(buffer, sizeof(buffer), backend_out_) == nullptr)
  {
    perror("fgets");
    return false;
  }

  response = buffer;
  while (!response.empty() && (response.back() == '\n' || response.back() == '\r'))
  {
    response.pop_back();
  }
  return true;
}

bool ODriveHardwareInterface::expect_ok(const std::string & command)
{
  std::string response;
  if (!send_command(command, response))
  {
    return false;
  }
  if (response != "OK")
  {
    fprintf(stderr, "Command '%s' failed with response '%s'\n", command.c_str(), response.c_str());
    return false;
  }
  return true;
}

bool ODriveHardwareInterface::read_state_from_backend()
{
  std::string response;
  if (!send_command("READ", response))
  {
    RCLCPP_ERROR(get_logger(), "Failed to read state from the ODrive backend.");
    return false;
  }
  if (!parse_state_response(response))
  {
    RCLCPP_ERROR(get_logger(), "Failed to parse ODrive backend state response.");
    return false;
  }
  return true;
}

bool ODriveHardwareInterface::parse_state_response(const std::string & response)
{
  std::istringstream stream(response);
  std::string token;
  stream >> token;
  if (token != "STATE")
  {
    RCLCPP_ERROR(get_logger(), "Unexpected ODrive backend state response: '%s'.", response.c_str());
    return false;
  }

  for (size_t i = 0; i < info_.joints.size(); ++i)
  {
    double rotations = 0.0;
    double rotations_per_second = 0.0;
    if (!(stream >> rotations >> rotations_per_second))
    {
      RCLCPP_ERROR(
        get_logger(), "Failed to parse ODrive backend state response: '%s'.", response.c_str());
      return false;
    }
    hw_positions_[i] = rotations * kTwoPi;
    hw_velocities_[i] = rotations_per_second * kTwoPi;
  }

  return true;
}

}  // namespace odrive_hardware_interface

PLUGINLIB_EXPORT_CLASS(
  odrive_hardware_interface::ODriveHardwareInterface, hardware_interface::SystemInterface)
