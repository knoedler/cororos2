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

#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace pwm_hardware_interface
{

class PwmHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(PwmHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  int16_t speed_to_pwm(double wheel_velocity_rad_s, bool invert) const;

  bool start_backend();
  void stop_backend();

  bool open_maestro();
  void close_maestro();
  bool configure_maestro_serial();
  bool write_maestro_target(uint8_t channel, uint16_t target);
  bool write_neutral();

  std::string device_path_;
  int maestro_fd_{-1};
  bool backend_running_{false};
  std::array<uint8_t, 4> channels_{0, 1, 2, 3};

  int pwm_min_{1000};
  int pwm_neutral_{1500};
  int pwm_max_{2000};
  int min_active_pwm_delta_{0};
  double wheel_radius_{0.205};
  double max_wheel_speed_mps_{2.8};
  bool invert_left_{false};
  bool invert_right_{false};

  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;

  std::array<int16_t, 4> last_pwm_{
    {static_cast<int16_t>(0), static_cast<int16_t>(0), static_cast<int16_t>(0),
     static_cast<int16_t>(0)}};
};

}  // namespace pwm_hardware_interface
