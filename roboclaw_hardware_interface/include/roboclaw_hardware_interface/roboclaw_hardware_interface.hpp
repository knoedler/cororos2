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

#include <chrono>
#include <optional>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "hardware_interface/system_interface.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "roboclaw_hardware_interface/roboclaw_protocol.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/float32.hpp"

namespace roboclaw_hardware_interface
{

class RoboclawHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(RoboclawHardwareInterface)
  ~RoboclawHardwareInterface() override;

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
  bool validate_joint_configuration() const;
  bool start_backend();
  void stop_backend();
  bool open_roboclaw();
  void close_roboclaw();
  bool configure_roboclaw_serial();
  bool ping_roboclaw();
  bool activate_backend();
  bool deactivate_backend();
  bool drive_backend(double left_mps, double right_mps);
  bool stop_motors();
  bool read_state_from_backend();
  bool read_status_from_backend(RoboclawTelemetry & telemetry);
  bool ensure_publishers();
  void release_publishers();
  void reset_command_and_state_buffers();
  void reset_runtime_state();
  bool is_active_lifecycle_state() const;
  bool can_read_in_current_state() const;
  bool poll_and_publish_status();
  void publish_status(const RoboclawTelemetry & telemetry);
  void reset_status_publish_state();

  std::string device_;
  int baud_{115200};
  int address_{128};
  bool use_encoder_{false};
  double max_speed_{1.2};
  double ticks_at_max_speed_{32760.0};
  int acceleration_{32000};
  double ticks_per_meter_{4342.2};
  double wheel_radius_{0.13};
  double status_interval_sec_{2.0};

  std::vector<double> hw_commands_;
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;

  bool backend_running_{false};
  RoboclawProtocol protocol_;
  std::optional<int32_t> last_left_ticks_;
  std::optional<int32_t> last_right_ticks_;
  std::chrono::steady_clock::time_point last_encoder_read_time_{};

  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr logic_battery_voltage_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m1_current_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr m2_current_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr temp1_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr temp2_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  std::chrono::steady_clock::time_point last_status_publish_{};
  bool has_published_status_{false};
  int last_error_word_{-2};
};

}  // namespace roboclaw_hardware_interface
