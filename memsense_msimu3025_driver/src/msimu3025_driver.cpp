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

#include <fcntl.h>
#include <linux/serial.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "sensor_msgs/msg/temperature.hpp"
#include "std_msgs/msg/float64.hpp"

namespace
{
constexpr int kPacketBytes = 54;
constexpr int kPacketDataBytes = kPacketBytes - 6;
constexpr size_t kReadChunkBytes = 256;
constexpr speed_t kBaudRate = B460800;

bool fletcher_valid(const std::array<uint8_t, kPacketBytes> & msg)
{
  int f1 = 0;
  int f2 = 0;
  for (int i = 0; i < (kPacketBytes - 2); ++i)
  {
    f1 += msg[static_cast<size_t>(i)];
    f2 += f1;
  }
  f1 %= 256;
  f2 %= 256;
  return f1 == msg[kPacketBytes - 2] && f2 == msg[kPacketBytes - 1];
}

float read_be_float(const std::array<uint8_t, kPacketBytes> & msg, size_t offset)
{
  const uint32_t raw =
    (static_cast<uint32_t>(msg[offset]) << 24) | (static_cast<uint32_t>(msg[offset + 1]) << 16) |
    (static_cast<uint32_t>(msg[offset + 2]) << 8) | static_cast<uint32_t>(msg[offset + 3]);
  float value = 0.0F;
  std::memcpy(&value, &raw, sizeof(float));
  return value;
}

bool has_packet_header(const std::vector<uint8_t> & buffer)
{
  return buffer.size() >= kPacketBytes && buffer[0] == 0xA5 && buffer[1] == 0xA5 &&
         buffer[2] == 0xA2 && buffer[3] == kPacketDataBytes;
}
}  // namespace

class Msimu3025Driver : public rclcpp::Node
{
public:
  Msimu3025Driver() : Node("msimu3025_driver"), tty_fd_(-1)
  {
    serial_path_ = declare_parameter<std::string>(
      "serial_path_msimu3025",
      "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0");
    frame_id_ = declare_parameter<std::string>("frame_id", "imu_sensor_link");
    imu_topic_ = declare_parameter<std::string>("imu_topic", "/msimu3025_raw");
    magnetic_topic_ = declare_parameter<std::string>("magnetic_topic", "/msimu3025_magnetic");
    temperature_topic_ =
      declare_parameter<std::string>("temperature_topic", "/msimu3025_temperature");
    gyro_bias_topic_ = declare_parameter<std::string>("gyro_bias_topic", "/msimu3025_gyro_z_bias");
    enable_gyro_z_bias_correction_ = declare_parameter<bool>("enable_gyro_z_bias_correction", true);
    motion_command_topic_ =
      declare_parameter<std::string>("motion_command_topic", "/diff_drive_controller/cmd_vel");
    motion_command_timeout_s_ = declare_parameter<double>("motion_command_timeout_s", 13.0);
    gyro_bias_recalibration_period_s_ =
      declare_parameter<double>("gyro_bias_recalibration_period_s", 16.0);
    gyro_bias_window_size_ = static_cast<int>(
      std::max<int64_t>(1, declare_parameter<int64_t>("gyro_bias_window_size", 2000)));
    gyro_bias_stationary_max_z_rate_rad_s_ =
      declare_parameter<double>("gyro_bias_stationary_max_z_rate_rad_s", 0.01);
    gyro_bias_stationary_max_sum_rad_s_ =
      declare_parameter<double>("gyro_bias_stationary_max_sum_rad_s", 2.0);
    motion_command_linear_threshold_ =
      declare_parameter<double>("motion_command_linear_threshold", 0.01);
    motion_command_angular_threshold_ =
      declare_parameter<double>("motion_command_angular_threshold", 0.01);

    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>(imu_topic_, 10);
    magnetic_pub_ = create_publisher<sensor_msgs::msg::MagneticField>(magnetic_topic_, 10);
    temperature_pub_ = create_publisher<sensor_msgs::msg::Temperature>(temperature_topic_, 10);
    gyro_bias_pub_ = create_publisher<std_msgs::msg::Float64>(gyro_bias_topic_, 10);

    rx_buffer_.reserve(kPacketBytes * 4);
    gyro_z_samples_.assign(static_cast<size_t>(gyro_bias_window_size_), 0.0);
    const auto startup_time_s = now().seconds();
    last_motion_time_s_ = startup_time_s;
    last_gyro_bias_calibration_time_s_ = startup_time_s;

    if (enable_gyro_z_bias_correction_)
    {
      motion_command_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
        motion_command_topic_, 10,
        std::bind(&Msimu3025Driver::motion_command_callback, this, std::placeholders::_1));
    }

    initialize_messages();

    reconnect_timer_ = create_wall_timer(
      std::chrono::seconds(1), std::bind(&Msimu3025Driver::ensure_serial_connection, this));
    read_timer_ = create_wall_timer(
      std::chrono::milliseconds(2), std::bind(&Msimu3025Driver::read_serial, this));
  }

  ~Msimu3025Driver() override { close_serial(); }

private:
  void initialize_messages()
  {
    imu_msg_.orientation.w = 1.0;
    imu_msg_.orientation_covariance[0] = -1.0;
    imu_msg_.header.frame_id = frame_id_;
    imu_msg_.angular_velocity_covariance[0] = 0.62 * 2.0 * M_PI / 360.0 / 3600.0;
    imu_msg_.angular_velocity_covariance[4] = 0.56 * 2.0 * M_PI / 360.0 / 3600.0;
    imu_msg_.angular_velocity_covariance[8] = 0.80 * 2.0 * M_PI / 360.0 / 3600.0;
    imu_msg_.linear_acceleration_covariance[0] = 2.6 * (1e-6) * 9.80665;
    imu_msg_.linear_acceleration_covariance[4] = 2.6 * (1e-6) * 9.80665;
    imu_msg_.linear_acceleration_covariance[8] = 6.7 * (1e-6) * 9.80665;

    magnetic_msg_.header.frame_id = frame_id_;
    magnetic_msg_.magnetic_field_covariance[0] = 0.0016 / 10000.0;
    magnetic_msg_.magnetic_field_covariance[4] = 0.0016 / 10000.0;
    magnetic_msg_.magnetic_field_covariance[8] = 0.0016 / 10000.0;

    temperature_msg_.header.frame_id = frame_id_;
    temperature_msg_.variance = 1.5;
  }

  void ensure_serial_connection()
  {
    if (tty_fd_ >= 0)
    {
      return;
    }

    tty_fd_ = open(serial_path_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (tty_fd_ < 0)
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000, "Failed to open Memsense serial port '%s': %s",
        serial_path_.c_str(), std::strerror(errno));
      return;
    }

    struct serial_struct serial_cfg;
    if (ioctl(tty_fd_, TIOCGSERIAL, &serial_cfg) == 0)
    {
      serial_cfg.flags |= ASYNC_LOW_LATENCY;
      ioctl(tty_fd_, TIOCSSERIAL, &serial_cfg);
    }

    struct termios tio;
    std::memset(&tio, 0, sizeof(tio));
    tio.c_iflag = 0;
    tio.c_oflag = 0;
    tio.c_cflag = CS8 | CREAD | CLOCAL;
    tio.c_lflag = 0;
    tio.c_cc[VMIN] = 0;
    tio.c_cc[VTIME] = 0;
    cfsetospeed(&tio, kBaudRate);
    cfsetispeed(&tio, kBaudRate);
    if (tcsetattr(tty_fd_, TCSANOW, &tio) != 0)
    {
      RCLCPP_ERROR(
        get_logger(), "Failed to configure Memsense serial port: %s", std::strerror(errno));
      close_serial();
      return;
    }

    tcflush(tty_fd_, TCIFLUSH);
    rx_buffer_.clear();
    RCLCPP_INFO(get_logger(), "Opened Memsense serial port: %s", serial_path_.c_str());
  }

  void close_serial()
  {
    if (tty_fd_ >= 0)
    {
      close(tty_fd_);
      tty_fd_ = -1;
    }
    rx_buffer_.clear();
  }

  void read_serial()
  {
    if (tty_fd_ < 0)
    {
      return;
    }

    std::array<uint8_t, kReadChunkBytes> read_chunk{};
    while (tty_fd_ >= 0)
    {
      const auto read_count = ::read(tty_fd_, read_chunk.data(), read_chunk.size());
      if (read_count > 0)
      {
        rx_buffer_.insert(
          rx_buffer_.end(), read_chunk.begin(),
          read_chunk.begin() + static_cast<size_t>(read_count));
        process_rx_buffer();
        continue;
      }

      if (read_count == 0 || errno == EAGAIN || errno == EWOULDBLOCK)
      {
        return;
      }

      RCLCPP_ERROR(get_logger(), "Read failed on Memsense serial port: %s", std::strerror(errno));
      close_serial();
      return;
    }
  }

  void process_rx_buffer()
  {
    while (rx_buffer_.size() >= kPacketBytes)
    {
      if (!has_packet_header(rx_buffer_))
      {
        rx_buffer_.erase(rx_buffer_.begin());
        continue;
      }

      std::array<uint8_t, kPacketBytes> packet{};
      std::copy_n(rx_buffer_.begin(), kPacketBytes, packet.begin());
      if (!fletcher_valid(packet))
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Invalid Memsense Fletcher checksum while resynchronizing");
        rx_buffer_.erase(rx_buffer_.begin());
        continue;
      }

      parse_packet(packet);
      rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + kPacketBytes);
    }
  }

  void motion_command_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    const auto & twist = msg->twist;
    if (
      std::fabs(twist.linear.x) > motion_command_linear_threshold_ ||
      std::fabs(twist.linear.y) > motion_command_linear_threshold_ ||
      std::fabs(twist.linear.z) > motion_command_linear_threshold_ ||
      std::fabs(twist.angular.x) > motion_command_angular_threshold_ ||
      std::fabs(twist.angular.y) > motion_command_angular_threshold_ ||
      std::fabs(twist.angular.z) > motion_command_angular_threshold_)
    {
      last_motion_time_s_ = now().seconds();
    }
  }

  void apply_gyro_z_bias_correction(double time_now_s)
  {
    if (!enable_gyro_z_bias_correction_ || gyro_z_samples_.empty())
    {
      return;
    }

    gyro_z_samples_[gyro_z_sample_index_] = imu_msg_.angular_velocity.z;
    gyro_z_sample_index_ = (gyro_z_sample_index_ + 1) % gyro_z_samples_.size();
    if (gyro_z_sample_index_ == 0)
    {
      gyro_z_samples_full_ = true;
    }

    if (!gyro_z_samples_full_)
    {
      imu_msg_.angular_velocity.z -= gyro_z_bias_rad_s_;
      return;
    }

    if ((time_now_s - last_gyro_bias_calibration_time_s_) > gyro_bias_recalibration_period_s_)
    {
      last_gyro_bias_calibration_time_s_ = time_now_s;

      if ((time_now_s - last_motion_time_s_) > motion_command_timeout_s_)
      {
        double z_sum = 0.0;
        double z_max = 0.0;
        for (const double sample : gyro_z_samples_)
        {
          z_sum += sample;
          z_max = std::max(z_max, std::fabs(sample));
        }

        if (
          z_max < gyro_bias_stationary_max_z_rate_rad_s_ &&
          std::fabs(z_sum) < gyro_bias_stationary_max_sum_rad_s_)
        {
          gyro_z_bias_rad_s_ = z_sum / static_cast<double>(gyro_z_samples_.size());
          RCLCPP_INFO(
            get_logger(), "Updated Memsense gyro z-bias estimate to %.6f rad/s",
            gyro_z_bias_rad_s_);
        }
      }
    }

    imu_msg_.angular_velocity.z -= gyro_z_bias_rad_s_;
  }

  void parse_packet(const std::array<uint8_t, kPacketBytes> & packet)
  {
    for (size_t i = 4; (i + 1) < static_cast<size_t>(kPacketBytes - 2);)
    {
      const auto field_id = packet[i];
      const auto field_len = packet[i + 1];
      const auto next_field = i + static_cast<size_t>(field_len) + 2;
      if (next_field > static_cast<size_t>(kPacketBytes - 2))
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Discarding malformed Memsense packet with invalid field length");
        return;
      }

      switch (field_id)
      {
        case 0x81:
          if (field_len == 0x0c)
          {
            imu_msg_.linear_acceleration.x =
              9.80665 * read_be_float(packet, static_cast<size_t>(i + 2));
            imu_msg_.linear_acceleration.y =
              9.80665 * read_be_float(packet, static_cast<size_t>(i + 6));
            imu_msg_.linear_acceleration.z =
              9.80665 * read_be_float(packet, static_cast<size_t>(i + 10));
          }
          break;
        case 0x82:
          if (field_len == 0x0c)
          {
            imu_msg_.angular_velocity.x =
              2.0 * M_PI * read_be_float(packet, static_cast<size_t>(i + 2)) / 360.0;
            imu_msg_.angular_velocity.y =
              2.0 * M_PI * read_be_float(packet, static_cast<size_t>(i + 6)) / 360.0;
            imu_msg_.angular_velocity.z =
              2.0 * M_PI * read_be_float(packet, static_cast<size_t>(i + 10)) / 360.0;
          }
          break;
        case 0x83:
          if (field_len == 0x0c)
          {
            magnetic_msg_.magnetic_field.x =
              read_be_float(packet, static_cast<size_t>(i + 2)) / 10000.0;
            magnetic_msg_.magnetic_field.y =
              read_be_float(packet, static_cast<size_t>(i + 6)) / 10000.0;
            magnetic_msg_.magnetic_field.z =
              read_be_float(packet, static_cast<size_t>(i + 10)) / 10000.0;
          }
          break;
        case 0x87:
          if (field_len == 0x04)
          {
            temperature_msg_.temperature = read_be_float(packet, static_cast<size_t>(i + 2));
          }
          break;
        default:
          break;
      }

      i = next_field;
    }

    if (
      std::fabs(imu_msg_.angular_velocity.x) >= 50.0 ||
      std::fabs(imu_msg_.angular_velocity.y) >= 50.0 ||
      std::fabs(imu_msg_.angular_velocity.z) >= 50.0 ||
      std::fabs(imu_msg_.linear_acceleration.x) >= 500.0 ||
      std::fabs(imu_msg_.linear_acceleration.y) >= 500.0 ||
      std::fabs(imu_msg_.linear_acceleration.z) >= 500.0)
    {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000, "Discarding implausible Memsense packet");
      return;
    }

    const auto stamp = now();
    const auto time_now_s = stamp.seconds();
    apply_gyro_z_bias_correction(time_now_s);

    imu_msg_.header.stamp = stamp;
    magnetic_msg_.header.stamp = stamp;
    temperature_msg_.header.stamp = stamp;
    gyro_bias_msg_.data = gyro_z_bias_rad_s_;

    imu_pub_->publish(imu_msg_);
    magnetic_pub_->publish(magnetic_msg_);
    temperature_pub_->publish(temperature_msg_);
    gyro_bias_pub_->publish(gyro_bias_msg_);
  }

  std::string serial_path_;
  std::string frame_id_;
  std::string imu_topic_;
  std::string magnetic_topic_;
  std::string temperature_topic_;
  std::string gyro_bias_topic_;
  bool enable_gyro_z_bias_correction_;
  std::string motion_command_topic_;
  double motion_command_timeout_s_;
  double gyro_bias_recalibration_period_s_;
  int gyro_bias_window_size_;
  double gyro_bias_stationary_max_z_rate_rad_s_;
  double gyro_bias_stationary_max_sum_rad_s_;
  double motion_command_linear_threshold_;
  double motion_command_angular_threshold_;

  int tty_fd_;
  std::vector<uint8_t> rx_buffer_;
  std::vector<double> gyro_z_samples_;
  size_t gyro_z_sample_index_{0};
  bool gyro_z_samples_full_{false};
  double gyro_z_bias_rad_s_{0.0};
  double last_motion_time_s_{0.0};
  double last_gyro_bias_calibration_time_s_{0.0};

  sensor_msgs::msg::Imu imu_msg_;
  sensor_msgs::msg::MagneticField magnetic_msg_;
  sensor_msgs::msg::Temperature temperature_msg_;
  std_msgs::msg::Float64 gyro_bias_msg_;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr motion_command_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr magnetic_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Temperature>::SharedPtr temperature_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr gyro_bias_pub_;

  rclcpp::TimerBase::SharedPtr reconnect_timer_;
  rclcpp::TimerBase::SharedPtr read_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Msimu3025Driver>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
