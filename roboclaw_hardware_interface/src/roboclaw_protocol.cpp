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

#include "roboclaw_hardware_interface/roboclaw_protocol.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <array>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <limits>
#include <sstream>

namespace
{

speed_t baud_rate_to_termios_speed(int baud)
{
  switch (baud)
  {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    default:
      return 0;
  }
}

namespace cmd
{
constexpr uint8_t M1_FORWARD = 0;
constexpr uint8_t M2_FORWARD = 4;
constexpr uint8_t GET_M1_ENC = 16;
constexpr uint8_t GET_M2_ENC = 17;
constexpr uint8_t RESET_ENC = 20;
constexpr uint8_t GET_VERSION = 21;
constexpr uint8_t GET_MAIN_BATTERY = 24;
constexpr uint8_t GET_LOGIC_BATTERY = 25;
constexpr uint8_t MIXED_SPEED = 37;
constexpr uint8_t GET_CURRENTS = 49;
constexpr uint8_t M1_DUTY_ACCEL = 52;
constexpr uint8_t M2_DUTY_ACCEL = 53;
constexpr uint8_t GET_TEMP = 82;
constexpr uint8_t GET_TEMP2 = 83;
constexpr uint8_t GET_ERROR = 90;
}  // namespace cmd

}  // namespace

namespace roboclaw_hardware_interface
{

bool RoboclawProtocol::connect(const std::string & device, int baud, int address)
{
  device_ = device;
  baud_ = baud;
  address_ = address;
  last_error_.clear();

  if (!open_device())
  {
    return false;
  }

  if (!configure_serial())
  {
    disconnect();
    return false;
  }

  return true;
}

void RoboclawProtocol::disconnect()
{
  if (fd_ >= 0)
  {
    close(fd_);
    fd_ = -1;
  }
}

bool RoboclawProtocol::is_connected() const { return fd_ >= 0; }

bool RoboclawProtocol::ping()
{
  for (int tries = 0; tries < 3; ++tries)
  {
    if (!flush_serial_io() || !send_packet_command(cmd::GET_VERSION))
    {
      continue;
    }

    bool passed = true;
    for (int i = 0; i < 48; ++i)
    {
      uint8_t value = 0;
      if (!read_byte(value))
      {
        passed = false;
        break;
      }
      if (value == 0)
      {
        break;
      }
    }

    if (passed && read_checksum())
    {
      return true;
    }
  }

  last_error_ = "Connected to Roboclaw but could not read controller version.";
  return false;
}

bool RoboclawProtocol::reset_encoders() { return write_no_arg_command(cmd::RESET_ENC); }

bool RoboclawProtocol::stop(bool use_encoder)
{
  if (use_encoder)
  {
    return write_byte_command(cmd::M1_FORWARD, 0) && write_byte_command(cmd::M2_FORWARD, 0);
  }

  return write_signed_word_uint32_command(cmd::M1_DUTY_ACCEL, 0, 0) &&
         write_signed_word_uint32_command(cmd::M2_DUTY_ACCEL, 0, 0);
}

bool RoboclawProtocol::drive_open_loop(int16_t m1_duty, int16_t m2_duty, uint32_t acceleration)
{
  return write_signed_word_uint32_command(cmd::M1_DUTY_ACCEL, m1_duty, acceleration) &&
         write_signed_word_uint32_command(cmd::M2_DUTY_ACCEL, m2_duty, acceleration);
}

bool RoboclawProtocol::drive_closed_loop(int32_t m1_speed, int32_t m2_speed)
{
  return write_signed_long_pair_command(cmd::MIXED_SPEED, m1_speed, m2_speed);
}

bool RoboclawProtocol::read_encoders(int32_t & m1_ticks, int32_t & m2_ticks)
{
  return read_encoder_command(cmd::GET_M1_ENC, m1_ticks) &&
         read_encoder_command(cmd::GET_M2_ENC, m2_ticks);
}

bool RoboclawProtocol::read_status(RoboclawTelemetry & telemetry)
{
  uint16_t main_battery = 0;
  uint16_t logic_battery = 0;
  uint32_t currents = 0;
  uint16_t temp1 = 0;
  uint16_t temp2 = 0;
  uint16_t error_word = 0;

  telemetry.main_battery_voltage = read_uint16_command(cmd::GET_MAIN_BATTERY, main_battery)
                                     ? main_battery / 10.0
                                     : std::numeric_limits<double>::quiet_NaN();
  telemetry.logic_battery_voltage = read_uint16_command(cmd::GET_LOGIC_BATTERY, logic_battery)
                                      ? logic_battery / 10.0
                                      : std::numeric_limits<double>::quiet_NaN();

  if (read_uint32_command(cmd::GET_CURRENTS, currents))
  {
    auto m1_current = static_cast<int16_t>((currents >> 16) & 0xFFFF);
    auto m2_current = static_cast<int16_t>(currents & 0xFFFF);
    telemetry.m1_current = static_cast<double>(m1_current) / 100.0;
    telemetry.m2_current = static_cast<double>(m2_current) / 100.0;
  }
  else
  {
    telemetry.m1_current = std::numeric_limits<double>::quiet_NaN();
    telemetry.m2_current = std::numeric_limits<double>::quiet_NaN();
  }

  telemetry.temp1_c = read_uint16_command(cmd::GET_TEMP, temp1)
                        ? temp1 / 10.0
                        : std::numeric_limits<double>::quiet_NaN();
  telemetry.temp2_c = read_uint16_command(cmd::GET_TEMP2, temp2)
                        ? temp2 / 10.0
                        : std::numeric_limits<double>::quiet_NaN();
  telemetry.error_word = read_uint16_command(cmd::GET_ERROR, error_word) ? error_word : -1;

  return true;
}

const std::string & RoboclawProtocol::last_error() const { return last_error_; }

bool RoboclawProtocol::open_device()
{
  fd_ = open(device_.c_str(), O_RDWR | O_NOCTTY);
  if (fd_ < 0)
  {
    std::ostringstream stream;
    stream << "Failed to open Roboclaw device '" << device_ << "': " << std::strerror(errno);
    last_error_ = stream.str();
    return false;
  }
  return true;
}

bool RoboclawProtocol::configure_serial()
{
  const speed_t speed = baud_rate_to_termios_speed(baud_);
  if (speed == 0)
  {
    std::ostringstream stream;
    stream << "Unsupported Roboclaw baud rate " << baud_ << '.';
    last_error_ = stream.str();
    return false;
  }

  struct termios options;
  if (tcgetattr(fd_, &options) < 0)
  {
    last_error_ = "Failed to get serial attributes for Roboclaw.";
    return false;
  }

  cfmakeraw(&options);
  cfsetispeed(&options, speed);
  cfsetospeed(&options, speed);
  options.c_cflag |= CLOCAL | CREAD;
  options.c_cflag &= ~CRTSCTS;
  options.c_cc[VMIN] = 0;
  options.c_cc[VTIME] = 1;

  if (tcsetattr(fd_, TCSANOW, &options) < 0)
  {
    last_error_ = "Failed to set serial attributes for Roboclaw.";
    return false;
  }

  return flush_serial_io();
}

bool RoboclawProtocol::flush_serial_io()
{
  if (tcflush(fd_, TCIOFLUSH) < 0)
  {
    last_error_ = "Failed to flush Roboclaw serial buffers.";
    return false;
  }
  return true;
}

bool RoboclawProtocol::write_all(const uint8_t * data, size_t size)
{
  size_t written = 0;
  while (written < size)
  {
    const ssize_t result = write(fd_, data + written, size - written);
    if (result < 0)
    {
      if (errno == EINTR)
      {
        continue;
      }

      std::ostringstream stream;
      stream << "Failed to write Roboclaw serial data: " << std::strerror(errno);
      last_error_ = stream.str();
      return false;
    }

    if (result == 0)
    {
      last_error_ = "Roboclaw serial write returned zero bytes.";
      return false;
    }

    written += static_cast<size_t>(result);
  }

  return true;
}

bool RoboclawProtocol::read_exact(uint8_t * data, size_t size)
{
  size_t read_count = 0;
  while (read_count < size)
  {
    const ssize_t result = read(fd_, data + read_count, size - read_count);
    if (result < 0)
    {
      if (errno == EINTR)
      {
        continue;
      }

      std::ostringstream stream;
      stream << "Failed to read Roboclaw serial data: " << std::strerror(errno);
      last_error_ = stream.str();
      return false;
    }

    if (result == 0)
    {
      last_error_ = "Roboclaw serial read returned zero bytes.";
      return false;
    }

    read_count += static_cast<size_t>(result);
  }

  return true;
}

void RoboclawProtocol::crc_clear() { crc_ = 0; }

void RoboclawProtocol::crc_update(uint8_t data)
{
  crc_ ^= static_cast<uint16_t>(data) << 8;
  for (int bit = 0; bit < 8; ++bit)
  {
    if ((crc_ & 0x8000) != 0)
    {
      crc_ = static_cast<uint16_t>((crc_ << 1) ^ 0x1021);
    }
    else
    {
      crc_ = static_cast<uint16_t>(crc_ << 1);
    }
  }
}

bool RoboclawProtocol::send_packet_command(uint8_t command)
{
  const std::array<uint8_t, 2> data = {static_cast<uint8_t>(address_), command};
  crc_clear();
  crc_update(data[0]);
  crc_update(data[1]);
  return write_all(data.data(), data.size());
}

bool RoboclawProtocol::write_byte(uint8_t value)
{
  crc_update(value);
  return write_all(&value, 1);
}

bool RoboclawProtocol::write_word(uint16_t value)
{
  return write_byte(static_cast<uint8_t>((value >> 8) & 0xFF)) &&
         write_byte(static_cast<uint8_t>(value & 0xFF));
}

bool RoboclawProtocol::write_long(uint32_t value)
{
  return write_byte(static_cast<uint8_t>((value >> 24) & 0xFF)) &&
         write_byte(static_cast<uint8_t>((value >> 16) & 0xFF)) &&
         write_byte(static_cast<uint8_t>((value >> 8) & 0xFF)) &&
         write_byte(static_cast<uint8_t>(value & 0xFF));
}

bool RoboclawProtocol::write_signed_word(int16_t value)
{
  return write_word(static_cast<uint16_t>(value));
}

bool RoboclawProtocol::write_signed_long(int32_t value)
{
  return write_long(static_cast<uint32_t>(value));
}

bool RoboclawProtocol::write_checksum_and_ack()
{
  if (!write_word(crc_))
  {
    return false;
  }

  uint8_t ack = 0;
  return read_exact(&ack, 1);
}

bool RoboclawProtocol::read_byte(uint8_t & value)
{
  if (!read_exact(&value, 1))
  {
    return false;
  }
  crc_update(value);
  return true;
}

bool RoboclawProtocol::read_word(uint16_t & value)
{
  uint8_t high = 0;
  uint8_t low = 0;
  if (!read_byte(high) || !read_byte(low))
  {
    return false;
  }
  value = static_cast<uint16_t>((high << 8) | low);
  return true;
}

bool RoboclawProtocol::read_long(uint32_t & value)
{
  uint8_t byte1 = 0;
  uint8_t byte2 = 0;
  uint8_t byte3 = 0;
  uint8_t byte4 = 0;
  if (!read_byte(byte1) || !read_byte(byte2) || !read_byte(byte3) || !read_byte(byte4))
  {
    return false;
  }

  value = (static_cast<uint32_t>(byte1) << 24) | (static_cast<uint32_t>(byte2) << 16) |
          (static_cast<uint32_t>(byte3) << 8) | static_cast<uint32_t>(byte4);
  return true;
}

bool RoboclawProtocol::read_signed_long(int32_t & value)
{
  uint32_t raw_value = 0;
  if (!read_long(raw_value))
  {
    return false;
  }
  value = static_cast<int32_t>(raw_value);
  return true;
}

bool RoboclawProtocol::read_checksum()
{
  uint8_t high = 0;
  uint8_t low = 0;
  if (!read_exact(&high, 1) || !read_exact(&low, 1))
  {
    return false;
  }

  const uint16_t received_crc = static_cast<uint16_t>((high << 8) | low);
  if ((crc_ & 0xFFFF) != received_crc)
  {
    std::ostringstream stream;
    stream << "Roboclaw CRC mismatch. Expected " << (crc_ & 0xFFFF) << ", got " << received_crc
           << '.';
    last_error_ = stream.str();
    return false;
  }

  return true;
}

bool RoboclawProtocol::read_uint16_command(uint8_t command, uint16_t & value)
{
  for (int tries = 0; tries < 3; ++tries)
  {
    if (flush_serial_io() && send_packet_command(command) && read_word(value) && read_checksum())
    {
      return true;
    }
  }
  return false;
}

bool RoboclawProtocol::read_uint32_command(uint8_t command, uint32_t & value)
{
  for (int tries = 0; tries < 3; ++tries)
  {
    if (flush_serial_io() && send_packet_command(command) && read_long(value) && read_checksum())
    {
      return true;
    }
  }
  return false;
}

bool RoboclawProtocol::read_encoder_command(uint8_t command, int32_t & ticks)
{
  for (int tries = 0; tries < 3; ++tries)
  {
    uint8_t status = 0;
    if (
      flush_serial_io() && send_packet_command(command) && read_signed_long(ticks) &&
      read_byte(status) && read_checksum())
    {
      return true;
    }
  }
  return false;
}

bool RoboclawProtocol::write_no_arg_command(uint8_t command)
{
  for (int tries = 0; tries < 3; ++tries)
  {
    if (flush_serial_io() && send_packet_command(command) && write_checksum_and_ack())
    {
      return true;
    }
  }
  return false;
}

bool RoboclawProtocol::write_byte_command(uint8_t command, uint8_t value)
{
  for (int tries = 0; tries < 3; ++tries)
  {
    if (
      flush_serial_io() && send_packet_command(command) && write_byte(value) &&
      write_checksum_and_ack())
    {
      return true;
    }
  }
  return false;
}

bool RoboclawProtocol::write_signed_word_uint32_command(
  uint8_t command, int16_t value1, uint32_t value2)
{
  for (int tries = 0; tries < 3; ++tries)
  {
    if (
      flush_serial_io() && send_packet_command(command) && write_signed_word(value1) &&
      write_long(value2) && write_checksum_and_ack())
    {
      return true;
    }
  }
  return false;
}

bool RoboclawProtocol::write_signed_long_pair_command(
  uint8_t command, int32_t value1, int32_t value2)
{
  for (int tries = 0; tries < 3; ++tries)
  {
    if (
      flush_serial_io() && send_packet_command(command) && write_signed_long(value1) &&
      write_signed_long(value2) && write_checksum_and_ack())
    {
      return true;
    }
  }
  return false;
}

}  // namespace roboclaw_hardware_interface
