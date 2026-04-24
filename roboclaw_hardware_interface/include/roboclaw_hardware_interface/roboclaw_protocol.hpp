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

#include <cstdint>
#include <string>

namespace roboclaw_hardware_interface
{

struct RoboclawTelemetry
{
  double main_battery_voltage{0.0};
  double logic_battery_voltage{0.0};
  double m1_current{0.0};
  double m2_current{0.0};
  double temp1_c{0.0};
  double temp2_c{0.0};
  int error_word{-1};
};

class RoboclawProtocol
{
public:
  bool connect(const std::string & device, int baud, int address);
  void disconnect();
  bool is_connected() const;

  bool ping();
  bool reset_encoders();
  bool stop(bool use_encoder);
  bool drive_open_loop(int16_t m1_duty, int16_t m2_duty, uint32_t acceleration);
  bool drive_closed_loop(int32_t m1_speed, int32_t m2_speed);
  bool read_encoders(int32_t & m1_ticks, int32_t & m2_ticks);
  bool read_status(RoboclawTelemetry & telemetry);

  const std::string & last_error() const;

private:
  bool open_device();
  bool configure_serial();
  bool flush_serial_io();
  bool write_all(const uint8_t * data, size_t size);
  bool read_exact(uint8_t * data, size_t size);
  void crc_clear();
  void crc_update(uint8_t data);
  bool send_packet_command(uint8_t command);
  bool write_byte(uint8_t value);
  bool write_word(uint16_t value);
  bool write_long(uint32_t value);
  bool write_signed_word(int16_t value);
  bool write_signed_long(int32_t value);
  bool write_checksum_and_ack();
  bool read_byte(uint8_t & value);
  bool read_word(uint16_t & value);
  bool read_long(uint32_t & value);
  bool read_signed_long(int32_t & value);
  bool read_checksum();
  bool read_uint16_command(uint8_t command, uint16_t & value);
  bool read_uint32_command(uint8_t command, uint32_t & value);
  bool read_encoder_command(uint8_t command, int32_t & ticks);
  bool write_no_arg_command(uint8_t command);
  bool write_byte_command(uint8_t command, uint8_t value);
  bool write_signed_word_uint32_command(uint8_t command, int16_t value1, uint32_t value2);
  bool write_signed_long_pair_command(uint8_t command, int32_t value1, int32_t value2);

  std::string device_;
  int baud_{115200};
  int address_{128};
  int fd_{-1};
  uint16_t crc_{0};
  std::string last_error_;
};

}  // namespace roboclaw_hardware_interface
