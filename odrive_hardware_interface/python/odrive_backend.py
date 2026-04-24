#!/usr/bin/env python3
# Copyright (c) 2026, b-robotized Group
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import argparse
import logging
import sys

IMPORT_ERROR = None

try:
    import odrive
    from odrive.enums import AXIS_STATE_CLOSED_LOOP_CONTROL, AXIS_STATE_IDLE, CONTROL_MODE_VELOCITY_CONTROL
except ModuleNotFoundError as exc:  # pragma: no cover - hardware env dependent
    odrive = None
    AXIS_STATE_CLOSED_LOOP_CONTROL = None
    AXIS_STATE_IDLE = None
    CONTROL_MODE_VELOCITY_CONTROL = None
    IMPORT_ERROR = exc


class ODriveBoard:
    def __init__(self, serial_number: str, right_axis: int, connect_timeout: float) -> None:
        self.serial_number = serial_number
        self.right_axis_index = right_axis
        self.connect_timeout = connect_timeout
        self.driver = None
        self.left_axis = None
        self.right_axis = None

    def connect(self) -> None:
        self.driver = odrive.find_any(path="usb", serial_number=self.serial_number, timeout=self.connect_timeout)
        self.right_axis = self.driver.axis0 if self.right_axis_index == 0 else self.driver.axis1
        self.left_axis = self.driver.axis1 if self.right_axis_index == 0 else self.driver.axis0

    def activate(self) -> None:
        for axis in (self.left_axis, self.right_axis):
            axis.config.watchdog_timeout = 1.0
            axis.controller.config.control_mode = CONTROL_MODE_VELOCITY_CONTROL
            axis.controller.input_vel = 0.0
            axis.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            axis.config.enable_watchdog = True
            axis.watchdog_feed()

    def deactivate(self) -> None:
        if not self.driver:
            return
        for axis in (self.left_axis, self.right_axis):
            axis.requested_state = AXIS_STATE_IDLE
            axis.config.enable_watchdog = False

    def drive(self, left_turns_per_second: float, right_turns_per_second: float) -> None:
        self.left_axis.watchdog_feed()
        self.right_axis.watchdog_feed()
        self.left_axis.controller.input_vel = left_turns_per_second
        self.right_axis.controller.input_vel = -right_turns_per_second

    def read(self) -> list[float]:
        return [
            float(self.left_axis.encoder.pos_estimate),
            float(self.left_axis.encoder.vel_estimate),
            float(-self.right_axis.encoder.pos_estimate),
            float(-self.right_axis.encoder.vel_estimate),
        ]

    def disconnect(self) -> None:
        try:
            self.deactivate()
        except Exception:
            pass
        self.driver = None
        self.left_axis = None
        self.right_axis = None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--front-serial", required=True)
    parser.add_argument("--rear-serial", required=True)
    parser.add_argument("--front-right-axis", type=int, default=0)
    parser.add_argument("--rear-right-axis", type=int, default=1)
    parser.add_argument("--connect-timeout", type=float, default=30.0)
    args = parser.parse_args()

    logging.basicConfig(level=logging.INFO, stream=sys.stderr)

    if IMPORT_ERROR is not None:
        print(f"ERROR missing python dependency: {IMPORT_ERROR}", flush=True)
        return 1

    front = ODriveBoard(args.front_serial, args.front_right_axis, args.connect_timeout)
    rear = ODriveBoard(args.rear_serial, args.rear_right_axis, args.connect_timeout)

    try:
        front.connect()
        rear.connect()
    except Exception as exc:
        print(f"ERROR connect failed: {exc}", flush=True)
        return 1

    active = False
    try:
        for raw_line in sys.stdin:
            line = raw_line.strip()
            if not line:
                continue
            parts = line.split()
            command = parts[0]

            if command == "PING":
                print("PONG", flush=True)
            elif command == "ACTIVATE":
                front.activate()
                rear.activate()
                active = True
                print("OK", flush=True)
            elif command == "DEACTIVATE":
                front.deactivate()
                rear.deactivate()
                active = False
                print("OK", flush=True)
            elif command == "WRITE":
                if len(parts) != 5:
                    print("ERROR bad write command", flush=True)
                    continue
                fl, fr, rl, rr = (float(value) for value in parts[1:5])
                if active:
                    front.drive(fl, fr)
                    rear.drive(rl, rr)
                print("OK", flush=True)
            elif command == "READ":
                values = front.read() + rear.read()
                print("STATE " + " ".join(f"{value:.10f}" for value in values), flush=True)
            elif command == "EXIT":
                front.disconnect()
                rear.disconnect()
                print("OK", flush=True)
                return 0
            else:
                print(f"ERROR unknown command: {command}", flush=True)
    finally:
        front.disconnect()
        rear.disconnect()

    return 0


if __name__ == "__main__":
    sys.exit(main())
