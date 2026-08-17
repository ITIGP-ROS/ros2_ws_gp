// Copyright 2021 ros2_control Development Team
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

#ifndef ACKERMANN_HARDWARE__ACKERMANN_SYSTEM_HPP_
#define ACKERMANN_HARDWARE__ACKERMANN_SYSTEM_HPP_

#include <cstddef>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "ackermann_hardware/visibility_control.h"
#include "ackermann_hardware/can_comms.hpp"
#include "ackermann_hardware/drive_wheel.hpp"
#include "ackermann_hardware/steering_wheel.hpp"

namespace ackermann_hardware
{
class AckermannHardwareSystem : public hardware_interface::SystemInterface
{

struct Config
{
  std::string left_wheel_name = "";
  std::string right_wheel_name = "";
  std::string left_steering_name = "";
  std::string right_steering_name = "";
  std::string can_interface = "";
  int enc_counts_per_rev = 0;
  std::string imu_name = "";
};

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(AckermannHardwareSystem);

  ACKERMANN_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  ACKERMANN_HARDWARE_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ACKERMANN_HARDWARE_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ACKERMANN_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  ACKERMANN_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  ACKERMANN_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  ACKERMANN_HARDWARE_PUBLIC
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  ACKERMANN_HARDWARE_PUBLIC
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  ACKERMANN_HARDWARE_PUBLIC
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:

  CanComms can_comms_;
  Config cfg_;
  DriveWheel wheel_l_;
  DriveWheel wheel_r_;
  SteeringWheel steering_;
  
  double imu_accel_[3] = {0.0, 0.0, 0.0};
  double imu_gyro_[3] = {0.0, 0.0, 0.0};
  double imu_orientation_[4] = {0.0, 0.0, 0.0, 1.0}; // Fake

  // IMU Calibration
  bool is_imu_calibrating_ = true;
  int imu_calibration_sample_count_ = 0;
  double imu_gyro_z_sum_ = 0.0;
  double imu_gyro_z_offset_ = 0.0;
  const int IMU_CALIBRATION_SAMPLES = 200; // ~4 seconds at 50Hz

  // Settle window before gyro sampling starts. on_activate() centres the steering,
  // which physically moves the vehicle; sampling the gyro during that motion biases
  // the offset. This USED TO BE a blocking rclcpp::sleep_for(1500ms) inside
  // on_activate(), and that block was itself a bug: it stalled the controller_manager
  // loop, which then ran ~45 cycles back to back to catch up and tripped the encoder
  // staleness check within a millisecond of "Successfully activated!".
  //
  // Counting cycles here instead costs nothing — calibration was ALREADY spread across
  // read() cycles (IMU_CALIBRATION_SAMPLES above) — and it keeps activation
  // non-blocking, which is what makes ENC_STALE_LIMIT_S below safe. Do not put the
  // sleep back.
  int activation_settle_cycles_ = 0;
  static constexpr int ACTIVATION_SETTLE_CYCLES = 45; // ~1.5 s at 30 Hz

  // hardware_interface::SystemInterface is not a Node and exposes no get_clock(),
  // so throttled logging needs its own clock. Steady time: this only paces log
  // output, and must not be affected by sim-time or wall-clock jumps.
  rclcpp::Clock throttle_clock_{RCL_STEADY_TIME};

  // Seconds elapsed since the last FRESH encoder frame. Velocity is divided by
  // this rather than by one controller period, so a missed 0x110 does not turn
  // into an (N+1)x spike when the next one arrives.
  double enc_dt_accum_ = 0.0;

  // CAN write-failure tracking (bus-off / link-down detection).
  // Counts CONSECUTIVE failed write cycles; any successful cycle resets it.
  size_t can_write_failures_ = 0;
  static constexpr size_t CAN_WRITE_FAILURE_LIMIT = 5; // ~166 ms at 30 Hz

  // Encoder-loss tracking (read side), gated on MEASURED TIME via enc_dt_accum_.
  // Tolerated for longer than the write side's 5 cycles because a transient encoder
  // drop is more plausible than a bus-off, while 333 ms is still far too short for
  // the robot to run away. The encoder is the EKF's ONLY forward-velocity source, so
  // sustained loss means there is no trustworthy vx at all.
  //
  // THIS USED TO COUNT CYCLES (ENC_READ_FAILURE_LIMIT = 10) AND ASSUMED 33.3 ms EACH.
  // That assumption fails exactly when it matters. The old blocking on_activate() made
  // controller_manager run its backlog back to back, so ten "consecutive cycles" could
  // elapse in ~1 ms — measured on the unit, the ERROR landed 1 ms after "Successfully
  // activated!" and the component left ACTIVE permanently: write() stops being called,
  // 0x100/0x120 stop, and every re-activation re-trips it, so the vehicle is dead for
  // the session with no operator-visible message.
  //
  // Holding the last good encoder value for 333 ms costs at most 0.2 m/s x 0.333 =
  // 6.7 cm of odometry error at campaign speed. That is the whole price, against
  // losing the remainder of a drive.
  //
  // Safe only because on_activate() no longer blocks — see ACTIVATION_SETTLE_CYCLES.
  static constexpr double ENC_STALE_LIMIT_S = 0.333;

  double dummy_front_wheel_pos_ = 0.0;
  double dummy_front_wheel_vel_ = 0.0;
};

}  // namespace ackermann_hardware

#endif  // ACKERMANN_HARDWARE__ACKERMANN_SYSTEM_HPP_