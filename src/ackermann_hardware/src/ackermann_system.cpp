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

#include "ackermann_hardware/ackermann_system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace ackermann_hardware
{
hardware_interface::CallbackReturn AckermannHardwareSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  cfg_.left_wheel_name = info_.hardware_parameters["left_wheel_name"];
  cfg_.right_wheel_name = info_.hardware_parameters["right_wheel_name"];
  cfg_.left_steering_name = info_.hardware_parameters["left_steering_name"];
  cfg_.right_steering_name = info_.hardware_parameters["right_steering_name"];
  cfg_.can_interface = info_.hardware_parameters["can_interface"];
  cfg_.enc_counts_per_rev = std::stoi(info_.hardware_parameters["enc_counts_per_rev"]);
  cfg_.imu_name = info_.hardware_parameters["imu_name"];

  wheel_l_.setup(cfg_.left_wheel_name, cfg_.enc_counts_per_rev);
  wheel_r_.setup(cfg_.right_wheel_name, cfg_.enc_counts_per_rev);
  steering_.setup(cfg_.left_steering_name, cfg_.right_steering_name);

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // DRIVE WHEELS
    if (joint.name == cfg_.left_wheel_name || joint.name == cfg_.right_wheel_name)
    {
       if (joint.command_interfaces.size() != 1)
       {
         RCLCPP_FATAL(
           rclcpp::get_logger("AckermannHardwareSystem"),
           "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
           joint.command_interfaces.size());
         return hardware_interface::CallbackReturn::ERROR;
       }
   
       if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
       {
         RCLCPP_FATAL(
           rclcpp::get_logger("AckermannHardwareSystem"),
           "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
           joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
         return hardware_interface::CallbackReturn::ERROR;
       }
   
       if (joint.state_interfaces.size() != 2)
       {
         RCLCPP_FATAL(
           rclcpp::get_logger("AckermannHardwareSystem"),
           "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
           joint.state_interfaces.size());
         return hardware_interface::CallbackReturn::ERROR;
       }
   
       if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
       {
         RCLCPP_FATAL(
           rclcpp::get_logger("AckermannHardwareSystem"),
           "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
           joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
         return hardware_interface::CallbackReturn::ERROR;
       }
   
       if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
       {
         RCLCPP_FATAL(
           rclcpp::get_logger("AckermannHardwareSystem"),
           "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
           joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
         return hardware_interface::CallbackReturn::ERROR;
       }
    }
    
    // STEERING JOINTS
    else if (joint.name == cfg_.left_steering_name || joint.name == cfg_.right_steering_name)
    {
       if (joint.command_interfaces.size() != 1)
       {
         RCLCPP_FATAL(
           rclcpp::get_logger("AckermannHardwareSystem"),
           "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
           joint.command_interfaces.size());
         return hardware_interface::CallbackReturn::ERROR;
       }
   
       if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
       {
         RCLCPP_FATAL(
           rclcpp::get_logger("AckermannHardwareSystem"),
           "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
           joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
         return hardware_interface::CallbackReturn::ERROR;
       }
   
       if (joint.state_interfaces.size() != 1)
       {
         RCLCPP_FATAL(
           rclcpp::get_logger("AckermannHardwareSystem"),
           "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
           joint.state_interfaces.size());
         return hardware_interface::CallbackReturn::ERROR;
       }
   
       if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
       {
         RCLCPP_FATAL(
           rclcpp::get_logger("AckermannHardwareSystem"),
           "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
           joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
         return hardware_interface::CallbackReturn::ERROR;
       }
    }
    // IMU SENSOR
    else if (joint.name == cfg_.imu_name)
    {
       if (joint.state_interfaces.size() != 10) // ax, ay, az, gx, gy, gz, ox, oy, oz, ow
       {
         RCLCPP_WARN(
           rclcpp::get_logger("AckermannHardwareSystem"),
           "Sensor '%s' state interface count not validated strictly.", joint.name.c_str());
       }
    }
    else
    {
        RCLCPP_WARN(rclcpp::get_logger("AckermannHardwareSystem"), "Joint '%s' is not listed in configuration", joint.name.c_str());
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> AckermannHardwareSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // Drive Wheels
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_POSITION, &wheel_l_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.vel));

  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_POSITION, &wheel_r_.pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.vel));

  // Steering Joints (Servo)
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    steering_.left_joint_name, hardware_interface::HW_IF_POSITION, &steering_.left_pos));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    steering_.right_joint_name, hardware_interface::HW_IF_POSITION, &steering_.right_pos));

  // Dummy Front Wheels for RVIZ
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "front_left_wheel_joint", hardware_interface::HW_IF_POSITION, &dummy_front_wheel_pos_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "front_left_wheel_joint", hardware_interface::HW_IF_VELOCITY, &dummy_front_wheel_vel_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "front_right_wheel_joint", hardware_interface::HW_IF_POSITION, &dummy_front_wheel_pos_));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    "front_right_wheel_joint", hardware_interface::HW_IF_VELOCITY, &dummy_front_wheel_vel_));

  // IMU State Interfaces
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    cfg_.imu_name, "linear_acceleration.x", &imu_accel_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    cfg_.imu_name, "linear_acceleration.y", &imu_accel_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    cfg_.imu_name, "linear_acceleration.z", &imu_accel_[2]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    cfg_.imu_name, "angular_velocity.x", &imu_gyro_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    cfg_.imu_name, "angular_velocity.y", &imu_gyro_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    cfg_.imu_name, "angular_velocity.z", &imu_gyro_[2]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    cfg_.imu_name, "orientation.x", &imu_orientation_[0]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    cfg_.imu_name, "orientation.y", &imu_orientation_[1]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    cfg_.imu_name, "orientation.z", &imu_orientation_[2]));
  state_interfaces.emplace_back(hardware_interface::StateInterface(
    cfg_.imu_name, "orientation.w", &imu_orientation_[3]));

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> AckermannHardwareSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // Drive Wheels
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_l_.name, hardware_interface::HW_IF_VELOCITY, &wheel_l_.cmd));

  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    wheel_r_.name, hardware_interface::HW_IF_VELOCITY, &wheel_r_.cmd));

  // Steering Joints (Servo)
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    steering_.left_joint_name, hardware_interface::HW_IF_POSITION, &steering_.left_cmd));
  command_interfaces.emplace_back(hardware_interface::CommandInterface(
    steering_.right_joint_name, hardware_interface::HW_IF_POSITION, &steering_.right_cmd));

  return command_interfaces;
}

hardware_interface::CallbackReturn AckermannHardwareSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("AckermannHardwareSystem"), "Configuring ...please wait...");
  if (can_comms_.connected())
  {
    can_comms_.disconnect();
  }
  if (!can_comms_.connect(cfg_.can_interface))
  {
    RCLCPP_FATAL(rclcpp::get_logger("AckermannHardwareSystem"), "Failed to connect to CAN interface: %s", cfg_.can_interface.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("AckermannHardwareSystem"), "Successfully configured!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AckermannHardwareSystem::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("AckermannHardwareSystem"), "Cleaning up ...please wait...");
  if (can_comms_.connected())
  {
    can_comms_.disconnect();
  }
  RCLCPP_INFO(rclcpp::get_logger("AckermannHardwareSystem"), "Successfully cleaned up!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AckermannHardwareSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("AckermannHardwareSystem"), "Activating ...please wait...");

  if (!can_comms_.connected())
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  // Zero the Tiva's cumulative encoder tick counters (0x140 ResetCommand).
  can_comms_.send_hardware_reset();

  // Re-baseline the HOST side to match. The Tiva's counters restart at 0, so
  // without this the first read after a re-activation differences a near-zero
  // tick count against a stale, possibly large `pos` and emits an enormous false
  // velocity — ~4,900x the physical maximum after only 100 m driven. Resetting
  // enc/pos/vel and the elapsed-time accumulator keeps host and Tiva consistent.
  wheel_l_.enc = 0;  wheel_l_.pos = 0.0;  wheel_l_.vel = 0.0;
  wheel_r_.enc = 0;  wheel_r_.pos = 0.0;  wheel_r_.vel = 0.0;
  enc_dt_accum_ = 0.0;   // fresh start: a prior session's loss must not carry over

  // Center steering — Tiva-C handles angle->PWM mapping locally
  can_comms_.set_steering(0.0f);

  // Let that steering movement settle before the gyro is sampled — but WITHOUT
  // blocking. This was rclcpp::sleep_for(1500ms), which stalled controller_manager's
  // loop for 1.5 s; it then ran the backlog back to back and the encoder staleness
  // check (which counted cycles, not time) fired 1 ms after "Successfully activated!",
  // knocking the component out of ACTIVE for the rest of the session.
  //
  // read() now counts the settle window down instead, so activation returns
  // immediately, there is no catch-up burst, and no cycle carries an oversized period.
  // ENC_STALE_LIMIT_S depends on that: do not reintroduce a blocking call here.
  activation_settle_cycles_ = ACTIVATION_SETTLE_CYCLES;

  // Reset IMU Calibration state on activate
  is_imu_calibrating_ = true;
  imu_calibration_sample_count_ = 0;
  imu_gyro_z_sum_ = 0.0;
  imu_gyro_z_offset_ = 0.0;

  RCLCPP_INFO(rclcpp::get_logger("AckermannHardwareSystem"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn AckermannHardwareSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("AckermannHardwareSystem"), "Deactivating ...please wait...");
  
  if (can_comms_.connected())
  {
    // Stop the drive, not just centre the steering. This covers the CLEAN
    // deactivate: writes stop afterwards, so without an explicit zero the Tiva
    // would hold the last commanded velocity until its own CMD_TIMEOUT fired.
    //
    // NOTE: this does NOT cover the runaway case — a *controller* deactivating
    // or crashing while this hardware component stays ACTIVE, where
    // controller_manager keeps calling write() at 30 Hz and re-sends the last
    // command from hardware-owned storage. The Tiva's CMD_TIMEOUT watches frame
    // ARRIVAL, not content, so a punctual stream of stale commands never trips
    // it. That needs a command-freshness/heartbeat mechanism — tracked
    // separately, deliberately not solved here.
    can_comms_.set_motor_values(0.0f, 0.0f);
    can_comms_.set_steering(0.0f);
  }

  RCLCPP_INFO(rclcpp::get_logger("AckermannHardwareSystem"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type AckermannHardwareSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!can_comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }
  
  int l_enc = 0, r_enc = 0;
  double imu_data[6] = {0.0};
  bool is_imu_reset = false;
  CanComms::SteeringFeedback steer_fb;
  
  // Per-source freshness. The encoder (0x110) and the IMU (0x150/0x160) are
  // independent frames on the wire, so each consumer below is gated on its OWN
  // source — mirroring how steering has always been gated on steer_fb.received.
  CanComms::SensorStatus status =
    can_comms_.read_sensor_values(l_enc, r_enc, imu_data, is_imu_reset, steer_fb);

  // Steering position comes from the rack-mounted pot (real Ackermann geometry),
  // regardless of encoder/IMU frame freshness.
  static bool prev_pot_fault = false;
  static bool prev_out_of_range = false;
  if (steer_fb.received) {
      if (steer_fb.pot_fault && !prev_pot_fault) {
          RCLCPP_WARN(rclcpp::get_logger("AckermannHardwareSystem"),
                      "Steering pot fault detected! Steering feedback unreliable.");
      }
      if (steer_fb.out_of_range && !prev_out_of_range) {
          RCLCPP_WARN(rclcpp::get_logger("AckermannHardwareSystem"),
                      "Steering command exceeded mechanical travel and was clamped!");
      }

      // pot_fault means the ANGLE IN THIS FRAME IS GARBAGE — observed at -1.335 rad,
      // roughly 4.7x the mechanical limit. It used to be written in anyway: the flag
      // was decoded, logged once, and then set_rack_position() ran regardless, so the
      // impossible value reached the steering state, joint states and odometry. Hold
      // the last good angle instead — the same thing the cluster is already told to do
      // when it greys out the display. Rare (order one frame in several thousand), so
      // a hold is a far better estimate than the fault value.
      if (!steer_fb.pot_fault) {
          // THE STEERING FEEDBACK CALIBRATION OF RECORD. Measured 2026-08-27.
          //
          // The Tiva firmware is FROZEN - it will not be changed - so this host-side
          // mapping is not a stopgap, it is where pot counts become wheel angles for this
          // vehicle. Treat it like any other calibration: if the linkage, the servo or
          // the pot centre is ever disturbed, RE-MEASURE IT (procedure at the end of this
          // comment) rather than nudging the constants.
          //
          // 0x130 steeringAngle does not agree with 0x120 steeringSetpoint. Measured on
          // the wheels-up vehicle, 13 commanded levels through the normal
          // /cmd_vel_teleop path, 12465 CAN frames:
          //
          //   SIGN IS INVERTED. Commanding LEFT drives the pot to the RIGHT of centre
          //   (cmd +0.2409 -> pot -0.1550, implying count ~2413 against a centre of
          //   1921). servo_feedback_cfg.h states the opposite convention explicitly -
          //   "higher count = RIGHT = negative angle (REP-103)" - and its calibration is
          //   internally coherent, so the hardware moves the other way from what that
          //   calibration assumes. That is a PHYSICAL inversion, most likely dating from
          //   the 2026-08-19 re-centre at the new 1520 us centre.
          //
          //   SCALE IS SHORT, AND ASYMMETRICALLY. Least squares per side:
          //       LEFT  (cmd>0)  pot = -0.60526*cmd - 0.01010   rms 0.0017 rad
          //       RIGHT (cmd<0)  pot = -0.78021*cmd - 0.01713   rms 0.0019 rad
          //   The 1.289 gain ratio matches the linkage asymmetry the firmware measured
          //   independently (1.196 from the pot, 1.210 from circle diameters), so the
          //   two sides are genuinely different and ONE constant will not do.
          //
          // The branch is chosen from the COMMAND, not from the sign of the feedback:
          // near centre the two fitted lines cross around -0.013 rad and the raw sign is
          // meaningless there, whereas the command is always known exactly.
          //
          // VALIDATED ON GROUND-DRIVING DATA, not just on the rig it was fitted on.
          // Bag 20260827_073550, 200 AMCL intervals, heading change vs AMCL:
          //     steering yaw, uncorrected   median 0.0832 rad
          //     steering yaw, corrected     median 0.0140 rad   <- 6x better
          //     IMU yaw (/imu_corrected)    median 0.0078 rad
          // The corrected estimate beats the IMU on p90 (0.0367 vs 0.0617) and on mean
          // (0.0180 vs 0.0217) while losing on the median: precise-but-tailed IMU
          // alongside coarser-but-better-behaved geometry. ekf.yaml now fuses BOTH, which
          // is only sound because this correction exists - and which also removes the
          // single point of failure imu_scale.py's own docstring warns about.
          //
          // RE-MEASUREMENT PROCEDURE (wheels up, ~4 minutes):
          //   drive /cmd_vel_teleop at a fixed vx with wz stepped so that
          //   delta = atan(L*wz/vx) walks +/-0.04 .. 0.24 rad, returning to centre between
          //   steps so backlash loads the same way; capture can0 ids 0x120 and 0x130;
          //   least-squares pot-vs-setpoint PER SIDE. Fit quality when these were taken
          //   was rms 0.0017 (left) / 0.0019 (right) over 13 levels, and the constants
          //   reproduced a held-out sweep to 0.0072 rad median.
          constexpr double FB_A_LEFT  = -0.60526, FB_B_LEFT  = -0.01010;
          constexpr double FB_A_RIGHT = -0.78021, FB_B_RIGHT = -0.01713;
          const bool cmd_left = steering_.get_average_steering_cmd() >= 0.0;
          const double corrected = cmd_left
            ? (steer_fb.angle - FB_B_LEFT)  / FB_A_LEFT
            : (steer_fb.angle - FB_B_RIGHT) / FB_A_RIGHT;
          steering_.set_rack_position(corrected);
      } else {
          // Edge-triggered warnings fire once. A pot that fails and STAYS failed would
          // then freeze steering feedback silently forever, which is the same class of
          // silent-wrong-data defect this guard exists to end. Keep it visible.
          RCLCPP_WARN_THROTTLE(
            rclcpp::get_logger("AckermannHardwareSystem"), throttle_clock_, 2000,
            "Steering pot fault SUSTAINED — holding last good steering position "
            "(L %.4f / R %.4f rad). Steering feedback, joint states and odometry are "
            "frozen on this axis.",
            steering_.left_pos, steering_.right_pos);
      }
  }
  prev_pot_fault = steer_fb.pot_fault;
  prev_out_of_range = steer_fb.out_of_range;
  
  // A frozen IMU (pair arrived, sequences matched each other, but the sequence did
  // not advance) is a real fault, unlike a transient dropped frame — surface it.
  // Throttled: at 30 Hz an unthrottled warn would be 30 lines/s from the control loop.
  if (status.imu_stale)
  {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger("AckermannHardwareSystem"), throttle_clock_, 2000,
      "IMU sequence frozen (seq not advancing) — treating IMU as missing. "
      "Wheel odometry is unaffected.");
  }

  // IMU: gated on the IMU pair alone. Missing, sequence-mismatched, or STALE IMU
  // frames no longer touch the encoder (defect R-E).
  if (status.imu)
  {
      static bool prev_imu_reset = false;
      if (is_imu_reset && !prev_imu_reset) {
          RCLCPP_WARN(rclcpp::get_logger("AckermannHardwareSystem"),
                      "Firmware reported IMU crash! Auto-healing, retaining previous IMU calibration...");
      }
      prev_imu_reset = is_imu_reset;

    imu_accel_[0] = imu_data[0];
    imu_accel_[1] = imu_data[1];
    imu_accel_[2] = imu_data[2];
    
    imu_gyro_[0] = imu_data[3];
    imu_gyro_[1] = imu_data[4];
    
    double raw_gyro_z = imu_data[5];
    
    if (is_imu_calibrating_) {
        // Settle window (replaces the old blocking sleep in on_activate). While it
        // runs, frames are read and discarded: centring the steering physically moves
        // the vehicle, and averaging gyro samples taken during that motion would bake
        // the movement into the zero offset. Sampling starts only once it expires.
        if (activation_settle_cycles_ > 0) {
            --activation_settle_cycles_;
        } else if (!is_imu_reset) {
            imu_gyro_z_sum_ += raw_gyro_z;
            imu_calibration_sample_count_++;

            if (imu_calibration_sample_count_ >= IMU_CALIBRATION_SAMPLES) {
                imu_gyro_z_offset_ = imu_gyro_z_sum_ / IMU_CALIBRATION_SAMPLES;
                is_imu_calibrating_ = false;
                RCLCPP_INFO(rclcpp::get_logger("AckermannHardwareSystem"), 
                            "IMU Calibration Complete. Z-axis Bias: %.4f rad/s", imu_gyro_z_offset_);
            }
        }
        
        // Enforce strict 0 to prevent drift in EKF during setup
        imu_gyro_[2] = 0.0; 
    } else if (is_imu_reset) {
        // Enforce strict 0 during hardware crash to prevent drift
        imu_gyro_[2] = 0.0; 
    } else {
        // Apply calibration offset
        imu_gyro_[2] = (raw_gyro_z - imu_gyro_z_offset_); 
    }
  }

  // Wheel odometry: gated on the ENCODER alone (defect R-E). Accumulate elapsed
  // time so that if a 0x110 frame is genuinely missed, the next one is divided by
  // the true interval rather than a single cycle — otherwise an N-cycle gap
  // produces an (N+1)x velocity spike.
  enc_dt_accum_ += period.seconds();

  if (status.enc)
  {
    wheel_l_.enc = l_enc;
    wheel_r_.enc = r_enc;

    const double l_pos_prev = wheel_l_.pos;
    const double r_pos_prev = wheel_r_.pos;

    wheel_l_.pos = wheel_l_.calc_enc_angle();
    wheel_r_.pos = wheel_r_.calc_enc_angle();

    if (enc_dt_accum_ > 0.0001) {
      wheel_l_.vel = (wheel_l_.pos - l_pos_prev) / enc_dt_accum_;
      wheel_r_.vel = (wheel_r_.pos - r_pos_prev) / enc_dt_accum_;
    }

    enc_dt_accum_ = 0.0;
  }
  else
  {
    // The encoder frame did not arrive this cycle. HOLD the previous pos and vel
    // rather than fabricating a zero — a brief hold is closer to the truth than a
    // confident "stopped", which is exactly what misled the EKF before.
    //
    // But a hold is only honest for a BRIEF drop. Sustained loss means there is no
    // trustworthy forward velocity at all (the encoder is the EKF's only vx
    // source), and holding a stale vx forever would be the same class of silent
    // wrong data that R-E was. So escalate, mirroring the write-side bus-off
    // counter: stop rather than drive blind.
    //
    // Gated on MEASURED elapsed time, not on a cycle count. enc_dt_accum_ is
    // incremented by the real period every cycle above and reset only by a fresh
    // 0x110, so it IS the age of the newest trustworthy encoder frame. Counting
    // cycles and multiplying by a nominal 33.3 ms lied whenever the loop was not at
    // rate — which is exactly when the loop is in trouble.
    if (enc_dt_accum_ >= ENC_STALE_LIMIT_S)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("AckermannHardwareSystem"),
        "No encoder (0x110) for %d ms of measured time (limit %d ms) — no trustworthy "
        "odometry; returning ERROR.",
        static_cast<int>(enc_dt_accum_ * 1000.0),
        static_cast<int>(ENC_STALE_LIMIT_S * 1000.0));
      return hardware_interface::return_type::ERROR;
    }
  }

  // Steering position is set from the steering pot feedback above

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AckermannHardwareSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (!can_comms_.connected())
  {
    return hardware_interface::return_type::ERROR;
  }

  double avg_steer_angle = steering_.get_average_steering_cmd();

  // Host-side safety clamp, matching the firmware's own clamp exactly. +left per
  // REP-103 / DBC steeringSetpoint. Its purpose is to bound bypass command paths
  // that have no Nav2 limit in front of them; it must therefore permit the full
  // real range and no less.
  //
  // 🔴 WAS +0.2421 / -0.3037, AND BOTH NUMBERS WERE WRONG (fixed 2026-08-27).
  //
  // Those were the PRE-2026-08-16 figures, and the firmware that supersedes them
  // is explicit about what they actually were (Vehicle-Control, servo_cfg.h):
  //
  //     "The previous values (0.2421 / 0.3037) were NOT wheel-angle measurements.
  //      0.3037 rad was the CAD DESIGN figure and 0.2421 was derived from it via
  //      the pot; neither was ever checked against the road wheels. They were
  //      wrong by ~2.3x (left) and ~1.9x (right) ... ROOT CAUSE: the pot is NOT
  //      on the road wheel - it is upstream on the servo horn / linkage."
  //
  // The recalibration measured traced circle DIAMETERS at known commands and fitted
  // delta_true = atan(L/(dia/2)) per side, so post-2026-08-16 the angle sent over CAN
  // IS the true bicycle steering angle. The firmware then clamps it, SYMMETRICALLY:
  //
  //     steering_control.c:70-71   SC_LIMIT_LEFT_RAD  = +0.286f
  //                                SC_LIMIT_RIGHT_RAD = -0.286f
  //
  // The old comment here claimed "the Tiva enforces the same limits in firmware".
  // It does not, and the mismatch was not symmetric in its effect: LEFT was clipped
  // at 0.2421 when the vehicle can reach 0.286 - 15% of the left lock discarded,
  // raising the achievable left radius from L/tan(0.286)=0.80 m to
  // L/tan(0.2421)=0.953 m - while RIGHT passed 0.3037 down only to be clamped to
  // 0.286 by the Tiva anyway.
  //
  // ⚠️ THAT CLIP WAS THE ONLY ASYMMETRY IN THE SYSTEM. The mechanism is symmetric
  // here, and vehicle data agrees: over 6342 samples with both wheels steering,
  // cot(outer)-cot(inner) = 0.510 with ZERO spread, exactly track/wheelbase for an
  // ideal symmetric linkage. nav2_amcl.yaml's "THE VEHICLE IS ASYMMETRIC" note, and
  // the rejection of a 0.84 m turning radius as "only achievable turning RIGHT",
  // both rest on this clamp rather than on the hardware. Revisit them together.
  //
  // Keep these two equal to SC_LIMIT_*_RAD. If the firmware clamp moves, move these.
  constexpr double STEER_MAX_LEFT  =  0.286;
  constexpr double STEER_MAX_RIGHT = -0.286;
  avg_steer_angle = std::clamp(avg_steer_angle, STEER_MAX_RIGHT, STEER_MAX_LEFT);

  // Host-side velocity sanity clamp (last ROS line of defense; guards bypass
  // command paths — there is no Nav2 limit in front of those). Bound is the
  // measured physical max: 178 rpm wheel (wheels-up) = 0.61 m/s / 0.0325 m
  // = 18.77 rad/s per wheel. This is NOT the operational cap (that is Nav2
  // max_vel_x, set conservatively for load); it only rejects the impossible.
  constexpr float WHEEL_VEL_MAX = 18.77f;  // rad/s, per wheel (= 0.61 m/s)
  float left_vel  = std::clamp(static_cast<float>(wheel_l_.cmd), -WHEEL_VEL_MAX, WHEEL_VEL_MAX);
  float right_vel = std::clamp(static_cast<float>(wheel_r_.cmd), -WHEEL_VEL_MAX, WHEEL_VEL_MAX);

  // Send physical steering angle in radians directly.
  // Tiva-C handles angle->PWM mapping locally per DBC.
  // Both sends are attempted every cycle even when one fails (no short-circuit),
  // so a single bad frame never suppresses the other axis.
  bool ok = can_comms_.set_steering(static_cast<float>(avg_steer_angle));
  ok &= can_comms_.set_motor_values(left_vel, right_vel);

  // Bus-off / link-down detection.
  //
  // ⚠️ THE OLD PREMISE HERE WAS WRONG AND IT COST THE VEHICLE. It read: "Our TX is ~1.6%
  // of a 500 kbit/s bus, so a failed write is never congestion — it means the link is
  // broken." That accounts for BUS bandwidth but not for the kernel's per-interface tx
  // QUEUE, which is what actually overflows. The socket is O_NONBLOCK, so a full queue
  // returns ENOBUFS/EAGAIN immediately, and can0 carries four other local writers
  // (update_coordinator, road_classification, liveliness_respond, the YOLO CAN node).
  // Measured 2026-08-26 on a battery boot: a ~166 ms ENOBUFS burst tripped the 5-cycle
  // limit, this function returned ERROR, ros2_control deactivated the hardware component,
  // and the vehicle was silently uncommandable with every controller still active. can0
  // read ERROR-ACTIVE, berr-counter 0/0, bus-off 0 throughout.
  //
  // So the two cases are now counted separately: a HARD error (a real errno, link gone)
  // still escalates in 5 cycles, while CONGESTION is ridden out for CAN_WRITE_CONGESTION_LIMIT.
  if (!ok)
  {
    if (can_comms_.last_write_transient())
    {
      can_write_failures_ = 0;   // not a link fault; do not age the bus-off counter
      if (++can_write_congestion_ >= CAN_WRITE_CONGESTION_LIMIT)
      {
        RCLCPP_ERROR(
          rclcpp::get_logger("AckermannHardwareSystem"),
          "CAN tx queue full for %zu consecutive cycles (~%d ms) — the bus or the queue is "
          "wedged, not a momentary burst; returning ERROR.",
          can_write_congestion_, static_cast<int>(can_write_congestion_ * 1000 / 30));
        return hardware_interface::return_type::ERROR;
      }
      // Rate-limited so a burst does not flood the journal the way the raw
      // "CAN write failed" line from can_comms does.
      RCLCPP_WARN_THROTTLE(
        rclcpp::get_logger("AckermannHardwareSystem"), throttle_clock_, 1000,
        "CAN tx queue full (ENOBUFS) — dropping this cycle's commands and continuing. "
        "%zu consecutive congested cycles.", can_write_congestion_);
    }
    else if (++can_write_failures_ >= CAN_WRITE_FAILURE_LIMIT)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("AckermannHardwareSystem"),
        "CAN write failed %zu consecutive cycles (~%d ms) — bus-off/link down; "
        "returning ERROR.",
        can_write_failures_, static_cast<int>(can_write_failures_ * 1000 / 30));
      return hardware_interface::return_type::ERROR;
    }
  }
  else
  {
    can_write_failures_ = 0;
    can_write_congestion_ = 0;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace ackermann_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ackermann_hardware::AckermannHardwareSystem, hardware_interface::SystemInterface)