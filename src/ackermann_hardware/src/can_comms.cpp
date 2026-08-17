#include "ackermann_hardware/can_comms.hpp"

bool CanComms::connect(const std::string &interface)
{
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) {
        std::cerr << "Failed to create CAN socket" << std::endl;
        return false;
    }

    int flags = fcntl(socket_fd_, F_GETFL, 0);
    if (flags < 0 || fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
        std::cerr << "Failed to set non-blocking mode" << std::endl;
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, interface.c_str(), IFNAMSIZ - 1);
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
        std::cerr << "Failed to get CAN interface index for " << interface << std::endl;
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    struct sockaddr_can addr {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        std::cerr << "Failed to bind CAN socket" << std::endl;
        close(socket_fd_);
        socket_fd_ = -1;
        return false;
    }

    return true;
}

void CanComms::disconnect()
{
    if (socket_fd_ >= 0) {
        close(socket_fd_);
        socket_fd_ = -1;
    }
}

bool CanComms::connected() const
{
    return socket_fd_ >= 0;
}

bool CanComms::send_hardware_reset()
{
    struct can_frame frame {};
    frame.can_id = V_PACE_DB_RESET_COMMAND_FRAME_ID;
    frame.can_dlc = V_PACE_DB_RESET_COMMAND_LENGTH;

    struct v_pace_db_reset_command_t msg;
    msg.reset = 1;
    v_pace_db_reset_command_pack(frame.data, &msg, sizeof(frame.data));

    ssize_t nbytes = write(socket_fd_, &frame, sizeof(struct can_frame));
    if (nbytes != sizeof(struct can_frame)) {
        std::cerr << "CAN write failed: " << strerror(errno) << std::endl;
        return false;
    }
    return true;
}

CanComms::SensorStatus CanComms::read_sensor_values(int &l_enc, int &r_enc, double imu[6],
                                                    bool &is_imu_reset,
                                                    SteeringFeedback &steer_fb)
{
    SensorStatus status;
    struct can_frame frame;
    bool got_enc = false, got_imu1 = false, got_imu2 = false;
    uint8_t seq1 = 0, seq2 = 0;

    while (true) {
        ssize_t nbytes = read(socket_fd_, &frame, sizeof(struct can_frame));
        if (nbytes != sizeof(struct can_frame)) break;

        switch (frame.can_id) {
            case V_PACE_DB_VELOCITY_FEEDBACK_FRAME_ID: {
                struct v_pace_db_velocity_feedback_t msg;
                v_pace_db_velocity_feedback_unpack(&msg, frame.data, frame.can_dlc);
                l_enc = msg.left_ticks;
                r_enc = msg.right_ticks;
                got_enc = true;
                break;
            }
            case V_PACE_DB_STEERING_FEEDBACK_FRAME_ID: {
                struct v_pace_db_steering_feedback_t msg;
                v_pace_db_steering_feedback_unpack(&msg, frame.data, frame.can_dlc);
                steer_fb.angle = v_pace_db_steering_feedback_steering_angle_decode(msg.steering_angle);
                steer_fb.at_target = (msg.at_target == 1);
                steer_fb.pot_fault = (msg.pot_fault == 1);
                steer_fb.out_of_range = (msg.out_of_range == 1);
                steer_fb.saturated = (msg.saturated == 1);
                steer_fb.received = true;
                break;
            }
            case V_PACE_DB_IMU_ACCEL_FRAME_ID: {
                struct v_pace_db_imu_accel_t msg;
                v_pace_db_imu_accel_unpack(&msg, frame.data, frame.can_dlc);
                imu[0] = v_pace_db_imu_accel_accel_x_decode(msg.accel_x);
                imu[1] = v_pace_db_imu_accel_accel_y_decode(msg.accel_y);
                imu[2] = v_pace_db_imu_accel_accel_z_decode(msg.accel_z);
                seq1 = msg.sequence;
                got_imu1 = true;
                break;
            }
            case V_PACE_DB_IMU_GYRO_FLAGS_FRAME_ID: {
                struct v_pace_db_imu_gyro_flags_t msg;
                v_pace_db_imu_gyro_flags_unpack(&msg, frame.data, frame.can_dlc);
                imu[3] = v_pace_db_imu_gyro_flags_gyro_x_decode(msg.gyro_x);
                imu[4] = v_pace_db_imu_gyro_flags_gyro_y_decode(msg.gyro_y);
                imu[5] = v_pace_db_imu_gyro_flags_gyro_z_decode(msg.gyro_z);
                is_imu_reset = (msg.imu_reset == 1);
                seq2 = msg.sequence;
                got_imu2 = true;
                break;
            }
        }
    }

    // The encoder frame stands on its own. 0x110 is an independent transmission
    // from 0x150/0x160 per the DBC, so IMU freshness must never invalidate it.
    status.enc = got_enc;

    // The IMU is valid only when BOTH halves arrived AND they describe the same
    // sample cycle (0x150 carries `sequence` in byte 6, 0x160 in byte 7 — the DBC
    // requires them to match). A mismatch invalidates the IMU PAIR ONLY; the
    // encoder read above is unaffected.
    if (got_imu1 && got_imu2 && seq1 == seq2) {
        // Both halves are from the same sample cycle. Now require the sample to be
        // NEW: a frozen sensor/firmware keeps re-sending the same sequence, which is
        // still internally self-consistent (seq1 == seq2) but stale. Reject staleness
        // so the EKF predicts through it rather than fusing a frozen reading as fresh.
        //
        // The Tiva emits 0x150+0x160 at 50 Hz with a proven-steady sequence (4,300
        // pairs, 0 mismatches), so there is no legitimate jitter to tolerate — any
        // non-advance is a genuine freeze. `sequence` is uint8 and wraps, so
        // "advanced" is tested as != rather than >, which makes 255 -> 0 a valid
        // advance rather than a false freeze.
        if (!seq_initialized_ || seq1 != expected_seq_) {
            status.imu = true;
            expected_seq_ = seq1;
            seq_initialized_ = true;
        } else {
            // Sequence unchanged since the last ACCEPTED sample => frozen IMU.
            // Treat exactly like a missing IMU (status.imu stays false). The encoder
            // is deliberately unaffected — see the R-E decoupling above.
            status.imu_stale = true;
        }
    }

    return status;
}

bool CanComms::set_motor_values(float left_vel, float right_vel)
{
    struct can_frame frame {};
    frame.can_id = V_PACE_DB_VELOCITY_COMMAND_FRAME_ID;
    frame.can_dlc = V_PACE_DB_VELOCITY_COMMAND_LENGTH;

    struct v_pace_db_velocity_command_t msg;
    msg.left_vel_setpoint = left_vel;
    msg.right_vel_setpoint = right_vel;

    v_pace_db_velocity_command_pack(frame.data, &msg, sizeof(frame.data));

    ssize_t nbytes = write(socket_fd_, &frame, sizeof(struct can_frame));
    if (nbytes != sizeof(struct can_frame)) {
        std::cerr << "CAN write failed: " << strerror(errno) << std::endl;
        return false;
    }
    return true;
}

bool CanComms::set_steering(float steer_angle)
{
    struct can_frame frame {};
    frame.can_id = V_PACE_DB_STEERING_COMMAND_FRAME_ID;
    frame.can_dlc = V_PACE_DB_STEERING_COMMAND_LENGTH;

    struct v_pace_db_steering_command_t msg;
    msg.steering_setpoint = steer_angle;

    v_pace_db_steering_command_pack(frame.data, &msg, sizeof(frame.data));

    ssize_t nbytes = write(socket_fd_, &frame, sizeof(struct can_frame));
    if (nbytes != sizeof(struct can_frame)) {
        std::cerr << "CAN write failed: " << strerror(errno) << std::endl;
        return false;
    }
    return true;
}