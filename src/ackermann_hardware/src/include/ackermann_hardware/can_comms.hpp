#ifndef CAN_COMMS_HPP
#define CAN_COMMS_HPP

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <string>
#include <iostream>

extern "C" {
#include "v_pace_db.h"
}

class CanComms
{
public:
    struct SteeringFeedback
    {
        double angle = 0.0;
        bool at_target = false;
        bool pot_fault = false;
        bool out_of_range = false;
        bool saturated = false;
        bool received = false;
    };

    // Per-source freshness for one read cycle. Reported separately because the
    // encoder and IMU arrive in INDEPENDENT CAN frames (0x110 vs 0x150/0x160 per
    // the DBC) — a lost IMU frame says nothing about the validity of the encoder.
    // Collapsing these into one flag was defect R-E: it froze wheel odometry
    // whenever an IMU frame went missing, feeding the EKF a confident false zero.
    struct SensorStatus
    {
        bool enc = false;   // 0x110 VelocityFeedback arrived this cycle
        bool imu = false;   // BOTH 0x150 and 0x160 arrived, sequences matched, AND advanced
        bool imu_stale = false;  // IMU pair arrived but sequence did NOT advance (frozen sensor)
    };

    CanComms() = default;
    ~CanComms() { disconnect(); }

    bool connect(const std::string &interface);
    void disconnect();
    bool connected() const;

    // TX functions return true when the full frame reached the socket, false on a
    // short/failed write. The caller uses this to detect a broken link (bus-off,
    // unplugged Tiva) — a dropped frame must not look like a delivered command.
    //
    // 🔴 BUT NOT EVERY FAILED WRITE IS A BROKEN LINK. The socket is O_NONBLOCK, so when
    // the kernel's CAN tx queue is momentarily full the write returns immediately with
    // ENOBUFS/EAGAIN. That is CONGESTION and it is the documented behaviour of a
    // non-blocking socket — the link is fine and the next cycle usually succeeds.
    // Measured on the vehicle 2026-08-26 (battery boot, five local writers on can0,
    // 331 rx frames/s, txqueuelen 10): a ~166 ms burst of ENOBUFS was read as bus-off,
    // write() returned ERROR, ros2_control deactivated the hardware component, and the
    // vehicle went silently uncommandable with every controller still reporting active.
    // can0 showed ERROR-ACTIVE with berr-counter 0/0 and bus-off 0 throughout — the link
    // had never failed.
    //
    // last_write_transient() distinguishes the two so the caller can ride out congestion
    // without tearing the hardware component down. Only meaningful right after a TX
    // function returned false.
    bool last_write_transient() const { return last_write_transient_; }
    bool send_hardware_reset();
    SensorStatus read_sensor_values(int &l_enc, int &r_enc, double imu[6], bool &is_imu_reset,
                                    SteeringFeedback &steer_fb);
    bool set_motor_values(float left_vel, float right_vel);
    bool set_steering(float steer_angle);

private:
    int socket_fd_ = -1;
    uint8_t expected_seq_ = 0;
    bool seq_initialized_ = false;
    // Set at every write site from errno; see last_write_transient() above.
    bool last_write_transient_ = false;
};

#endif
