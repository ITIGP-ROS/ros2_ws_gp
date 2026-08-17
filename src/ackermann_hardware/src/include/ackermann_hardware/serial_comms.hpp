/*
The MCU sends a 36-byte packed binary struct containing encoder and IMU data.
Format (36 bytes):
  [0]    0xAA (Header 1)
  [1]    0x55 (Header 2)
  [2-5]  Left Motor Ticks (int32_t)
  [6-9]  Right Motor Ticks (int32_t)
  [10-13] Accel X (float)
  [14-17] Accel Y (float)
  [18-21] Accel Z (float)
  [22-25] Gyro X (float)
  [26-29] Gyro Y (float)
  [30-33] Gyro Z (float)
  [34]   IMU Reset Flag (uint8_t, 1 = reset occurred)
  [35]   Checksum (uint8_t, simple algebraic sum of bytes 0-34)
 
Sending to MCU (Motor Commands)
  Motor velocities are sent as an ASCII string terminated by a newline.
  Format: "<left_motor_vel_rad_s> <right_motor_vel_rad_s>\n"
  Example: "10.5 10.5\n"
 
Hardware Reset Command
  Sent to the MCU to trigger a reset of its state and encoders.
  Format: { '\n', '\n', '\n', '\n', 0xAA, 0x55, 0xFF, 0x00 }
*/

#ifndef ACKERMANN_SERIAL_SERIAL_COMMS_HPP
#define ACKERMANN_SERIAL_SERIAL_COMMS_HPP

#include <libserial/SerialPort.h>
#include <iostream>
#include <vector>
#include <cstring>

#pragma pack(push, 1)
struct FirmwareData {
    uint8_t header1; // 0xAA
    uint8_t header2; // 0x55
    int32_t leftTicks;
    int32_t rightTicks;
    float accelX;
    float accelY;
    float accelZ;
    float gyroX;
    float gyroY;
    float gyroZ;
    uint8_t isImuReset;
    uint8_t checksum;
};
#pragma pack(pop)


LibSerial::BaudRate convert_baud_rate(int baud_rate)
{
  switch (baud_rate)
  {
    case 1200: return LibSerial::BaudRate::BAUD_1200;
    case 1800: return LibSerial::BaudRate::BAUD_1800;
    case 2400: return LibSerial::BaudRate::BAUD_2400;
    case 4800: return LibSerial::BaudRate::BAUD_4800;
    case 9600: return LibSerial::BaudRate::BAUD_9600;
    case 19200: return LibSerial::BaudRate::BAUD_19200;
    case 38400: return LibSerial::BaudRate::BAUD_38400;
    case 57600: return LibSerial::BaudRate::BAUD_57600;
    case 115200: return LibSerial::BaudRate::BAUD_115200;
    case 230400: return LibSerial::BaudRate::BAUD_230400;
    default:
      std::cout << "Error! Baud rate " << baud_rate << " not supported! Default to 115200" << std::endl;
      return LibSerial::BaudRate::BAUD_115200;
  }
}

class SerialComms
{

public:

  SerialComms() = default;

  void connect(const std::string &serial_device, int32_t baud_rate, int32_t timeout_ms)
  {  
    timeout_ms_ = timeout_ms;
    serial_conn_.Open(serial_device);
    serial_conn_.SetBaudRate(convert_baud_rate(baud_rate));
  }

  void disconnect()
  {
    serial_conn_.Close();
  }

  bool connected() const
  {
    return serial_conn_.IsOpen();
  }


  void send_hardware_reset()
  {
    std::vector<uint8_t> reset_cmd = {'\n', '\n', '\n', '\n', 0xAA, 0x55, 0xFF, 0x00};
    serial_conn_.FlushIOBuffers();
    serial_conn_.Write(reset_cmd);
  }

  bool read_sensor_values(int &val_1, int &val_2, double imu[6], bool &is_imu_reset)
  {
    size_t available = serial_conn_.GetNumberOfBytesAvailable();
    if (available > 0) 
    {
        // Read all available bytes without blocking
        LibSerial::DataBuffer new_data;
        serial_conn_.Read(new_data, available, 1); // 1ms timeout, effectively non-blocking since we know bytes exist
        buffer_.insert(buffer_.end(), new_data.begin(), new_data.end());
    }

    // We need at least 36 bytes for a full packet.
    if (buffer_.size() < 36) {
        return false;
    }

    // Safety cap: prevent unbounded growth from sustained corrupted data
    if (buffer_.size() > 512) {
        buffer_.erase(buffer_.begin(), buffer_.end() - 35);
        return false;
    }

    // Search backwards for the NEWEST valid packet.
    int best_idx = -1;
    for (int i = buffer_.size() - 36; i >= 0; --i) {
        if (buffer_[i] == 0xAA && buffer_[i+1] == 0x55) {
            FirmwareData fw_data;
            std::memcpy(&fw_data, &buffer_[i], 36);
            
            uint8_t checksum = 0;
            uint8_t* ptr = reinterpret_cast<uint8_t*>(&fw_data);
            for (size_t j = 0; j < sizeof(fw_data) - 1; ++j) {
                checksum += ptr[j];
            }

            if (checksum == fw_data.checksum) {
                best_idx = i;
                break; // Found the freshest valid packet!
            }
        }
    }

    if (best_idx != -1) {
        FirmwareData fw_data;
        std::memcpy(&fw_data, &buffer_[best_idx], 36);

        val_1 = fw_data.leftTicks;
        val_2 = fw_data.rightTicks;
        imu[0] = fw_data.accelX;
        imu[1] = fw_data.accelY;
        imu[2] = fw_data.accelZ;
        imu[3] = fw_data.gyroX;
        imu[4] = fw_data.gyroY;
        imu[5] = fw_data.gyroZ;
        is_imu_reset = (fw_data.isImuReset == 1);
        
        buffer_.erase(buffer_.begin(), buffer_.begin() + best_idx + 36);
        return true;
    } else {
        // No valid packet found, but we have >= 36 bytes (Likely startup trash).
        // Erase everything except the last 35 bytes (in case they are the start of a new packet).
        buffer_.erase(buffer_.begin(), buffer_.end() - 35);
    }
    
    return false;
  }


  void set_motor_values(float val_1, float val_2)
  {
    std::stringstream ss;
    ss << val_1 << " " << val_2 << "\n";

    serial_conn_.Write(ss.str());
  }


private:
    LibSerial::SerialPort serial_conn_;
    int timeout_ms_;
    std::vector<uint8_t> buffer_; // Internal buffer for non-blocking reads
};

#endif // ACKERMANN_SERIAL_SERIAL_COMMS_HPP