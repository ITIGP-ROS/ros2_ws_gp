#ifndef ACKERMANN_SERIAL_I2C_COMMS_HPP
#define ACKERMANN_SERIAL_I2C_COMMS_HPP

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <cmath>
#include <iostream>
#include <string>
#include <stdexcept>

class I2CComms
{
public:
  I2CComms() = default;

  ~I2CComms()
  {
    if (connected())
    {
      disconnect();
    }
  }

  bool connect(const std::string &device, int tca_addr = 0x70, int tca_channel = 0, int pca_addr = 0x40)
  {
    device_file_ = device;
    tca_addr_ = tca_addr;
    tca_channel_ = tca_channel;
    pca_addr_ = pca_addr;

    fd_ = open(device_file_.c_str(), O_RDWR);
    if (fd_ < 0)
    {
      std::cerr << "Failed to open I2C device: " << device_file_ << std::endl;
      return false;
    }

    if (!setup_tca()) return false;

    if (!setup_pca()) return false;
    
    return true;
  }

  void disconnect()
  {
    if (fd_ >= 0)
    {
      close(fd_);
      fd_ = -1;
    }
  }

  bool connected() const
  {
    return fd_ >= 0;
  }

  bool set_servo_duty(int channel, int on_counts)
  {
    if (!connected()) return false;

    // PCA9685 is 12-bit (0-4095). Clamp to valid range.
    if (on_counts < 0) on_counts = 0;
    if (on_counts > 4095) on_counts = 4095;

    uint8_t base_reg = 0x06 + (4 * channel);

    // Batch all 4 registers in a single I2C transaction (PCA9685 auto-increment)
    // This is done to reduce I2C overhead and jitter
    uint8_t buf[5] = {
        base_reg,
        0,                            // ON_L
        0,                            // ON_H
        (uint8_t)(on_counts & 0xFF),  // OFF_L
        (uint8_t)((on_counts >> 8) & 0x0F)  // OFF_H (12-bit: lower 4 bits valid)
    };

    if (ioctl(fd_, I2C_SLAVE, pca_addr_) < 0) return false;
    if (write(fd_, buf, 5) != 5) return false;
    
    return true;
  }

private:
  int fd_ = -1;
  std::string device_file_;
  int tca_addr_;
  int tca_channel_;
  int pca_addr_;

  const int PRE_SCALE_REG = 0xFE;
  const int MODE1_REG = 0x00;
  const int MODE2_REG = 0x01;

  bool setup_tca()
  {
    uint8_t data = 1 << tca_channel_;
    if (!write_raw(tca_addr_, data)) return false;
    
    // TCA9548A switching time is fast, but bus stabilization might need a moment
    usleep(50000); 
    return true;
  }

  bool setup_pca()
  {
    const uint8_t ALLCALL = 0x01;
    const uint8_t AI = 0x20;
    const uint8_t SLEEP = 0x10;
    const uint8_t OUTDRV = 0x04;
    
    if (!write_register(pca_addr_, MODE2_REG, OUTDRV)) return false;
    
    if (!write_register(pca_addr_, MODE1_REG, ALLCALL | AI)) return false;
    usleep(5000);
    
    uint8_t mode1 = read_register(pca_addr_, MODE1_REG);
    mode1 = mode1 & ~SLEEP;
    if (!write_register(pca_addr_, MODE1_REG, mode1)) return false;
    usleep(5000);
    
    int prescale = 121; // 25MHz / (4096 * 50) - 1
    
    uint8_t old_mode = read_register(pca_addr_, MODE1_REG);
    uint8_t new_mode = (old_mode & 0x7F) | SLEEP; 
    if (!write_register(pca_addr_, MODE1_REG, new_mode)) return false;
    
    if (!write_register(pca_addr_, PRE_SCALE_REG, prescale)) return false;
    
    if (!write_register(pca_addr_, MODE1_REG, old_mode)) return false;
    usleep(5000);
    
    if (!write_register(pca_addr_, MODE1_REG, old_mode | 0x80)) return false;
    
    return true;
  }


  bool write_raw(int addr, uint8_t data)
  {
    if (ioctl(fd_, I2C_SLAVE, addr) < 0)
    {
      std::cerr << "Failed to set I2C slave address " << std::hex << addr << std::endl;
      return false;
    }
    if (write(fd_, &data, 1) != 1)
    {
      std::cerr << "Failed to write raw byte to " << std::hex << addr << std::endl;
      return false;
    }
    return true;
  }

  bool write_register(int dev_addr, uint8_t reg, uint8_t val)
  {
    if (ioctl(fd_, I2C_SLAVE, dev_addr) < 0)
    {
        std::cerr << "Failed to set I2C slave address " << std::hex << dev_addr << std::endl;
        return false;
    }
    
    uint8_t buf[2];
    buf[0] = reg;
    buf[1] = val;
    
    if (write(fd_, buf, 2) != 2)
    {
        std::cerr << "Failed to write register " << std::hex << (int)reg << " to device " << dev_addr << std::endl;
        return false;
    }
    return true;
  }
  
  uint8_t read_register(int dev_addr, uint8_t reg)
  {
      if (ioctl(fd_, I2C_SLAVE, dev_addr) < 0) return 0;
      
      if (write(fd_, &reg, 1) != 1) return 0;
      
      uint8_t val = 0;
      if (read(fd_, &val, 1) != 1) return 0;
      return val;
  }
};

#endif // ACKERMANN_SERIAL_I2C_COMMS_HPP
