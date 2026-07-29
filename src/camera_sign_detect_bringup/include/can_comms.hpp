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

class CanComms
{
public:
    CanComms() = default;
    ~CanComms() { disconnect(); }

    bool connect(const std::string &interface);
    void disconnect();
    bool connected() const;

    bool send(uint32_t can_id, const uint8_t *data, uint8_t len);

private:
    int socket_fd_ = -1;
};

#endif