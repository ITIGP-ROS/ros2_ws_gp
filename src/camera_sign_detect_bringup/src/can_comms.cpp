#include "can_comms.hpp"

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

bool CanComms::send(uint32_t can_id, const uint8_t *data, uint8_t len)
{
    if (socket_fd_ < 0) {
        std::cerr << "CAN not connected" << std::endl;
        return false;
    }

    if (len > CAN_MAX_DLEN) {
        std::cerr << "CAN payload too long: " << static_cast<int>(len) << std::endl;
        return false;
    }

    struct can_frame frame {};
    frame.can_id = can_id;
    frame.can_dlc = len;
    std::memcpy(frame.data, data, len);

    ssize_t nbytes = write(socket_fd_, &frame, sizeof(struct can_frame));
    if (nbytes != sizeof(struct can_frame)) {
        std::cerr << "CAN write failed: " << strerror(errno) << std::endl;
        return false;
    }

    return true;
}