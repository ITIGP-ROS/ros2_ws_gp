#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>

#include <cstring>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"

static constexpr uint32_t ECU_CLASSIFICATION_LOG_CAN_ID = 0x240u;

class RoadClassificationNode : public rclcpp::Node
{
public:
  RoadClassificationNode()
  : Node("road_classification_node"), socket_fd_(-1), is_rough_(false), confidence_(0)
  {
    declare_parameter("can_interface", "can0");
    declare_parameter("confidence_threshold", 50);
    declare_parameter("rough_speed_limit", 50.0);
    declare_parameter("smooth_speed_limit", 0.0);
    declare_parameter("publish_rate", 10.0);

    can_interface_ = get_parameter("can_interface").as_string();
    confidence_threshold_ = get_parameter("confidence_threshold").as_int();
    rough_speed_limit_ = get_parameter("rough_speed_limit").as_double();
    smooth_speed_limit_ = get_parameter("smooth_speed_limit").as_double();
    double publish_rate = get_parameter("publish_rate").as_double();

    speed_limit_pub_ = create_publisher<nav2_msgs::msg::SpeedLimit>("speed_limit", rclcpp::QoS(10).reliable());

    if (!connectCAN()) {
      RCLCPP_ERROR(get_logger(), "Failed to connect to CAN interface: %s", can_interface_.c_str());
    }

    timer_ = create_wall_timer(
      std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate)),
      std::bind(&RoadClassificationNode::timerCallback, this));

    RCLCPP_INFO(
      get_logger(),
      "Road classification node started. CAN: %s, confidence_threshold: %d, "
      "rough_limit: %.1f%%, smooth_limit: %.1f%%, rate: %.1f Hz",
      can_interface_.c_str(), confidence_threshold_,
      rough_speed_limit_, smooth_speed_limit_, publish_rate);
  }

  ~RoadClassificationNode()
  {
    disconnectCAN();
  }

private:
  bool connectCAN()
  {
    socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to create CAN socket: %s", strerror(errno));
      return false;
    }

    int flags = fcntl(socket_fd_, F_GETFL, 0);
    if (flags < 0 || fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to set non-blocking mode: %s", strerror(errno));
      close(socket_fd_);
      socket_fd_ = -1;
      return false;
    }

    struct ifreq ifr;
    std::strncpy(ifr.ifr_name, can_interface_.c_str(), IFNAMSIZ - 1);
    ifr.ifr_name[IFNAMSIZ - 1] = '\0';
    if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to get CAN interface index for %s: %s",
        can_interface_.c_str(), strerror(errno));
      close(socket_fd_);
      socket_fd_ = -1;
      return false;
    }

    struct sockaddr_can addr{};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to bind CAN socket: %s", strerror(errno));
      close(socket_fd_);
      socket_fd_ = -1;
      return false;
    }

    struct can_filter rfilter{};
    rfilter.can_id = ECU_CLASSIFICATION_LOG_CAN_ID;
    rfilter.can_mask = CAN_SFF_MASK;
    if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter)) < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to set CAN filter: %s", strerror(errno));
      close(socket_fd_);
      socket_fd_ = -1;
      return false;
    }

    RCLCPP_INFO(get_logger(), "Connected to CAN interface: %s", can_interface_.c_str());
    return true;
  }

  void disconnectCAN()
  {
    if (socket_fd_ >= 0) {
      close(socket_fd_);
      socket_fd_ = -1;
    }
  }

  void readCANFrames()
  {
    if (socket_fd_ < 0) {
      return;
    }

    struct can_frame frame;
    while (true) {
      ssize_t nbytes = read(socket_fd_, &frame, sizeof(frame));
      if (nbytes < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
          break;
        }
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "CAN read error: %s", strerror(errno));
        break;
      }
      if (nbytes < static_cast<ssize_t>(sizeof(frame))) {
        break;
      }

      if (frame.can_id == ECU_CLASSIFICATION_LOG_CAN_ID && frame.can_dlc >= 6) {
        uint8_t label = frame.data[0];
        uint8_t conf = frame.data[1];

        bool was_rough = is_rough_;
        is_rough_ = (label == 1 && conf >= static_cast<uint8_t>(confidence_threshold_));
        confidence_ = conf;

        if (is_rough_ != was_rough) {
          RCLCPP_INFO(get_logger(), "Road classification changed: %s (label=%u, confidence=%u%%)",
            is_rough_ ? "ROUGH" : "SMOOTH", label, conf);
        }
      }
    }
  }

  void timerCallback()
  {
    readCANFrames();

    auto msg = nav2_msgs::msg::SpeedLimit();
    msg.header.stamp = now();
    msg.percentage = true;

    if (is_rough_) {
      msg.speed_limit = rough_speed_limit_;
    } else {
      msg.speed_limit = smooth_speed_limit_;
    }

    speed_limit_pub_->publish(msg);
  }

  int socket_fd_;
  std::string can_interface_;
  int confidence_threshold_;
  double rough_speed_limit_;
  double smooth_speed_limit_;

  bool is_rough_;
  uint8_t confidence_;

  rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr speed_limit_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RoadClassificationNode>());
  rclcpp::shutdown();
  return 0;
}
