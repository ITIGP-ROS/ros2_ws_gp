#pragma once

#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include "can_comms.hpp"

class YoloClassCanNode : public rclcpp::Node {
public:
  explicit YoloClassCanNode(const rclcpp::NodeOptions& options);

private:
  void onDetections(const vision_msgs::msg::Detection2DArray::SharedPtr msg);

  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr sub_;
  std::unique_ptr<CanComms> can_;
  double conf_threshold_{0.5};
  uint32_t can_id_{0x203};
};