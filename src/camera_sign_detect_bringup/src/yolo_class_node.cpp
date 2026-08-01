#include "yolo_class_node.hpp"
#include <unordered_set>

YoloClassCanNode::YoloClassCanNode(const rclcpp::NodeOptions& options)
  : Node("yolo_class_can", options) {
  
  declare_parameter("can_interface", "can0");
  declare_parameter("detections_topic", "/detections");
  declare_parameter("conf_threshold", 0.5);
  declare_parameter("can_id", 533);  // 0x215 decimal

  const std::string iface = get_parameter("can_interface").as_string();
  const std::string topic = get_parameter("detections_topic").as_string();
  conf_threshold_ = get_parameter("conf_threshold").as_double();
  can_id_ = static_cast<uint32_t>(get_parameter("can_id").as_int());

  can_ = std::make_unique<CanComms>();
  if (!can_->connect(iface)) {
    RCLCPP_FATAL(get_logger(), "Failed to open CAN interface %s", iface.c_str());
    throw std::runtime_error("CAN init failed");
  }

  sub_ = create_subscription<vision_msgs::msg::Detection2DArray>(
    topic, rclcpp::SensorDataQoS(),
    std::bind(&YoloClassCanNode::onDetections, this, std::placeholders::_1));

  RCLCPP_INFO(get_logger(),
    "Ready: topic=%s -> CAN id=0x%03X on %s (conf≥%.2f)",
    topic.c_str(), can_id_, iface.c_str(), conf_threshold_);
}

void YoloClassCanNode::onDetections(
    const vision_msgs::msg::Detection2DArray::ConstSharedPtr& msg) {
  
  std::unordered_set<int> unique_classes;

  for (const auto& det : msg->detections) {
    for (const auto& res : det.results) {
      if (res.hypothesis.score < conf_threshold_) continue;

      int cls = 0;
      try {
        cls = std::stoi(res.hypothesis.class_id);
      } catch (...) {
        continue;
      }

      if (cls >= 0 && cls <= 14) {
        unique_classes.insert(cls);
      }
    }
  }

  for (int cls : unique_classes) {
    uint8_t data = static_cast<uint8_t>(cls);
    if (!can_->send(can_id_, &data, 1)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Failed to send CAN frame for class %d", cls);
    }
  }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(YoloClassCanNode)