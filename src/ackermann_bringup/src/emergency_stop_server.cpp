#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "action_msgs/srv/cancel_goal.hpp"

class EmergencyStopServer : public rclcpp::Node
{
public:
  EmergencyStopServer() : Node("emergency_stop_server"), locked_(false)
  {
    pub_ = create_publisher<std_msgs::msg::Bool>("emergency_lock", 10);

    cancel_goal_client_ = create_client<action_msgs::srv::CancelGoal>("/navigate_to_pose/_action/cancel_goal");

    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      [this]() {
        if (locked_) {
          auto msg = std_msgs::msg::Bool();
          msg.data = true;
          pub_->publish(msg);
        }
      });

    srv_lock_ = create_service<std_srvs::srv::Trigger>(
      "emergency_stop/lock",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
      {
        locked_ = true;
        auto msg = std_msgs::msg::Bool();
        msg.data = true;
        pub_->publish(msg);

        auto cancel_request = std::make_shared<action_msgs::srv::CancelGoal::Request>();
        cancel_goal_client_->async_send_request(cancel_request);

        resp->success = true;
        resp->message = "Emergency lock activated";
        RCLCPP_INFO(get_logger(), "Emergency lock activated");
      });

    srv_unlock_ = create_service<std_srvs::srv::Trigger>(
      "emergency_stop/unlock",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
      {
        locked_ = false;
        auto msg = std_msgs::msg::Bool();
        msg.data = false;
        pub_->publish(msg);
        resp->success = true;
        resp->message = "Emergency lock deactivated";
        RCLCPP_INFO(get_logger(), "Emergency lock deactivated");
      });
  }

private:
  bool locked_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
  rclcpp::Client<action_msgs::srv::CancelGoal>::SharedPtr cancel_goal_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_lock_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_unlock_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EmergencyStopServer>());
  rclcpp::shutdown();
  return 0;
}
