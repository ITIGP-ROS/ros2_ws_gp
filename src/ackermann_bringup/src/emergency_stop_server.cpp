#include <memory>
#include <chrono>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "action_msgs/msg/goal_status_array.hpp"
#include "action_msgs/srv/cancel_goal.hpp"

// Locking the vehicle cancels whatever nav2 was driving towards. That cancel is
// the point -- an OTA update must not run under a moving car. But the goal
// itself was never the problem, and losing it means the driver has to re-send
// it by hand after every update.
//
// So: remember the goal that was in flight at lock time, and re-publish it a
// couple of seconds after the unlock. bt_navigator subscribes to /goal_pose and
// turns it back into a NavigateToPose goal, which is the same path the goal
// originally came in on (rviz's SetGoal tool).
class EmergencyStopServer : public rclcpp::Node
{
public:
  EmergencyStopServer() : Node("emergency_stop_server"), locked_(false), nav_goal_active_(false)
  {
    restore_goal_ = declare_parameter<bool>("restore_goal", true);
    restore_delay_ = declare_parameter<double>("goal_restore_delay_sec", 2.0);

    pub_ = create_publisher<std_msgs::msg::Bool>("emergency_lock", 10);
    goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);

    cancel_goal_client_ = create_client<action_msgs::srv::CancelGoal>("/navigate_to_pose/_action/cancel_goal");

    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/goal_pose", 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) { onGoalPose(*msg); });

    // Matches rcl_action's status QoS (depth 1, reliable, transient local), so
    // we also pick up the current state on late join instead of waiting for the
    // next transition.
    rclcpp::QoS status_qos(rclcpp::KeepLast(1));
    status_qos.reliable().transient_local();
    status_sub_ = create_subscription<action_msgs::msg::GoalStatusArray>(
      "/navigate_to_pose/_action/status", status_qos,
      [this](action_msgs::msg::GoalStatusArray::ConstSharedPtr msg) { onNavStatus(*msg); });

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
        // Snapshot before cancelling -- once the cancel lands, nav_goal_active_
        // goes false and there is nothing left to remember.
        saveGoalForRestore();

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

        scheduleGoalRestore();

        resp->success = true;
        resp->message = "Emergency lock deactivated";
        RCLCPP_INFO(get_logger(), "Emergency lock deactivated");
      });

    if (restore_goal_) {
      RCLCPP_INFO(
        get_logger(), "nav2 goal restore enabled (%.1fs after unlock)", restore_delay_);
    }
  }

private:
  void onGoalPose(const geometry_msgs::msg::PoseStamped & msg)
  {
    last_goal_ = msg;

    // A goal sent while we are locked is the driver's current intent; it
    // supersedes whatever we were holding on to.
    if (locked_ && pending_restore_.has_value()) {
      pending_restore_.reset();
      RCLCPP_INFO(get_logger(), "New goal received while locked — dropping saved goal");
    }
  }

  void onNavStatus(const action_msgs::msg::GoalStatusArray & msg)
  {
    bool active = false;
    for (const auto & status : msg.status_list) {
      if (status.status == action_msgs::msg::GoalStatus::STATUS_ACCEPTED ||
          status.status == action_msgs::msg::GoalStatus::STATUS_EXECUTING)
      {
        active = true;
        break;
      }
    }
    nav_goal_active_ = active;
  }

  void saveGoalForRestore()
  {
    cancelPendingRestore();

    if (!restore_goal_) {
      return;
    }
    // Only worth restoring if the robot was actually driving somewhere. A goal
    // that already succeeded (or was never sent) must not be resurrected.
    if (!nav_goal_active_ || !last_goal_.has_value()) {
      return;
    }

    pending_restore_ = last_goal_;
    RCLCPP_INFO(
      get_logger(), "Saved in-flight nav2 goal (%.2f, %.2f in '%s') for restore after unlock",
      pending_restore_->pose.position.x, pending_restore_->pose.position.y,
      pending_restore_->header.frame_id.c_str());
  }

  void cancelPendingRestore()
  {
    if (restore_timer_) {
      restore_timer_->cancel();
      restore_timer_.reset();
    }
  }

  void scheduleGoalRestore()
  {
    if (!pending_restore_.has_value()) {
      return;
    }

    cancelPendingRestore();
    restore_timer_ = create_wall_timer(
      std::chrono::duration<double>(restore_delay_),
      [this]() {
        restore_timer_->cancel();

        if (locked_ || !pending_restore_.has_value()) {
          return;
        }

        auto goal = *pending_restore_;
        pending_restore_.reset();

        // Re-stamp: the original stamp is now seconds old and nav2 transforms
        // the goal through tf on the way in.
        goal.header.stamp = now();
        goal_pub_->publish(goal);

        RCLCPP_INFO(
          get_logger(), "Republished saved nav2 goal (%.2f, %.2f in '%s')",
          goal.pose.position.x, goal.pose.position.y, goal.header.frame_id.c_str());
      });
  }

  bool locked_;
  bool nav_goal_active_;
  bool restore_goal_;
  double restore_delay_;

  std::optional<geometry_msgs::msg::PoseStamped> last_goal_;
  std::optional<geometry_msgs::msg::PoseStamped> pending_restore_;

  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr status_sub_;
  rclcpp::Client<action_msgs::srv::CancelGoal>::SharedPtr cancel_goal_client_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr restore_timer_;
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
