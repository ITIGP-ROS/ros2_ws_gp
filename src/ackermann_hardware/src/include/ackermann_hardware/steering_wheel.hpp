#ifndef ACKERMANN_SERIAL_STEERING_WHEEL_HPP
#define ACKERMANN_SERIAL_STEERING_WHEEL_HPP

#include <string>
#include <cmath>

class SteeringWheel
{
public:
  std::string left_joint_name = "";
  std::string right_joint_name = "";

  double left_pos = 0.0;
  double right_pos = 0.0;
  double left_cmd = 0.0;
  double right_cmd = 0.0;

  SteeringWheel() = default;

  void setup(const std::string &left_name, const std::string &right_name)
  {
    left_joint_name = left_name;
    right_joint_name = right_name;
  }

  double get_average_steering_cmd()
  {
      return (left_cmd + right_cmd) / 2.0;
  }

  // Convert the rack-mounted pot reading (bicycle steering angle, +left, REP-103)
  // into per-wheel steering joint positions using inverse Ackermann geometry.
  // Keep L and d in sync with controllers.yaml / base.xacro.
  void set_rack_position(double rack_angle)
  {
      constexpr double L = 0.23529; // wheelbase
      constexpr double d = 0.12;    // front pivot track

      double t = std::tan(rack_angle);
      left_pos  = std::atan(L * t / (L - (d / 2.0) * t)); // inner wheel
      right_pos = std::atan(L * t / (L + (d / 2.0) * t)); // outer wheel
  }
};

#endif // ACKERMANN_SERIAL_STEERING_WHEEL_HPP
