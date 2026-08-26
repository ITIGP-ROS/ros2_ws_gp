#!/usr/bin/env python3
"""imu_scale.py -- correct the Tiva IMU's yaw-rate scale error.

Launched by robot.launch.py alongside imu_broadcaster; it is not something an operator
normally starts by hand. To run it standalone (bench work, or to re-measure k):

    ros2 run ackermann_bringup imu_scale.py                    (k defaults to 1.148)
    ros2 run ackermann_bringup imu_scale.py --ros-args -p k:=1.148

WHY THIS EXISTS -- measured twice, two independent ways
-------------------------------------------------------
The EKF's ONLY yaw source is the Tiva IMU's yaw velocity (ekf.yaml: odom0 fuses x-velocity
only, imu0 fuses yaw-velocity only). That gyro under-reports rotation:

  * Ground run 2026-08-25, 24 turn intervals, AMCL as reference:
    odom/real yaw ratio median 0.871 (stdev 0.080) -> k = 1/0.871 = 1.148.
    After correction the median residual is 6.5% -- a CONSTANT scale error, correctable.
  * Independently, the Tiva IMU was earlier measured to under-rotate ~14%. 1.148 = +14.8%.

The symptom on the vehicle: odometry lags real rotation in every turn, MPPI over-corrects,
AMCL yanks the pose back (14 jumps of ~0.31 m in one run), manoeuvres wobble and abort,
and occasionally the yank lands the footprint in lethal space ("Starting point in lethal
space!" with nothing actually wrong).

WIRING
------
subscribes /imu_broadcaster/imu  ->  multiplies angular_velocity.z by k  ->  publishes
/imu_corrected. ekf.yaml's imu0 is pointed at /imu_corrected.

🔴 IF THIS NODE IS NOT RUNNING, THE EKF HAS NO YAW SOURCE AT ALL and odometry heading
freezes -- ekf.yaml's imu0 names /imu_corrected and nothing else publishes it. That is
why this node lives in the package and is spawned by robot.launch.py rather than being
started by hand: it comes up with imu_broadcaster, in the same OnProcessStart handler, so
the topic exists before the EKF has anything to fuse.

HISTORY: until 2026-08-26 this file sat in /data/collection and was started only by
ground_up.sh, a driver script on a developer's PC. The EKF config shipped in the image
therefore depended on a script that was in no repository -- if ground_up had not been run,
odometry heading silently froze. Moving it here is what closed that gap.

BENCH VERIFICATION (do once, hands required)
--------------------------------------------
Put the vehicle on the floor, mark its heading, rotate it BY HAND through exactly one full
turn (360 deg), and compare /odom yaw before and after:
  ros2 topic echo /odom --field pose.pose.orientation | ...
Without this node odom reads ~313 deg for a real 360. With it, ~360 +- 7%.
The long-term fix is the gyro scale in the Tiva firmware (teammate's code -- recommended,
not edited here).
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu


class ImuScale(Node):
    def __init__(self):
        super().__init__('imu_scale')
        self.declare_parameter('k', 1.148)
        self.declare_parameter('bias_samples', 300)
        self.k = float(self.get_parameter('k').value)
        # 🔴 BIAS REMOVAL. Measured 2026-08-25 with the vehicle parked: the gyro reports a
        # constant -0.00086 rad/s, which integrates to -3.3 deg/min of pure heading drift
        # (predicted -3.38, measured -3.29 -- exact). That drift slowly rotates the world
        # in the odom frame, smearing wall marks across the rolling costmap: the lethal
        # count climbed +120 cells in 158 s with an EMPTY scene, which read as "the
        # costmap does not update correctly". The bias is estimated from the first
        # bias_samples messages. robot.launch.py brings this node up with
        # imu_broadcaster at controller_manager start, i.e. while the vehicle is still
        # parked, so the window is stationary by construction.
        self._bias_n = int(self.get_parameter('bias_samples').value)
        self._bias_acc = 0.0
        self._bias_cnt = 0
        self.bias = None
        q = QoSProfile(depth=50)
        q.reliability = ReliabilityPolicy.BEST_EFFORT
        qr = QoSProfile(depth=50)
        qr.reliability = ReliabilityPolicy.RELIABLE
        self.pub = self.create_publisher(Imu, '/imu_corrected', qr)
        # 🔴 BEST_EFFORT subscription on purpose: it matches BOTH reliable and
        # best-effort publishers, so a QoS change upstream can never silently starve the
        # EKF of yaw. (A RELIABLE subscription would match only reliable publishers.)
        self.create_subscription(Imu, '/imu_broadcaster/imu', self.cb, q)
        self.n = 0
        self.get_logger().info('imu yaw-rate scale k=%.3f  (/imu_broadcaster/imu -> /imu_corrected)' % self.k)

    def cb(self, m):
        if self.bias is None:
            self._bias_acc += m.angular_velocity.z
            self._bias_cnt += 1
            if self._bias_cnt >= self._bias_n:
                self.bias = self._bias_acc / self._bias_cnt
                self.get_logger().info('gyro bias calibrated: %+.5f rad/s (%d samples)'
                                       % (self.bias, self._bias_cnt))
            # publish uncorrected-bias but scaled during calibration (~10 s, parked)
            m.angular_velocity.z *= self.k
            self.pub.publish(m)
            self.n += 1
            return
        m.angular_velocity.z = (m.angular_velocity.z - self.bias) * self.k
        # covariance grows with the correction so the EKF weighs it honestly
        if len(m.angular_velocity_covariance) == 9 and m.angular_velocity_covariance[8] > 0:
            m.angular_velocity_covariance[8] *= self.k * self.k
        self.pub.publish(m)
        self.n += 1


def main():
    rclpy.init()
    n = ImuScale()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
