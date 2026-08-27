#!/usr/bin/env python3
"""Send goals, measure what the vehicle actually did, print numbers you can compare.

    python3 nav_probe.py 5,4 -5,-3            # two goals, one run
    python3 nav_probe.py --course             # the standard 4-goal course
    python3 nav_probe.py --course --tag base  # also write runs/base.json

Requires sim.launch.py and nav2_sim.launch.py to be up.

WHY THIS EXISTS. Tuning by watching RViz answers "did that look better?", which is not a
question you can answer twice the same way. Every number below is one the vehicle's own
nav2_amcl.yaml already argues about in prose - steering saturation against the 0.286
clamp (item 105), curvature reversals per metre (smoothfix.py, measured 0.46/m on the
ground bag), cross-track error (median 0.033 m on that same bag) - so a simulated run can
be put beside a real one rather than just beside the previous simulated one.

GROUND TRUTH is taken from /gazebo/model_states, not from AMCL. That separation is the
point: it makes "the vehicle went to the wrong place" distinguishable from "the vehicle
went to the right place and AMCL thinks otherwise", which look identical in RViz and have
opposite fixes.

Each goal starts from a reset: the model is teleported back to the start pose, AMCL is
re-seeded there, and both costmaps are cleared. Runs are therefore independent - a run
does not inherit the previous run's ending pose or its accumulated obstacle marks.
"""
import argparse
import json
import math
import os
import statistics
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, TwistStamped
from nav_msgs.msg import Path
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from sensor_msgs.msg import JointState

# From ackermann_bringup/config/controllers.yaml and ackermann_description/urdf/base.xacro.
WHEELBASE = 0.23529
STEER_CLAMP = 0.286
MODEL_NAME = 'ackermann'

# The standard course. Each leg is chosen to exercise something different in arena.world:
#   1  open straight run, nothing in the way
#   2  around the interior stub, forcing a real detour
#   3  a long diagonal across the whole arena
#   4  past the far end of the other stub, into the corner behind it
COURSE = [(5.0, 4.0), (-5.0, -3.0), (6.0, -4.0), (-7.0, 1.5)]


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def ang_diff(a, b):
    return abs(math.atan2(math.sin(a - b), math.cos(a - b)))


def point_seg_dist(p, a, b):
    ax, ay = a
    bx, by = b
    px, py = p
    dx, dy = bx - ax, by - ay
    d2 = dx * dx + dy * dy
    if d2 == 0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / d2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def split_at_cusps(pts):
    """Split a plan into runs of consistent travel direction.

    Smac runs REEDS_SHEPP with enforce_path_inversion, so a plan legitimately contains
    CUSPS - points where the vehicle stops and reverses. At a cusp three consecutive
    points double back on themselves, and the circle through them has a radius near zero.
    Measuring curvature straight down the raw point list therefore reports a ~0.05 m
    minimum radius on any plan containing an inversion, which looks like the planner
    demanding the impossible when it is really just the metric mis-reading a manoeuvre
    the planner is entitled to produce.

    Cusps are found by the sign of the dot product between consecutive segment vectors:
    a direction reversal makes it negative. Curvature is then measured only WITHIN each
    run, where it means what it is supposed to mean.
    """
    runs, cur = [], [pts[0]] if pts else []
    for i in range(1, len(pts) - 1):
        v1 = (pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
        v2 = (pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
        cur.append(pts[i])
        if v1[0] * v2[0] + v1[1] * v2[1] < 0:      # doubled back: this is a cusp
            runs.append(cur)
            cur = [pts[i]]
    if pts:
        cur.append(pts[-1])
        runs.append(cur)
    return [r for r in runs if len(r) >= 3]


def path_metrics(pts):
    """Curvature reversals per metre, and the tightest radius the path actually asks for.

    Reversals are counted on the SIGN of the discrete curvature through consecutive
    triples. A path that alternates left and right arcs down what should be a straight
    line is the specific defect smoothfix.py was written to chase; it shows up here as a
    reversal rate well above zero on a leg with no obstacle in it. Their ground bag
    measured 0.46 reversals/m.

    Both figures are computed per cusp-free run - see split_at_cusps. Cusps themselves are
    reported separately, as a count, because a plan full of inversions is its own finding.
    """
    if len(pts) < 3:
        return None
    length = sum(math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))
    if length < 0.5:
        return None
    runs = split_at_cusps(pts)
    signs_total, reversals, radii = 0, 0, []
    for run in runs:
        signs = []
        for i in range(1, len(run) - 1):
            (x1, y1), (x2, y2), (x3, y3) = run[i - 1], run[i], run[i + 1]
            cross = (x2 - x1) * (y3 - y2) - (y2 - y1) * (x3 - x2)
            a = math.dist(run[i - 1], run[i])
            b = math.dist(run[i], run[i + 1])
            c = math.dist(run[i - 1], run[i + 1])
            if abs(cross) > 1e-9 and a * b * c > 1e-9:
                signs.append(1 if cross > 0 else -1)
                radii.append(a * b * c / (2 * abs(cross)))
        reversals += sum(1 for i in range(1, len(signs)) if signs[i] != signs[i - 1])
        signs_total += len(signs)
    return {'length': length,
            'reversals_per_m': reversals / length,
            'min_radius': min(radii) if radii else float('inf'),
            'cusps': max(0, len(runs) - 1)}


class Probe(Node):
    def __init__(self):
        super().__init__('nav_probe')
        self.set_parameters([rclpy.Parameter('use_sim_time', value=True)])
        self.truth = None
        self.amcl = None
        self.plan = []
        self.samples = []
        self.trace = []
        self.create_subscription(ModelStates, '/gazebo/model_states', self._truth, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._amcl, 10)
        self.create_subscription(Path, '/plan', self._plan, 10)
        self.create_subscription(JointState, '/joint_states', self._joints, qos_profile_sensor_data)
        self.create_subscription(TwistStamped, '/ackermann_controller/reference', self._cmd, 10)
        # The velocity_smoother's own input and output. Measuring BOTH is what separates
        # "MPPI asked for an infeasible turn" from "the smoother made it infeasible":
        # on hardware MPPI never exceeded atan(L/min_turning_r) while the smoother
        # exceeded the steering clamp on 2.5% of commands, purely in ramp transients.
        self.create_subscription(Twist, '/cmd_vel_nav', self._nav_in, 10)
        self.create_subscription(Twist, '/cmd_vel_nav_smoothed', self._nav_out, 10)
        self.nav_in = []
        self.nav_out = []
        self.steer = []      # actual steering angle at the wheels
        self.cmd_steer = []  # steering the command implies: atan(L*wz/vx)
        self.cmd_vx = []
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.set_state = self.create_client(SetEntityState, '/gazebo/set_entity_state')
        self.clear_global = self.create_client(
            ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
        self.clear_local = self.create_client(
            ClearEntireCostmap, '/local_costmap/clear_entirely_local_costmap')
        self.ac = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def _truth(self, m):
        if MODEL_NAME in m.name:
            self.truth = m.pose[m.name.index(MODEL_NAME)]

    def _amcl(self, m):
        self.amcl = m.pose.pose

    def _plan(self, m):
        self.plan = [(p.pose.position.x, p.pose.position.y) for p in m.poses]

    def _joints(self, m):
        for name in ('front_left_steering_joint', 'front_right_steering_joint'):
            if name in m.name:
                self.steer.append(m.position[m.name.index(name)])

    def _delta(self, vx, wz):
        return math.atan(WHEELBASE * wz / abs(vx)) if abs(vx) > 0.02 else None

    def _nav_in(self, m):
        d = self._delta(m.linear.x, m.angular.z)
        if d is not None:
            self.nav_in.append((time.time(), abs(d)))

    def _nav_out(self, m):
        d = self._delta(m.linear.x, m.angular.z)
        if d is not None:
            self.nav_out.append((time.time(), abs(d), m.linear.x))

    def _cmd(self, m):
        vx, wz = m.twist.linear.x, m.twist.angular.z
        self.cmd_vx.append(vx)
        if abs(vx) > 0.02:
            self.cmd_steer.append(math.atan(WHEELBASE * wz / abs(vx)))

    def spin(self, seconds):
        t0 = time.time()
        while time.time() - t0 < seconds:
            rclpy.spin_once(self, timeout_sec=0.05)

    def reset(self, x, y, yaw):
        req = SetEntityState.Request()
        req.state.name = MODEL_NAME
        req.state.pose.position.x = float(x)
        req.state.pose.position.y = float(y)
        req.state.pose.position.z = 0.05
        req.state.pose.orientation.z = math.sin(yaw / 2)
        req.state.pose.orientation.w = math.cos(yaw / 2)
        req.state.reference_frame = 'world'
        if self.set_state.wait_for_service(timeout_sec=5):
            rclpy.spin_until_future_complete(self, self.set_state.call_async(req), timeout_sec=5)
        self.spin(1.0)

        p = PoseWithCovarianceStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.get_clock().now().to_msg()
        p.pose.pose.position.x = float(x)
        p.pose.pose.position.y = float(y)
        p.pose.pose.orientation.z = math.sin(yaw / 2)
        p.pose.pose.orientation.w = math.cos(yaw / 2)
        # AMCL's own default covariance for a hand-set pose.
        p.pose.covariance[0] = p.pose.covariance[7] = 0.25
        p.pose.covariance[35] = 0.068
        self.initial_pose_pub.publish(p)
        self.spin(2.0)

        for cli in (self.clear_global, self.clear_local):
            if cli.wait_for_service(timeout_sec=3):
                rclpy.spin_until_future_complete(
                    self, cli.call_async(ClearEntireCostmap.Request()), timeout_sec=5)
        self.spin(1.0)

    def run_goal(self, gx, gy, timeout, gyaw=0.0):
        self.samples.clear()
        self.trace.clear()
        self.nav_in.clear()
        self.nav_out.clear()
        self.steer.clear()
        self.cmd_steer.clear()
        self.cmd_vx.clear()
        plan_snapshots = []

        g = NavigateToPose.Goal()
        g.pose.header.frame_id = 'map'
        g.pose.pose.position.x = float(gx)
        g.pose.pose.position.y = float(gy)
        g.pose.pose.orientation.z = math.sin(gyaw / 2)
        g.pose.pose.orientation.w = math.cos(gyaw / 2)
        fut = self.ac.send_goal_async(g)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=15)
        gh = fut.result()
        if gh is None or not gh.accepted:
            return {'outcome': 'REJECTED'}

        res = gh.get_result_async()
        t0 = time.time()
        while time.time() - t0 < timeout and not res.done():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.truth:
                tp = (self.truth.position.x, self.truth.position.y)
                xte = None
                if len(self.plan) >= 2:
                    xte = min(point_seg_dist(tp, self.plan[i], self.plan[i + 1])
                              for i in range(len(self.plan) - 1))
                self.samples.append((tp, yaw_of(self.truth.orientation), xte,
                                     (self.amcl.position.x, self.amcl.position.y) if self.amcl else None))
                self.trace.append((round(time.time() - t0, 2),
                                   round(tp[0], 3), round(tp[1], 3),
                                   round(math.hypot(tp[0] - gx, tp[1] - gy), 3),
                                   round(self.cmd_vx[-1], 3) if self.cmd_vx else 0.0,
                                   round(self.steer[-1], 4) if self.steer else 0.0,
                                   len(self.plan)))
                if self.plan and (not plan_snapshots or plan_snapshots[-1] != self.plan):
                    plan_snapshots.append(list(self.plan))

        dur = time.time() - t0
        status = res.result().status if res.done() else None
        outcome = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}.get(status, 'TIMEOUT')

        out = {'goal': [gx, gy], 'outcome': outcome, 'duration_s': round(dur, 1)}
        if self.samples:
            (fx, fy), fyaw, _, famcl = self.samples[-1]
            out['final_xy_error_m'] = round(math.hypot(fx - gx, fy - gy), 3)
            out['final_pose'] = [round(fx, 2), round(fy, 2)]
            if famcl:
                out['loc_error_final_m'] = round(math.hypot(fx - famcl[0], fy - famcl[1]), 3)
            locs = [math.dist(s[0], s[3]) for s in self.samples if s[3]]
            if locs:
                out['loc_error_median_m'] = round(statistics.median(locs), 3)
                out['loc_error_p95_m'] = round(sorted(locs)[int(0.95 * (len(locs) - 1))], 3)
            xtes = [s[2] for s in self.samples if s[2] is not None]
            if xtes:
                out['xtrack_median_m'] = round(statistics.median(xtes), 3)
                out['xtrack_p95_m'] = round(sorted(xtes)[int(0.95 * (len(xtes) - 1))], 3)
            travelled = sum(math.dist(self.samples[i][0], self.samples[i + 1][0])
                            for i in range(len(self.samples) - 1))
            out['distance_travelled_m'] = round(travelled, 2)
        if self.steer:
            a = sorted(abs(s) for s in self.steer)
            out['steer_p95_rad'] = round(a[int(0.95 * (len(a) - 1))], 4)
            out['steer_max_rad'] = round(a[-1], 4)
            out['steer_at_clamp_pct'] = round(
                100.0 * sum(1 for s in a if s >= 0.95 * STEER_CLAMP) / len(a), 1)
        if self.cmd_steer:
            a = sorted(abs(s) for s in self.cmd_steer)
            out['cmd_steer_p95_rad'] = round(a[int(0.95 * (len(a) - 1))], 4)
        if self.cmd_vx:
            out['reverse_pct'] = round(100.0 * sum(1 for v in self.cmd_vx if v < -0.01)
                                       / len(self.cmd_vx), 1)
        # smoother distortion: does the ramp push implied steering past the clamp?
        if self.nav_in and self.nav_out:
            ins = sorted(d for _, d in self.nav_in)
            outs = sorted(d for _, d, _ in self.nav_out)
            out['mppi_steer_max'] = round(ins[-1], 4)
            out['smoothed_steer_max'] = round(outs[-1], 4)
            out['mppi_past_clamp_pct'] = round(
                100.0 * sum(1 for d in ins if d > STEER_CLAMP) / len(ins), 2)
            out['smoothed_past_clamp_pct'] = round(
                100.0 * sum(1 for d in outs if d > STEER_CLAMP) / len(outs), 2)
            pairs, i = [], 0
            for t, d, _vx in self.nav_out:
                while i + 1 < len(self.nav_in) and self.nav_in[i + 1][0] <= t:
                    i += 1
                pairs.append((self.nav_in[i][1], d))
            if pairs:
                out['smoother_inflated_pct'] = round(
                    100.0 * sum(1 for a, b in pairs if b > a + 1e-4) / len(pairs), 1)

        if plan_snapshots:
            pms = [path_metrics(p) for p in plan_snapshots]
            pms = [m for m in pms if m]
            if pms:
                out['plans'] = len(pms)
                out['reversals_per_m'] = round(
                    statistics.median(m['reversals_per_m'] for m in pms), 3)
                out['plan_min_radius_m'] = round(
                    statistics.median(m['min_radius'] for m in pms), 3)
                out['plan_cusps'] = round(statistics.median(m['cusps'] for m in pms), 1)
        return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('goals', nargs='*',
                    help='goals as x,y or x,y,yaw (yaw in rad). Orientation MATTERS: a '
                         'goal behind the vehicle with yaw 0 is satisfied by simply '
                         'reversing to it, which does not test turning at all.')
    ap.add_argument('--course', action='store_true', help='use the standard 4-goal course')
    ap.add_argument('--start', default='0,0,0', help='reset pose x,y,yaw before each goal')
    ap.add_argument('--timeout', type=float, default=90.0)
    ap.add_argument('--tag', help='also write runs/<tag>.json next to this script')
    ap.add_argument('--trace', help='write a per-sample CSV of the LAST goal here')
    args = ap.parse_args()

    goals = ([(x, y, 0.0) for x, y in COURSE] if args.course
             else [tuple(float(v) for v in g.split(',')) + (0.0,) * (3 - len(g.split(',')))
                   for g in args.goals])
    if not goals:
        ap.error('give goals as x,y or pass --course')
    sx, sy, syaw = (float(v) for v in args.start.split(','))

    rclpy.init()
    n = Probe()
    if not n.ac.wait_for_server(timeout_sec=30):
        print('navigate_to_pose not available - is nav2_sim.launch.py running?')
        return 1
    n.spin(2.0)

    results = []
    for gx, gy, gyaw in goals:
        n.reset(sx, sy, syaw)
        r = n.run_goal(gx, gy, args.timeout, gyaw)
        results.append(r)
        print(f"goal ({gx:5.1f},{gy:5.1f})  {r['outcome']:9s}  " + '  '.join(
            f'{k}={v}' for k, v in r.items() if k not in ('goal', 'outcome', 'final_pose')))

    ok = sum(1 for r in results if r['outcome'] == 'SUCCEEDED')
    print(f'\n{ok}/{len(results)} succeeded')
    for key, label in (('final_xy_error_m', 'final xy error'),
                       ('xtrack_median_m', 'cross-track median'),
                       ('steer_p95_rad', 'steering p95'),
                       ('reversals_per_m', 'curvature reversals/m'),
                       ('plan_min_radius_m', 'plan min radius (want >=1.20)'),
                       ('plan_cusps', 'plan cusps (reversals)'),
                       ('smoothed_past_clamp_pct', 'smoothed past steer clamp %'),
                       ('smoother_inflated_pct', 'smoother inflated steering %'),
                       ('loc_error_median_m', 'localisation error median')):
        vals = [r[key] for r in results if key in r]
        if vals:
            print(f'  {label:26s} {statistics.median(vals):.3f}   (per goal: '
                  + ', '.join(f'{v:.3f}' for v in vals) + ')')

    if args.trace:
        with open(args.trace, 'w') as f:
            f.write('t,x,y,dist_to_goal,cmd_vx,steer,plan_len\n')
            for row in n.trace:
                f.write(','.join(str(v) for v in row) + '\n')
        print('wrote', args.trace)

    if args.tag:
        d = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'runs')
        os.makedirs(d, exist_ok=True)
        with open(os.path.join(d, args.tag + '.json'), 'w') as f:
            json.dump(results, f, indent=2)
        print('wrote', os.path.join(d, args.tag + '.json'))

    n.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
