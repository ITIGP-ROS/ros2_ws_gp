#!/usr/bin/env python3
"""Drive the nav2 lifecycle nodes to ACTIVE by hand.

At the finer 0.05 m search grid the Smac planner spends ~14 s building its Reeds-Shepp
lookup table during configure, which overruns lifecycle_manager's service timeout -- so
bringup aborts even though every node configured correctly. The nodes are already alive and
configured at that point, so this just issues the remaining transitions directly, in
dependency order.

Run by ackermann-navigation.service as ExecStartPost, so it EXITS NON-ZERO unless the
three nodes that make the stack usable -- controller_server, planner_server and
bt_navigator -- all reach active. Without that, systemd would report the unit started
while nav2 sat half-activated and silently ignored every goal, which is the exact failure
this script exists to prevent. Safe to re-run: it is a no-op on nodes already active.
"""
import time
import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import GetState, ChangeState
from lifecycle_msgs.msg import Transition

ORDER = ['controller_server', 'smoother_server', 'planner_server', 'behavior_server',
         'bt_navigator', 'waypoint_follower', 'velocity_smoother']
WANT = {'unconfigured': Transition.TRANSITION_CONFIGURE,
        'inactive': Transition.TRANSITION_ACTIVATE}

rclpy.init()
n = Node('activator')


def state_of(name):
    c = n.create_client(GetState, '/%s/get_state' % name)
    if not c.wait_for_service(timeout_sec=5):
        return None
    f = c.call_async(GetState.Request())
    rclpy.spin_until_future_complete(n, f, timeout_sec=25)
    return f.result().current_state.label if f.result() else None


def transition(name, tid):
    c = n.create_client(ChangeState, '/%s/change_state' % name)
    if not c.wait_for_service(timeout_sec=5):
        return False
    r = ChangeState.Request()
    r.transition.id = tid
    f = c.call_async(r)
    # generous: configure can take ~15 s at the finer grid
    rclpy.spin_until_future_complete(n, f, timeout_sec=90)
    return bool(f.result() and f.result().success)


for name in ORDER:
    for _ in range(3):
        st = state_of(name)
        if st is None:
            print("  %-20s NO SERVICE" % name)
            break
        if st == 'active':
            print("  %-20s active" % name)
            break
        tid = WANT.get(st)
        if tid is None:
            print("  %-20s stuck in '%s'" % (name, st))
            break
        ok = transition(name, tid)
        print("  %-20s %s -> %s" % (name, st, "ok" if ok else "FAILED"))
        time.sleep(0.5)

print("\n  final states:")
final = {}
for name in ORDER:
    final[name] = state_of(name)
    print("    %-20s %s" % (name, final[name]))
rclpy.shutdown()

# These three are the difference between "nav2 is up" and "nav2 accepts goals":
# without bt_navigator nothing receives a NavigateToPose, and without the planner or
# controller nothing acts on one. The other four are useful but not load-bearing.
REQUIRED = ('controller_server', 'planner_server', 'bt_navigator')
missing = [k for k in REQUIRED if final.get(k) != 'active']
if missing:
    print("\n  NOT ACTIVE: %s" % ", ".join(missing))
    raise SystemExit(1)
print("\n  controller_server, planner_server and bt_navigator are active")
