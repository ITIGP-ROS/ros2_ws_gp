#!/usr/bin/env python3
"""
xbox_override.py -- the manual override. Runs ON THE JETSON. Phase 0.1.

WHAT THIS IS FOR
----------------
Until this exists there is NO manual override: nothing publishes /cmd_vel_teleop, so the
top-priority twist_mux slot is empty and once Nav2 drives, the operator cannot take control
except by killing the stack or cutting power. Nothing autonomous runs until D.1-D.4 pass.

CONTROLS
    RB              ENGAGE. Held = "I own the vehicle". Nothing moves without it.
    RT (analogue)   forward,  proportional
    LT (analogue)   reverse,  proportional
    left stick X    steering ONLY

WHY RB AS WELL AS THE TRIGGERS
------------------------------
The triggers are the MOTION deadman -- release them and the vehicle stops within one control
cycle. RB is not a second motion interlock, it is the ARBITRATION boundary.

twist_mux selects the highest-priority input that published within its timeout. This node sits
at priority 150, above recovery (100) and navigation (10). A node that publishes continuously
-- including zero Twists when idle -- is therefore selected ALWAYS, and Nav2 is locked out for
good. twist_mux_topics.yaml:9-13 warns about exactly that.

So "engaged" has to be an explicit state, not an inference from trigger position:

    RB HELD       publish 20 Hz, zeros included. A zero here is an ACTIVE STOP and it holds
                  the mux slot, so Nav2 cannot grab the vehicle back mid-manoeuvre.
    RB RELEASED   publish zeros for --tail seconds (an active stop, 2x the mux timeout),
                  then GO SILENT so twist_mux falls through to Nav2.

Without RB, "zero throttle" and "not driving" are indistinguishable to the mux.

THE DEADMAN, AND WHAT ACTUALLY STOPS THE WHEELS
-----------------------------------------------
Publishing continuously at 20 Hz is what buys the failsafe. If this node dies, the controller
is unplugged, or ssh drops, publication stops and:

    ackermann_controller reference_timeout  0.4 s   <- THIS zeroes the wheels. It fires FIRST.
    twist_mux            timeout            0.5 s   <- deselects the slot

Note twist_mux NEVER publishes a zero on timeout, it just stops relaying -- so the controller's
0.4 s reference_timeout is the real deadman.

*** SAFETY SEAM, KNOWN AND UNSOLVED (MEMORY §4 open question) ***
If this node dies while RB is HELD, the operator believes they own the vehicle. Publication
stops, the mux deselects after 0.5 s, and NAV2 BECOMES SELECTED. The wheels stop first (0.4 s),
but Nav2 can then command motion, with the operator holding a button that does nothing.
Mitigation for D.1-D.4: Nav2 is not running. Do not skip that.

WHY THE TRIGGER TRANSFORM LOOKS BACKWARDS
-----------------------------------------
Through joy_node (SDL2) the triggers REST at +1.0 and go to -1.0 fully pressed. Through raw
joydev they rest at -1.0. Measured both ways on the same controller. So:

    throttle = (1.0 - axis) / 2.0        0.0 released, 1.0 fully pressed

An all-zero axes array -- a Joy message received before the device is initialised, or a
default-constructed one -- evaluates to (1-0)/2 = 0.5, i.e. HALF THROTTLE ON BOTH TRIGGERS.
Hence ARM-ON-RELEASE below: no trigger contributes anything until it has been SEEN at rest.

STEERING IS AN ANGLE, NOT A RATE
--------------------------------
The stick sets a steering angle delta, capped at the measured servo stall onset (0.222 rad),
and wz is derived: wz = vx * tan(delta) / L. Two reasons:
  * wz is then identically 0 whenever vx is 0, so the "what does the controller do when you
    steer while stationary" question NEVER REACHES the controller. drive_segment.py says it
    drives the servo to its clamp; the board runs steering_controllers_library 2.52.1 and the
    laptop 2.53.1, so it could not be settled off-vehicle. This design does not care.
  * a raw wz mapping FLIPS the wheel angle in reverse. An angle does not.
  => The front wheels do NOT move while the vehicle is stationary. Steer and throttle together.

USAGE (on the Jetson, with the drive stack up)
    # 1. get the SDL name -- it is NOT the kernel name, and the board's SDL differs from the
    #    laptop's (board 2.30.1, laptop 2.0.20), so it must be read ON THE BOARD:
    /opt/ros/humble/lib/joy/joy_enumerate_devices

    # 2. start joy_node with that exact name
    ros2 run joy joy_node --ros-args \
        -p device_name:="<name from step 1>" -p autorepeat_rate:=20.0 -p deadzone:=0.05

    # 3. start this
    ./xbox_override.py --device-checked
"""

import argparse
import math
import sys
import time

# --- geometry and caps. Every number here has a source. -----------------------------------
WHEELBASE = 0.23529        # controllers.yaml:37
# 0.286 rad -- the Tiva's own clamp, i.e. the full range the vehicle allows.
# Raised from the old conservative 0.222 on 2026-08-22 at the owner's request and
# verified on the wire the same day (MEMORY §5.1a): 500 frames of 0x120 decoded
# during a two-lock sweep gave RIGHT -0.2874 rad (full) and LEFT +0.2421 rad.
# ⚠️ The LEFT side is cut by ackermann_hardware::write()'s 0.2421 clamp, which is
# DOWNSTREAM of this node -- raising this value cannot open it. Left radius stays
# 0.953 m until Youhana's change lands; right is now 0.796 m.
# ⚠️ The old name STALL_ONSET_RAD was already wrong before this change: it was never
# the stall onset under the post-2026-08-16 servo map. Kept only so the CLI flag and
# any caller still resolve; the meaning is "steering cap".
# Measured pad maps, keyed by (axes, buttons). See resolve_pad_map() for why this is
# detected rather than assumed -- over Bluetooth, ax2 is not a trigger and rests at 0.0,
# which the (1-axis)/2 throttle transform reads as HALF PRESSED (i.e. creeping REVERSE).
PAD_MAPS = {
    (8, 12): dict(rt_axis=5, lt_axis=2, steer_axis=0, rb_button=5,  name="USB"),
    (6, 16): dict(rt_axis=5, lt_axis=4, steer_axis=0, rb_button=10, name="BLUETOOTH"),
}
USB_MAP = PAD_MAPS[(8, 12)]

STEER_CAP_RAD = 0.286
STALL_ONSET_RAD = STEER_CAP_RAD    # backwards-compatible alias
MAX_FWD = 0.20             # proven first-ground-drive speed; half of MPPI vx_max (0.4)
MAX_REV = 0.10             # half of forward. There is NO REAR SENSING on this vehicle.

# A trigger counts as "at rest" above this. Rest is +1.0; fully pressed is -1.0.
REST_THRESH = 0.90
STICK_DEADZONE = 0.12      # left stick X drifted a few counts at rest on the laptop probe
TRIGGER_DEADZONE = 0.05    # see trigger(): rest is +1.0, so joy_node's deadzone does not apply


def main():
    ap = argparse.ArgumentParser(description="Manual override -- publishes /cmd_vel_teleop")
    ap.add_argument("--rate", type=float, default=20.0, help="publish Hz while engaged")
    ap.add_argument("--tail", type=float, default=1.0,
                    help="seconds of zeros after disengage, then silence (2x mux timeout)")
    ap.add_argument("--joy-timeout", type=float, default=0.25,
                    help="no /joy for this long -> disengage (covers an unplugged controller)")
    ap.add_argument("--max-fwd", type=float, default=MAX_FWD)
    ap.add_argument("--max-rev", type=float, default=MAX_REV)
    ap.add_argument("--max-steer", type=float, default=STALL_ONSET_RAD)
    # Indices are PARAMETERS, never assumptions: SDL 2.30 (board) may map differently from
    # SDL 2.0.20 (laptop, where these defaults were measured). Confirm with Step 0-J.
    # Left as None so the code can tell "the operator chose this" from "nobody said".
    # Unset ones are filled in from the MEASURED per-transport map once /joy arrives.
    ap.add_argument("--rb-button", type=int, default=None, help="ENGAGE button index")
    ap.add_argument("--rt-axis", type=int, default=None, help="forward trigger axis")
    ap.add_argument("--lt-axis", type=int, default=None, help="reverse trigger axis")
    ap.add_argument("--steer-axis", type=int, default=None, help="left stick X")
    ap.add_argument("--steer-sign", type=float, default=1.0,
                    help="set -1.0 if steering comes out mirrored")
    ap.add_argument("--topic", default="/cmd_vel_teleop")
    ap.add_argument("--input", choices=["joy","evdev"], default="joy",
                    help="joy = /joy from joy_node (needs xpad+joydev). "
                         "evdev = read /dev/input/eventN DIRECTLY -- needs NEITHER.")
    ap.add_argument("--event", default=None,
                    help="evdev mode: /dev/input/eventN. Default: auto-detect a gamepad.")
    ap.add_argument("--device-checked", action="store_true",
                    help="confirms you ran joy_enumerate_devices ON THIS MACHINE (Step 0-J)")
    # ros2 launch appends "--ros-args ..." whenever the Node action carries a name,
    # parameters or remappings, and this script parses with plain argparse -- which
    # rejects them and exits 2 before a single frame is published. That is a SILENT loss
    # of the manual override: joy_node still comes up, /joy still flows, the unit still
    # reports active, and nothing holds twist_mux priority 150.
    #
    # Split at --ros-args rather than using parse_known_args(): the latter would also
    # swallow a mistyped --max-fwd and fall back to the default speed cap without saying
    # so, and a speed cap that silently reverts is not something this node should allow.
    # sys.argv is left untouched, so rclpy still applies the remaps it finds there.
    _argv = sys.argv[1:]
    if "--ros-args" in _argv:
        _argv = _argv[:_argv.index("--ros-args")]
    args = ap.parse_args(_argv)

    # Same guard drive_segment.py carries. /cmd_vel and /cmd_vel_stamped BYPASS twist_mux and
    # the emergency lock -- odom_test.py and circle_test.py do exactly that and must not be
    # used to move the vehicle.
    if args.topic != "/cmd_vel_teleop":
        sys.exit("REFUSING: only /cmd_vel_teleop keeps twist_mux and the emergency lock in path")

    if not args.device_checked:
        sys.exit(
            "REFUSING: run Step 0-J first, on THIS machine:\n"
            "    /opt/ros/humble/lib/joy/joy_enumerate_devices     <- the SDL name\n"
            "    ros2 topic echo /joy                              <- axes/buttons\n"
            "Confirm: triggers REST at +1.0 and reach -1.0 pressed; note the RB button index.\n"
            "The board's SDL is 2.30.1, the laptop's 2.0.20 -- the name and the indices are\n"
            "NOT guaranteed to match. Then re-run with --device-checked.")

    # ------------------------------------------------------------------ evdev
    # 🔑 WHY THIS MODE EXISTS.
    # joy_node needs SDL, which needs /dev/input/jsN, which needs CONFIG_INPUT_JOYDEV.
    # That module WAS missing when this mode was written (handoff item 26); it has since
    # been baked -- orinivi-image.bb installs kernel-module-xpad, kernel-module-joydev and
    # kernel-module-uhid, and both .ko files are on the unit. So the joy path is the normal
    # one now and this mode is a FALLBACK, not the only option. Kept because it is still
    # the shortest route to a working pad on any image that lacks those modules. But
    # CONFIG_INPUT_EVDEV=y is BUILT IN and demonstrably working, and CONFIG_HID_GENERIC=y
    # and CONFIG_USB_HID=y are built in too.
    #
    # So a GENERIC USB HID gamepad binds and appears as /dev/input/eventN TODAY, with no
    # bake at all. Reading evdev directly skips joy_node, joydev and SDL entirely.
    # (An Xbox pad still needs xpad -- its USB protocol is not standard HID.)
    #
    # Same deadman, same caps, same arming, same topic. The ONLY thing that changes is
    # where the axis values come from.
    joy_from_evdev = None
    if args.input == "evdev":
        # evdev synthesises the Joy message itself using these indices, BEFORE any /joy
        # exists, so they cannot be left unresolved. The synthetic layout is ours by
        # construction, so the USB profile is the right one by definition here.
        for _k in ("rt_axis", "lt_axis", "steer_axis", "rb_button"):
            if getattr(args, _k) is None:
                setattr(args, _k, USB_MAP[_k])
        import struct, fcntl, glob, select
        EVIOCGNAME = 0x82004506
        def devname(path):
            try:
                with open(path, 'rb') as f:
                    buf = bytearray(256)
                    fcntl.ioctl(f, EVIOCGNAME, buf)
                    return buf.split(b'\x00')[0].decode(errors='replace')
            except Exception:
                return ""
        dev = args.event
        if not dev:
            for c in sorted(glob.glob('/dev/input/event*')):
                nm = devname(c).lower()
                if any(k in nm for k in ('pad','joystick','controller','gamepad')):
                    dev = c
                    print(f"[evdev] auto-detected: {c}  '{devname(c)}'")
                    break
        if not dev:
            sys.exit("REFUSING: --input evdev but no gamepad-looking /dev/input/event* found.\n"
                     "  list them with:  for f in /dev/input/event*; do echo -n \"$f \"; "
                     "cat /sys/class/input/$(basename $f)/device/name; done\n"
                     "  then pass it explicitly:  --event /dev/input/eventN")
        joy_from_evdev = dev

    import rclpy
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import Joy

    rclpy.init()
    node = rclpy.create_node("xbox_override")
    pub = node.create_publisher(Twist, args.topic, 10)

    st = {
        "joy": None,          # last Joy message
        "joy_t": 0.0,         # when it arrived (monotonic)
        "seen_joy": False,    # have we EVER received one
        "armed_rt": False,    # trigger observed at rest since start/dropout
        "armed_lt": False,
        "nag_t": 0.0,         # last time we said WHICH trigger is still missing
        "engaged": False,
        "tail_until": 0.0,
        "warned_idx": False,
    }

    # ---------------------------------------------------------------- pad map
    # THE AXIS MAP DEPENDS ON THE TRANSPORT. Both halves measured on this vehicle
    # 2026-08-22 (MEMORY 5.1e), same pad, same board, same SDL:
    #
    #   USB : 8 axes, 12 buttons -- RT ax5, LT ax2, stick ax0, RB btn5
    #   BT  : 6 axes, 16 buttons -- RT ax5, LT ax4, stick ax0, RB btn10
    #
    # 🔴 Getting LT wrong is NOT cosmetic. The throttle transform is (1 - axis)/2, and
    # over Bluetooth ax2 is not a trigger -- it rests at 0.0, which that transform reads
    # as HALF PRESSED. The vehicle would creep in REVERSE for as long as it was engaged.
    #
    # So the map is DETECTED, never assumed. An explicit flag always wins; if the shape
    # matches neither profile we refuse to run rather than guess.
    def resolve_pad_map(n_ax, n_btn):
        prof = PAD_MAPS.get((n_ax, n_btn))
        chosen = {k: getattr(args, k) for k in
                  ("rt_axis", "lt_axis", "steer_axis", "rb_button")}
        missing = [k for k, v in chosen.items() if v is None]
        if not missing:
            node.get_logger().info("pad map: fully specified on the command line")
            return
        if prof is None:
            node.get_logger().error(
                f"UNKNOWN PAD: {n_ax} axes / {n_btn} buttons matches no measured profile "
                f"(USB is 8/12, Bluetooth is 6/16). Refusing to guess -- a wrong LT axis "
                f"drives the vehicle BACKWARDS. Re-run Step 0-J and pass the indices "
                f"explicitly.")
            raise SystemExit(2)
        for k in missing:
            setattr(args, k, prof[k])
        node.get_logger().info(
            f"pad map: {prof['name']} ({n_ax} axes/{n_btn} buttons) -- "
            f"RT ax{args.rt_axis}, LT ax{args.lt_axis}, "
            f"stick ax{args.steer_axis}, RB btn{args.rb_button}"
            + (f" (overridden: {', '.join(k for k in chosen if chosen[k] is not None)})"
               if any(v is not None for v in chosen.values()) else ""))

    def on_joy(msg):
        st["joy"] = msg
        st["joy_t"] = time.monotonic()
        if not st["seen_joy"]:
            st["seen_joy"] = True
            resolve_pad_map(len(msg.axes), len(msg.buttons))
            node.get_logger().info(
                f"/joy is live: {len(msg.axes)} axes, {len(msg.buttons)} buttons. "
                "Release both triggers to ARM.")

    if joy_from_evdev is None:
        node.create_subscription(Joy, "/joy", on_joy, 10)
    else:
        # Read /dev/input/eventN in a thread and synthesise the SAME Joy message the
        # subscription would have delivered, so EVERYTHING downstream -- arming, the
        # both-triggers rule, the tail, the staleness timeout -- is byte-for-byte the
        # code path that joy mode uses. No second implementation to keep in sync.
        import threading, struct, fcntl, array
        EV_KEY, EV_ABS = 0x01, 0x03
        EVIOCGABS = lambda a: 0x80184540 + a          # EVIOCGABS(abs)
        try:
            fd = open(joy_from_evdev, 'rb', buffering=0)
        except PermissionError:
            sys.exit(f"REFUSING: no permission to read {joy_from_evdev}.\n"
                     "  /dev/input/event* is root:input mode 660. On the Jetson we run as\n"
                     "  root so this does not arise. On a laptop, either run as root or\n"
                     "  add yourself to the 'input' group (needs a re-login):\n"
                     "      sudo usermod -aG input $USER")
        except OSError as e:
            sys.exit(f"REFUSING: cannot open {joy_from_evdev}: {e}")
        # Read each axis's real min/max so we can normalise to -1..+1 the way SDL does.
        rng = {}
        for ax in range(0x40):
            try:
                buf = array.array('i', [0]*6)
                fcntl.ioctl(fd, EVIOCGABS(ax), buf, True)
                if buf[1] != buf[2]:
                    rng[ax] = (buf[1], buf[2])
            except Exception:
                pass
        node.get_logger().info(f"[evdev] {joy_from_evdev}: {len(rng)} absolute axes")
        # 🔑 SDL normalises a trigger to +1 RELEASED / -1 PRESSED, and the whole arming
        # and throttle transform below is written for that convention. evdev gives raw
        # min..max with min = released. So map to SDL's convention EXACTLY, or a released
        # trigger reads as FULL THROTTLE.
        axes = [0.0]*max(8, (max(rng)+1) if rng else 8)
        for ax,(lo,hi) in rng.items():
            if ax < len(axes):
                axes[ax] = 1.0 if ax in (args.rt_axis, args.lt_axis) else 0.0
        btns = [0]*16
        SZ = struct.calcsize('llHHi')
        def pump():
            while rclpy.ok():
                try:
                    data = fd.read(SZ)
                except Exception:
                    time.sleep(0.05); continue
                if not data or len(data) < SZ: continue
                _, _, typ, code, val = struct.unpack('llHHi', data)
                if typ == EV_ABS and code in rng and code < len(axes):
                    lo, hi = rng[code]
                    if code in (args.rt_axis, args.lt_axis):
                        # released(lo) -> +1.0 ; pressed(hi) -> -1.0   [SDL convention]
                        axes[code] = 1.0 - 2.0*((val-lo)/(hi-lo) if hi > lo else 0.0)
                    else:
                        axes[code] = 2.0*((val-lo)/(hi-lo) if hi > lo else 0.5) - 1.0
                elif typ == EV_KEY and (code-0x130) < len(btns) and code >= 0x130:
                    btns[code-0x130] = 1 if val else 0
                m = Joy(); m.axes = list(axes); m.buttons = list(btns)
                on_joy(m)
        threading.Thread(target=pump, daemon=True).start()

    def disarm(reason):
        if st["armed_rt"] or st["armed_lt"] or st["engaged"]:
            node.get_logger().warn(f"DISARMED: {reason}. Release both triggers to re-arm.")
        st["armed_rt"] = st["armed_lt"] = False
        if st["engaged"]:
            st["engaged"] = False
            st["tail_until"] = time.monotonic() + args.tail

    def trigger(axis_val, armed_key):
        """(1-x)/2, but ONLY once this trigger has been seen at rest. Never trust a zero."""
        if axis_val >= REST_THRESH:
            if not st[armed_key]:
                st[armed_key] = True
                if st["armed_rt"] and st["armed_lt"]:
                    node.get_logger().info("ARMED. Hold RB to engage.")
        if not st[armed_key]:
            return 0.0
        t = max(0.0, min(1.0, (1.0 - axis_val) / 2.0))
        # Deadzone AFTER the transform. joy_node's own `deadzone` zeroes around 0.0, but these
        # triggers rest at +1.0, so it does not cover them at all. Without this, a trigger
        # resting at 0.98 instead of 1.00 yields t = 0.01 -- nonzero -- and then EVERY press of
        # the OTHER trigger trips the both-pressed rule and pins vx to 0 forever. Fails safe
        # (the vehicle refuses to move) but it would look exactly like a broken D.1.
        return 0.0 if t < TRIGGER_DEADZONE else t

    def tick():
        now = time.monotonic()
        j = st["joy"]

        # No /joy at all, or gone stale -> disengage. Covers an unplugged controller, which
        # simply stops producing messages.
        if j is None or (now - st["joy_t"]) > args.joy_timeout:
            if st["seen_joy"] and j is not None:
                disarm("/joy went stale (controller unplugged? joy_node dead?)")
            elif st["engaged"]:
                disarm("/joy lost")
            _tail_or_silence(now)
            return

        n_ax, n_btn = len(j.axes), len(j.buttons)
        if max(args.rt_axis, args.lt_axis, args.steer_axis) >= n_ax or args.rb_button >= n_btn:
            if not st["warned_idx"]:
                st["warned_idx"] = True
                node.get_logger().error(
                    f"index out of range: /joy has {n_ax} axes, {n_btn} buttons. "
                    "Re-run Step 0-J and pass the right --rt-axis/--lt-axis/--rb-button.")
            _tail_or_silence(now)
            return

        fwd = trigger(j.axes[args.rt_axis], "armed_rt")
        rev = trigger(j.axes[args.lt_axis], "armed_lt")
        armed = st["armed_rt"] and st["armed_lt"]

        # SAY WHICH TRIGGER IS MISSING, and keep saying it.
        #
        # Measured 2026-08-23 on the Bluetooth pad: RT (axes[5]) reports +1.000 in the
        # very FIRST /joy message and arms instantly, while LT (axes[4]) sits at 0.000
        # and stays there until it is physically moved. So the operator presses RT --
        # the throttle, the one they actually want -- sees nothing happen, presses it
        # again, and the node waits forever on a trigger they have no reason to touch.
        # It cost several sessions and looked like "the joystick is broken".
        #
        # The node has always KNOWN which one was outstanding. It just never said.
        if not armed and (now - st["nag_t"]) > 3.0:
            st["nag_t"] = now
            missing = []
            if not st["armed_rt"]:
                missing.append("RT (axes[%d], reads %+.2f)" % (args.rt_axis, j.axes[args.rt_axis]))
            if not st["armed_lt"]:
                missing.append("LT (axes[%d], reads %+.2f)" % (args.lt_axis, j.axes[args.lt_axis]))
            node.get_logger().warning(
                "NOT ARMED -- still waiting on %s. PULL THAT TRIGGER FULLY AND RELEASE IT "
                "(it must read >= %.2f). The other one is already fine."
                % (" and ".join(missing), REST_THRESH))
        rb = bool(j.buttons[args.rb_button])

        if rb and armed:
            if not st["engaged"]:
                node.get_logger().info("ENGAGED -- operator has the vehicle")
            st["engaged"] = True
        else:
            if st["engaged"]:
                node.get_logger().info(
                    f"disengaged -- zeros for {args.tail:.1f}s, then Nav2 may resume")
                st["engaged"] = False
                st["tail_until"] = now + args.tail

        if not st["engaged"]:
            _tail_or_silence(now)
            return

        # Both triggers pressed -> ZERO. Not "last one wins", not undefined.
        if fwd > 0.0 and rev > 0.0:
            vx = 0.0
        else:
            vx = fwd * args.max_fwd - rev * args.max_rev

        stick = st["joy"].axes[args.steer_axis] * args.steer_sign
        if abs(stick) < STICK_DEADZONE:
            stick = 0.0
        delta = max(-args.max_steer, min(args.max_steer, stick * args.max_steer))

        # wz derived from the ANGLE -> identically 0 when vx is 0, and consistent in reverse.
        wz = vx * math.tan(delta) / WHEELBASE

        m = Twist()
        m.linear.x = vx
        m.angular.z = wz
        pub.publish(m)

    def _tail_or_silence(now):
        """Zeros for the tail window (an ACTIVE stop), then genuine silence so Nav2 resumes."""
        if now < st["tail_until"]:
            pub.publish(Twist())

    node.create_timer(1.0 / args.rate, tick)

    node.get_logger().info(
        f"override up: {args.topic} @ {args.rate:.0f} Hz, "
        f"fwd<={args.max_fwd} m/s rev<={args.max_rev} m/s |delta|<={args.max_steer} rad. "
        "Waiting for /joy -- nothing moves until both triggers are seen at rest.")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Always leave a stop behind, even on exception. Publish a burst of zeros so one lost
        # message cannot leave the vehicle commanded.
        try:
            for _ in range(5):
                pub.publish(Twist())
                time.sleep(0.02)
        except Exception:
            pass
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
