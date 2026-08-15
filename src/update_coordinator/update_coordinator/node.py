#!/usr/bin/env python3

import os
import sys
import json
import time
import queue
import threading
from collections import namedtuple
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_srvs.srv import Trigger

from .secoc_utils import (
    load_key, FvStore, secoc_verify, secoc_build,
    can_open, can_recv, can_send, SECOC_FRAME_LEN
)

CAN_ID_OTA_REQ         = 0x300   # cluster/host -> Jetson (request / running)
CAN_ID_OTA_APPROVE     = 0x301   # Jetson -> cluster/host (approve / deny)
CAN_ID_ECU_OTA_REQ     = 0x310   # ESP32 -> Jetson (request / running)
CAN_ID_ECU_OTA_APPROVE = 0x311   # Jetson -> ESP32 (approve / deny)

OTA_REQ_MAGIC = 0xA5
OTA_RUN_MAGIC = 0x5A

DID_CLUSTER_REQUEST = 1
DID_CLUSTER_RUNNING = 2
DID_CLUSTER_APPROVE = 3

DID_ECU_REQUEST = 4
DID_ECU_RUNNING = 5
DID_ECU_APPROVE = 6

VERDICT_APPROVE = 1
VERDICT_DENY    = 0

UPDATE_TIMEOUT_S = 300.0
STATE_DIR = "/var/lib/update_coordinator"

# --------------------------------------------------------------------------
# HUMAN APPROVAL
#
# A REQUEST used to be answered with APPROVE the instant its SecOC verified.
# Now it is put in front of the driver first, and only an approval locks the
# vehicle. Deny puts a 0 on the bus, nothing is locked, and the car keeps
# driving — both ECUs already fail closed on a deny, so no firmware change was
# needed on either side.
#
# The transport is the file spool the IVI head unit already watches. See
# IVI/OTA_APPROVAL_PROTOCOL.md; this node is a third producer alongside
# ivi_ota_agent.sh. Files rather than a topic because the head unit is a
# separate image that does not run ROS.
#
# THE ASYMMETRY THAT MATTERS: asking costs time, and the two requesters give us
# wildly different amounts of it.
#
#   cluster  0x300  the QNX bridge waits 60 s   (mcp2515_can_udp.c,
#                                                OTA_APPROVE_TIMEOUT_S)
#   esp32    0x310  the ECU waits 5 s, once     (ECU/ESP32/src/logs/can.c,
#                                                k_msgq_get(..., K_MSEC(5000)))
#
# Five seconds is the whole design constraint. The ESP32 asks at the moment it
# is about to act — right before it reboots into new firmware, or right before
# it drops the STM32 into its bootloader — and if no verdict lands in time it
# abandons the update outright.
#
# The PROMPT IS 5 s FOR EVERY TARGET, deliberately. A driver should not get a
# window whose length depends on which ECU happens to be asking. Five seconds,
# for everyone.
#
# What differs per target is deadline_ms: how long WE wait before giving up and
# applying on_no_verdict. It has to sit above the prompt plus the latency around
# it (notice the offer, drop the card in, poll for the verdict) and below the
# requester's wall with room for the SecOC build and the frame itself.
#
# The cluster has 60 s to play with, so its deadline sits well past the prompt
# and the driver's answer always decides. The ESP32 does not: its wall is 5000 ms
# — the same length as the prompt — so there is nowhere to put a deadline that is
# both above the prompt and below the wall. deadline_ms=4400 therefore expires
# BEFORE the popup's own countdown, and an untouched ESP32 offer is resolved by
# on_no_verdict rather than by the popup self-accepting. Accept and Deny inside
# those 4.4 s are honoured exactly as they are everywhere else; only the tail of
# the countdown is decoration on this one path. The alternative was a shorter
# prompt for the ESP32, and a prompt whose length you cannot predict is one you
# cannot learn to react to.
#
# ui_ms is a REQUEST, not a command: the head unit clamps it to its own maximum
# and can refuse auto-accept entirely.
Budget = namedtuple("Budget", "peer_wait_ms ui_ms deadline_ms")

BUDGETS = {
    "cluster": Budget(peer_wait_ms=60000, ui_ms=5000, deadline_ms=30000),
    "esp32":   Budget(peer_wait_ms=5000,  ui_ms=5000, deadline_ms=4400),
}

# How often we look for a verdict once one is outstanding. At 4 s of ESP32
# budget this is 1.6% of the window, which is noise; the timer only exists
# while something is actually pending.
APPROVAL_POLL_S = 0.1

APPROVAL_DIR = "/run/ota-approval"


class UpdateCoordinator(Node):
    def __init__(self):
        super().__init__("update_coordinator")

        self.declare_parameter("secoc_key", "/etc/ota_secoc.key")
        self.declare_parameter("can_iface", "can0")
        self.declare_parameter("state_dir", STATE_DIR)
        self.declare_parameter("timeout_sec", UPDATE_TIMEOUT_S)

        # Human approval. require_approval=False restores the old behaviour of
        # answering every REQUEST automatically, which is what bring-up on a
        # bench with no head unit attached wants.
        self.declare_parameter("require_approval", True)
        self.declare_parameter("approval_dir", APPROVAL_DIR)
        self.declare_parameter("ui_alive_max_age_s", 10.0)
        self.declare_parameter("on_no_verdict", "approve")

        key_path  = self.get_parameter("secoc_key").value
        iface     = self.get_parameter("can_iface").value
        state_dir = self.get_parameter("state_dir").value
        self._timeout_sec = self.get_parameter("timeout_sec").value

        self._require_approval = bool(self.get_parameter("require_approval").value)
        approval_dir           = self.get_parameter("approval_dir").value
        self._ui_max_age       = float(self.get_parameter("ui_alive_max_age_s").value)

        # What to do when the head unit is up but never answers. "approve"
        # matches ivi_ota_agent.sh's ON_NO_UI and the head unit's own
        # auto-accept: across this whole handshake, "nobody is paying attention"
        # consistently resolves to "go ahead" rather than to a car that cannot
        # be updated. Set "deny" to invert it.
        self._on_no_verdict = str(self.get_parameter("on_no_verdict").value).lower()
        if self._on_no_verdict not in ("approve", "deny"):
            self.get_logger().warn(
                f"on_no_verdict='{self._on_no_verdict}' is not approve|deny — using approve")
            self._on_no_verdict = "approve"

        self._offers_dir   = os.path.join(approval_dir, "offers")
        self._verdicts_dir = os.path.join(approval_dir, "verdicts")
        self._alive_path   = os.path.join(approval_dir, "ui-alive")

        self._key = load_key(key_path)
        self._store = FvStore(state_dir)
        self._self_update_flag = Path(state_dir) / "jetson_updating.flag"

        # Filter in the kernel to the only two IDs _handle_can() dispatches on.
        # Everything else on the bus used to be woken up for and thrown away.
        self.can_sock = can_open(iface, (CAN_ID_OTA_REQ, CAN_ID_ECU_OTA_REQ))
        self._shutdown_event = threading.Event()
        self.can_queue = queue.Queue()
        self._can_thread = threading.Thread(target=self._can_loop, daemon=True)
        self._can_thread.start()

        self._gc = self.create_guard_condition(callback=self._on_can_event)

        self.lock_client   = self.create_client(Trigger, "/emergency_stop/lock")
        self.unlock_client = self.create_client(Trigger, "/emergency_stop/unlock")

        self.create_service(Trigger, "/update_coordinator/self_start", self._on_self_start)
        self.create_service(Trigger, "/update_coordinator/self_done",  self._on_self_done)

        self.active_ecus = {}
        self.is_locked = False
        self._lock_state_pending = False

        self._timeout_timer = None

        # name -> pending approval. At most one per requester.
        self._pending = {}
        self._poll_timer = None
        self._offer_seq = 0

        self._recover_self_update()
        self._sweep_stale_offers()

        self.get_logger().info(
            f"Coordinator ready on {iface} | "
            f"cluster=0x{CAN_ID_OTA_REQ:03X}->0x{CAN_ID_OTA_APPROVE:03X} "
            f"ecu=0x{CAN_ID_ECU_OTA_REQ:03X}->0x{CAN_ID_ECU_OTA_APPROVE:03X}"
        )
        if self._require_approval:
            self.get_logger().info(
                f"Driver approval REQUIRED via {approval_dir} "
                f"(no verdict -> {self._on_no_verdict})"
            )
            if not os.path.isdir(self._verdicts_dir):
                # Worth shouting about: the failure is invisible otherwise. No
                # spool means no prompt can ever be answered, we fall straight
                # through to on_no_verdict, and with the default that looks
                # exactly like the old auto-approving coordinator.
                self.get_logger().error(
                    f"{self._verdicts_dir} does NOT exist — nothing can answer a "
                    f"prompt, so every request will resolve to "
                    f"'{self._on_no_verdict}'. Is ivi-ota-agent (which ships the "
                    f"tmpfiles fragment) installed in this image?"
                )
        else:
            self.get_logger().warn(
                "require_approval=False — every OTA request is approved "
                "automatically, with no driver prompt"
            )

    def _can_loop(self):
        while not self._shutdown_event.is_set() and rclpy.ok():
            try:
                msg = can_recv(self.can_sock, timeout=0.1)
                if msg:
                    self.can_queue.put(msg)
                    self._gc.trigger()
            except Exception as e:
                if not self._shutdown_event.is_set():
                    self.get_logger().error(f"CAN recv error: {e}")
                break

    def _on_can_event(self):
        while not self.can_queue.empty():
            can_id, dlc, data = self.can_queue.get()
            self._handle_can(can_id, dlc, data)
        self._update_lock_state()

    def _handle_can(self, can_id, dlc, data):
        if dlc < SECOC_FRAME_LEN:
            return

        flows = {
            CAN_ID_OTA_REQ: (
                "cluster", DID_CLUSTER_REQUEST, DID_CLUSTER_RUNNING,
                DID_CLUSTER_APPROVE, CAN_ID_OTA_APPROVE,
            ),
            CAN_ID_ECU_OTA_REQ: (
                "esp32", DID_ECU_REQUEST, DID_ECU_RUNNING,
                DID_ECU_APPROVE, CAN_ID_ECU_OTA_APPROVE,
            ),
        }
        flow = flows.get(can_id)
        if flow is None:
            return

        name, did_req, did_run, did_appr, appr_can_id = flow
        magic = data[0]

        if magic == OTA_REQ_MAGIC:
            did, what = did_req, "REQUEST"
        elif magic == OTA_RUN_MAGIC:
            did, what = did_run, "RUNNING"
        else:
            self.get_logger().warn(f"{name} unknown magic 0x{magic:02X} on 0x{can_id:03X}")
            return

        payload, res = secoc_verify(self._key, self._store, did, data)
        if payload is None:
            self.get_logger().warn(f"SecOC REJECT from {name}: {res}")
            return

        slot = payload[1]

        if what == "REQUEST":
            self._begin_approval(name, slot, did_appr, appr_can_id)
        else:
            if name in self.active_ecus:
                del self.active_ecus[name]
            else:
                self.get_logger().warn(f"{name} sent RUNNING but was not active")
            self.get_logger().info(f"{name} RUNNING on slot={slot} — update complete")

        self._update_lock_state()

    # ---------------------------------------------------------------- approval

    def _ui_available(self):
        """True if a head unit is up and able to answer a prompt.

        Stale liveness means the app crashed, was never started, or the image
        does not ship the spool. In every one of those cases nobody can press
        anything, so asking would only burn the requester's timeout before we
        fell back to the default anyway — and on the ESP32's 5 s path that
        difference decides whether the update happens at all.
        """
        if not os.path.isdir(self._offers_dir):
            return False
        try:
            age = time.time() - os.path.getmtime(self._alive_path)
        except OSError:
            return False
        return age <= self._ui_max_age

    def _write_offer(self, oid, target, slot, budget):
        payload = {
            "id":             oid,
            "target":         target,
            "version":        "",
            "slot":           chr(slot) if 32 <= slot < 127 else str(slot),
            "requested_at":   int(time.time()),
            "expires_at":     int(time.time() + budget.deadline_ms / 1000.0),
            "auto_accept_ms": budget.ui_ms,
            "stops_vehicle":  True,
        }
        final = os.path.join(self._offers_dir, oid + ".json")
        tmp   = final + ".tmp"
        try:
            with open(tmp, "w") as f:
                f.write(json.dumps(payload))
                f.flush()
                os.fsync(f.fileno())
            # Rename, never write in place. inotify fires on the first byte, and
            # a head unit that reads a half-written offer logs it as malformed
            # and ignores it — which on the ESP32 path costs the whole budget.
            os.replace(tmp, final)
            return True
        except OSError as e:
            self.get_logger().error(f"cannot write offer {oid}: {e}")
            try:
                os.unlink(tmp)
            except OSError:
                pass
            return False

    def _withdraw_offer(self, oid):
        for path in (os.path.join(self._offers_dir, oid + ".json"),
                     os.path.join(self._verdicts_dir, oid)):
            try:
                os.unlink(path)
            except OSError:
                pass

    def _read_verdict(self, oid):
        """True/False once the head unit answers, None while it has not."""
        try:
            with open(os.path.join(self._verdicts_dir, oid)) as f:
                word = f.read(32).strip().lower()
        except OSError:
            return None
        if word == "approve":
            return True
        if word == "deny":
            return False
        # Absence means nobody is home and we fail open; garbage means something
        # answered and got it wrong, which is not the same thing and must not be
        # read as consent.
        self.get_logger().error(
            f"verdict for {oid} was '{word}', not approve|deny — treating as DENY")
        return False

    def _sweep_stale_offers(self):
        """Drop offers left behind by a previous run of this node.

        Our own offers are meaningless once we restart: the requester has long
        since timed out, and the CAN socket that would carry the verdict is a
        different socket now. Leaving them would put a prompt on the driver's
        screen for a question nobody is waiting on the answer to.
        """
        try:
            names = os.listdir(self._offers_dir)
        except OSError:
            return
        for fn in names:
            if not fn.endswith(".json"):
                continue
            oid = fn[:-len(".json")]
            if oid.split("-", 1)[0] in BUDGETS:
                self._withdraw_offer(oid)
                self.get_logger().info(f"cleared stale offer {oid} from a previous run")

    def _begin_approval(self, name, slot, did_appr, appr_can_id):
        budget = BUDGETS[name]

        if not self._require_approval or not self._ui_available():
            why = ("gate disabled" if not self._require_approval
                   else "no head unit — failing open")
            self._send_verdict(name, did_appr, appr_can_id, slot, True, why)
            self.active_ecus[name] = time.time()
            return

        # A second REQUEST means the first one is dead — neither requester asks
        # twice while it is still waiting. Take the old prompt down rather than
        # leave the driver two cards for the same update.
        if name in self._pending:
            old = self._pending.pop(name)
            self._withdraw_offer(old["id"])
            self.get_logger().warn(
                f"{name} re-requested while {old['id']} was pending — dropped it")

        self._offer_seq += 1
        oid = f"{name}-{int(time.time())}-{self._offer_seq}"

        if not self._write_offer(oid, name, slot, budget):
            self._send_verdict(name, did_appr, appr_can_id, slot, True,
                               "offer could not be written — failing open")
            self.active_ecus[name] = time.time()
            return

        self._pending[name] = {
            "id":          oid,
            "slot":        slot,
            "did_appr":    did_appr,
            "appr_can_id": appr_can_id,
            "t0":          time.time(),
            "deadline":    time.time() + budget.deadline_ms / 1000.0,
        }
        self._start_poll_timer()
        self.get_logger().info(
            f"{name} REQUEST slot={slot} -> asking the driver ({oid}, "
            f"prompt {budget.ui_ms} ms, giving up at {budget.deadline_ms} ms, "
            f"requester waits {budget.peer_wait_ms} ms)"
        )

    def _send_verdict(self, name, did_appr, appr_can_id, slot, approved, why):
        verdict = VERDICT_APPROVE if approved else VERDICT_DENY
        frame, fv = secoc_build(self._key, self._store, did_appr,
                                bytes([verdict, slot]))
        can_send(self.can_sock, appr_can_id, frame)
        self.get_logger().info(
            f"{name} slot={slot} -> {'APPROVE' if approved else 'DENY'} "
            f"on 0x{appr_can_id:03X} ({why}, fv={fv})"
        )

    def _resolve(self, name, approved, why):
        p = self._pending.pop(name, None)
        if p is None:
            return
        self._withdraw_offer(p["id"])
        elapsed_ms = int((time.time() - p["t0"]) * 1000)
        self._send_verdict(name, p["did_appr"], p["appr_can_id"], p["slot"],
                           approved, f"{why} after {elapsed_ms} ms")

        # Only an approval holds the vehicle. A deny means the update is not
        # happening, so there is nothing to stand still for and the driver keeps
        # driving — which is the entire point of offering them the choice.
        if approved:
            self.active_ecus[name] = time.time()

        self._update_lock_state()

    def _start_poll_timer(self):
        if self._poll_timer is None:
            self._poll_timer = self.create_timer(APPROVAL_POLL_S, self._poll_approvals)

    def _stop_poll_timer(self):
        if self._poll_timer is not None:
            self.destroy_timer(self._poll_timer)
            self._poll_timer = None

    def _poll_approvals(self):
        now = time.time()
        for name in list(self._pending):
            p = self._pending[name]
            answer = self._read_verdict(p["id"])
            if answer is not None:
                self._resolve(name, answer,
                              "driver " + ("APPROVED" if answer else "DENIED"))
            elif now >= p["deadline"]:
                self._resolve(name, self._on_no_verdict == "approve",
                              f"no answer, on_no_verdict={self._on_no_verdict},")

        if not self._pending:
            self._stop_poll_timer()

    def _start_timeout_timer(self):
        if self._timeout_timer is None:
            self._timeout_timer = self.create_timer(1.0, self._timeout_check)
            self.get_logger().debug("Timeout timer STARTED")

    def _stop_timeout_timer(self):
        if self._timeout_timer is not None:
            self.destroy_timer(self._timeout_timer)
            self._timeout_timer = None
            self.get_logger().debug("Timeout timer STOPPED")

    def _timeout_check(self):
        now = time.time()
        expired = [
            name for name, t in self.active_ecus.items()
            if now - t > self._timeout_sec
        ]
        for name in expired:
            self.get_logger().error(
                f"{name} update timed out after {self._timeout_sec}s — forcing release"
            )
            del self.active_ecus[name]

        if expired:
            self._update_lock_state()

    def _update_lock_state(self):
        if self._lock_state_pending:
            return

        should_lock = len(self.active_ecus) > 0

        if should_lock:
            self._start_timeout_timer()
        else:
            self._stop_timeout_timer()

        if should_lock and not self.is_locked:
            self._call_service(self.lock_client, "lock")
        elif not should_lock and self.is_locked:
            self._call_service(self.unlock_client, "unlock")

    def _call_service(self, client, label):
        if not client.service_is_ready():
            self.get_logger().warning(f"emergency_stop/{label} not ready")
            return

        self._lock_state_pending = True
        future = client.call_async(Trigger.Request())
        future.add_done_callback(lambda f, lbl=label: self._on_service_done(f, lbl))

    def _on_service_done(self, future, label):
        self._lock_state_pending = False
        try:
            response = future.result()
            if response.success:
                self.is_locked = (label == "lock")
                self.get_logger().info(f"System {'LOCKED' if self.is_locked else 'UNLOCKED'}")
            else:
                self.get_logger().error(f"emergency_stop/{label} rejected: {response.message}")
        except Exception as e:
            self.get_logger().error(f"emergency_stop/{label} call failed: {e}")

    def _on_self_start(self, request, response):
        self.active_ecus["jetson"] = time.time()
        self._update_lock_state()
        self._self_update_flag.parent.mkdir(parents=True, exist_ok=True)
        self._self_update_flag.write_text("1")
        self.get_logger().info("jetson self-update START registered")
        response.success = True
        response.message = "jetson update registered, system locked"
        return response

    def _on_self_done(self, request, response):
        if "jetson" in self.active_ecus:
            del self.active_ecus["jetson"]
        self._self_update_flag.unlink(missing_ok=True)
        self._update_lock_state()
        self.get_logger().info("jetson self-update DONE")
        response.success = True
        response.message = "jetson update complete"
        return response

    def _recover_self_update(self):
        if not self._self_update_flag.exists():
            return
        self._self_update_flag.unlink()
        if "jetson" in self.active_ecus:
            del self.active_ecus["jetson"]
            self.get_logger().info("Recovered from jetson self-update restart")
        self._update_lock_state()

    def destroy_node(self):
        self.get_logger().info("Shutting down coordinator...")
        # Take our prompts down on the way out. We are about to stop listening,
        # so an answer would land nowhere — but the card would stay on the
        # driver's screen looking like it still means something.
        for name, p in list(self._pending.items()):
            self._withdraw_offer(p["id"])
            self.get_logger().warn(f"withdrew pending offer {p['id']} ({name}) — shutting down")
        self._pending.clear()
        self._shutdown_event.set()
        if self.can_sock:
            self.can_sock.close()
        self._can_thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        sys.stdout.reconfigure(line_buffering=True)
    except AttributeError:
        pass

    node = UpdateCoordinator()

    executor = SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
