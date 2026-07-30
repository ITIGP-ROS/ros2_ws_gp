#!/usr/bin/env python3

import os
import sys
import time
import queue
import threading
from pathlib import Path

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from std_srvs.srv import Trigger

from .secoc_utils import (
    load_key, FvStore, secoc_verify,
    can_open, can_recv, SECOC_FRAME_LEN
)

CAN_ID_UPDATE_CLUSTER = 0x320
CAN_ID_UPDATE_EDGE    = 0x321

DID_UPDATE_CLUSTER = 4
DID_UPDATE_EDGE    = 5

CMD_START = 0x01
CMD_DONE  = 0x02

UPDATE_TIMEOUT_S = 300.0
STATE_DIR = "/var/lib/update_coordinator"


class UpdateCoordinator(Node):
    def __init__(self):
        super().__init__("update_coordinator")

        self.declare_parameter("secoc_key", "/etc/ota_secoc.key")
        self.declare_parameter("can_iface", "can0")
        self.declare_parameter("state_dir", STATE_DIR)
        self.declare_parameter("timeout_sec", UPDATE_TIMEOUT_S)

        key_path  = self.get_parameter("secoc_key").value
        iface     = self.get_parameter("can_iface").value
        state_dir = self.get_parameter("state_dir").value
        self._timeout_sec = self.get_parameter("timeout_sec").value

        self._key = load_key(key_path)
        self._store = FvStore(state_dir)
        self._self_update_flag = Path(state_dir) / "jetson_updating.flag"

        self.can_sock = can_open(iface)
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

        self._recover_self_update()

        self.get_logger().info(
            f"Coordinator ready on {iface} | "
            f"cluster=0x{CAN_ID_UPDATE_CLUSTER:03X} edge=0x{CAN_ID_UPDATE_EDGE:03X}"
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

        if can_id == CAN_ID_UPDATE_CLUSTER:
            did, name = DID_UPDATE_CLUSTER, "cluster"
        elif can_id == CAN_ID_UPDATE_EDGE:
            did, name = DID_UPDATE_EDGE, "edge"
        else:
            return

        payload, res = secoc_verify(self._key, self._store, did, data)
        if payload is None:
            self.get_logger().warn(f"SecOC REJECT from {name}: {res}")
            return

        cmd = payload[0]
        if cmd == CMD_START:
            self.active_ecus[name] = time.time()
            self.get_logger().info(f"{name} START updating")
        elif cmd == CMD_DONE:
            if name in self.active_ecus:
                del self.active_ecus[name]
                self.get_logger().info(f"{name} DONE updating")
            else:
                self.get_logger().warn(f"{name} sent DONE but was not active")
        else:
            self.get_logger().warn(f"{name} unknown cmd 0x{cmd:02X}")

        self._update_lock_state()

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
