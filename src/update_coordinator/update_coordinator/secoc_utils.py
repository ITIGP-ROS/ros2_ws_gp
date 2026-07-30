import os
import struct
import socket
import select

from cryptography.hazmat.primitives.cmac import CMAC
from cryptography.hazmat.primitives.ciphers import algorithms

SECOC_PAYLOAD_LEN = 2
SECOC_MAC_LEN     = 4
SECOC_FRAME_LEN   = 8
SECOC_FV_MAX_JUMP = 0x8000

CAN_FRAME_FMT  = "=IB3x8s"
CAN_FRAME_SIZE = struct.calcsize(CAN_FRAME_FMT)
CAN_SFF_MASK   = 0x7FF


def aes_cmac(key: bytes, msg: bytes) -> bytes:
    c = CMAC(algorithms.AES(key))
    c.update(msg)
    return c.finalize()


def load_key(path):
    with open(path, "rb") as f:
        buf = f.read(128)
    if len(buf) == 16:
        return buf
    txt = buf.rstrip(b" \t\r\n")
    if len(txt) != 32:
        raise ValueError(f"key must be 16 raw bytes or 32 hex chars (got {len(txt)})")
    return bytes.fromhex(txt.decode("ascii"))


class FvStore:
    def __init__(self, directory):
        self.dir = directory
        os.makedirs(directory, exist_ok=True)

    def _path(self, kind, did):
        return os.path.join(self.dir, f"secoc_{kind}fv_{did}")

    def load(self, kind, did):
        try:
            with open(self._path(kind, did)) as f:
                return int(f.read().strip() or 0)
        except (OSError, ValueError):
            return 0

    def store(self, kind, did, fv):
        tmp = self._path(kind, did) + ".tmp"
        with open(tmp, "w") as f:
            f.write(f"{fv}\n")
        os.replace(tmp, self._path(kind, did))


def secoc_mac(key, did, payload, fv):
    m = struct.pack(">H", did) + bytes(payload) + struct.pack(">I", fv)
    return aes_cmac(key, m)[:SECOC_MAC_LEN]


def secoc_build(key, store, did, payload):
    fv = store.load("tx", did) + 1
    mac = secoc_mac(key, did, payload, fv)
    frame = bytes(payload) + mac + struct.pack(">H", fv & 0xFFFF)
    store.store("tx", did, fv)
    return frame, fv


def secoc_verify(key, store, did, frame):
    if len(frame) < SECOC_FRAME_LEN:
        return None, "short frame"

    floor = store.load("rx", did)
    rx_low = struct.unpack(">H", frame[6:8])[0]
    cand = (floor & 0xFFFF0000) | rx_low
    if cand <= floor:
        cand += 0x10000

    mac = secoc_mac(key, did, frame[0:2], cand)
    if mac != frame[2:6]:
        return None, "MAC mismatch"

    if cand - floor > SECOC_FV_MAX_JUMP:
        return None, "freshness outside accept window"

    store.store("rx", did, cand)
    return frame[0:2], cand


def can_open(iface):
    s = socket.socket(socket.PF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    s.bind((iface,))
    return s


def can_send(sock, can_id, data):
    frame = struct.pack(CAN_FRAME_FMT, can_id, len(data), bytes(data).ljust(8, b"\x00"))
    sock.send(frame)


def can_recv(sock, timeout):
    r, _, _ = select.select([sock], [], [], timeout)
    if not r:
        return None
    raw = sock.recv(CAN_FRAME_SIZE)
    can_id, dlc, data = struct.unpack(CAN_FRAME_FMT, raw)
    return (can_id & CAN_SFF_MASK), dlc, data[:dlc]
