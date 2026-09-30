"""`vla` harness: drive a Robo Use episode with any served VLA policy. Standard library + numpy only (it runs inside the
agent sandbox or the hub agent image, like every harness; it reaches the robot only through the `robo` socket).

One loop for every policy: observe (state + camera images) -> build the policy's observation from a declarative
template -> one request over the policy's own wire protocol -> decode the action chunk -> map it onto the task's
action vector -> execute through a chunk controller -> repeat until the episode ends.

Wire protocols (`wire` in the policy config):
  openpi      openpi policy server (pi0, pi0-FAST, pi0.5): websocket, msgpack with openpi's numpy extension; the server
              sends its metadata first, then one obs dict -> one {"actions": [H, D]} per request
  xpolicylab  XPolicyLab policy server (40+ policies: pi0/pi0.5, GR00T, OpenVLA-OFT, RDT, SmolVLA, ACT, DP, ...):
              websocket, msgpack + msgpack-numpy envelope (hello / reset / infer / trial_end), Observation Data Format v1.0
  openvla     OpenVLA `vla-scripts/deploy.py`: HTTP POST /act, json-numpy body {"image", "instruction", "unnorm_key"} ->
              one action
  gr00t       Isaac GR00T N1.6/N1.7 policy server: ZeroMQ REQ/REP, msgpack + msgpack-numpy,
              {"endpoint": "get_action", "data": {"observation": {...}}} -> [action dict {key: [B, H, D]}, info]
  http-json   plain HTTP JSON (src/robouse/harnesses/vla/lerobot_server.py, for any LeRobot policy including MolmoAct2:
              SmolVLA, pi0, ACT, diffusion): POST /reset, POST /act {"task", "images": {slot: base64 PNG}, "state"} ->
              {"actions": [[...]]}

Config: ROBOUSE_VLA_CONFIG (JSON written by the harness from a preset in presets.json plus overrides), ROBOUSE_VLA_URL
(server address), ROBOUSE_VLA_TOKEN (optional auth), ROBOUSE_TASK_LANGUAGE (the instruction given to the policy).
Output: one JSON line per inference on stdout (agent/stdout.jsonl), converted to the trial's ATIF trajectory.
"""

from __future__ import annotations

import base64
import hashlib
import json
import os
import socket
import ssl
import struct
import sys
import time
import urllib.parse
import urllib.request
import uuid
import zlib

import numpy as np

# ======================================================================================================================
# robo socket
# ======================================================================================================================


def robo(req: dict) -> dict:
    s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    s.settimeout(300)
    try:
        s.connect(os.environ["ROBOUSE_SOCKET"])
        s.sendall((json.dumps(req) + "\n").encode())
        buf = b""
        while not buf.endswith(b"\n"):
            chunk = s.recv(1 << 20)
            if not chunk:
                break
            buf += chunk
        return json.loads(buf.decode() or '{"ok": false, "error": "empty response"}')
    except (FileNotFoundError, ConnectionRefusedError):
        return {"ok": False, "error": "episode server is not running"}
    finally:
        s.close()


def emit(**kw) -> None:
    print(json.dumps(kw, default=lambda o: o.tolist() if hasattr(o, "tolist") else str(o)), flush=True)


# ======================================================================================================================
# msgpack (the subset every policy server uses) and the numpy extensions of openpi, msgpack-numpy and json-numpy
# ======================================================================================================================


def mp_pack(obj, default=None) -> bytes:
    out = bytearray()

    def p(o):
        if o is None:
            out.append(0xC0)
        elif o is True:
            out.append(0xC3)
        elif o is False:
            out.append(0xC2)
        elif isinstance(o, int) and not isinstance(o, bool):
            if 0 <= o < 128:
                out.append(o)
            elif -32 <= o < 0:
                out.append(o & 0xFF)
            elif 0 <= o < 2**8:
                out.extend(b"\xcc" + struct.pack(">B", o))
            elif 0 <= o < 2**16:
                out.extend(b"\xcd" + struct.pack(">H", o))
            elif 0 <= o < 2**32:
                out.extend(b"\xce" + struct.pack(">I", o))
            elif o >= 0:
                out.extend(b"\xcf" + struct.pack(">Q", o))
            elif o >= -(2**7):
                out.extend(b"\xd0" + struct.pack(">b", o))
            elif o >= -(2**15):
                out.extend(b"\xd1" + struct.pack(">h", o))
            elif o >= -(2**31):
                out.extend(b"\xd2" + struct.pack(">i", o))
            else:
                out.extend(b"\xd3" + struct.pack(">q", o))
        elif isinstance(o, float):
            out.extend(b"\xcb" + struct.pack(">d", o))
        elif isinstance(o, str):
            b = o.encode()
            n = len(b)
            if n < 32:
                out.append(0xA0 | n)
            elif n < 2**8:
                out.extend(b"\xd9" + struct.pack(">B", n))
            elif n < 2**16:
                out.extend(b"\xda" + struct.pack(">H", n))
            else:
                out.extend(b"\xdb" + struct.pack(">I", n))
            out.extend(b)
        elif isinstance(o, (bytes, bytearray, memoryview)):
            b = bytes(o)
            n = len(b)
            if n < 2**8:
                out.extend(b"\xc4" + struct.pack(">B", n))
            elif n < 2**16:
                out.extend(b"\xc5" + struct.pack(">H", n))
            else:
                out.extend(b"\xc6" + struct.pack(">I", n))
            out.extend(b)
        elif isinstance(o, (list, tuple)):
            n = len(o)
            if n < 16:
                out.append(0x90 | n)
            elif n < 2**16:
                out.extend(b"\xdc" + struct.pack(">H", n))
            else:
                out.extend(b"\xdd" + struct.pack(">I", n))
            for x in o:
                p(x)
        elif isinstance(o, dict):
            n = len(o)
            if n < 16:
                out.append(0x80 | n)
            elif n < 2**16:
                out.extend(b"\xde" + struct.pack(">H", n))
            else:
                out.extend(b"\xdf" + struct.pack(">I", n))
            for k, v in o.items():
                p(k)
                p(v)
        elif default is not None:
            r = default(o)
            if r is o:
                raise TypeError(f"cannot msgpack {type(o).__name__}")
            p(r)
        else:
            raise TypeError(f"cannot msgpack {type(o).__name__}")

    p(obj)
    return bytes(out)


def mp_unpack(data: bytes, object_hook=None):
    mv = memoryview(data)
    pos = 0

    def take(n):
        nonlocal pos
        b = mv[pos : pos + n]
        pos += n
        return bytes(b)

    def u():
        nonlocal pos
        t = mv[pos]
        pos += 1
        if t < 0x80:
            return t
        if t >= 0xE0:
            return t - 0x100
        if 0x80 <= t <= 0x8F:
            return m(t & 0x0F)
        if 0x90 <= t <= 0x9F:
            return [u() for _ in range(t & 0x0F)]
        if 0xA0 <= t <= 0xBF:
            return take(t & 0x1F).decode()
        simple = {0xC0: None, 0xC2: False, 0xC3: True}
        if t in simple:
            return simple[t]
        if t in (0xC4, 0xC5, 0xC6):
            n = struct.unpack({0xC4: ">B", 0xC5: ">H", 0xC6: ">I"}[t], take({0xC4: 1, 0xC5: 2, 0xC6: 4}[t]))[0]
            return take(n)
        if t in (0xC7, 0xC8, 0xC9):  # ext: kept as (type, bytes)
            n = struct.unpack({0xC7: ">B", 0xC8: ">H", 0xC9: ">I"}[t], take({0xC7: 1, 0xC8: 2, 0xC9: 4}[t]))[0]
            et = struct.unpack(">b", take(1))[0]
            return ("ext", et, take(n))
        if t in (0xD4, 0xD5, 0xD6, 0xD7, 0xD8):
            n = {0xD4: 1, 0xD5: 2, 0xD6: 4, 0xD7: 8, 0xD8: 16}[t]
            et = struct.unpack(">b", take(1))[0]
            return ("ext", et, take(n))
        fmt = {
            0xCA: (">f", 4),
            0xCB: (">d", 8),
            0xCC: (">B", 1),
            0xCD: (">H", 2),
            0xCE: (">I", 4),
            0xCF: (">Q", 8),
            0xD0: (">b", 1),
            0xD1: (">h", 2),
            0xD2: (">i", 4),
            0xD3: (">q", 8),
        }
        if t in fmt:
            f, n = fmt[t]
            return struct.unpack(f, take(n))[0]
        if t in (0xD9, 0xDA, 0xDB):
            n = struct.unpack({0xD9: ">B", 0xDA: ">H", 0xDB: ">I"}[t], take({0xD9: 1, 0xDA: 2, 0xDB: 4}[t]))[0]
            return take(n).decode()
        if t in (0xDC, 0xDD):
            n = struct.unpack(">H" if t == 0xDC else ">I", take(2 if t == 0xDC else 4))[0]
            return [u() for _ in range(n)]
        if t in (0xDE, 0xDF):
            n = struct.unpack(">H" if t == 0xDE else ">I", take(2 if t == 0xDE else 4))[0]
            return m(n)
        raise ValueError(f"msgpack: unsupported type byte 0x{t:02x}")

    def m(n):
        d = {}
        for _ in range(n):
            k = u()
            d[k] = u()
        return object_hook(d) if object_hook else d

    return u()


def _k(d: dict, name: str):
    """msgpack map key lookup that accepts both bin (bytes) and str keys."""
    if name.encode() in d:
        return d[name.encode()]
    return d.get(name)


def np_default_openpi(o):
    if isinstance(o, np.ndarray):
        return {b"__ndarray__": True, b"data": o.tobytes(), b"dtype": o.dtype.str, b"shape": list(o.shape)}
    if isinstance(o, np.generic):
        return {b"__npgeneric__": True, b"data": o.item(), b"dtype": o.dtype.str}
    return o


def np_default_mnp(o):  # lebedov/msgpack-numpy (GR00T, XPolicyLab)
    if isinstance(o, np.ndarray):
        return {b"nd": True, b"type": o.dtype.str, b"kind": b"", b"shape": list(o.shape), b"data": o.tobytes()}
    if isinstance(o, np.generic):
        return {b"nd": False, b"type": o.dtype.str, b"data": o.tobytes()}
    return o


def np_hook(d: dict):
    """Decode arrays in any of the three numpy-in-msgpack conventions."""
    if _k(d, "__ndarray__"):
        dt = _k(d, "dtype")
        return np.frombuffer(_k(d, "data"), dtype=np.dtype(dt.decode() if isinstance(dt, bytes) else dt)).reshape(
            _k(d, "shape")
        )
    if _k(d, "__npgeneric__"):
        dt = _k(d, "dtype")
        return np.dtype(dt.decode() if isinstance(dt, bytes) else dt).type(_k(d, "data"))
    nd = _k(d, "nd")
    if nd is not None and _k(d, "type") is not None and _k(d, "data") is not None:
        t = _k(d, "type")
        t = t.decode() if isinstance(t, bytes) else t
        kind = _k(d, "kind") or b""
        if kind in (b"O", "O", b"V", "V"):
            raise ValueError("refusing object/structured numpy payloads (pickle)")
        if nd:
            return np.frombuffer(_k(d, "data"), dtype=np.dtype(t)).reshape(_k(d, "shape"))
        return np.frombuffer(_k(d, "data"), dtype=np.dtype(t))[0]
    return d


def json_numpy_default(o):
    if isinstance(o, np.ndarray):
        return {
            "__numpy__": base64.b64encode(np.ascontiguousarray(o).tobytes()).decode(),
            "dtype": o.dtype.str,
            "shape": list(o.shape),
        }
    if isinstance(o, np.generic):
        return o.item()
    raise TypeError(type(o).__name__)


def json_numpy_hook(d: dict):
    if "__numpy__" in d:
        a = np.frombuffer(base64.b64decode(d["__numpy__"]), dtype=np.dtype(d["dtype"]))
        return a.reshape(d["shape"]) if d["shape"] else a[0]
    return d


# ======================================================================================================================
# websocket client (RFC 6455) and ZeroMQ REQ client (ZMTP 3.0, NULL mechanism)
# ======================================================================================================================


class WebSocket:
    def __init__(self, url: str, headers: dict | None = None, timeout: float = 120.0):
        u = urllib.parse.urlparse(url)
        port = u.port or (443 if u.scheme == "wss" else 80)
        raw = socket.create_connection((u.hostname, port), timeout=timeout)
        self.sock = (
            ssl.create_default_context().wrap_socket(raw, server_hostname=u.hostname) if u.scheme == "wss" else raw
        )
        key = base64.b64encode(os.urandom(16)).decode()
        path = (u.path or "/") + (f"?{u.query}" if u.query else "")
        hdr = "".join(f"{k}: {v}\r\n" for k, v in (headers or {}).items())
        self.sock.sendall(
            (
                f"GET {path} HTTP/1.1\r\nHost: {u.hostname}:{port}\r\nUpgrade: websocket\r\nConnection: Upgrade\r\n"
                f"Sec-WebSocket-Key: {key}\r\nSec-WebSocket-Version: 13\r\n{hdr}\r\n"
            ).encode()
        )
        resp = b""
        while b"\r\n\r\n" not in resp:
            c = self.sock.recv(4096)
            if not c:
                raise ConnectionError("websocket handshake: connection closed")
            resp += c
        head, self._buf = resp.split(b"\r\n\r\n", 1)
        status = head.split(b"\r\n")[0]
        if b" 101 " not in status:
            raise ConnectionError("websocket handshake failed: " + status.decode(errors="replace"))
        accept = base64.b64encode(
            hashlib.sha1((key + "258EAFA5-E914-47DA-95CA-C5AB0DC85B11").encode()).digest()
        ).decode()
        if accept.encode() not in head:
            raise ConnectionError("websocket handshake: bad Sec-WebSocket-Accept")

    def _read(self, n: int) -> bytes:
        while len(self._buf) < n:
            c = self.sock.recv(max(65536, n - len(self._buf)))
            if not c:
                raise ConnectionError("websocket closed by the server")
            self._buf += c
        out, self._buf = self._buf[:n], self._buf[n:]
        return out

    def _frame(self, opcode: int, payload: bytes) -> None:
        n = len(payload)
        head = bytes([0x80 | opcode])
        if n < 126:
            head += bytes([0x80 | n])
        elif n < 2**16:
            head += bytes([0x80 | 126]) + struct.pack(">H", n)
        else:
            head += bytes([0x80 | 127]) + struct.pack(">Q", n)
        mask = os.urandom(4)
        arr = np.frombuffer(payload, dtype=np.uint8)
        m = np.frombuffer((mask * (n // 4 + 1))[:n], dtype=np.uint8)
        self.sock.sendall(head + mask + (arr ^ m).tobytes())

    def send(self, data) -> None:
        self._frame(0x1 if isinstance(data, str) else 0x2, data.encode() if isinstance(data, str) else bytes(data))

    def recv(self):
        parts, op0 = [], None
        while True:
            b0, b1 = self._read(2)
            op, fin = b0 & 0x0F, b0 & 0x80
            n = b1 & 0x7F
            if n == 126:
                n = struct.unpack(">H", self._read(2))[0]
            elif n == 127:
                n = struct.unpack(">Q", self._read(8))[0]
            mask = self._read(4) if b1 & 0x80 else None
            payload = self._read(n)
            if mask:
                payload = (
                    np.frombuffer(payload, np.uint8) ^ np.frombuffer((mask * (n // 4 + 1))[:n], np.uint8)
                ).tobytes()
            if op == 0x9:  # ping
                self._frame(0xA, payload)
                continue
            if op == 0xA:
                continue
            if op == 0x8:
                raise ConnectionError(f"websocket closed by the server: {payload[2:].decode(errors='replace')}")
            if op0 is None:
                op0 = op
            parts.append(payload)
            if fin:
                data = b"".join(parts)
                return data.decode() if op0 == 0x1 else data

    def close(self) -> None:
        try:
            self._frame(0x8, struct.pack(">H", 1000))
        except OSError:
            pass
        self.sock.close()


class ZmqReq:
    """ZeroMQ REQ socket over ZMTP 3.0 with the NULL security mechanism (what pyzmq uses by default)."""

    def __init__(self, url: str, timeout: float = 120.0):
        u = urllib.parse.urlparse(url)
        self.sock = socket.create_connection((u.hostname, u.port or 5555), timeout=timeout)
        greeting = b"\xff" + b"\x00" * 8 + b"\x7f" + bytes([3, 0]) + b"NULL".ljust(20, b"\x00") + b"\x00" + b"\x00" * 31
        self.sock.sendall(greeting)
        peer = self._read(64)
        if peer[0] != 0xFF or peer[9] != 0x7F:
            raise ConnectionError("not a ZeroMQ peer")
        self._send_frame(b"\x05READY" + self._prop(b"Socket-Type", b"REQ"), command=True)
        while True:  # the peer's READY
            flags, body = self._recv_frame()
            if flags & 0x04:
                break

    @staticmethod
    def _prop(name: bytes, value: bytes) -> bytes:
        return bytes([len(name)]) + name + struct.pack(">I", len(value)) + value

    def _read(self, n: int) -> bytes:
        buf = b""
        while len(buf) < n:
            c = self.sock.recv(n - len(buf))
            if not c:
                raise ConnectionError("zmq peer closed the connection")
            buf += c
        return buf

    def _send_frame(self, body: bytes, more: bool = False, command: bool = False) -> None:
        flags = (0x01 if more else 0) | (0x04 if command else 0)
        if len(body) > 255:
            self.sock.sendall(bytes([flags | 0x02]) + struct.pack(">Q", len(body)) + body)
        else:
            self.sock.sendall(bytes([flags, len(body)]) + body)

    def _recv_frame(self):
        flags = self._read(1)[0]
        n = struct.unpack(">Q", self._read(8))[0] if flags & 0x02 else self._read(1)[0]
        return flags, self._read(n)

    def request(self, payload: bytes) -> bytes:
        self._send_frame(b"", more=True)  # REQ envelope delimiter
        self._send_frame(payload)
        frames = []
        while True:
            flags, body = self._recv_frame()
            if flags & 0x04:  # command (e.g. PING): ignore
                continue
            frames.append(body)
            if not flags & 0x01:
                break
        if frames and frames[0] == b"":
            frames = frames[1:]
        return b"".join(frames)

    def close(self) -> None:
        self.sock.close()


# ======================================================================================================================
# images
# ======================================================================================================================


def decode_png(path: str) -> np.ndarray:
    try:
        from PIL import Image

        return np.asarray(Image.open(path).convert("RGB"))
    except ImportError:
        pass
    with open(path, "rb") as f:
        data = f.read()
    assert data[:8] == b"\x89PNG\r\n\x1a\n", "not a PNG"
    pos, idat, w, h, ctype = 8, b"", 0, 0, 2
    while pos < len(data):
        n = struct.unpack(">I", data[pos : pos + 4])[0]
        typ = data[pos + 4 : pos + 8]
        body = data[pos + 8 : pos + 8 + n]
        pos += 12 + n
        if typ == b"IHDR":
            w, h, depth, ctype, _, _, interlace = struct.unpack(">IIBBBBB", body)
            assert depth == 8 and interlace == 0 and ctype in (2, 6), "only 8-bit RGB/RGBA non-interlaced PNGs"
        elif typ == b"IDAT":
            idat += body
    ch = 3 if ctype == 2 else 4
    raw = zlib.decompress(idat)
    stride = w * ch
    out = np.zeros((h, stride), dtype=np.uint8)
    prev = np.zeros(stride, dtype=np.int32)
    for y in range(h):
        f = raw[y * (stride + 1)]
        line = np.frombuffer(raw, np.uint8, stride, y * (stride + 1) + 1).astype(np.int32)
        if f == 1:
            for x in range(ch, stride):
                line[x] = (line[x] + line[x - ch]) & 0xFF
        elif f == 2:
            line = (line + prev) & 0xFF
        elif f == 3:
            for x in range(stride):
                line[x] = (line[x] + ((line[x - ch] if x >= ch else 0) + prev[x]) // 2) & 0xFF
        elif f == 4:
            for x in range(stride):
                a = line[x - ch] if x >= ch else 0
                b, c = prev[x], (prev[x - ch] if x >= ch else 0)
                pa, pb, pc = abs(b - c), abs(a - c), abs(a + b - 2 * c)
                line[x] = (line[x] + (a if pa <= pb and pa <= pc else b if pb <= pc else c)) & 0xFF
        out[y] = line
        prev = line
    return out.reshape(h, w, ch)[:, :, :3]


def resize(img: np.ndarray, size) -> np.ndarray:
    """Bilinear resize to (width, height); `pad` keeps the aspect ratio with black borders (openpi resize_with_pad)."""
    w, h = int(size[0]), int(size[1])
    H, W = img.shape[:2]
    ys = np.clip((np.arange(h) + 0.5) * H / h - 0.5, 0, H - 1)
    xs = np.clip((np.arange(w) + 0.5) * W / w - 0.5, 0, W - 1)
    y0, x0 = np.floor(ys).astype(int), np.floor(xs).astype(int)
    y1, x1 = np.minimum(y0 + 1, H - 1), np.minimum(x0 + 1, W - 1)
    wy, wx = (ys - y0)[:, None, None], (xs - x0)[None, :, None]
    f = img.astype(np.float32)
    top = f[y0][:, x0] * (1 - wx) + f[y0][:, x1] * wx
    bot = f[y1][:, x0] * (1 - wx) + f[y1][:, x1] * wx
    return np.clip(top * (1 - wy) + bot * wy + 0.5, 0, 255).astype(np.uint8)


def resize_with_pad(img: np.ndarray, size) -> np.ndarray:
    w, h = int(size[0]), int(size[1])
    H, W = img.shape[:2]
    s = min(w / W, h / H)
    nw, nh = max(1, int(round(W * s))), max(1, int(round(H * s)))
    r = resize(img, (nw, nh))
    out = np.zeros((h, w, 3), dtype=np.uint8)
    oy, ox = (h - nh) // 2, (w - nw) // 2
    out[oy : oy + nh, ox : ox + nw] = r
    return out


def png_b64(img: np.ndarray) -> str:
    def chunk(t, b):
        return struct.pack(">I", len(b)) + t + b + struct.pack(">I", zlib.crc32(t + b) & 0xFFFFFFFF)

    h, w = img.shape[:2]
    raw = b"".join(b"\x00" + img[y].astype(np.uint8).tobytes() for y in range(h))
    png = (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
        + chunk(b"IDAT", zlib.compress(raw, 6))
        + chunk(b"IEND", b"")
    )
    return base64.b64encode(png).decode()


# ======================================================================================================================
# observation template and action mapping
# ======================================================================================================================


def _set(d: dict, dotted: str, value, nested: bool) -> None:
    if not nested:
        d[dotted] = value
        return
    parts = dotted.split(".")
    for p in parts[:-1]:
        d = d.setdefault(p, {})
    d[parts[-1]] = value


def _state_vector(state: dict, fields: list) -> np.ndarray:
    """Concatenate state fields. A field is a dotted name, or {"const": [...]}, or {"field": name, "scale": s, "fn": f}
    (scale, then one of STATE_FNS) for embodiments that report the quantity differently."""
    vals = []
    for f in fields:
        if isinstance(f, dict):
            if "const" in f:
                vals.extend(np.asarray(f["const"], dtype=np.float64).ravel().tolist())
                continue
            v = _state_vector(state, [f["field"]]) * float(f.get("scale", 1.0))
            vals.extend((STATE_FNS[f["fn"]](v) if f.get("fn") else v).ravel().tolist())
            continue
        v = state
        for part in f.split("."):
            v = v[part]
        if isinstance(v, dict):
            v = list(v.values())
        vals.extend(np.asarray(v, dtype=np.float64).ravel().tolist())
    return np.asarray(vals, dtype=np.float64)


def quat_to_axis_angle(q) -> np.ndarray:
    """Quaternion (x, y, z, w) -> axis-angle (3), as robosuite's quat2axisangle (LIBERO's state)."""
    q = np.asarray(q, dtype=np.float64)[:4]
    w = float(np.clip(q[3], -1.0, 1.0))
    den = np.sqrt(1.0 - w * w)
    return np.zeros(3) if den < 1e-10 else q[:3] * 2.0 * np.arccos(w) / den


# state transforms for embodiments that expose the same quantity in another form (zero-shot mappings):
# a hand quaternion to LIBERO's axis-angle, one gripper width to two finger joints (+w/2, -w/2)
STATE_FNS = {
    "quat_to_axis_angle": quat_to_axis_angle,
    "width_to_fingers": lambda v: np.asarray([v[0] / 2.0, -v[0] / 2.0], dtype=np.float64),
}


class Observer:
    """Builds the policy observation from `robo observe`, following cfg["obs"] ({dotted key: source})."""

    def __init__(self, cfg: dict, info: dict, language: str):
        self.cfg, self.info, self.language = cfg, info, language
        self.cams = sorted(
            {s.get("camera") or "" for s in cfg["obs"].values() if isinstance(s, dict) and "camera" in s}
        )
        self.encoding = cfg.get("image_encoding", "array")

    def observe(self):
        images, state, steps = {}, None, None
        for cam in self.cams or [""]:
            r = robo({"op": "observe", "image": bool(self.cams), **({"camera": cam} if cam else {})})
            if not r.get("ok"):
                return None, r.get("error")
            state, steps = r["result"]["state"], r["result"].get("steps_used")
            if self.cams:
                p = r["result"]["image_path"]
                images[cam] = decode_png(p if isinstance(p, str) else p[0])
        return (images, state, steps), None

    def build(self, images: dict, state: dict) -> dict:
        out: dict = {}
        nested = bool(self.cfg.get("nested_obs", False))
        for key, src in self.cfg["obs"].items():
            if not isinstance(src, dict):
                _set(out, key, src, nested)
                continue
            if "const" in src:
                val = src["const"]
            elif "text" in src:
                val = self.language
                if src.get("batch"):
                    val = [[val]] if src["batch"] == 2 else [val]
            elif "camera" in src:
                img = images[src.get("camera") or ""]
                flip = src.get(
                    "flip"
                )  # "h" mirror left-right, "v" upside down, "hv" rotate 180 (robouse images are upright)
                if flip in ("h", "hv"):
                    img = img[:, ::-1]
                if flip in ("v", "hv"):
                    img = img[::-1]
                if src.get("size"):
                    img = (resize_with_pad if src.get("pad") else resize)(img, src["size"])
                img = np.ascontiguousarray(img, dtype=np.uint8)
                if src.get("encoding", self.encoding) == "png_b64":
                    val = png_b64(img)
                else:
                    val = img.reshape((1,) * int(src.get("batch", 0)) + img.shape)
            elif "state" in src:
                v = _state_vector(state, src["state"])
                if src.get("fn"):
                    v = STATE_FNS[src["fn"]](v)
                if src.get("slice"):
                    v = v[int(src["slice"][0]) : int(src["slice"][1])]
                if src.get("scale") is not None:
                    v = v * np.asarray(src["scale"], dtype=np.float64)
                v = v.astype(src.get("dtype", "float32"))
                val = (
                    v.reshape((1,) * int(src.get("batch", 0)) + v.shape)
                    if src.get("encoding") != "list"
                    else v.tolist()
                )
            else:
                raise ValueError(f"obs source for {key!r} needs one of camera/state/text/const")
            _set(out, key, val, nested)
        return out


class ActionMap:
    """Policy action vector -> the task's action vector: select/reorder (index), scale, offset, sign, gripper convention,
    padding, then clip to the task's bounds from `robo info` (or cfg["action"]["clip"])."""

    def __init__(self, cfg: dict, info: dict):
        a = cfg.get("action", {})
        self.a = a
        self.dim = len(info["action"]["names"])
        self.low = np.asarray(info["action"]["low"], dtype=float)
        self.high = np.asarray(info["action"]["high"], dtype=float)
        self.relative_to = a.get(
            "relative_to"
        )  # e.g. "joint_pos": policy outputs deltas, the task takes absolute targets

    def expected_policy_dim(self):
        return self.a.get("policy_dim")

    def __call__(self, v, state: dict | None = None) -> list:
        v = np.array(v, dtype=np.float64).ravel()  # a copy: decoded arrays may be read-only views
        if self.a.get("index") is not None:
            v = np.asarray([v[i] if i >= 0 else 0.0 for i in self.a["index"]], dtype=np.float64)
        if self.a.get("sign") is not None:
            v = v * np.asarray(self.a["sign"], dtype=np.float64)
        if self.a.get("scale") is not None:
            v = v * np.asarray(self.a["scale"], dtype=np.float64)
        if self.a.get("offset") is not None:
            v = v + np.asarray(self.a["offset"], dtype=np.float64)
        g = self.a.get("gripper")  # {"index": i, "from": "0..1 open=1", "to": "-1 open, +1 close"}
        if g:
            i = g["index"]
            if g.get("mode") == "openvla":  # OpenVLA: [0, 1] with 1 = open -> binarized, -1 = open, +1 = close
                v[i] = -np.sign(2 * v[i] - 1) if v[i] != 0.5 else -1.0
            elif g.get("mode") == "binarize":
                v[i] = 1.0 if v[i] > g.get("threshold", 0.0) else -1.0
        if self.relative_to and state is not None:
            v = v + _state_vector(state, [self.relative_to])[: len(v)]
        if len(v) < self.dim:
            v = np.concatenate([v, np.full(self.dim - len(v), float(self.a.get("pad", 0.0)))])
        v = v[: self.dim]
        clip = self.a.get("clip")
        lo, hi = (np.full(self.dim, clip[0]), np.full(self.dim, clip[1])) if clip else (self.low, self.high)
        return [float(x) for x in np.clip(v, lo, hi)]


# ======================================================================================================================
# wires
# ======================================================================================================================


class Wire:
    name = "base"

    def __init__(self, cfg: dict, url: str):
        self.cfg, self.url = cfg, url
        self.headers = {}
        tok = os.environ.get("ROBOUSE_VLA_TOKEN", "")
        if tok:
            hdr = cfg.get("token_header", "Authorization")
            self.headers[hdr] = cfg.get("token_format", "Api-Key {}").format(tok) if hdr == "Authorization" else tok
        self.metadata: dict = {}

    def reset(self, episode: str) -> None:
        pass

    def infer(self, obs: dict) -> np.ndarray:
        raise NotImplementedError

    def close(self) -> None:
        pass


class OpenPI(Wire):
    name = "openpi"

    def __init__(self, cfg, url):
        super().__init__(cfg, url)
        self.ws = _retry(lambda: WebSocket(url, self.headers, timeout=float(cfg.get("timeout_s", 120))), cfg)
        meta = self.ws.recv()
        self.metadata = mp_unpack(meta, np_hook) if isinstance(meta, bytes) else {"raw": meta}

    def infer(self, obs):
        self.ws.send(mp_pack(obs, np_default_openpi))
        r = self.ws.recv()
        if isinstance(r, str):
            raise RuntimeError(f"policy server error: {r[:500]}")
        out = mp_unpack(r, np_hook)
        self.last_timing = out.get("server_timing") if isinstance(out, dict) else None
        return np.asarray(_k(out, self.cfg.get("actions_key", "actions")), dtype=np.float64)

    def close(self):
        self.ws.close()


class XPolicyLab(Wire):
    name = "xpolicylab"

    def __init__(self, cfg, url):
        super().__init__(cfg, url)
        self.eval_id = f"robouse-{uuid.uuid4().hex[:8]}"
        self.trial = None
        self.step = 0
        self.ws = _retry(lambda: WebSocket(url, self.headers, timeout=float(cfg.get("timeout_s", 120))), cfg)
        self.metadata = self._req("hello", {})

    def _req(self, mtype: str, payload: dict, trial=None, step=0) -> dict:
        rid = uuid.uuid4().hex
        frame = {
            "message_type": mtype,
            "message_id": rid,
            "evaluation_id": self.eval_id,
            "action_case_id": None,
            "trial_id": trial,
            "repeat_index": None,
            "step": step,
            "sent_at": time.strftime("%Y-%m-%dT%H:%M:%S+00:00", time.gmtime()),
            "payload": payload,
        }
        self.ws.send(mp_pack(frame, np_default_mnp))
        want = {"hello": "hello_ack", "reset": "reset_result", "infer": "infer_result", "trial_end": "trial_end_ack"}[
            mtype
        ]
        while True:
            raw = self.ws.recv()
            if not isinstance(raw, bytes):
                continue
            r = mp_unpack(raw, np_hook)
            if _k(r, "message_id") != rid:
                continue
            if _k(r, "message_type") == "error":
                p = _k(r, "payload") or {}
                raise RuntimeError(f"policy server error {_k(p, 'code')}: {_k(p, 'message')}")
            if _k(r, "message_type") != want:
                raise RuntimeError(f"expected {want}, got {_k(r, 'message_type')}")
            return _k(r, "payload") or {}

    def reset(self, episode):
        if self.trial:
            self._req("trial_end", {}, trial=self.trial)
        self.trial, self.step = episode, 0
        self._req("reset", {}, trial=self.trial)

    def infer(self, obs):
        p = self._req("infer", {"observation": obs}, trial=self.trial, step=self.step)
        self.step += 1
        acts = _k(p, "actions") or []
        keys = self.cfg.get("action_keys") or ["arm_joint_state", "ee_joint_state"]
        rows = []
        for a in acts:
            rows.append(np.concatenate([np.asarray(_k(a, k), dtype=np.float64).ravel() for k in keys]))
        self.last_timing = {"latency_ms": _k(p, "latency_ms")}
        return np.asarray(rows)

    def close(self):
        try:
            if self.trial:
                self._req("trial_end", {}, trial=self.trial)
        finally:
            self.ws.close()


class OpenVLA(Wire):
    name = "openvla"

    def infer(self, obs):
        body = json.dumps(obs, default=json_numpy_default).encode()
        req = urllib.request.Request(
            self.url.rstrip("/") + self.cfg.get("path", "/act"),
            data=body,
            headers={"Content-Type": "application/json", **self.headers},
            method="POST",
        )
        with urllib.request.urlopen(req, timeout=float(self.cfg.get("timeout_s", 120))) as r:
            txt = r.read().decode()
        out = json.loads(txt, object_hook=json_numpy_hook)
        if isinstance(out, str):
            out = json.loads(out, object_hook=json_numpy_hook) if out.startswith(("{", "[")) else out
        if isinstance(out, str):
            raise RuntimeError(f"policy server error: {out[:300]}")
        a = np.asarray(out["action"] if isinstance(out, dict) else out, dtype=np.float64)
        return a.reshape(1, -1) if a.ndim == 1 else a


class Gr00t(Wire):
    name = "gr00t"

    def __init__(self, cfg, url):
        super().__init__(cfg, url)
        self.zmq = _retry(lambda: ZmqReq(url, timeout=float(cfg.get("timeout_s", 120))), cfg)
        self.token = os.environ.get("ROBOUSE_VLA_TOKEN") or None
        self.metadata = self._call("ping", None) or {}

    def _call(self, endpoint, data):
        req = {"endpoint": endpoint}
        if data is not None:
            req["data"] = data
        if self.token:
            req["api_token"] = self.token
        out = mp_unpack(self.zmq.request(mp_pack(req, np_default_mnp)), np_hook)
        if isinstance(out, dict) and _k(out, "error"):
            raise RuntimeError(f"policy server error: {_k(out, 'error')}")
        return out

    def reset(self, episode):
        try:
            self._call("reset", {"options": None})
        except RuntimeError:
            pass

    def infer(self, obs):
        out = self._call("get_action", {"observation": obs, "options": None})
        act = out[0] if isinstance(out, list) else out
        keys = self.cfg.get("action_keys") or sorted(k if isinstance(k, str) else k.decode() for k in act)
        parts = [np.asarray(_k(act, k), dtype=np.float64) for k in keys]
        parts = [p[0] if p.ndim == 3 else p for p in parts]  # drop the batch dimension
        parts = [p[:, None] if p.ndim == 1 else p for p in parts]
        return np.concatenate(parts, axis=1)


class HttpJson(Wire):
    name = "http-json"

    def _post(self, path: str, body: dict) -> dict:
        last = None
        for attempt in range(int(self.cfg.get("retries", 4))):
            try:
                req = urllib.request.Request(
                    self.url.rstrip("/") + path,
                    data=json.dumps(body, default=json_numpy_default).encode(),
                    headers={"Content-Type": "application/json", **self.headers},
                    method="POST",
                )
                with urllib.request.urlopen(req, timeout=float(self.cfg.get("timeout_s", 120))) as r:
                    return json.loads(r.read().decode())
            except Exception as e:  # noqa: BLE001
                last = e
                time.sleep(2 * (attempt + 1))
        raise RuntimeError(f"policy server unreachable: {last}")

    def reset(self, episode):
        self.episode = episode
        self._post("/reset", {"episode": episode})

    def infer(self, obs):
        out = self._post("/act", {"episode": getattr(self, "episode", "e"), **obs})
        if "actions" not in out:
            raise RuntimeError(f"policy server error: {out.get('error')}")
        self.last_timing = {"latency_s": out.get("latency_s")}
        return np.asarray(out["actions"], dtype=np.float64)


WIRES = {w.name: w for w in (OpenPI, XPolicyLab, OpenVLA, Gr00t, HttpJson)}


def _retry(fn, cfg):
    last = None
    for _attempt in range(int(cfg.get("connect_attempts", 10))):
        try:
            return fn()
        except OSError as e:  # the server may still be loading weights
            last = e
            time.sleep(float(cfg.get("connect_retry_s", 3)))
    raise ConnectionError(f"could not connect to the policy server: {last}")


# ======================================================================================================================
# chunk controllers
# ======================================================================================================================


class Controller:
    """open_loop: execute `replan` actions of each chunk (default all), then re-infer.
    ensemble: ACT temporal ensembling (infer every step; average all predictions for the current step with weights
    exp(-m * age), oldest first as in ACT). smooth: exponential moving average (alpha) over executed actions."""

    def __init__(self, cfg: dict):
        c = cfg.get("controller", {})
        self.mode = c.get("mode", "open_loop")
        self.replan = c.get("replan")
        self.m = float(c.get("m", 0.01))
        self.alpha = c.get("smooth")
        self.buf: dict[int, list] = {}
        self.prev = None

    def plan(self, t: int, chunk: np.ndarray) -> list:
        """Actions to execute now, given the chunk inferred at step t."""
        if self.mode == "ensemble":
            for i, a in enumerate(chunk):
                self.buf.setdefault(t + i, []).append(a)
            preds = self.buf.pop(t, [chunk[0]])
            w = np.exp(-self.m * np.arange(len(preds)))
            acts = [np.sum(np.asarray(preds) * w[:, None], axis=0) / w.sum()]
            for k in [k for k in self.buf if k < t]:
                del self.buf[k]
        else:
            n = len(chunk) if not self.replan else min(len(chunk), int(self.replan))
            acts = list(chunk[:n])
        if self.alpha is not None:
            out = []
            for a in acts:
                self.prev = (
                    a if self.prev is None else float(self.alpha) * np.asarray(a) + (1 - float(self.alpha)) * self.prev
                )
                out.append(self.prev)
            acts = out
        return acts


# ======================================================================================================================
# main loop
# ======================================================================================================================


def check_compat(cfg: dict, info: dict, amap: ActionMap, chunk: np.ndarray) -> str | None:
    """Fail fast (before anything moves) when the policy and the embodiment do not fit together."""
    backends = cfg.get("backends")
    if backends and info.get("backend") not in backends:
        return f"policy preset is for backends {backends}, this task uses {info.get('backend')!r}"
    if chunk.ndim != 2 or chunk.shape[0] == 0:
        return f"policy returned no action chunk (shape {chunk.shape})"
    want = amap.expected_policy_dim()
    if want and chunk.shape[1] != want:
        return f"policy action dimension {chunk.shape[1]} != preset's {want}"
    a = cfg.get("action", {})
    if a.get("index") is None and "pad" not in a and chunk.shape[1] != amap.dim:
        return (
            f"policy actions have {chunk.shape[1]} values but the task takes {amap.dim} "
            f"({', '.join(info['action']['names'])}); map them with action.index / action.pad"
        )
    need = a.get("mode")
    modes = [g.get("mode") for g in (info.get("embodiment") or {}).get("action_groups", [])]
    if need and modes and need not in modes:
        return f"policy outputs {need!r} actions, the embodiment's action groups are {modes}"
    return None


def main() -> int:
    cfg = json.loads(os.environ.get("ROBOUSE_VLA_CONFIG") or "{}")
    url = os.environ.get("ROBOUSE_VLA_URL") or cfg.get("url")
    info = robo({"op": "info"})
    if not info.get("ok"):
        emit(type="error", error=info.get("error"))
        return 1
    info = info["result"]
    # per_backend: {backend: {top-level keys}} merged over the preset for that simulator (one job across suites)
    cfg = {**cfg, **(cfg.get("per_backend") or {}).get(str(info.get("backend")), {})}
    language = os.environ.get("ROBOUSE_TASK_LANGUAGE", "").strip()
    emit(
        type="start",
        task=info.get("task"),
        backend=info.get("backend"),
        action_dim=len(info["action"]["names"]),
        max_steps=info["max_steps"],
        language=language,
        wire=cfg.get("wire"),
        preset=cfg.get("preset"),
        server=url,
    )
    if not url or cfg.get("wire") not in WIRES:
        emit(type="error", error=f"need ROBOUSE_VLA_URL and a wire in {sorted(WIRES)}")
        robo({"op": "give_up", "text": "vla harness misconfigured"})
        return 1
    wire = WIRES[cfg["wire"]](cfg, url)
    emit(type="connected", metadata=wire.metadata if isinstance(wire.metadata, dict) else str(wire.metadata))
    obsr, amap, ctl = Observer(cfg, info, language), ActionMap(cfg, info), Controller(cfg)
    wire.reset(f"{info.get('task')}-{os.getpid()}-{int(time.time())}")
    t, n_inf, checked = 0, 0, False
    max_inf = int(cfg.get("max_inferences", 100000))
    max_steps = int(info["max_steps"])
    stop = cfg.get("stop", {})  # {"still_steps": N, "tol": x}: claim done once the commanded action stops changing
    still_n, still_tol, still, last_cmd = int(stop.get("still_steps", 0)), float(stop.get("tol", 1e-3)), 0, None
    try:
        while n_inf < max_inf:
            got, err = obsr.observe()
            if got is None:
                emit(type="end", reason=err)
                return 0
            images, state, steps = got
            obs = obsr.build(images, state)
            t0 = time.time()
            chunk = np.atleast_2d(wire.infer(obs))
            n_inf += 1
            if not checked:
                why = check_compat(cfg, info, amap, chunk)
                if why:
                    emit(type="incompatible", reason=why)
                    robo({"op": "give_up", "text": f"vla: incompatible policy/embodiment: {why}"})
                    return 2
                checked = True
            acts = ctl.plan(t, chunk)
            emit(
                type="chunk",
                inference=n_inf,
                steps_used=steps,
                chunk_shape=list(chunk.shape),
                executed=len(acts),
                roundtrip_s=round(time.time() - t0, 3),
                server_timing=getattr(wire, "last_timing", None),
                actions=[[round(float(x), 5) for x in a] for a in acts],
            )
            for a in acts:
                cmd = amap(a, state if amap.relative_to else None)
                if last_cmd is not None and max(abs(x - y) for x, y in zip(cmd, last_cmd, strict=False)) < still_tol:
                    still += 1
                else:
                    still = 0
                last_cmd = cmd
                if (still_n and still >= still_n) or (steps is not None and steps >= max_steps - 1):
                    # a VLA never says "done": the run ends when the policy has settled or the budget is about to run
                    # out, and the episode server judges the state it left
                    robo(
                        {
                            "op": "done",
                            "text": "vla: policy settled"
                            if still_n and still >= still_n
                            else "vla: step budget reached",
                        }
                    )
                    emit(
                        type="end",
                        reason="done",
                        steps_used=steps,
                        why="settled" if still_n and still >= still_n else "budget",
                    )
                    return 0
                res = robo({"op": "act", "action": cmd, "repeat": 1})
                t += 1
                if not res.get("ok"):
                    emit(type="end", reason=res.get("error"))
                    return 0
                if "episode" in res["result"]:
                    emit(type="end", reason=res["result"]["episode"], steps_used=res["result"].get("steps_used"))
                    return 0
                state = res["result"].get("state", state)
                steps = res["result"].get("steps_used", steps)
        robo({"op": "done", "text": "vla: inference budget reached"})
    finally:
        try:
            wire.close()
        except Exception:  # noqa: BLE001
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
