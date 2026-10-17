"""Mock rosbridge server (standard library only): a simulated ROS arm behind rosbridge's JSON protocol, for testing the
rosbridge driver without ROS or hardware.

It accepts websocket clients, honours subscribe / advertise / publish / unsubscribe / unadvertise, publishes
sensor_msgs/JointState on /joint_states at 20 Hz and a sensor_msgs/CompressedImage (JPEG, a schematic of the joint
angles) on each camera topic at 5 Hz, and moves the joints toward the last commanded positions (JointTrajectory's
first point, or Float64MultiArray data) with a first-order lag and a speed limit. Every command is logged in `commands`.
"""

from __future__ import annotations

import base64
import hashlib
import io
import json
import math
import socket
import struct
import threading
import time


class _Conn:
    def __init__(self, sock: socket.socket):
        self.sock, self.buf, self.lock = sock, b"", threading.Lock()
        self.subs: set[str] = set()

    def _read(self, n: int) -> bytes:
        while len(self.buf) < n:
            c = self.sock.recv(65536)
            if not c:
                raise ConnectionError("closed")
            self.buf += c
        out, self.buf = self.buf[:n], self.buf[n:]
        return out

    def recv(self) -> str | None:
        b0, b1 = self._read(2)
        op, n = b0 & 0x0F, b1 & 0x7F
        if n == 126:
            n = struct.unpack(">H", self._read(2))[0]
        elif n == 127:
            n = struct.unpack(">Q", self._read(8))[0]
        mask = self._read(4) if b1 & 0x80 else b"\0\0\0\0"
        data = bytes(c ^ mask[i % 4] for i, c in enumerate(self._read(n)))
        if op == 0x8:
            raise ConnectionError("close frame")
        return data.decode() if op in (0x1, 0x2) else None

    def send(self, text: str) -> None:
        p = text.encode()
        n = len(p)
        head = bytes([0x81]) + (
            bytes([n])
            if n < 126
            else bytes([126]) + struct.pack(">H", n)
            if n < 65536
            else bytes([127]) + struct.pack(">Q", n)
        )
        with self.lock:
            self.sock.sendall(head + p)


class MockRosbridge:
    def __init__(
        self,
        joints: list[str],
        start_rad: list[float] | None = None,
        port: int = 0,
        cameras: list[str] | None = None,
        state_topic: str = "/joint_states",
        gain: float = 0.5,
        max_rad_per_tick: float = 0.05,
    ):
        self.joints = list(joints)
        self.q = list(start_rad or [0.0] * len(joints))
        self.target = list(self.q)
        self.cameras = list(cameras or [])
        self.state_topic, self.gain, self.vmax = state_topic, gain, max_rad_per_tick
        self.commands: list[list[float]] = []
        self.conns: list[_Conn] = []
        self.srv = socket.socket()
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(("127.0.0.1", port))
        self.srv.listen(4)
        self.port = self.srv.getsockname()[1]
        self.url = f"ws://127.0.0.1:{self.port}"
        self._stop = threading.Event()
        threading.Thread(target=self._accept, daemon=True).start()
        threading.Thread(target=self._tick, daemon=True).start()

    def _accept(self) -> None:
        while not self._stop.is_set():
            try:
                s, _ = self.srv.accept()
            except OSError:
                return
            req = b""
            while b"\r\n\r\n" not in req:
                req += s.recv(4096)
            key = [
                l.split(b":", 1)[1].strip() for l in req.split(b"\r\n") if l.lower().startswith(b"sec-websocket-key")
            ][0]
            acc = base64.b64encode(hashlib.sha1(key + b"258EAFA5-E914-47DA-95CA-C5AB0DC85B11").digest())
            s.sendall(
                b"HTTP/1.1 101 Switching Protocols\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Accept: "
                + acc
                + b"\r\n\r\n"
            )
            c = _Conn(s)
            self.conns.append(c)
            threading.Thread(target=self._serve, args=(c,), daemon=True).start()

    def _serve(self, c: _Conn) -> None:
        try:
            while True:
                t = c.recv()
                if t is None:
                    continue
                m = json.loads(t)
                if m.get("op") == "subscribe":
                    c.subs.add(m["topic"])
                elif m.get("op") == "unsubscribe":
                    c.subs.discard(m["topic"])
                elif m.get("op") == "publish":
                    msg = m.get("msg") or {}
                    if "points" in msg:
                        names = msg.get("joint_names") or self.joints
                        pos = dict(zip(names, msg["points"][0]["positions"], strict=False))
                        tgt = [float(pos.get(j, self.target[i])) for i, j in enumerate(self.joints)]
                    else:
                        tgt = [float(x) for x in msg.get("data", self.target)]
                    self.target = tgt
                    self.commands.append(list(tgt))
        except (ConnectionError, OSError, ValueError):
            if c in self.conns:
                self.conns.remove(c)

    def _image(self) -> str:
        from PIL import Image, ImageDraw

        im = Image.new("RGB", (160, 120), (235, 235, 235))
        d = ImageDraw.Draw(im)
        x, y, a = 80.0, 110.0, -math.pi / 2
        for q in self.q[:4]:
            a += q
            nx, ny = x + 25 * math.cos(a), y + 25 * math.sin(a)
            d.line([(x, y), (nx, ny)], fill=(60, 90, 160), width=4)
            x, y = nx, ny
        b = io.BytesIO()
        im.save(b, "JPEG", quality=80)
        return base64.b64encode(b.getvalue()).decode()

    def _tick(self) -> None:
        k = 0
        while not self._stop.is_set():
            self.q = [
                q + max(-self.vmax, min(self.vmax, self.gain * (t - q)))
                for q, t in zip(self.q, self.target, strict=False)
            ]
            now = time.time()
            js = json.dumps(
                {
                    "op": "publish",
                    "topic": self.state_topic,
                    "msg": {
                        "header": {"stamp": {"sec": int(now), "nanosec": int((now % 1) * 1e9)}},
                        "name": self.joints,
                        "position": self.q,
                        "velocity": [0.0] * len(self.q),
                        "effort": [0.0] * len(self.q),
                    },
                }
            )
            img = None
            if k % 4 == 0 and self.cameras:
                img = self._image()
            for c in list(self.conns):
                try:
                    if self.state_topic in c.subs:
                        c.send(js)
                    for cam in self.cameras:
                        if img and cam in c.subs:
                            c.send(json.dumps({"op": "publish", "topic": cam, "msg": {"format": "jpeg", "data": img}}))
                except OSError:
                    pass
            k += 1
            time.sleep(0.05)

    def close(self) -> None:
        self._stop.set()
        self.srv.close()
