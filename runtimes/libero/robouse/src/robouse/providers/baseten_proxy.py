#!/usr/bin/env python3
"""Localhost pass-through proxy between Claude Code and Baseten's Anthropic-compatible API (run_agent.py --provider baseten).

Why: Baseten's GLM-5.3 endpoint rejects a Messages request with more than 8 images ("400 Too many images: 9 (max 8)"),
and Claude Code resends every screenshot of a computer-use session on every turn. The proxy removes the oldest image
blocks from each /v1/messages request so at most MAX_IMAGES remain. It removes them in batches (keeping between
MAX_IMAGES - BATCH + 1 and MAX_IMAGES of the newest), so the request prefix, and with it the provider's prompt cache,
stays the same for BATCH turns at a time. A removed image becomes a short text block. Nothing else in the request or the
response changes; responses (SSE streams included) are relayed byte for byte.

It also holds the Baseten key: Claude Code gets a dummy ANTHROPIC_AUTH_TOKEN and the proxy sends the real Bearer
token upstream, so the key is in no child process environment. Never printed or written.
"""
from __future__ import annotations

import http.client
import json
import os
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

UPSTREAM = "inference.baseten.co"
MAX_IMAGES, BATCH = 8, 5
# BF_BASETEN_KEEP_IMAGES=k (1..7): a sensitivity setting that keeps only the newest k screenshots on every request
KEEP = int(os.environ.get("BF_BASETEN_KEEP_IMAGES") or 0)
PLACEHOLDER = "[older screenshot removed: the model endpoint accepts at most 8 images per request]"
HOP = {"host", "authorization", "x-api-key", "content-length", "connection", "keep-alive", "transfer-encoding", "accept-encoding"}


def removed_count(n: int) -> int:
    """How many of n images to drop: none up to MAX_IMAGES, then in steps of BATCH (keeps MAX_IMAGES-BATCH+1..MAX_IMAGES)."""
    if 0 < KEEP < MAX_IMAGES:
        return max(0, n - KEEP)
    if n <= MAX_IMAGES:
        return 0
    return BATCH * ((n - (MAX_IMAGES - BATCH + 1)) // BATCH)


def prune(body: dict, stats: dict) -> dict:
    """Replace the oldest image blocks (in messages, including inside tool_result content) with a text placeholder."""
    refs = []   # (container list, index) of every image block, in conversation order
    for m in body.get("messages") or []:
        content = m.get("content")
        if not isinstance(content, list):
            continue
        for i, blk in enumerate(content):
            if not isinstance(blk, dict):
                continue
            if blk.get("type") == "image":
                refs.append((content, i))
            elif blk.get("type") == "tool_result" and isinstance(blk.get("content"), list):
                for j, sub in enumerate(blk["content"]):
                    if isinstance(sub, dict) and sub.get("type") == "image":
                        refs.append((blk["content"], j))
    k = removed_count(len(refs))
    for lst, i in refs[:k]:
        lst[i] = {"type": "text", "text": PLACEHOLDER}
    stats["max_images_seen"] = max(stats["max_images_seen"], len(refs))
    stats["images_removed_last"] = k
    return body


def start(key: str, stats: dict | None = None) -> tuple[int, dict]:
    """Serve on 127.0.0.1 (a free port) in a daemon thread. Returns (port, stats)."""
    stats = stats if stats is not None else {}
    stats.update({"keep_images": KEEP or f"{MAX_IMAGES - BATCH + 1}-{MAX_IMAGES}", "requests": 0, "pruned_requests": 0, "max_images_seen": 0, "images_removed_last": 0,
                  "upstream_errors": 0, "status_counts": {}})
    lock = threading.Lock()

    class H(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.0"   # the response ends when the connection closes, so SSE relays without re-chunking

        def log_message(self, *a):  # noqa: D401  (quiet)
            pass

        def _relay(self, method: str) -> None:
            n = int(self.headers.get("content-length") or 0)
            data = self.rfile.read(n) if n else b""
            if method == "POST" and "/messages" in self.path and data:
                try:
                    body = prune(json.loads(data), stats)
                    if stats["images_removed_last"]:
                        with lock:
                            stats["pruned_requests"] += 1
                    data = json.dumps(body).encode()
                except ValueError:
                    pass
            hdrs = {k: v for k, v in self.headers.items() if k.lower() not in HOP}
            hdrs.update({"Authorization": f"Bearer {key}", "Content-Length": str(len(data)), "Accept-Encoding": "identity"})
            with lock:
                stats["requests"] += 1
            try:
                conn = http.client.HTTPSConnection(UPSTREAM, timeout=900)
                conn.request(method, self.path, body=data if data else None, headers=hdrs)
                r = conn.getresponse()
            except Exception as e:  # noqa: BLE001
                with lock:
                    stats["upstream_errors"] += 1
                self.send_response(502)
                self.send_header("content-type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps({"type": "error", "error": {"type": "api_error", "message": f"proxy: {type(e).__name__}"}}).encode())
                return
            with lock:
                stats["status_counts"][str(r.status)] = stats["status_counts"].get(str(r.status), 0) + 1
            self.send_response(r.status)
            for k, v in r.getheaders():
                if k.lower() not in ("transfer-encoding", "connection", "content-length"):
                    self.send_header(k, v)
            self.send_header("connection", "close")
            self.end_headers()
            try:
                while True:
                    chunk = r.read1(65536)
                    if not chunk:
                        break
                    self.wfile.write(chunk)
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError):
                pass
            finally:
                conn.close()

        def do_POST(self):  # noqa: N802
            self._relay("POST")

        def do_GET(self):  # noqa: N802
            self._relay("GET")

    srv = ThreadingHTTPServer(("127.0.0.1", 0), H)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv.server_address[1], stats
