#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""极简 WebSocket 广播服务（RFC6455，纯标准库）

网页仿真（playground/index.js）连 ws://<host>:8765，
收 {"type":"motor_angles","payload":{"angles":[6 个角度]}} 驱动 URDF 模型。

这里只做服务端 → 浏览器 的单向推送，够用且不需要 websockets 库。
"""

import base64
import hashlib
import json
import socket
import struct
import threading
import time

GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"


def _accept_key(key):
    return base64.b64encode(hashlib.sha1((key + GUID).encode()).digest()).decode()


def _frame(text):
    data = text.encode("utf-8")
    n = len(data)
    if n < 126:
        head = struct.pack("!BB", 0x81, n)
    elif n < 65536:
        head = struct.pack("!BBH", 0x81, 126, n)
    else:
        head = struct.pack("!BBQ", 0x81, 127, n)
    return head + data


class SimBridge:
    """把六个关节角推给网页仿真"""

    def __init__(self, host="0.0.0.0", port=8765, log=None):
        self.host = host
        self.port = port
        self.log = log or (lambda msg: None)
        self._clients = []
        self._lock = threading.Lock()
        self._alive = True
        self._srv = None
        self.started = False
        self._last_push = 0.0
        self.interval = 0.1        # 10 Hz

    # ---------------- 启停 ----------------
    def start(self):
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((self.host, self.port))
            s.listen(8)
            s.settimeout(1.0)
            self._srv = s
        except OSError as ex:
            self.log("网页仿真桥启动失败（端口 %d 被占用？）：%s" % (self.port, ex))
            return False
        threading.Thread(target=self._accept_loop, daemon=True).start()
        self.started = True
        self.log("网页仿真桥已启动：ws://127.0.0.1:%d" % self.port)
        return True

    def stop(self):
        self._alive = False
        try:
            if self._srv:
                self._srv.close()
        except Exception:
            pass
        with self._lock:
            for c in self._clients:
                try:
                    c.close()
                except Exception:
                    pass
            self._clients.clear()

    def client_count(self):
        with self._lock:
            return len(self._clients)

    # ---------------- 连接 ----------------
    def _accept_loop(self):
        while self._alive:
            try:
                conn, addr = self._srv.accept()
            except socket.timeout:
                continue
            except OSError:
                break
            threading.Thread(target=self._serve, args=(conn,), daemon=True).start()

    def _serve(self, conn):
        try:
            conn.settimeout(5)
            req = b""
            while b"\r\n\r\n" not in req:
                chunk = conn.recv(1024)
                if not chunk:
                    return
                req += chunk
                if len(req) > 16384:
                    return
            text = req.decode("latin-1")
            if "upgrade: websocket" not in text.lower():
                conn.sendall(b"HTTP/1.1 400 Bad Request\r\n\r\n")
                return
            key = ""
            for line in text.split("\r\n")[1:]:
                if ":" in line:
                    k, v = line.split(":", 1)
                    if k.strip().lower() == "sec-websocket-key":
                        key = v.strip()
            if not key:
                return
            conn.sendall(("HTTP/1.1 101 Switching Protocols\r\n"
                          "Upgrade: websocket\r\n"
                          "Connection: Upgrade\r\n"
                          "Sec-WebSocket-Accept: %s\r\n\r\n" % _accept_key(key)).encode())
            conn.settimeout(None)
            with self._lock:
                self._clients.append(conn)
            self.log("网页仿真已连接（当前 %d 个页面）" % self.client_count())
            while self._alive:
                try:
                    if not conn.recv(4096):     # 上行帧直接丢弃
                        break
                except OSError:
                    break
        except Exception:
            pass
        finally:
            with self._lock:
                if conn in self._clients:
                    self._clients.remove(conn)
            try:
                conn.close()
            except Exception:
                pass

    # ---------------- 推送 ----------------
    def push(self, angles, force=False):
        now = time.time()
        if not force and now - self._last_push < self.interval:
            return
        self._last_push = now
        with self._lock:
            if not self._clients:
                return
            msg = _frame(json.dumps({
                "type": "motor_angles",
                "payload": {"angles": [round(float(a), 2) for a in angles]},
            }))
            dead = []
            for c in self._clients:
                try:
                    c.sendall(msg)
                except OSError:
                    dead.append(c)
            for c in dead:
                self._clients.remove(c)
                try:
                    c.close()
                except Exception:
                    pass
