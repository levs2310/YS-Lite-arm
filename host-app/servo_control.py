#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""众灵总线舵机机械臂 · 上位机 (Windows / tkinter)

连接方式：USB-TTL 单线半双工，默认 115200-8-N-1
控制方式：输入角度(°) → 换算 PWM → 下发 #IDPxxxxTyyyy!
功能：六轴状态监控、参数设置、急停释放力矩/恢复、网页仿真联动（可选方向）

无第三方依赖（串口用 Win32 API，via ctypes；网页仿真走标准库 socket + http.server）。
调试：python servo_control.py --shot 截图.png   生成界面截图
"""

import base64
import collections
import ctypes
import ctypes.wintypes as wt
import functools
import hashlib
import http.server
import json
import os
import queue
import re
import socket
import struct
import sys
import threading
import time
import tkinter as tk
import webbrowser
from tkinter import ttk, messagebox
from tkinter import scrolledtext, filedialog

# ============================================================
# 基础配置
# ============================================================
APP_NAME = "远山Lite机械臂上位机"
APP_VER = "1.0"
UI_FONT = "Microsoft YaHei UI"   # 中文+英文混排必须用具体字族，空字族会让 ASCII 字形挤压
BAUD_DEFAULT = 115200
SERVO_IDS = [1, 2, 3, 4, 5, 6]
JOINT_NAMES = {1: "底座", 2: "肩", 3: "肘", 4: "腕转", 5: "腕摆", 6: "夹爪"}

# 手册取值范围
PWM_MIN_LIMIT = 500
PWM_MAX_LIMIT = 2500
ID_MIN, ID_MAX = 0, 254
BAUD_TABLE = [("9600", 0), ("19200", 1), ("38400", 2), ("57600", 3),
              ("115200", 4), ("128000", 5), ("256000", 6), ("1000000", 7)]


LINK_DIRECT = "直连舵机总线"
LINK_MODES = [LINK_DIRECT]

MOVE_STAGGER_MS = 150      # 多轴下发时间隔（错开启动，避免电流尖峰）

MODE_TEXT = {
    1: "角度模式 · 正转",
    2: "角度模式 · 反转",
}
KEY_MAP = {
    "a": (1, +1), "d": (1, -1),      # 1 号：底座
    "w": (2, +1), "s": (2, -1),      # 2 号：肩
    "i": (3, +1), "k": (3, -1),      # 3 号：肘
    "j": (4, +1), "l": (4, -1),      # 4 号：腕转
    "f": (5, +1), "h": (5, -1),      # 5 号：腕摆
    "z": (6, +1), "x": (6, -1),      # 6 号：夹爪
}

PWM_PER_DEG = 2000.0 / 270.0
VEL_PWM_PER_DPS = 3.33     # 预留
VEL_MAX = 200.0            # 预留

IS_FROZEN = bool(getattr(sys, "frozen", False))


def app_dir():
    if IS_FROZEN:
        return os.path.dirname(os.path.abspath(sys.executable))
    return os.path.dirname(os.path.abspath(__file__))


BASE_DIR = app_dir()
CONFIG_PATH = os.path.join(BASE_DIR, "arm_config.json")
REC_PERIOD_MS = 50        # 轨迹采样周期（20 帧/秒）
PLAY_STEP_S = 0.05        # 回放插值步长（20 次/秒）


def default_joints():
    js = []
    for i, sid in enumerate(SERVO_IDS):
        js.append({
            "id": sid,
            "name": JOINT_NAMES.get(sid, "关节%d" % sid),
            "amin": 0, "amax": 270,
            "pmin": PWM_MIN_LIMIT, "pmax": PWM_MAX_LIMIT,
            "invert": False,
            "time": 1000,
            "speed": 90,
            "comp": 36,
            "home": 0.0,
            "mode": 1,
            "mode_want": None,
            "vel": 10.0,
            "limit_min": PWM_MIN_LIMIT,
            "limit_max": PWM_MAX_LIMIT,
            "center": 1500,
        })
    return js


def load_config():
    cfg = {"port": "", "baud": BAUD_DEFAULT, "link": LINK_DIRECT,
           "autohome": False, "kb_speed": 10.0, "joints": default_joints()}
    try:
        with open(CONFIG_PATH, "r", encoding="utf-8") as f:
            data = json.load(f)
        if isinstance(data.get("joints"), list) and len(data["joints"]) == len(SERVO_IDS):
            for a, b in zip(cfg["joints"], data["joints"]):
                a.update({k: v for k, v in b.items() if k in a})
        if isinstance(data.get("marks"), dict):
            cfg["marks"] = data["marks"]
        if isinstance(data.get("kb_speed"), (int, float)):
            cfg["kb_speed"] = float(data["kb_speed"])
        if isinstance(data.get("port"), str):
            cfg["port"] = data["port"]
        if isinstance(data.get("baud"), int):
            cfg["baud"] = data["baud"]
        if data.get("link") in LINK_MODES:
            cfg["link"] = data["link"]
        cfg["autohome"] = bool(data.get("autohome", False))
    except Exception:
        pass
    return cfg


def save_config(cfg):
    try:
        with open(CONFIG_PATH, "w", encoding="utf-8") as f:
            json.dump(cfg, f, ensure_ascii=False, indent=2)
        return True
    except Exception:
        return False


# ============================================================
# Win32 串口（ctypes，无需 pyserial）
# ============================================================
if sys.platform == "win32":
    kernel32 = ctypes.WinDLL("kernel32", use_last_error=True)
    GENERIC_READ = 0x80000000
    GENERIC_WRITE = 0x40000000
    OPEN_EXISTING = 3
    INVALID_HANDLE_VALUE = wt.HANDLE(-1).value
    PURGE_TXABORT = 0x0001
    PURGE_RXABORT = 0x0002
    PURGE_TXCLEAR = 0x0004
    PURGE_RXCLEAR = 0x0008

    class DCB(ctypes.Structure):
        _fields_ = [
            ("DCBlength", wt.DWORD), ("BaudRate", wt.DWORD),
            ("fBinary", wt.DWORD, 1), ("fParity", wt.DWORD, 1),
            ("fOutxCtsFlow", wt.DWORD, 1), ("fOutxDsrFlow", wt.DWORD, 1),
            ("fDtrControl", wt.DWORD, 2), ("fDsrSensitivity", wt.DWORD, 1),
            ("fTXContinueOnXoff", wt.DWORD, 1), ("fOutX", wt.DWORD, 1),
            ("fInX", wt.DWORD, 1), ("fErrorChar", wt.DWORD, 1),
            ("fNull", wt.DWORD, 1), ("fRtsControl", wt.DWORD, 2),
            ("fAbortOnError", wt.DWORD, 1), ("fDummy2", wt.DWORD, 17),
            ("wReserved", wt.WORD), ("XonLim", wt.WORD), ("XoffLim", wt.WORD),
            ("ByteSize", wt.BYTE), ("Parity", wt.BYTE), ("StopBits", wt.BYTE),
            ("XonChar", ctypes.c_char), ("XoffChar", ctypes.c_char),
            ("ErrorChar", ctypes.c_char), ("EofChar", ctypes.c_char),
            ("EvtChar", ctypes.c_char), ("wReserved1", wt.WORD),
        ]

    class COMMTIMEOUTS(ctypes.Structure):
        _fields_ = [
            ("ReadIntervalTimeout", wt.DWORD), ("ReadTotalTimeoutMultiplier", wt.DWORD),
            ("ReadTotalTimeoutConstant", wt.DWORD), ("WriteTotalTimeoutMultiplier", wt.DWORD),
            ("WriteTotalTimeoutConstant", wt.DWORD),
        ]

    kernel32.CreateFileW.restype = wt.HANDLE
    kernel32.CreateFileW.argtypes = [wt.LPCWSTR, wt.DWORD, wt.DWORD, wt.LPVOID,
                                     wt.DWORD, wt.DWORD, wt.HANDLE]


class SerialError(Exception):
    pass


class WinSerial:
    """单线半双工 USB-TTL 串口（同步句柄 + 非阻塞读）"""

    def __init__(self, port, baud=BAUD_DEFAULT):
        self.port = port
        self.baud = baud
        self.h = None

    def open(self):
        if sys.platform != "win32":
            raise SerialError("仅支持 Windows")
        path = "\\\\.\\" + self.port
        h = kernel32.CreateFileW(path, GENERIC_READ | GENERIC_WRITE, 0, None,
                                 OPEN_EXISTING, 0, None)
        if not h or h == INVALID_HANDLE_VALUE:
            err = ctypes.get_last_error()
            raise SerialError("打开 %s 失败 (错误码 %s)，检查端口是否被占用" % (self.port, err))
        self.h = h
        kernel32.SetupComm(h, 8192, 8192)

        dcb = DCB()
        dcb.DCBlength = ctypes.sizeof(DCB)
        dcb.BaudRate = self.baud
        dcb.ByteSize = 8
        dcb.Parity = 0          # NOPARITY
        dcb.StopBits = 0        # ONESTOPBIT
        dcb.fBinary = 1
        dcb.fParity = 0
        dcb.fOutxCtsFlow = 0
        dcb.fOutxDsrFlow = 0
        dcb.fDtrControl = 1     # ENABLE
        dcb.fDsrSensitivity = 0
        dcb.fTXContinueOnXoff = 1
        dcb.fOutX = 0
        dcb.fInX = 0
        dcb.fErrorChar = 0
        dcb.fNull = 0
        dcb.fRtsControl = 1     # ENABLE
        dcb.fAbortOnError = 0
        if not kernel32.SetCommState(h, ctypes.byref(dcb)):
            self.close()
            raise SerialError("设置串口参数失败（%s）" % self.port)

        to = COMMTIMEOUTS()
        to.ReadIntervalTimeout = 0xFFFFFFFF     # 有数据立即返回
        to.ReadTotalTimeoutMultiplier = 0
        to.ReadTotalTimeoutConstant = 0
        to.WriteTotalTimeoutMultiplier = 0
        to.WriteTotalTimeoutConstant = 500
        kernel32.SetCommTimeouts(h, ctypes.byref(to))

        kernel32.PurgeComm(h, PURGE_TXCLEAR | PURGE_RXCLEAR)
        kernel32.PurgeComm(h, PURGE_TXABORT | PURGE_RXABORT)

    def close(self):
        if self.h:
            try:
                kernel32.CloseHandle(self.h)
            except Exception:
                pass
            self.h = None

    def write(self, data):
        if not self.h:
            raise SerialError("串口未打开")
        n = wt.DWORD(0)
        buf = ctypes.create_string_buffer(data, len(data))
        ok = kernel32.WriteFile(self.h, buf, len(data), ctypes.byref(n), None)
        if not ok:
            raise SerialError("写串口失败 (错误码 %s)" % ctypes.get_last_error())
        return n.value

    def read(self, maxlen=512):
        if not self.h:
            return b""
        buf = ctypes.create_string_buffer(maxlen)
        n = wt.DWORD(0)
        ok = kernel32.ReadFile(self.h, buf, maxlen, ctypes.byref(n), None)
        if not ok:
            return b""
        return buf.raw[:n.value]

    def flush(self):
        if self.h:
            kernel32.PurgeComm(self.h, PURGE_TXCLEAR | PURGE_RXCLEAR)


def list_com_ports():
    ports = []
    try:
        import winreg
        key = winreg.OpenKey(winreg.HKEY_LOCAL_MACHINE,
                             r"HARDWARE\DEVICEMAP\SERIALCOMM")
        i = 0
        while True:
            try:
                _name, value, _typ = winreg.EnumValue(key, i)
            except OSError:
                break
            ports.append(str(value))
            i += 1
    except Exception:
        pass

    def keyf(p):
        m = re.search(r"(\d+)", p)
        return (int(m.group(1)) if m else 999, p)

    return sorted(set(ports), key=keyf)


# ============================================================
# 协议封装
# ============================================================
def frame_position(sid, pwm, t_ms):
    return "#%03dP%04dT%04d!" % (sid, pwm, t_ms)


def frame_read_pos(sid):
    return "#%03dPRAD!" % sid


def frame_ulk(sid):
    return "#%03dPULK!" % sid


def frame_ulr(sid):
    return "#%03dPULR!" % sid


def frame_mod_get(sid):
    return "#%03dPMOD!" % sid


def frame_mod_set(sid, mode):
    return "#%03dPMOD%d!" % (sid, mode)


def frame_sck(sid):
    return "#%03dPSCK!" % sid


def frame_smi(sid):
    return "#%03dPSMI!" % sid


def frame_smx(sid):
    return "#%03dPSMX!" % sid


def frame_pid(sid, new_id):
    return "#%03dPID%03d!" % (sid, new_id)


def frame_baud(sid, code):
    return "#%03dPBD%d!" % (sid, code)


def frame_csd(sid, pwm):
    return "#%03dPCSD%04d!" % (sid, pwm)


def frame_cleo(sid):
    return "#%03dPCLEO!" % sid


def frame_cle(sid):
    return "#%03dPCLE!" % sid


def frame_ver(sid):
    return "#%03dPVER!" % sid


def frame_prte(sid):
    return "#%03dPRTE!" % sid


REPLY_RE = re.compile(r"#(\d{3})P([^!]*)!")
MOD_RE = re.compile(r"#(\d{3})MOD(\d+)!")


def angle_to_pwm(joint, angle):
    amin, amax = float(joint["amin"]), float(joint["amax"])
    pmin, pmax = int(joint["pmin"]), int(joint["pmax"])
    angle = max(amin, min(amax, float(angle)))
    ratio = 0.0 if amax == amin else (angle - amin) / (amax - amin)
    if joint.get("invert"):
        ratio = 1.0 - ratio
    pwm = pmin + ratio * (pmax - pmin)
    return int(round(pwm)), angle


def pwm_to_angle(joint, pwm):
    pmin, pmax = int(joint["pmin"]), int(joint["pmax"])
    amin, amax = float(joint["amin"]), float(joint["amax"])
    ratio = 0.0 if pmax == pmin else (float(pwm) - pmin) / (pmax - pmin)
    ratio = max(0.0, min(1.0, ratio))
    if joint.get("invert"):
        ratio = 1.0 - ratio
    return amin + ratio * (amax - amin)


# ============================================================
# 网页仿真联动（WebSocket 桥 + 静态服务，全部用标准库）
# ============================================================
# 协议与 网页仿真/playground 对齐：
#   上位机 → 页面   {"type":"motor_angles","payload":{"angles":[6 个值]}}
#   页面 → 上位机   {"type":"sim_angles","payload":{"angles":[6 个值],"t":秒}}
# 前 5 个值是关节角(度)；第 6 个是夹爪开合量 0-400（0=闭合，400=全开）。

SIM_WS_PORT = 8765
SIM_HTTP_PORT = 8766
SIM_GUID = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11"
SIM_GRIPPER_SPAN = 400.0
# 仿真回写舵机时的死区（单位同协议）：小于该差值认为是自己推送的回声，忽略
SIM_DEADBAND = (1.5, 1.5, 1.5, 1.5, 1.5, 3.0)

SIM_MODE_TO_SIM = "真机 → 仿真"
SIM_MODE_FROM_SIM = "仿真 → 真机"
SIM_MODE_OFF = "关闭"
SIM_MODES = [SIM_MODE_OFF, SIM_MODE_TO_SIM, SIM_MODE_FROM_SIM]
# 下发给页面的机器可读模式名
SIM_MODE_TOKENS = {
    SIM_MODE_OFF: "off",
    SIM_MODE_TO_SIM: "to_sim",
    SIM_MODE_FROM_SIM: "from_sim",
}

# 仿真 → 真机的电流保护：电源电流有限，六轴同时启动会把电压拉低、总线跟着超时。
# 所以一次只允许有限几路同时动，且按固定间隔依次下发。
SIM_MAX_CONCURRENT = 2     # 同时运动的轴最多 2 路
SIM_MOVE_GAP_MS = 80       # 两次下发的间隔
SIM_MOVE_SPEED = 45.0      # 仿真下发用的速度(°/s)，比手动慢，峰值电流更低
SIM_MOVE_MAX_MS = 700      # 单次动作时长上限


def _ws_accept_key(key):
    return base64.b64encode(hashlib.sha1((key + SIM_GUID).encode()).digest()).decode()


def _ws_frame(text):
    data = text.encode("utf-8")
    n = len(data)
    if n < 126:
        head = struct.pack("!BB", 0x81, n)
    elif n < 65536:
        head = struct.pack("!BBH", 0x81, 126, n)
    else:
        head = struct.pack("!BBQ", 0x81, 127, n)
    return head + data


def _ws_recv_exact(sock, n):
    buf = b""
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return None
        buf += chunk
    return buf


def _ws_recv_frame(sock):
    """读一帧。浏览器发来的帧一定带掩码。返回 (opcode, payload) 或 None"""
    head = _ws_recv_exact(sock, 2)
    if not head:
        return None
    opcode = head[0] & 0x0F
    masked = bool(head[1] & 0x80)
    length = head[1] & 0x7F
    if length == 126:
        ext = _ws_recv_exact(sock, 2)
        if ext is None:
            return None
        length = struct.unpack("!H", ext)[0]
    elif length == 127:
        ext = _ws_recv_exact(sock, 8)
        if ext is None:
            return None
        length = struct.unpack("!Q", ext)[0]
    if length > (1 << 20):          # 1 MB 上限，异常数据直接断开
        return None
    mask = _ws_recv_exact(sock, 4) if masked else b"\x00\x00\x00\x00"
    if mask is None:
        return None
    payload = _ws_recv_exact(sock, length) if length else b""
    if payload is None:
        return None
    if masked:
        payload = bytes(payload[i] ^ mask[i % 4] for i in range(len(payload)))
    return opcode, payload


def sim_web_dir():
    """网页仿真的构建产物目录（playground/dist）"""
    base = app_dir()
    candidates = [
        os.path.join(base, "..", "网页仿真", "playground", "dist"),
        os.path.join(base, "网页仿真", "playground", "dist"),
        os.path.join(base, "..", "..", "网页仿真", "playground", "dist"),
    ]
    for p in candidates:
        p = os.path.abspath(p)
        if os.path.isdir(p):
            return p
    return os.path.abspath(candidates[0])


class SimBridge:
    """网页仿真桥：把关节角推给页面，同时接收页面拖动的角度"""

    def __init__(self, host="127.0.0.1", port=SIM_WS_PORT, log=None, on_message=None):
        self.host = host
        self.port = port
        self.log = log or (lambda msg: None)
        self.on_message = on_message or (lambda msg: None)
        self._clients = []
        self._lock = threading.Lock()
        self._alive = True
        self._srv = None
        self.started = False

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
        self._alive = True
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
        self.started = False

    def client_count(self):
        with self._lock:
            return len(self._clients)

    # ---------------- 连接 ----------------
    def _accept_loop(self):
        while self._alive:
            try:
                conn, _addr = self._srv.accept()
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
                          "Sec-WebSocket-Accept: %s\r\n\r\n" % _ws_accept_key(key)).encode())
            conn.settimeout(None)
            with self._lock:
                self._clients.append(conn)
            self.log("网页仿真已连接（当前 %d 个页面）" % self.client_count())
            while self._alive:
                frame = _ws_recv_frame(conn)
                if frame is None:
                    break
                opcode, payload = frame
                if opcode == 0x8:            # close
                    break
                if opcode == 0x9:            # ping → pong
                    try:
                        conn.sendall(struct.pack("!BB", 0x8A, len(payload)) + payload)
                    except OSError:
                        break
                    continue
                if opcode != 0x1:            # 只要文本帧
                    continue
                try:
                    msg = json.loads(payload.decode("utf-8"))
                except Exception:
                    continue
                try:
                    self.on_message(msg)
                except Exception as ex:
                    self.log("网页仿真消息处理失败：%s" % ex)
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
    def broadcast(self, obj):
        """给所有已连接的页面发一条 JSON 消息"""
        with self._lock:
            if not self._clients:
                return False
            msg = _ws_frame(json.dumps(obj, ensure_ascii=False))
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
            return True

    def push(self, angles):
        self.broadcast({
            "type": "motor_angles",
            "payload": {"angles": [round(float(a), 2) for a in angles]},
        })


class _QuietHandler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass


class StaticServer:
    """把网页仿真的构建产物用 HTTP 提供出去，点一下按钮就能打开"""

    def __init__(self, directory, port=SIM_HTTP_PORT, log=None):
        self.directory = directory
        self.port = port
        self.log = log or (lambda msg: None)
        self._srv = None
        self.started = False

    def start(self):
        if not os.path.isdir(self.directory):
            self.log("网页仿真目录不存在：%s" % self.directory)
            return False
        handler = functools.partial(_QuietHandler, directory=self.directory)
        try:
            self._srv = http.server.ThreadingHTTPServer(("127.0.0.1", self.port), handler)
        except OSError as ex:
            self.log("网页仿真静态服务启动失败（端口 %d）：%s" % (self.port, ex))
            return False
        threading.Thread(target=self._srv.serve_forever, daemon=True).start()
        self.started = True
        self.log("网页仿真已就绪：http://127.0.0.1:%d/" % self.port)
        return True

    def stop(self):
        try:
            if self._srv:
                self._srv.shutdown()
                self._srv.server_close()
        except Exception:
            pass
        self._srv = None
        self.started = False


# ============================================================
# 主界面
# ============================================================
class ArmApp:
    def __init__(self, root):
        self.root = root
        self.cfg = load_config()
        self.joints = {j["id"]: j for j in self.cfg["joints"]}

        root.title("%s v%s" % (APP_NAME, APP_VER))
        root.geometry("920x780")
        root.minsize(780, 700)

        # 连接状态
        self.ser = None
        self.connected = False
        self.estop = False
        self.polling = tk.BooleanVar(value=True)
        self._poll_on = True          # 工作线程只读这个布尔量（Tk 变量不能跨线程读）
        self.link_var = tk.StringVar(value=LINK_DIRECT)
        self.autohome_var = tk.BooleanVar(value=bool(self.cfg.get("autohome", False)))
        self._stm32 = False          # 只支持直连总线
        self._posq = collections.deque()                # STM32 模式下 P 查询的待回队列

        # 共享状态（工作线程写，UI 线程读）
        self.lock = threading.Lock()
        self.state = {
            sid: {"pwm": None, "angle": None, "t": 0.0, "ok": False,
                  "mode": None, "ver": None, "err": ""}
            for sid in SERVO_IDS
        }
        self._rx_err = ""
        self._rx_count = 0           # 收到的应答帧数
        self._saw_banner = False     # 是否见到 STM32 上电横幅
        self._cnt_fail = 0           # 最近一轮自检里 FAIL/ERR 的条数
        self._cnt_err = 0
        self._scan_ids = []          # ID 扫描队列
        self._scan_cur = None
        self._scan_found = []
        self._scan_active = False
        self._scan_pending = None
        self._poll_was_on = True

        # 线程与队列
        self._alive = True
        self._tx = queue.Queue()
        self._logq = queue.Queue()
        self._poll_idx = 0
        self._poll_i = 0
        self.torque_on = {sid: True for sid in SERVO_IDS}   # 失能=灰、使能=绿
        self.turns = {sid: 0 for sid in SERVO_IDS}          # 连续角度：累计圈数
        self._last_base = {}
        self._rec = []
        self._rec_on = False
        self._rec_t0 = 0.0
        self._play_on = False
        self._play_i = 0                                # 上一帧的基础角（-135..135）
        self._absent_until = {}      # 回 FAIL 的 ID 暂时跳过，避免每次轮询都等满超时
        self._pause_poll_until = 0.0 # 动作执行期间暂停轮询（舵机运动中不回 PRAD）
        self._poll_gap = 0.020       # 轮询间隔；录制时临时收紧到 9ms（约 18 Hz/轴）
        self._fail_streak = {}       # 连续失败计数
        self._last_poll = 0.0
        self._recent_tx = []          # [(frame, t)] 用于过滤半双工回显
        self._buf = b""

        # 变量
        self.port_var = tk.StringVar(value=self.cfg.get("port") or "")
        self.row_vars = {}
        for sid in SERVO_IDS:
            self.row_vars[sid] = {
                "angle": tk.StringVar(value="--"),
                "target": tk.StringVar(value=""),
                "speed": tk.StringVar(value=str(self.joints[sid].get("speed", 90))),
                "state": tk.StringVar(value="未知"),
            }

        # 网页仿真联动（默认关闭，需要手动打开）
        self.sim_mode = tk.StringVar(value=SIM_MODE_OFF)
        self.sim = SimBridge(log=self.log, on_message=self._on_sim_message)
        self._sim_ok = self.sim.start()
        self._sim_http = None
        self._sim_last_push = {}      # sid → 最近一次推给页面的值（用于回声抑制）
        self._sim_pending = {}        # sid → 待下发的目标角度（合并最新值）
        self._sim_active = {}         # sid → 该轴动作结束时刻（限制同时运动的路数）
        self._sim_next_ok = 0.0
        self._sim_tx_warn = 0.0
        self._sim_mode_sent = None    # 已告知页面的 (模式, 页面数)，变了才重发

        self._build_ui()
        self._refresh_ports()
        # 依据内容自适应窗口大小，避免不同 DPI 下右侧被裁切
        try:
            self.root.update_idletasks()
            rw, rh = self.root.winfo_reqwidth(), self.root.winfo_reqheight()
            sw, sh = self.root.winfo_screenwidth(), self.root.winfo_screenheight()
            w = min(max(rw + 10, 780), int(sw * 0.98))
            h = min(max(rh + 10, 720), int(sh * 0.94))
            self.root.geometry("%dx%d+%d+%d" % (w, h, max(0, (sw - w) // 2), max(0, (sh - h) // 3)))
        except Exception:
            pass
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        threading.Thread(target=self._worker, daemon=True).start()
        self._pump_id = self.root.after(100, self._pump)
        self.log("程序启动，等待连接串口")

    # ---------------- UI ----------------
    def _build_ui(self):
        self._build_top()
        self._build_table()
        self._build_toolbar()
        self._build_sim_bar()
        self._build_log()

    def _build_top(self):
        cfg = ttk.LabelFrame(self.root, text="全局配置")
        cfg.pack(fill="x", padx=8, pady=4)

        ttk.Label(cfg, text="串口:").grid(row=0, column=0, padx=(6, 2), pady=6, sticky="w")
        self.port_box = ttk.Combobox(cfg, textvariable=self.port_var, width=11, state="readonly")
        self.port_box.grid(row=0, column=1, padx=2)
        ttk.Button(cfg, text="刷新", width=6,
                   command=self._refresh_ports).grid(row=0, column=2, padx=4)

        ttk.Label(cfg, text="波特率:").grid(row=0, column=3, padx=(10, 2))
        self.baud_var = tk.StringVar(value=str(self.cfg.get("baud", BAUD_DEFAULT)))
        ttk.Combobox(cfg, textvariable=self.baud_var, width=8, state="readonly",
                     values=[str(b) for b, _ in BAUD_TABLE]).grid(row=0, column=4, padx=2)

        self.btn_conn = tk.Button(cfg, text="连接串口", width=9,
                                  font=(UI_FONT, 10, "bold"), command=self.toggle_connect)
        self.btn_conn.grid(row=0, column=5, padx=4)

        self.btn_estop = tk.Button(cfg, text="急 停", width=6, bg="#D02020", fg="white",
                                   activebackground="#FF4040", activeforeground="white",
                                   font=(UI_FONT, 10, "bold"), command=self.emergency_stop)
        self.btn_estop.grid(row=0, column=6, padx=3)

        self.btn_restore = tk.Button(cfg, text="使能", width=6, bg="#20A040", fg="white",
                                     activebackground="#40C060", activeforeground="white",
                                     font=(UI_FONT, 10, "bold"), command=self.restore_all)
        self.btn_restore.grid(row=0, column=7, padx=3)

        tk.Button(cfg, text="参数配置", width=8,
                  command=self.open_params).grid(row=0, column=8, padx=(6, 4))

    def _build_table(self):
        box = ttk.LabelFrame(self.root, text="六轴状态与控制（ID 001–006）")
        box.pack(fill="x", padx=8, pady=4)

        heads = ["ID", "关节", "当前角度(°)", "目标角度(°)", "", "状态"]
        for c, h in enumerate(heads):
            ttk.Label(box, text=h, anchor="center",
                      font=(UI_FONT, 9, "bold")).grid(row=0, column=c, padx=8, pady=4, sticky="ew")

        self.id_labels = {}
        self.state_labels = {}
        self.angle_labels = {}
        self.name_vars = {}
        self.mode_vars = {}
        for r, sid in enumerate(SERVO_IDS, start=1):
            j = self.joints[sid]
            lbl = tk.Label(box, text="%03d" % sid, font=("Consolas", 10, "bold"),
                           width=4, bg="#8FD19E", fg="black")
            lbl.grid(row=r, column=0, padx=8, pady=3)
            self.id_labels[sid] = lbl

            nm = tk.StringVar(value=j["name"])
            self.name_vars[sid] = nm
            ttk.Label(box, textvariable=nm, width=9, anchor="center").grid(row=r, column=1, padx=6)

            ang_lbl = tk.Label(box, textvariable=self.row_vars[sid]["angle"], width=12,
                               anchor="center", font=("Consolas", 12, "bold"),
                               fg="#1E7F3E", bg="#F5FBF6")
            ang_lbl.grid(row=r, column=2, padx=6, pady=2)
            self.angle_labels[sid] = ang_lbl

            e = ttk.Entry(box, textvariable=self.row_vars[sid]["target"], width=12,
                          font=("Consolas", 11), justify="center")
            e.grid(row=r, column=3, padx=6, pady=3)
            e.bind("<Return>", lambda _e, s=sid: self.send_one(s))
            ttk.Button(box, text="发送", width=5,
                       command=lambda s=sid: self.send_one(s)).grid(row=r, column=4, padx=4)

            st = tk.Label(box, textvariable=self.row_vars[sid]["state"],
                          width=8, anchor="center", font=(UI_FONT, 9), fg="#666666")
            st.grid(row=r, column=5, padx=8)
            self.state_labels[sid] = st

        ttk.Label(box, text="在「目标角度」里输入角度后，点「发送」或按回车",
                  foreground="#8A8A8A").grid(row=len(SERVO_IDS) + 1, column=0,
                                             columnspan=6, sticky="w", padx=8, pady=(0, 6))

    def _build_toolbar(self):
        bar = ttk.Frame(self.root)
        bar.pack(fill="x", padx=8, pady=4)
        tk.Button(bar, text="回到中位", width=9,
                  command=self.go_center).pack(side="left")
        ttk.Separator(bar, orient="vertical").pack(side="left", fill="y", padx=14)
        ttk.Button(bar, text="清空日志", width=9,
                   command=self.clear_log).pack(side="left")

        traj = ttk.LabelFrame(self.root, text="轨迹录制 / 回放")
        traj.pack(fill="x", padx=8, pady=(0, 4))
        self.rec_btn = ttk.Button(traj, text="● 录制", width=9, command=self.toggle_record)
        self.rec_btn.pack(side="left", padx=6, pady=4)
        self.play_btn = ttk.Button(traj, text="▶ 回放", width=9, command=self.toggle_replay)
        self.play_btn.pack(side="left")
        self.traj_status = tk.StringVar(value="未录制")
        ttk.Label(traj, textvariable=self.traj_status,
                  foreground="#0066CC").pack(side="left", padx=14)
        ttk.Button(traj, text="载入…", width=7,
                   command=self.load_track).pack(side="left", padx=6)

    def _build_sim_bar(self):
        box = ttk.LabelFrame(self.root, text="网页仿真")
        box.pack(fill="x", padx=8, pady=(0, 4))

        ttk.Button(box, text="打开网页仿真", width=13,
                   command=self.open_sim).pack(side="left", padx=6, pady=4)
        ttk.Label(box, text="联动").pack(side="left", padx=(10, 4))
        cb = ttk.Combobox(box, textvariable=self.sim_mode, width=11, state="readonly",
                          values=SIM_MODES)
        cb.pack(side="left")
        cb.bind("<<ComboboxSelected>>", lambda _e: self._on_sim_mode())

        self.sim_status = tk.StringVar(value="未连接")
        ttk.Label(box, textvariable=self.sim_status,
                  foreground="#0066CC").pack(side="left", padx=14)
        ttk.Label(box, text="真机→仿真：把舵机角度推给页面　仿真→真机：拖动页面驱动舵机",
                  foreground="#8A8A8A").pack(side="left", padx=(6, 4))

    def _build_log(self):
        box = ttk.LabelFrame(self.root, text="日志")
        box.pack(fill="both", expand=True, padx=8, pady=(2, 8))
        self.log_text = scrolledtext.ScrolledText(box, height=10, font=("Consolas", 9),
                                                  state="disabled", wrap="none")
        self.log_text.pack(fill="both", expand=True, padx=4, pady=4)
        self.log_text.tag_config("tx", foreground="#2D6CDF")
        self.log_text.tag_config("rx", foreground="#1E7F3E")
        self.log_text.tag_config("err", foreground="#C02020")
        self.log_text.tag_config("info", foreground="#666666")

    def _set_title(self, suffix):
        try:
            self.root.title("%s v%s — %s" % (APP_NAME, APP_VER, suffix))
        except Exception:
            pass

    # ---------------- 日志 ----------------
    def log(self, msg, tag="info"):
        ts = time.strftime("%H:%M:%S")
        self._logq.put((ts, msg, tag))

    def clear_log(self):
        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.configure(state="disabled")

    # ---------------- 串口 ----------------
    def _refresh_ports(self):
        ports = list_com_ports()
        cur = self.port_var.get()
        self.port_box["values"] = ports
        if ports and (not cur or cur not in ports):
            self.port_var.set(ports[0])
        elif not ports:
            self.port_var.set("")

    def toggle_connect(self):
        if self.connected:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        port = self.port_var.get().strip()
        if not port:
            messagebox.showwarning("提示", "请选择串口（USB-TTL 模块的 COM 口）")
            return
        self._stm32 = False
        try:
            baud = int(self.baud_var.get())
        except Exception:
            baud = BAUD_DEFAULT
        try:
            ser = WinSerial(port, baud)
            ser.open()
        except Exception as ex:
            messagebox.showerror("打开串口失败", str(ex))
            self.log("打开 %s 失败: %s" % (port, ex), "err")
            return

        self.ser = ser
        self.connected = True
        self.estop = False
        self.cfg["port"] = port
        self.cfg["baud"] = baud
        self.cfg["link"] = LINK_DIRECT
        save_config(self.cfg)
        self.btn_conn.configure(text="断开串口")
        self._set_title("已连接 %s @ %d" % (port, baud))
        self.log("已连接 %s @ %d 8N1" % (port, baud), "info")
        self._buf = b""
        self._recent_tx.clear()
        self._posq.clear()
        self._absent_until.clear()
        for sid in SERVO_IDS:
            self.turns[sid] = 0
            self._last_base.pop(sid, None)
        self.read_all()
        self.root.after(400, self._query_modes)
        self.root.after(1600, self._enforce_modes)
        if self.autohome_var.get():
            self.root.after(1500, self._auto_home)

    def disconnect(self):
        self.connected = False
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        self.btn_conn.configure(text="连接串口")
        self._set_title("未连接")
        for sid in SERVO_IDS:
            self._set_state(sid, None, None, ok=False, err="")
            self.row_vars[sid]["state"].set("未知")
            self.state_labels[sid].configure(fg="#666666")
            self.id_labels[sid].configure(bg="#C8C8C8")
        self.log("已断开串口", "info")

    # ---------------- 发送 ----------------
    def _on_link_changed(self):
        self._stm32 = self.link_var.get().startswith("经下位机")
        if self.connected:
            self.log("链路已切换为「%s」，请断开后重新连接" % self.link_var.get(), "err")
        self._set_title("链路 %s" % self.link_var.get())

    def _is_stm32(self):
        return self._stm32

    def _tx_frame(self, text, note="", quiet=False):
        """text: 不带换行的命令；quiet=True 时不写日志（轮询用）"""
        if not self.connected:
            self.log("未连接，忽略: %s" % text, "err")
            return False
        line = text + ("\n" if self._is_stm32() else "")
        self._tx.put(line)
        self._recent_tx.append((text.rstrip("!"), time.time()))
        if len(self._recent_tx) > 16:
            self._recent_tx.pop(0)
        if not quiet:
            self.log("→ %s%s" % (text, ("  " + note) if note else ""), "tx")
        return True

    # -------- 命令构造（两种链路格式不同） --------
    def _cmd_move(self, sid, pwm, t_ms):
        if self._is_stm32():
            return "M%d,%d,%d" % (sid, pwm, t_ms)
        return "#%03dP%04dT%04d!" % (sid, pwm, t_ms)

    def _cmd_read(self, sid):
        if self._is_stm32():
            now = time.time()
            while self._posq and now - self._posq[0][1] > 0.6:
                self._posq.popleft()
            self._posq.append((sid, now))
            return "P%d" % sid
        return "#%03dPRAD!" % sid

    def _cmd_torque(self, sid, on):
        if self._is_stm32():
            return ("L%d" % sid) if on else ("U%d" % sid)
        return ("#%03dPULR!" if on else "#%03dPULK!") % sid

    def _cmd_mode(self, sid, mode):
        if self._is_stm32():
            return "W%d,%d" % (sid, mode)
        return "#%03dPMOD%d!" % (sid, mode)

    def _cmd_mod_get(self, sid):
        return "#%03dPMOD!" % sid       # 两种链路都可走透传

    def _cmd_id(self, sid, new_id):
        if self._is_stm32():
            return "B%d,%d" % (sid, new_id)
        return "#%03dPID%03d!" % (sid, new_id)

    def _cmd_ver(self, sid):
        if self._is_stm32():
            return "V%d" % sid
        return "#%03dPVER!" % sid

    def _row_speed(self, sid):
        try:
            sp = float(self.joints[sid].get("speed", 90))
        except Exception:
            sp = 90.0
        return max(0.0, sp)

    def _apply_comp(self, sid, target_pwm):
        """停止补偿已按需求去掉：直接返回目标值"""
        return int(round(target_pwm)), 0

    def send_one(self, sid):
        """单轴下发：目标角度 → PWM + 动作时间"""
        if not self.connected:
            messagebox.showwarning("提示", "请先连接串口")
            return
        j = self.joints[sid]
        raw = self.row_vars[sid]["target"].get().strip()
        if not raw:
            messagebox.showinfo("提示", "请先在「目标角度」里填一个角度")
            return
        try:
            ang = float(raw)
        except ValueError:
            messagebox.showerror("错误", "目标角度要填数字")
            return
        amin, amax = float(j["amin"]), float(j["amax"])
        if not (amin <= ang <= amax):
            if not messagebox.askyesno(
                    "超出范围", "目标 %g° 超出该轴范围 %g…%g°。\n仍要限幅发送吗？"
                    % (ang, amin, amax)):
                return
            ang = max(amin, min(amax, ang))
            self.row_vars[sid]["target"].set("%g" % ang)
        self._move_to(sid, ang)

    def _move_time(self, sid, target_pwm, speed=None):
        """行程时间 T(ms) = 角度差 / 速度(°/s) * 1000；位置未知/过期按满行程算，绝不全速"""
        if speed is None:
            speed = self._row_speed(sid)
        j = self.joints[sid]
        cur = self.state[sid]["pwm"]
        tgt = pwm_to_angle(j, target_pwm)
        span = abs(float(j["amax"]) - float(j["amin"])) or 270.0
        age = time.time() - self.state[sid].get("t", 0.0)
        if cur is None or age > 1.5:
            delta = span
        else:
            delta = abs(tgt - pwm_to_angle(j, cur))
        if speed <= 0:
            return 0
        return max(60, min(9999, int(round(delta / speed * 1000.0))))

    def _move_to(self, sid, angle, note=""):
        """按角度下发一次（含速度换算）"""
        j = self.joints[sid]
        pwm, ang2 = angle_to_pwm(j, angle)
        sp = self._row_speed(sid)
        t = self._move_time(sid, pwm, sp)
        self._tx_frame(self._cmd_move(sid, pwm, t),
                       note or ("%s → %g°  %dms" % (j["name"], ang2, t)))
        self._pause_poll_until = max(self._pause_poll_until,
                                     time.time() + t / 1000.0 + 0.5)
        self._absent_until.pop(sid, None)
        self._fail_streak[sid] = 0
        return pwm, t

    def send_all(self):
        if not self.connected:
            messagebox.showwarning("提示", "请先连接串口")
            return
        jobs = []
        for sid in SERVO_IDS:
            raw = self.row_vars[sid]["target"].get().strip()
            if not raw:
                continue
            try:
                ang = float(raw)
            except ValueError:
                continue
            j = self.joints[sid]
            pwm, ang2 = angle_to_pwm(j, ang)
            cmd, applied = self._apply_comp(sid, pwm)
            sp = self._row_speed(sid)
            t = self._move_time(sid, cmd, sp)
            note = "(%s → %g°  %dms%s)" % (j["name"], ang2, t,
                                            ("  补偿%+d" % applied) if applied else "")
            jobs.append((sid, cmd, t, note))
        if not jobs:
            messagebox.showinfo("提示", "没有填写任何目标角度")
            return

        # 分批下发：每轴错开 60ms。六个舵机同时启动的电流尖峰会把 USB 适配器拉掉线。
        def fire(idx=0):
            if idx >= len(jobs) or not self.connected:
                return
            sid, cmd, t, note = jobs[idx]
            self._tx_frame(self._cmd_move(sid, cmd, t), note)
            self._absent_until.pop(sid, None)
            self._fail_streak[sid] = 0
            if idx + 1 < len(jobs):
                self.root.after(MOVE_STAGGER_MS, lambda: fire(idx + 1))

        self._pause_poll_until = max(self._pause_poll_until,
                                     time.time() + MOVE_STAGGER_MS / 1000.0 * len(jobs) +
                                     max(x[2] for x in jobs) / 1000.0 + 0.5)
        fire(0)

    def go_center(self):
        if not self.connected:
            messagebox.showwarning("提示", "请先连接串口")
            return
        for sid in SERVO_IDS:
            j = self.joints[sid]
            mid = int((int(j["pmin"]) + int(j["pmax"])) / 2)
            self.row_vars[sid]["target"].set("%.1f" % pwm_to_angle(j, mid))
        self.send_all()

    def self_check(self):
        """一键诊断：判断是「完全没数据」还是「下位机在但舵机没回」"""
        if not self.connected:
            messagebox.showwarning(
                "连接自检", "还没有连接串口。\n\n请先选 COM 口 → 点「连接」。")
            return
        self._rx_count = 0
        self._saw_banner = False
        self._cnt_fail = 0
        self._cnt_err = 0
        self.log("—— 开始自检：向 6 个 ID 发读取命令 ——", "info")
        self.read_all()
        self.root.after(1500, self._self_check_report)

    def _self_check_report(self):
        got = self._rx_count
        fresh = [sid for sid in SERVO_IDS
                 if self.state[sid]["pwm"] is not None
                 and time.time() - self.state[sid]["t"] < 1.6]
        lines = []
        lines.append("链路：%s" % self.link_var.get())
        lines.append("本次收到应答：%d 条" % got)
        lines.append("有实时角度的轴：%s" % ("、".join("%03d" % x for x in fresh) if fresh else "无"))
        lines.append("FAIL：%d 条    ERR：%d 条" % (self._cnt_fail, self._cnt_err))
        if self._saw_banner:
            lines.append("下位机横幅：已收到（STM32 固件在运行）")

        if got == 0:
            lines.append("")
            lines.append("→ 一点数据都没收到，重点查这三处：")
            lines.append("1. 「链路」是否选错：接的是 STM32 就选「经下位机」，直接接舵机 S 线就选「直连舵机总线」")
            lines.append("2. USB-TTL 的 TX/RX：接 STM32 时 PA9→RXD、PA10→TXD（交叉）；只接 PA2 到 S 时 TX 必须接 S")
            lines.append("3. 端口/波特率：COM 口选对没、是否被别的软件占用；波特率要 115200")
        elif not fresh:
            lines.append("")
            if self._cnt_err > 0 and self._cnt_fail == 0:
                lines.append("→ 下位机回了 ERR（命令没被识别）")
                lines.append("1. 确认「链路」选的是「经下位机 STM32」")
                lines.append("2. 下位机固件是否就是 下位机/ 里这份（协议 M/P/U/L/W/B/V）")
            elif self._cnt_fail > 0 and self._cnt_err == 0:
                lines.append("→ 下位机回了 FAIL：命令收到了，但舵机没应答")
                lines.append("1. 舵机电源：6–8.4V 独立供电、与 STM32 共地")
                lines.append("2. 信号线：STM32 的 PA2 一定要接到舵机 S；要读角度需 PA2 与 PA3 短接后一起接 S")
                lines.append("3. ID 不对：点「ID 扫描」，看舵机实际是哪个 ID（出厂可能全是 000）")
            else:
                lines.append("→ 能收到数据，但没有位置应答：")
                lines.append("1. 舵机电源是否为 6–8.4V 独立供电、是否共地")
                lines.append("2. STM32 的 PA2 是否接到舵机的 S（只接 TX 也能动，但读不到角度）")
                lines.append("3. 点「ID 扫描」，确认舵机实际 ID")
        else:
            lines.append("")
            lines.append("→ 正常。绿色角度会持续刷新；若个别轴不动，检查该轴 ID 与接线。")

        self.log("—— 自检结果 ——\n" + "\n".join(lines), "info")
        (messagebox.showinfo if got else messagebox.showwarning)("连接自检", "\n".join(lines))

    def scan_ids(self):
        """依次查询 ID 0–9，找出总线上真正在应答的舵机"""
        if not self.connected:
            messagebox.showwarning("ID 扫描", "还没有连接串口。")
            return
        self._scan_ids = list(range(0, 10))
        self._scan_found = []
        self._scan_cur = None
        self._scan_active = True
        self._poll_was_on = self._poll_on
        self._poll_on = False          # 扫描期间停轮询，避免回包错配
        self._posq.clear()
        try:                           # 清掉排队中的命令，避免回包串台
            while True:
                self._tx.get_nowait()
        except queue.Empty:
            pass
        self.log("—— ID 扫描：0–9，每个约 0.25s ——", "info")
        self.root.after(80, self._scan_step)

    def _scan_step(self):
        if not self._scan_ids:
            self._scan_active = False
            self._scan_pending = None
            self._poll_on = self._poll_was_on
            self._scan_report()
            return
        sid = self._scan_ids.pop(0)
        self._scan_cur = sid
        self._scan_pending = sid
        if self._stm32:
            line = "P%d\n" % sid
            self._recent_tx.append(("P%d" % sid, time.time()))
        else:
            line = "#%03dPRAD!" % sid
            self._recent_tx.append(("#%03dPRAD" % sid, time.time()))
        try:
            self.ser.write(line.encode("latin-1"))
        except Exception as ex:
            self.log("扫描发送失败: %s" % ex, "err")
        self.root.after(250, self._scan_step)

    def _scan_report(self):
        found = sorted(self._scan_found)
        if found:
            msg = "应答的 ID：%s" % "、".join("%03d" % x for x in found)
            if set(found) == {0}:
                msg += "\n\n→ 只有一个 ID 000 在应答：六个舵机很可能都是出厂默认 ID 000。\n" \
                       "   必须逐个改成 001–006：每次只给一个舵机上电，用参数页改 ID。"
            elif not set(found) & set(SERVO_IDS):
                msg += "\n\n→ 应答的 ID 不在 001–006 范围内，请在参数页按实际 ID 改写。"
            else:
                missing = [x for x in SERVO_IDS if x not in found]
                if missing:
                    msg += "\n\n→ 这些 ID 没应答：%s" % "、".join("%03d" % x for x in missing)
                    if len(found) == 1:
                        msg += ("\n\n（若你现在只接了 1 个舵机，其余轴在表格里显示「超时」是正常的，"
                                "只要这一个轴变绿即可）")
                    else:
                        msg += "\n（检查这些舵机的接线/供电）"
        else:
            msg = ("0–9 全部无应答。\n\n"
                   "→ 舵机侧完全没通：\n"
                   "1. 舵机电源（6–8.4V）是否上电，是否与 STM32 共地\n"
                   "2. STM32 PA2 是否接到舵机 S\n"
                   "3. 若只接了 PA2 没接回读线，读取一定失败，但运动命令应能生效——可先发一个角度试试")
        self.log("—— 扫描结果 ——\n" + msg, "info")
        messagebox.showinfo("ID 扫描", msg)

    def apply_speed_all(self):
        try:
            sp = float(self.speed_all_var.get())
        except ValueError:
            messagebox.showerror("输入错误", "速度必须是数字（°/s）")
            return
        if sp < 0:
            messagebox.showerror("输入错误", "速度不能为负")
            return
        for sid in SERVO_IDS:
            self.row_vars[sid]["speed"].set(str(sp))
            self.joints[sid]["speed"] = sp
        save_config(self.cfg)
        self.log("全部关节速度已设为 %g°/s" % sp, "info")

    def read_one(self, sid):
        self._tx_frame(self._cmd_read(sid))

    def read_all(self):
        for sid in SERVO_IDS:
            self._tx_frame(self._cmd_read(sid), quiet=True)

    # ---------------- 安全 ----------------
    def emergency_stop(self):
        if not self.connected:
            messagebox.showwarning("提示", "请先连接串口")
            return
        # 清空待发队列，避免急停后仍在执行旧动作
        try:
            while True:
                self._tx.get_nowait()
        except queue.Empty:
            pass
        self.estop = True
        for sid in SERVO_IDS:
            self.torque_on[sid] = False
            self._tx_frame(self._cmd_torque(sid, False), "(失能)")
        self._set_title("急停：已释放力矩")
        self.log("急停 —— 全部舵机已松轴（ULK）", "err")

    def restore_all(self):
        if not self.connected:
            messagebox.showwarning("提示", "请先连接串口")
            return
        for sid in SERVO_IDS:
            self.torque_on[sid] = True
            self._tx_frame(self._cmd_torque(sid, True), "(使能)")
        self.estop = False
        self._set_title("已恢复力矩")
        self.log("全部舵机恢复力矩", "info")

    # ---------------- 轮询与显示 ----------------
    def toggle_poll(self):
        v = not self.polling.get()
        self.polling.set(v)
        self._poll_on = v
        if hasattr(self, "poll_btn"):
            self.poll_btn.configure(text="回读: %s" % ("开" if v else "关"),
                                    bg="#BFE6C8" if v else "#E8E8E8")

    def _track_turns(self, sid, base):
        """舵机回读在两端会被压平，跨过 90° 死区时从 -135 跳到 +135（或反之）。
        用这个跳变来累计圈数，才能显示连续的 -225 / +225。"""
        last = self._last_base.get(sid)
        if last is not None:
            d = base - last
            if d > 135:          # -135 → +135：实际在往下转
                self.turns[sid] = self.turns.get(sid, 0) - 1
            elif d < -135:       # +135 → -135：实际在往上转
                self.turns[sid] = self.turns.get(sid, 0) + 1
        self._last_base[sid] = base
        return base + 360.0 * self.turns.get(sid, 0)

    def _set_state(self, sid, pwm, angle, ok=True, err=""):
        with self.lock:
            s = self.state[sid]
            if pwm is not None:
                s["pwm"] = pwm
                base = pwm_to_angle(self.joints[sid], pwm)
                s["angle"] = self._track_turns(sid, base)
            elif angle is not None:
                s["angle"] = angle
            s["t"] = time.time()
            s["ok"] = ok
            s["err"] = err

    def _pump(self):
        # 日志
        try:
            while True:
                ts, msg, tag = self._logq.get_nowait()
                self.log_text.configure(state="normal")
                self.log_text.insert("end", "[%s] %s\n" % (ts, msg), tag)
                self.log_text.see("end")
                self.log_text.configure(state="disabled")
        except queue.Empty:
            pass

        # 状态
        now = time.time()
        with self.lock:
            snapshot = {sid: dict(self.state[sid]) for sid in SERVO_IDS}
        for sid in SERVO_IDS:
            s = snapshot[sid]
            if s["pwm"] is None:
                self.row_vars[sid]["angle"].set("--")
                self.row_vars[sid]["state"].set("未知")
                self.state_labels[sid].configure(fg="#666666")
                self.id_labels[sid].configure(
                    bg="#8FD19E" if self.torque_on.get(sid, True) else "#C8C8C8")
                continue
            if s["angle"] is None:
                self.row_vars[sid]["angle"].set("--")
            else:
                self.row_vars[sid]["angle"].set("%.1f" % s["angle"])
            age = now - s["t"]
            if s["ok"] and age < 1.5:
                self.row_vars[sid]["state"].set("正常")
                self.state_labels[sid].configure(fg="#1E7F3E")
            elif age >= 1.5:
                self.row_vars[sid]["state"].set("超时")
                self.state_labels[sid].configure(fg="#C08000")
            else:
                self.row_vars[sid]["state"].set("错误")
                self.state_labels[sid].configure(fg="#C02020")
            # 色块表示力矩状态：失能灰、使能绿
            self.id_labels[sid].configure(
                bg="#8FD19E" if self.torque_on.get(sid, True) else "#C8C8C8")

        if getattr(self, "_lost", False):
            self._lost = False
            self.disconnect()
            messagebox.showwarning(
                "串口掉线",
                "USB 适配器掉线了（多半是舵机同时启动导致电压跌落）。\n\n"
                "已自动断开。请检查舵机电源，然后重新连接。")

        if getattr(self, "sim", None):
            self._announce_sim_mode()
            self._push_sim()
            self._sim_step()
            if getattr(self, "sim_status", None) is not None:
                if self.sim_mode.get() == SIM_MODE_OFF:
                    self.sim_status.set("已关闭（未联动）")
                else:
                    n = self.sim.client_count()
                    self.sim_status.set(("已连接 %d 个页面" % n) if n else "等待页面连接")

        if self.estop:
            self.btn_estop.configure(text="急停中")
        else:
            self.btn_estop.configure(text="急 停")

        self._pump_id = self.root.after(100, self._pump)

    MODE_SHORT = {1: "角度+", 2: "角度−"}

    def _refresh_mode(self, sid):
        v = self.mode_vars.get(sid)
        if v is None:
            return
        m = self.state[sid].get("mode")
        txt = self.MODE_SHORT.get(m, "?") if m else "?"
        want = self.joints[sid].get("mode_want")
        if want and m and want != m:
            txt += "·待重启"
        v.set(txt)

    def _refresh_names(self):
        """关节名后面标注当前模式"""
        try:
            for sid in SERVO_IDS:
                j = self.joints[sid]
                nm = j["name"]
                if int(j.get("mode", 1) or 1) in (5, 6):
                    nm += " 速"
                v = self.name_vars.get(sid)
                if v is not None:
                    v.set(nm)
        except Exception:
            pass

    def _enforce_modes(self):
        """连接后按 arm_config 里保存的模式校正舵机：默认（角度模式）会把 5/6 拉回 1/2"""
        if not self.connected:
            return
        jobs = []
        for sid in SERVO_IDS:
            want = int(self.joints[sid].get("mode", 1) or 1)
            now = self.state[sid].get("mode")
            if now is None or now == want:
                continue
            if self.joints[sid].get("mode_want") == want:
                continue          # 已经请求过，等舵机重启
            jobs.append((sid, want))
        if not jobs:
            return
        self.log("按配置校正模式：%s（改完要断电重启才生效）"
                 % "、".join("%s→%d" % (self.joints[x]["name"], m) for x, m in jobs), "info")
        def fire(i=0):
            if i >= len(jobs) or not self.connected:
                return
            sid, mode = jobs[i]
            self.joints[sid]["mode_want"] = mode
            self._tx_frame("#%03dPMOD%d!" % (sid, mode),
                           "(%s 设为模式%d)" % (self.joints[sid]["name"], mode))
            self._refresh_mode(sid)
            if i + 1 < len(jobs):
                self.root.after(MOVE_STAGGER_MS, lambda: fire(i + 1))
        save_config(self.cfg)
        fire(0)

    def _query_modes(self):
        """连上后读一次六轴模式（当前真实模式，顺便记住）"""
        if not self.connected:
            return
        jobs = [(sid, "#%03dPMOD!" % sid) for sid in SERVO_IDS]
        def fire(i=0):
            if i >= len(jobs) or not self.connected:
                return
            sid, frame = jobs[i]
            self._tx_frame(frame, "(读模式)")
            if i + 1 < len(jobs):
                self.root.after(MOVE_STAGGER_MS, lambda: fire(i + 1))
        fire(0)

    # ---------------- 轨迹录制 / 回放 ----------------
    def _track_dir(self):
        d = os.path.join(BASE_DIR, "tracks")
        try:
            os.makedirs(d, exist_ok=True)
        except Exception:
            pass
        return d

    def toggle_record(self):
        if self._rec_on:
            self._rec_on = False
            self._poll_gap = 0.020
            self.rec_btn.configure(text="● 录制")
            self._save_track()
            return
        if not self.connected:
            messagebox.showwarning("提示", "请先连接串口")
            return
        self._rec_on = True
        self._rec = []
        self._rec_t0 = time.time()
        self.rec_btn.configure(text="■ 停止录制")
        self._poll_gap = 0.007        # 录制期间加快回读（约 24 Hz/轴 @115200）
        self.log("开始录制轨迹（20 帧/秒，回读已提速）", "info")
        self._rec_tick()

    def _rec_tick(self):
        if not self._rec_on:
            return
        if not self.connected:
            self._rec_on = False
            self.rec_btn.configure(text="● 录制")
            return
        row = {}
        for sid in SERVO_IDS:
            a = self.state[sid]["angle"]
            if a is not None:
                row[str(sid)] = round(float(a), 1)
        if len(row) >= 1:
            self._rec.append((round(time.time() - self._rec_t0, 3), row))
        self.traj_status.set("录制中… %d 帧 / %.1f 秒"
                             % (len(self._rec), self._rec[-1][0] if self._rec else 0.0))
        self.root.after(REC_PERIOD_MS, self._rec_tick)

    def _smooth_track(self):
        """5 点二项式平滑 (1,2,3,2,1)/9：压掉 ±1 PWM 抖动，保留大动作"""
        n = len(self._rec)
        if n < 3:
            return
        w = [(0, 1), (1, 2), (2, 3), (3, 2), (4, 1)]
        out = []
        for i, (t, row) in enumerate(self._rec):
            new = {}
            for k, v in row.items():
                tot = wsum = 0.0
                for off, wt in w:
                    j = i + off - 2
                    if 0 <= j < n and k in self._rec[j][1]:
                        tot += self._rec[j][1][k] * wt
                        wsum += wt
                new[k] = round(tot / wsum, 2) if wsum else v
            out.append((t, new))
        self._rec = out

    def _save_track(self):
        self._smooth_track()
        if not self._rec:
            self.traj_status.set("未录制")
            return
        path = os.path.join(self._track_dir(),
                            "轨迹_%s.csv" % time.strftime("%Y%m%d_%H%M%S"))
        try:
            with open(path, "w", encoding="utf-8-sig") as f:
                f.write("t," + ",".join("s%d" % i for i in SERVO_IDS) + "\n")
                for t, row in self._rec:
                    f.write("%.3f," % t + ",".join(
                        ("%g" % row.get(str(i), "")) if str(i) in row else ""
                        for i in SERVO_IDS) + "\n")
            self.traj_status.set("已保存 %d 帧 / %.1f 秒" % (len(self._rec), self._rec[-1][0]))
            self.log("轨迹已保存：%s（%d 帧）" % (path, len(self._rec)), "info")
        except Exception as ex:
            self.log("轨迹保存失败：%s" % ex, "err")

    def load_track(self):
        path = filedialog.askopenfilename(
            title="载入轨迹", initialdir=self._track_dir(),
            filetypes=[("CSV 轨迹", "*.csv"), ("全部文件", "*.*")])
        if not path:
            return
        try:
            rows = []
            with open(path, "r", encoding="utf-8-sig") as f:
                head = f.readline()
                for line in f:
                    parts = [x.strip() for x in line.strip().split(",")]
                    if len(parts) < 2 or not parts[0]:
                        continue
                    t = float(parts[0])
                    row = {}
                    for i, sid in enumerate(SERVO_IDS):
                        if i + 1 < len(parts) and parts[i + 1] != "":
                            row[str(sid)] = float(parts[i + 1])
                    if row:
                        rows.append((t, row))
            if not rows:
                raise ValueError("文件里没有有效帧")
            self._rec = rows
            self._smooth_track()
            self.traj_status.set("已载入 %d 帧 / %.1f 秒" % (len(rows), rows[-1][0]))
            self.log("轨迹已载入：%s（%d 帧）" % (path, len(rows)), "info")
        except Exception as ex:
            messagebox.showerror("载入失败", str(ex))

    def toggle_replay(self):
        if self._play_on:
            self._stop_replay()
            return
        if not self._rec:
            messagebox.showinfo("提示", "还没有轨迹，先录制或用「载入…」")
            return
        if not self.connected:
            messagebox.showwarning("提示", "请先连接串口")
            return
        self._play_on = True
        self._play_i = 0
        self._play_t0 = time.time()
        self.play_btn.configure(text="■ 停止回放")
        self._pause_poll_until = time.time() + 3600      # 回放期间不轮询
        self.log("开始回放轨迹（%d 帧）" % len(self._rec), "info")
        self._play_step()

    def _stop_replay(self):
        self._play_on = False
        self.play_btn.configure(text="▶ 回放")
        self._pause_poll_until = time.time() + 0.3
        self.log("回放已停止", "info")

    def _track_at(self, t):
        """在录制数据上按时间线性插值，得到 t 时刻的六轴角度"""
        rec = self._rec
        if not rec:
            return {}
        def as_int(d):
            return {int(k): v for k, v in d.items()}
        if t <= rec[0][0]:
            return as_int(rec[0][1])
        if t >= rec[-1][0]:
            return as_int(rec[-1][1])
        lo, hi = 0, len(rec) - 1
        while lo + 1 < hi:
            mid = (lo + hi) // 2
            if rec[mid][0] <= t:
                lo = mid
            else:
                hi = mid
        t0, a = rec[lo]
        t1, b = rec[hi]
        span = max(1e-6, t1 - t0)
        k = (t - t0) / span
        out = {}
        for sid in SERVO_IDS:
            kx = str(sid)
            if kx in a and kx in b:
                out[sid] = a[kx] + (b[kx] - a[kx]) * k
            elif kx in a:
                out[sid] = a[kx]
            elif kx in b:
                out[sid] = b[kx]
        return out

    def _play_step(self):
        if not self._play_on:
            return
        if not self.connected:
            self._stop_replay()
            return
        dur = self._rec[-1][0] if self._rec else 0.0
        idx = self._play_i
        t = idx * PLAY_STEP_S
        if t > dur + 0.05:
            self.log("回放结束", "info")
            self._stop_replay()
            self.traj_status.set("回放完成（%.1f 秒）" % dur)
            return
        ang = self._track_at(min(t, dur))
        # T 给 1.5 倍步长：下一条到达时舵机还在走，才不会「走一下停一下」
        t_ms = int(PLAY_STEP_S * 1000 * 1.5)
        for sid, a in ang.items():
            j = self.joints[sid]
            pwm, _ = angle_to_pwm(j, a)
            self._tx_frame("#%03dP%04dT%04d!" % (sid, pwm, t_ms), quiet=True)
        pct = min(100.0, 100.0 * t / dur) if dur > 0 else 100.0
        self.traj_status.set("回放中… %.1f / %.1f 秒（%d%%）" % (t, dur, pct))
        self._play_i += 1
        nxt = self._play_t0 + self._play_i * PLAY_STEP_S
        self.root.after(max(5, int((nxt - time.time()) * 1000)), self._play_step)

    def _sim_angles(self):
        """给网页仿真的六路角度：1-5 轴用显示角度，第 6 轴换算成 0-400° 夹爪开合"""
        out = []
        for sid in SERVO_IDS[:5]:
            a = self.state[sid]["angle"]
            out.append(0.0 if a is None else float(a))
        pwm6 = self.state[6]["pwm"]
        out.append(0.0 if pwm6 is None else (pwm6 - 500) / 2000.0 * 400.0)
        return out

    # ---------------- 网页仿真 ----------------
    def _on_sim_mode(self):
        mode = self.sim_mode.get()
        if mode != SIM_MODE_FROM_SIM:
            self._sim_pending.clear()       # 切走时丢弃还没下发的目标
            self._sim_active.clear()
        self.log("网页仿真联动：%s" % mode)
        if mode == SIM_MODE_OFF:
            self.sim_status.set("已关闭")
        else:
            self.sim_status.set("已连接 %d 个页面" % self.sim.client_count()
                                if self.sim.client_count() else "等待页面连接")

    def _announce_sim_mode(self):
        """把当前联动模式告诉页面：页面据此禁用「当前末端逆解姿态」等会驱动真机的按钮"""
        key = (self.sim_mode.get(), self.sim.client_count())
        if key == self._sim_mode_sent:
            return
        mode = self.sim_mode.get()
        self.sim.broadcast({
            "type": "sim_mode",
            "payload": {"mode": SIM_MODE_TOKENS.get(mode, "off"), "label": mode},
        })
        self._sim_mode_sent = key

    def open_sim(self):
        """起静态服务 + 打开浏览器"""
        if not getattr(self, "_sim_ok", False):
            self._sim_ok = self.sim.start()
        web = sim_web_dir()
        if not os.path.isdir(web):
            messagebox.showinfo(
                "网页仿真",
                "没有找到网页仿真的构建产物：\n%s\n\n"
                "请先在 网页仿真/playground 目录执行：\n    npm install && npm run build" % web)
            return
        if self._sim_http is None or not self._sim_http.started:
            self._sim_http = StaticServer(web, SIM_HTTP_PORT, log=self.log)
            if not self._sim_http.start():
                return
        url = "http://127.0.0.1:%d/" % SIM_HTTP_PORT
        webbrowser.open(url)
        self.log("已打开网页仿真：%s（联动模式：%s）" % (url, self.sim_mode.get()))

    def _on_sim_message(self, msg):
        """工作线程收到页面消息 → 切回 UI 线程处理"""
        if not isinstance(msg, dict):
            return
        if msg.get("type") != "sim_angles":
            return
        payload = msg.get("payload") or {}
        angles = payload.get("angles")
        if isinstance(angles, list) and len(angles) >= 6:
            try:
                self.root.after(0, lambda: self._apply_sim_angles(angles))
            except Exception:
                pass

    def _apply_sim_angles(self, angles):
        """仿真 → 真机：这里只记录目标，真正下发交给 _sim_step 串行排队"""
        if self.sim_mode.get() != SIM_MODE_FROM_SIM:
            return
        if self.estop:
            return
        if not self.connected:
            if time.time() - self._sim_tx_warn > 5:
                self._sim_tx_warn = time.time()
                self.log("网页仿真在发角度，但串口未连接，已忽略", "err")
            return

        for idx, sid in enumerate(SERVO_IDS):
            try:
                val = float(angles[idx])
            except (TypeError, ValueError, IndexError):
                continue
            last = self._sim_last_push.get(sid)
            if last is not None and abs(val - last) < SIM_DEADBAND[idx]:
                continue
            j = self.joints[sid]
            if sid == 6:
                pwm = PWM_MIN_LIMIT + val / SIM_GRIPPER_SPAN * (PWM_MAX_LIMIT - PWM_MIN_LIMIT)
                pwm = max(PWM_MIN_LIMIT, min(PWM_MAX_LIMIT, pwm))
                ang = pwm_to_angle(j, pwm)
            else:
                ang = max(float(j.get("amin", -135)), min(float(j.get("amax", 135)), val))
            self._sim_pending[sid] = ang      # 同轴只留最新值，避免积压

    def _sim_step(self):
        """把仿真目标串行下发：限制同时运动的轴数，避免六轴一起启动把电源拉低"""
        if self.sim_mode.get() != SIM_MODE_FROM_SIM:
            self._sim_pending.clear()
            self._sim_active.clear()
            return
        if not (self.connected and self._sim_pending) or self.estop:
            return

        now = time.time()
        for sid in [s for s, end in self._sim_active.items() if end <= now]:
            self._sim_active.pop(sid, None)
        if len(self._sim_active) >= SIM_MAX_CONCURRENT:
            return
        if now < self._sim_next_ok:
            return

        def gap_to(sid):
            cur = self.state[sid]["angle"]
            if cur is None:
                return 999.0
            return abs(self._sim_pending[sid] - cur)

        sid = max(self._sim_pending, key=gap_to)
        ang = self._sim_pending.pop(sid)
        j = self.joints[sid]
        pwm, ang2 = angle_to_pwm(j, ang)
        cmd, applied = self._apply_comp(sid, pwm)
        t = max(120, min(SIM_MOVE_MAX_MS, self._move_time(sid, cmd, SIM_MOVE_SPEED)))

        self.row_vars[sid]["target"].set("%.1f" % ang2)
        self._tx_frame(self._cmd_move(sid, cmd, t),
                       "(仿真 → %s %.1f°  %dms)" % (j["name"], ang2, t))
        # 动作期间跳过这一路的位置查询，但不动全局暂停，其他轴的状态仍能刷新
        self._absent_until[sid] = now + t / 1000.0 + 0.1
        self._fail_streak[sid] = 0
        self._sim_active[sid] = now + t / 1000.0
        self._sim_next_ok = now + SIM_MOVE_GAP_MS / 1000.0

    def _push_sim(self):
        """真机 → 仿真：按联动模式把当前角度推给页面"""
        if self.sim_mode.get() != SIM_MODE_TO_SIM:
            return
        if not self.sim.client_count():
            return
        angles = self._sim_angles()
        for i, sid in enumerate(SERVO_IDS):
            self._sim_last_push[sid] = angles[i]
        self.sim.push(angles)

    # ---------------- 工作线程 ----------------
    def _on_autohome_toggle(self):
        self.cfg["autohome"] = bool(self.autohome_var.get())
        save_config(self.cfg)
        self.log("上电回零：%s" % ("开" if self.autohome_var.get() else "关"), "info")

    def _next_poll_sid(self, now):
        """轮询游标：跳过最近回 FAIL 的 ID（5 秒后再试）"""
        n = len(SERVO_IDS)
        for k in range(n):
            sid = SERVO_IDS[(self._poll_i + k) % n]
            if self._absent_until.get(sid, 0.0) > now:
                continue
            self._poll_i = (self._poll_i + k + 1) % n
            return sid
        return None

    def _worker(self):
        while self._alive:
            try:
                frame = self._tx.get(timeout=0.01)
            except queue.Empty:
                frame = None

            if frame and self.connected and self.ser:
                try:
                    self.ser.write(frame.encode("latin-1"))
                    self._recent_tx.append((frame.rstrip("!"), time.time()))
                    if len(self._recent_tx) > 16:
                        self._recent_tx.pop(0)
                except Exception as ex:
                    self.log("发送失败: %s" % ex, "err")
                    self._lost = True

            # 自动轮询位置：一问一答，等上一条回包（或超时）再问下一条，否则回包会串位
            now = time.time()
            while self._posq and now - self._posq[0][1] > 0.5:
                self._posq.popleft()
            if (self.connected and self._poll_on and not self._posq
                    and now >= self._pause_poll_until
                    and now - self._last_poll > self._poll_gap):
                sid = self._next_poll_sid(now)
                self._last_poll = now
                if sid is not None:
                    try:
                        cmd = self._cmd_read(sid)
                        line = cmd + ("\n" if self._stm32 else "")
                        self.ser.write(line.encode("latin-1"))
                        self._recent_tx.append((cmd.rstrip("!"), time.time()))
                        if len(self._recent_tx) > 24:
                            self._recent_tx.pop(0)
                    except Exception:
                        pass

            if self.connected and self.ser:
                try:
                    data = self.ser.read(512)
                except Exception:
                    data = b""
                if data:
                    self._buf += data
                    self._parse_buffer()

            time.sleep(0.004)

    def _parse_buffer(self):
        """按 '!'（直连总线）或 '\\n'（STM32 行协议）切分接收缓存"""
        while True:
            ie = self._buf.find(b"!")
            inl = self._buf.find(b"\n")
            cands = [x for x in (ie, inl) if x >= 0]
            if not cands:
                break
            k = min(cands)
            chunk, self._buf = self._buf[:k], self._buf[k + 1:]
            text = chunk.decode("latin-1").strip().strip("\r")
            if text:
                self._handle_frame(text)

    def _is_echo(self, text):
        now = time.time()
        self._recent_tx = [(f, t) for f, t in self._recent_tx if now - t < 0.6]
        t = text.rstrip("!")
        for f, _t in self._recent_tx:
            if f == t:
                return True
        return False

    def _on_pos(self, val):
        """STM32 的 P 查询回的是 POS,<pwm>，不带 ID，用待回队列配对
        （轮询回包不写日志，否则日志会被刷屏）"""
        try:
            pwm = int(str(val).strip())
        except ValueError:
            return
        if self._scan_active:
            sid = self._scan_pending
            self._scan_pending = None
            if sid is not None and sid not in self._scan_found:
                self._scan_found.append(sid)
                self.log("    ✓ ID %03d 在线（PWM=%d）" % (sid, pwm), "info")
            return
        sid = None
        now = time.time()
        while self._posq:
            s_, t_ = self._posq.popleft()
            if now - t_ < 0.8:
                sid = s_
                break
        if sid is None:
            self.log("← POS,%d（没有匹配的查询，已忽略）" % pwm, "err")
            return
        if self._scan_active and self._scan_cur == sid and sid not in self._scan_found:
            self._scan_found.append(sid)
            self.log("    ✓ ID %03d 在线（PWM=%d）" % (sid, pwm), "info")
        if sid not in SERVO_IDS:
            self.log("← POS,%d（ID %03d 不在 001-006 里）" % (pwm, sid), "err")
            return
        if not (500 <= pwm <= 2500):
            # 帧被打坏时会解析出离谱的值，直接丢弃，让下一轮重读
            self.log("← POS,%d 超出 500-2500，判为坏帧，丢弃" % pwm, "err")
            return
        self._absent_until.pop(sid, None)
        self._fail_streak[sid] = 0
        self._set_state(sid, pwm, pwm_to_angle(self.joints[sid], pwm), ok=True)

    def _handle_frame(self, text):
        if self._is_echo(text):
            return
        self._rx_count += 1
        up = text.upper().strip()
        if up.startswith("SERVO BUS READY"):
            self._saw_banner = True
            self.log("← 下位机就绪：%s" % text, "rx")
            return
        if up.startswith("POS,"):
            self._on_pos(text.split(",", 1)[1])
            return
        if up.startswith("VER,"):
            self.log("← 固件版本 %s" % text.split(",", 1)[1], "rx")
            return
        if up == "OK":
            self.log("← OK", "rx")
            return
        if up in ("ERR", "FAIL"):
            if up == "FAIL":
                self._cnt_fail += 1
                now = time.time()
                while self._posq:                  # 一次查询对应一条回包，消耗掉
                    _s, _t = self._posq.popleft()
                    if now - _t < 1.0:
                        self._fail_streak[_s] = self._fail_streak.get(_s, 0) + 1
                        if self._fail_streak[_s] >= 3:
                            self._absent_until[_s] = now + 5.0   # 连续 3 次没回才跳过
                        break
            else:
                self._cnt_err += 1
            self.log("← %s" % text, "err")
            return
        if not text.startswith("#"):
            self.log("← %s" % text, "rx")
            return
        if not text.endswith("!"):
            text += "!"
        mm = MOD_RE.match(text)
        if mm:
            sid = int(mm.group(1))
            mode = int(mm.group(2))
            if sid in SERVO_IDS:
                with self.lock:
                    self.state[sid]["mode"] = mode
                # 注意：只记录实际值，不覆盖 arm_config 里的期望模式
                self._refresh_mode(sid)
                self.log("    %s 当前模式 %d（%s）"
                         % (self.joints[sid]["name"], mode, MODE_TEXT.get(mode, "")), "info")
            return
        m = REPLY_RE.match(text)
        if not m:
            self.log("← %s" % text, "rx")
            return
        sid = int(m.group(1))
        body = m.group(2)
        if not body.isdigit():          # 位置回包不刷日志（表格里已经实时显示）
            self.log("← %s" % text, "rx")

        if sid not in SERVO_IDS:
            return
        if body.isdigit():
            pwm = int(body)
            if self._scan_active and self._scan_cur == sid and sid not in self._scan_found:
                self._scan_found.append(sid)
            self._set_state(sid, pwm, pwm_to_angle(self.joints[sid], pwm), ok=True)
        elif body.startswith("MOD"):
            mode = body[3:]
            if mode.isdigit():
                with self.lock:
                    self.state[sid]["mode"] = int(mode)
            self.log("    %s 当前模式 %s（%s）" %
                     (self.joints[sid]["name"], mode, MODE_TEXT.get(int(mode or 0), "")), "info")
        elif body.startswith("V"):
            self.log("    %s 固件版本 %s" % (self.joints[sid]["name"], body[1:]), "info")
        elif body.startswith("T"):
            self.log("    %s 温度/电压 %s" % (self.joints[sid]["name"], body[1:]), "info")
        elif body == "":
            self.log("    %s 在线（ID 匹配）" % self.joints[sid]["name"], "info")

    # ---------------- 参数设置 ----------------
    def open_params(self):
        ParamWindow(self)

    # ---------------- 关闭 ----------------
    def on_close(self):
        self._alive = False
        try:
            if getattr(self, "sim", None):
                self.sim.stop()
        except Exception:
            pass
        try:
            if getattr(self, "_sim_http", None):
                self._sim_http.stop()
        except Exception:
            pass
        try:
            if getattr(self, "_pump_id", None):
                self.root.after_cancel(self._pump_id)
        except Exception:
            pass
        try:
            if self.ser:
                self.ser.close()
        except Exception:
            pass
        self.root.destroy()


# ============================================================
# 参数设置窗口
# ============================================================
class ParamWindow:
    """参数配置：一张表，每行一个舵机，改完点该行「保存」写入 arm_config.json"""

    COLS = ["ID", "名称", "速度(°/s)", "模式", "保存"]

    def __init__(self, app):
        self.app = app
        self.win = tk.Toplevel(app.root)
        self.win.title("参数配置")
        self.win.transient(app.root)
        self.win.resizable(False, False)

        outer = ttk.Frame(self.win, padding=(10, 10))
        outer.pack(fill="both", expand=True)

        frame = ttk.LabelFrame(
            outer, text=" 各舵机参数（改完点该行「保存」写入 arm_config.json） ",
            padding=(8, 6))
        frame.pack(fill="both", expand=True)

        for c, h in enumerate(self.COLS):
            ttk.Label(frame, text=h, font=(UI_FONT, 9, "bold")).grid(
                row=0, column=c, padx=5, pady=4)

        self.vars = {}
        for r, sid in enumerate(SERVO_IDS, start=1):
            j = app.joints[sid]
            tk.Label(frame, text="%03d" % sid,
                     font=("Consolas", 10, "bold")).grid(row=r, column=0, padx=5, pady=3)
            ttk.Label(frame, text=j["name"], width=5,
                      anchor="center").grid(row=r, column=1, padx=3)

            v = {}
            sv = tk.StringVar(value="%g" % j["speed"])
            ttk.Entry(frame, textvariable=sv, width=8,
                      justify="center").grid(row=r, column=2, padx=4)
            v["speed"] = sv

            mode0 = int(j.get("mode", 1) or 1)
            if mode0 not in MODE_TEXT:
                mode0 = 1
            mv = tk.StringVar(value=str(mode0))
            ttk.Combobox(frame, textvariable=mv, width=10, state="readonly",
                         values=["1", "2"]).grid(row=r, column=3, padx=4)
            v["mode"] = mv

            ttk.Button(frame, text="保存", width=6,
                       command=lambda x=sid: self.save_row(x)).grid(row=r, column=4, padx=6, pady=3)

            self.vars[sid] = v

        # ---------------- 手动微调中位 ----------------
        ops = ttk.LabelFrame(outer, text=" 手动微调中位 ", padding=(8, 6))
        ops.pack(fill="x", pady=(10, 0))

        r1 = ttk.Frame(ops)
        r1.pack(fill="x")
        ttk.Label(r1, text="目标关节").pack(side="left")
        self.sid_var = tk.StringVar(value="001")
        self.sid_box = ttk.Combobox(
            r1, width=12, state="readonly",
            values=["%03d  %s" % (s, app.joints[s]["name"]) for s in SERVO_IDS])
        self.sid_box.current(0)
        self.sid_box.pack(side="left", padx=(6, 12))
        self.sid_box.bind("<<ComboboxSelected>>", self._on_pick)

        ttk.Button(r1, text="设为 1500 中位", width=13,
                   command=self.apply_sck).pack(side="left", padx=6)

        # ---------------- 修改ID（与手动微调中位隔开） ----------------
        pid = ttk.LabelFrame(outer, text=" 修改ID ", padding=(8, 6))
        pid.pack(fill="x", pady=(10, 0))

        r2 = ttk.Frame(pid)
        r2.pack(fill="x")
        ttk.Label(r2, text="原 ID").pack(side="left")
        self.oldid_var = tk.StringVar(value="000")
        ttk.Entry(r2, textvariable=self.oldid_var, width=5,
                  justify="center").pack(side="left", padx=5)
        ttk.Label(r2, text="→ 新 ID").pack(side="left")
        self.newid_var = tk.StringVar(value="006")
        ttk.Entry(r2, textvariable=self.newid_var, width=5,
                  justify="center").pack(side="left", padx=5)
        ttk.Button(r2, text="修改 ID", width=8, command=self.apply_pid).pack(side="left", padx=4)

        ttk.Button(r2, text="半恢复出厂", width=11, command=self.on_cleo).pack(side="left", padx=(16, 4))
        ttk.Button(r2, text="全恢复出厂", width=11, command=self.on_cle).pack(side="left")

        self.win.update_idletasks()
        w = self.win.winfo_reqwidth() + 14
        h = self.win.winfo_reqheight() + 34
        sw, sh = self.win.winfo_screenwidth(), self.win.winfo_screenheight()
        self.win.geometry("%dx%d+%d+%d" % (min(w, int(sw * 0.95)), min(h, int(sh * 0.9)),
                                           (sw - min(w, int(sw * 0.95))) // 2,
                                           max(30, (sh - min(h, int(sh * 0.9))) // 4)))

    # ---------------- 通用 ----------------
    def _sel_sid(self):
        try:
            return int(self.sid_var.get())
        except Exception:
            return 1

    def _on_pick(self, _e=None):
        idx = self.sid_box.current()
        if idx >= 0:
            self.sid_var.set("%03d" % SERVO_IDS[idx])

    def send(self, frame, note=""):
        self.app._tx_frame(frame, note)

    # ---------------- 表格保存 ----------------
    def save_row(self, sid):
        j = self.app.joints[sid]
        v = self.vars[sid]
        try:
            sp = float(v["speed"].get())
            mode = int(v["mode"].get())
        except ValueError:
            messagebox.showerror("错误", "%03d 这一行有非数字输入" % sid, parent=self.win)
            return
        if sp < 0:
            messagebox.showerror("错误", "速度必须 ≥ 0", parent=self.win)
            return
        old_mode = int(j.get("mode", 1) or 1)
        j.update({"speed": sp, "mode": mode})
        save_config(self.app.cfg)
        if mode != old_mode or self.app.state[sid].get("mode") != mode:
            self.app.joints[sid]["mode_want"] = mode
            self.send("#%03dPMOD%d!" % (sid, mode), "(下发模式%d)" % mode)
            self.app.log("%03d %s 模式设为 %d（断电重启后生效）" % (sid, j["name"], mode), "info")
        self.app.log("%03d %s 参数已保存：速度 %g°/s · 模式 %d"
                     % (sid, j["name"], sp, mode), "info")

    # ---------------- 舵机侧指令 ----------------
    def read_status(self):
        sid = self._sel_sid()
        self.send(self.app._cmd_mod_get(sid), "(读模式)")
        self.send(self.app._cmd_read(sid), "(读位置)")

    def on_read_ver(self):
        self.send(self.app._cmd_ver(self._sel_sid()), "(读版本)")

    def apply_mode(self):
        sid = self._sel_sid()
        try:
            mode = int(self.vars[sid]["mode"].get())
        except ValueError:
            mode = 1
        self.send(self.app._cmd_mode(sid, mode), "(设模式 %d)" % mode)
        self.app.joints[sid]["mode"] = mode
        save_config(self.app.cfg)
        self.app.log("%s 模式已下发并保存为 %d" % (self.app.joints[sid]["name"], mode), "info")

    def apply_sck(self):
        sid = self._sel_sid()
        if not messagebox.askyesno("确认", "把 %s 当前位置设为 1500 中位（纠偏）？"
                                   % self.app.joints[sid]["name"], parent=self.win):
            return
        self.send(frame_sck(sid), "(设为 1500 中位)")
        self.app.joints[sid]["center"] = 1500
        save_config(self.app.cfg)

    def zero_at_center(self):
        """把「中位」定义成 0°：角度范围变成 -span/2 … +span/2（270° 舵机即 -135…135）"""
        sid = self._sel_sid()
        j = self.app.joints[sid]
        try:
            span = float(self.vars[sid]["amax"].get()) - float(self.vars[sid]["amin"].get())
        except ValueError:
            span = 270.0
        span = abs(span) or 270.0
        j["amin"], j["amax"] = -span / 2.0, span / 2.0
        j["home"] = 0.0
        j["center"] = 1500
        save_config(self.app.cfg)
        for key, val in (("amin", j["amin"]), ("amax", j["amax"]), ("home", 0.0)):
            try:
                self.vars[sid][key].set("%g" % val)
            except Exception:
                pass
        self.app.log("%s 中位已设为 0°，角度范围 %g…%g°"
                     % (j["name"], j["amin"], j["amax"]), "info")

    def mark_limit(self, which):
        sid = self._sel_sid()
        j = self.app.joints[sid]
        cur = self.app.state[sid]["pwm"]
        if cur is None:
            messagebox.showwarning("提示", "还没读到该轴位置，先连接", parent=self.win)
            return
        j["limit_min" if which == "min" else "limit_max"] = int(cur)
        self.send(frame_smi(sid) if which == "min" else frame_smx(sid),
                  "(%s限位 = 当前 PWM %d)" % ("最小" if which == "min" else "最大", cur))
        save_config(self.app.cfg)
        self.app.log("%s %s限位已设为 PWM %d（已保存）"
                     % (j["name"], "最小" if which == "min" else "最大", cur), "info")

    def apply_pid(self):
        try:
            old = int(self.oldid_var.get())
            nid = int(self.newid_var.get())
        except ValueError:
            messagebox.showerror("错误", "原 ID / 新 ID 都要填 0–254 的整数", parent=self.win)
            return
        if old == nid:
            messagebox.showerror("错误", "原 ID 和新 ID 不能相同", parent=self.win)
            return
        if not messagebox.askyesno(
                "确认", "把 ID %03d 改成 %03d？\n\n注意：总线上同 ID 的舵机会一起被改，\n"
                "建议一次只接一个舵机。" % (old, nid), parent=self.win):
            return
        self.send(self.app._cmd_id(old, nid))
        self.app.log("修改 ID：%03d → %03d（改完用「ID 扫描」确认）" % (old, nid), "info")

    def on_check_id(self):
        try:
            old = int(self.oldid_var.get())
        except ValueError:
            old = self._sel_sid()
        self.send("#%03dPID!" % old, "(检测 ID %03d 是否在线)" % old)

    def on_cleo(self):
        sid = self._sel_sid()
        if messagebox.askyesno("确认", "半恢复出厂：ID 保留，模式/波特率/中位/限位全部还原。\n继续吗？",
                               parent=self.win):
            self.send(frame_cleo(sid))

    def on_cle(self):
        sid = self._sel_sid()
        if messagebox.askyesno("确认", "全恢复出厂：ID 会变成 000，需要重新改 ID。\n继续吗？",
                               parent=self.win):
            self.send(frame_cle(sid))


class IdWizard:
    """逐个把舵机改成 001–006：每次只给一个舵机上电，点一下对应的按钮"""

    def __init__(self, app, parent=None):
        self.app = app
        self.win = tk.Toplevel(app.root)
        self.win.title("舵机 ID 设置向导")
        self.win.transient(parent or app.root)
        self.win.resizable(False, False)

        f = ttk.Frame(self.win, padding=(18, 14))
        f.pack(fill="both", expand=True)

        ttk.Label(f, text="步骤：一次只给一个舵机上电（其余断电）→ 点下面它对应的按钮 →\n"
                          "听到/看到改好后断电，换下一个舵机，重复到 006。",
                  justify="left", foreground="#666666").grid(
            row=0, column=0, columnspan=2, sticky="w", pady=(0, 12))

        top = ttk.Frame(f)
        top.grid(row=1, column=0, columnspan=2, sticky="w")
        ttk.Label(top, text="舵机现在的 ID").pack(side="left")
        self.old_var = tk.StringVar(value="000")
        ttk.Entry(top, textvariable=self.old_var, width=6,
                  justify="center").pack(side="left", padx=8)
        ttk.Label(top, text="（出厂都是 000）", foreground="#8A8A8A").pack(side="left")
        ttk.Button(top, text="扫描总线", width=9,
                   command=self.app.scan_ids).pack(side="left", padx=12)

        g = ttk.LabelFrame(f, text=" 目标 ID ", padding=(12, 10))
        g.grid(row=2, column=0, columnspan=2, sticky="ew", pady=(14, 0))

        self.status_vars = {}
        for i, sid in enumerate(SERVO_IDS):
            r, c = divmod(i, 2)
            ttk.Button(g, text="改为 %03d" % sid, width=10,
                       command=lambda s=sid: self.change(s)).grid(
                row=r, column=c * 2, padx=(0, 6), pady=4, sticky="w")
            v = tk.StringVar(value="待设置")
            self.status_vars[sid] = v
            ttk.Label(g, textvariable=v, width=8,
                      foreground="#8A8A8A").grid(row=r, column=c * 2 + 1, padx=(0, 18), sticky="w")

        ttk.Label(f, text="改完一个后，把这个舵机断电，再给下一个上电——\n"
                          "如果两个都在总线上且 ID 相同，会被一起改掉。\n"
                          "点了没反应？先查波特率：新舵机出厂是 115200，改过的可能是 57600。",
                  justify="left", foreground="#8A8A8A").grid(
            row=3, column=0, columnspan=2, sticky="w", pady=(12, 0))

        self.win.update_idletasks()
        w = self.win.winfo_reqwidth() + 12
        h = self.win.winfo_reqheight() + 16
        sw, sh = self.win.winfo_screenwidth(), self.win.winfo_screenheight()
        self.win.geometry("%dx%d+%d+%d" % (w, h, (sw - w) // 2, max(40, (sh - h) // 3)))

    def verify(self):
        try:
            self.app.scan_ids()
        except Exception:
            pass

    def change(self, target):
        try:
            old = int(self.old_var.get())
        except ValueError:
            messagebox.showerror("错误", "舵机现在的 ID 要填数字", parent=self.win)
            return
        if old == target:
            messagebox.showinfo("提示", "原 ID 和目标 ID 相同，不用改", parent=self.win)
            return
        if not messagebox.askyesno(
                "确认", "把当前这个舵机的 ID 从 %03d 改成 %03d？\n\n"
                "请确认：总线上现在只接了这一个舵机。" % (old, target), parent=self.win):
            return
        self.app._tx_frame(self.app._cmd_id(old, target),
                           "(ID 向导：%03d → %03d)" % (old, target))
        self.status_vars[target].set("已发送")
        self.app.log("ID 向导：%03d → %03d 已下发；断电换下一个舵机（新舵机仍是 %03d）"
                     % (old, target, old), "info")
        self.win.after(800, self.verify)


# ============================================================
def main():
    root = tk.Tk()
    app = ArmApp(root)
    if "--shot" in sys.argv:
        idx = sys.argv.index("--shot")
        path = sys.argv[idx + 1] if len(sys.argv) > idx + 1 else "ui_shot.png"

        def _grab():
            try:
                root.update_idletasks()
                root.update()
                from PIL import ImageGrab
                full = ImageGrab.grab(all_screens=True)
                k = full.width / max(1, root.winfo_screenwidth())   # DPI 缩放系数
                x, y = root.winfo_rootx() * k, root.winfo_rooty() * k
                w, h = root.winfo_width() * k, root.winfo_height() * k
                full.crop((int(x - 10), int(y - 36), int(x + w + 10), int(y + h + 10))).save(path)
            except Exception as ex:
                print("shot failed:", ex)
            root.destroy()

        root.after(2200, _grab)
    if "--selftest" in sys.argv:
        root.after(1500, root.destroy)
    root.mainloop()


if __name__ == "__main__":
    main()
