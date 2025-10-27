# robot_monitor.py — PyQt AC-HUD 完成版（薄型HPバー＋数値オーバーレイ）
# 要件: 映像（黒背景可）にHPバー・弾数・接続状態・試合時間・最後の被弾センサ等を重畳表示
# UDPでJSON状態パケットを受信（非同期スレッド）／ログをCSV出力／Qtプラグインパス自己解決

import os
import sys
import json
import csv
import socket
import threading
import time
from collections import deque
from datetime import datetime

import numpy as np
import cv2

# --- Ensure Qt platform plugin path (Qt5/Qt both) ---
def _setup_qt_plugin_path():
    cands = []
    sp = os.path.join(os.path.dirname(sys.executable), "Lib", "site-packages", "PyQt5")
    for d in ("Qt5", "Qt"):
        p = os.path.join(sp, d)
        if os.path.isdir(p):
            cands.append(p)
    try:
        import PyQt5  # type: ignore
        pkgdir = os.path.dirname(PyQt5.__file__)
        for d in ("Qt5", "Qt"):
            p = os.path.join(pkgdir, d)
            if os.path.isdir(p):
                cands.append(p)
    except Exception:
        pass

    for base in cands:
        platforms = os.path.join(base, "plugins", "platforms")
        bin_dir = os.path.join(base, "bin")
        if os.path.exists(os.path.join(platforms, "qwindows.dll")):
            os.environ.setdefault("QT_QPA_PLATFORM_PLUGIN_PATH", platforms)
            os.environ["PATH"] = bin_dir + os.pathsep + os.environ.get("PATH", "")
            break

_setup_qt_plugin_path()

from PyQt5 import QtCore, QtGui, QtWidgets

# --- Font loader (AC用) ---
def load_font_or(system_fallback: str, *paths: str) -> str:
    for p in paths:
        if os.path.exists(p):
            fid = QtGui.QFontDatabase.addApplicationFont(p)
            fams = QtGui.QFontDatabase.applicationFontFamilies(fid)
            if fams:
                return fams[0]
    return system_fallback

BASE_DIR = os.path.dirname(__file__)
FONT_HEAD = load_font_or(
    "Arial",
    os.path.join(BASE_DIR, "assets", "fonts", "Orbitron-VariableFont_wght.ttf"),
    os.path.join(BASE_DIR, "assets", "fonts", "Orbitron-Regular.ttf"),
)
FONT_NUM = load_font_or(
    "Consolas",
    os.path.join(BASE_DIR, "assets", "fonts", "RobotoMono-VariableFont_wght.ttf"),
)

_pp = os.environ.get("QT_QPA_PLATFORM_PLUGIN_PATH", "")
if _pp:
    QtCore.QCoreApplication.addLibraryPath(os.path.dirname(_pp))
    QtCore.QCoreApplication.addLibraryPath(_pp)

# ------------- 設定 -------------
UDP_IP = "0.0.0.0"
UDP_PORT = 5005
LOG_CSV = True
LOG_FILENAME = f"match_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
RTSP_URL = None            # None=黒背景, 0=Webカメラ, "rtsp://..."=RTSP
FRAME_W, FRAME_H = 1280, 720
FPS = 30
# ---------------------------------

# デフォルトステート
state_lock = threading.Lock()
state = {
    "hp": 100,
    "max_hp": 100,
    "ammo": 5,
    "max_ammo": 5,
    "sensors": [0] * 8,
    "last_hit": None,
    "ping_ms": None,
    "connected": False,
    "timestamp": None,
}

# ログ用キュー
log_queue = deque(maxlen=1000)

# UDP受信スレッド

def udp_listener(ip, port):
    global state
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((ip, port))
    sock.settimeout(1.0)
    print(f"[UDP] listening on {ip}:{port}")
    while True:
        try:
            data, _ = sock.recvfrom(4096)
            t_recv = time.time()
            try:
                obj = json.loads(data.decode("utf-8"))
            except Exception as e:
                print("[UDP] json decode error:", e)
                continue
            with state_lock:
                for k in ("hp", "max_hp", "ammo", "max_ammo", "sensors", "last_hit"):
                    if k in obj:
                        state[k] = obj[k]
                state["connected"] = True
                state["timestamp"] = t_recv
                if "sent_ts" in obj:
                    state["ping_ms"] = int((t_recv - float(obj["sent_ts"])) * 1000)
                else:
                    state["ping_ms"] = None
                log_queue.append({
                    "t": datetime.fromtimestamp(t_recv).isoformat(timespec="seconds"),
                    "hp": state.get("hp"),
                    "ammo": state.get("ammo"),
                    "last_hit": state.get("last_hit"),
                })
        except socket.timeout:
            with state_lock:
                if state["timestamp"] is None or time.time() - state["timestamp"] > 1.5:
                    state["connected"] = False
        except Exception as e:
            print("[UDP] listener error:", e)
            time.sleep(0.5)

# ログスレッド

def logger_worker(filename: str):
    if not LOG_CSV:
        return
    with open(filename, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=["t", "hp", "ammo", "last_hit"])
        writer.writeheader()
        while True:
            while log_queue:
                writer.writerow(log_queue.popleft())
                f.flush()
            time.sleep(0.2)

# --- テーマ & 描画ユーティリティ ---

class HUDTheme:
    def __init__(self, name="ac_cyan"):
        if name == "ac_cyan":
            self.bg = QtGui.QColor(0, 0, 0, 255)
            self.panel = QtGui.QColor(12, 14, 18, 210)
            self.stroke = QtGui.QColor(190, 210, 230, 220)
            self.hp_ok = QtGui.QColor(80, 220, 140)
            self.hp_low = QtGui.QColor(255, 88, 72)
            self.ammo = QtGui.QColor(240, 240, 240)
            self.warn = QtGui.QColor(255, 120, 72)
            self.accent = QtGui.QColor(140, 190, 255)
            self.glow = QtGui.QColor(120, 170, 255, 100)
            self.shadow = QtGui.QColor(0, 0, 0, 160)
            self.crosshair = QtGui.QColor(222, 232, 245, 220)
        else:
            self.bg = QtGui.QColor(0, 0, 0, 255)
            self.panel = QtGui.QColor(20, 20, 24, 200)
            self.stroke = QtGui.QColor(220, 220, 230, 220)
            self.hp_ok = QtGui.QColor(60, 220, 120)
            self.hp_low = QtGui.QColor(255, 70, 70)
            self.ammo = QtGui.QColor(240, 240, 240)
            self.warn = QtGui.QColor(255, 100, 40)
            self.accent = QtGui.QColor(120, 170, 255)
            self.glow = QtGui.QColor(120, 170, 255, 120)
            self.shadow = QtGui.QColor(0, 0, 0, 160)
            self.crosshair = QtGui.QColor(220, 220, 220, 200)

        # フォント/ペン
        self.font_title = QtGui.QFont(FONT_HEAD, 18)
        self.font_body  = QtGui.QFont(FONT_HEAD, 14)
        self.font_num   = QtGui.QFont(FONT_NUM, 16)
        self.font_small = QtGui.QFont(FONT_HEAD, 12)

        self.pen_stroke = QtGui.QPen(self.stroke)
        self.pen_warn   = QtGui.QPen(self.warn,   6, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap)
        self.pen_accent = QtGui.QPen(self.accent, 3, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap)


def ease_out_cubic(t: float) -> float:
    t = max(0.0, min(1.0, t))
    return 1 - pow(1 - t, 3)


class FPSMeter:
    def __init__(self, n=60):
        self.times = deque(maxlen=n)
        self.last = time.time()

    def tick(self):
        now = time.time()
        dt = now - self.last
        self.last = now
        self.times.append(dt)

    def fps(self) -> float:
        if not self.times:
            return 0.0
        avg = sum(self.times) / len(self.times)
        return 1.0 / avg if avg > 0 else 0.0


class DamageFlash:
    """被弾時に画面端を赤くパルスする"""

    def __init__(self, decay=0.7):
        self.t0 = None
        self.duration = 0.6
        self.decay = decay

    def trigger(self):
        self.t0 = time.time()

    def alpha(self) -> float:
        if self.t0 is None:
            return 0.0
        t = (time.time() - self.t0) / self.duration
        if t >= 1.0:
            self.t0 = None
            return 0.0
        return ease_out_cubic(1.0 - t) ** self.decay


def draw_crosshair(p: QtGui.QPainter, cx, cy, size, color: QtGui.QColor, thick=2):
    p.setRenderHint(QtGui.QPainter.Antialiasing, True)
    p.setPen(QtGui.QPen(color, thick, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap))
    gap = int(size * 0.35)
    for dx, dy in [(1, 0), (-1, 0), (0, 1), (0, -1)]:
        x1 = cx + dx * gap
        y1 = cy + dy * gap
        x2 = cx + dx * size
        y2 = cy + dy * size
        p.drawLine(x1, y1, x2, y2)


def draw_corner_brackets(p: QtGui.QPainter, x, y, w, h, color: QtGui.QColor, len_px=14, thick=2):
    pen = QtGui.QPen(color, thick, QtCore.Qt.SolidLine, QtCore.Qt.SquareCap)
    p.setPen(pen)
    p.setBrush(QtCore.Qt.NoBrush)
    # 左上
    p.drawLine(x, y, x + len_px, y)
    p.drawLine(x, y, x, y + len_px)
    # 右上
    p.drawLine(x + w - len_px, y, x + w, y)
    p.drawLine(x + w, y, x + w, y + len_px)
    # 左下
    p.drawLine(x, y + h - len_px, x, y + h)
    p.drawLine(x, y + h, x + len_px, y + h)
    # 右下
    p.drawLine(x + w - len_px, y + h, x + w, y + h)
    p.drawLine(x + w, y + h - len_px, x + w, y + h)


def draw_dotted_divider(p: QtGui.QPainter, x1, y, x2, color: QtGui.QColor, dash=4, gap=4, thick=1):
    pen = QtGui.QPen(color, thick)
    pen.setStyle(QtCore.Qt.CustomDashLine)
    pen.setDashPattern([dash, gap])
    p.setPen(pen)
    p.drawLine(x1, y, x2, y)


def draw_sensor_ring(p: QtGui.QPainter, cx, cy, r, hit_idx, color_base: QtGui.QColor, color_hit: QtGui.QColor):
    p.setRenderHint(QtGui.QPainter.Antialiasing, True)
    segs = 8
    base_pen = QtGui.QPen(color_base, 3, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap)
    p.setPen(base_pen)
    RAD2QT = 180.0 * 16.0 / np.pi
    for i in range(segs):
        a0 = (2 * np.pi) * (i / segs)
        a1 = (2 * np.pi) * ((i + 0.7) / segs)
        start16 = int(-a0 * RAD2QT)
        span16 = int(-(a1 - a0) * RAD2QT)
        p.setPen(base_pen)
        p.drawArc(cx - r, cy - r, 2 * r, 2 * r, start16, span16)
        if hit_idx is not None and i == hit_idx % segs:
            glow = QtGui.QPen(color_hit, 6, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap)
            p.setPen(glow)
            p.drawArc(cx - r, cy - r, 2 * r, 2 * r, start16, span16)


def draw_ammo_icons(p: QtGui.QPainter, x, y, count, max_count, color_fill: QtGui.QColor, color_line: QtGui.QColor, scale=1.0):
    size = int(18 * scale)
    gap = int(8 * scale)
    for i in range(max_count):
        rect = QtCore.QRect(x + i * (size + gap), y, size, size)
        if i < count:
            p.setBrush(QtGui.QBrush(color_fill))
            p.setPen(QtGui.QPen(QtGui.QColor(0, 0, 0, 160), 1))
            p.drawEllipse(rect)
        else:
            p.setBrush(QtCore.Qt.NoBrush)
            p.setPen(QtGui.QPen(color_line, 2))
            p.drawEllipse(rect)


def draw_glass_panel(p: QtGui.QPainter, x, y, w, h, stroke: QtGui.QColor, r=12):
    p.setPen(QtCore.Qt.NoPen)
    grad = QtGui.QLinearGradient(x, y, x, y + h)
    grad.setColorAt(0.0, QtGui.QColor(255, 255, 255, 22))
    grad.setColorAt(1.0, QtGui.QColor(255, 255, 255, 6))
    p.setBrush(QtGui.QBrush(grad))
    p.drawRoundedRect(x, y, w, h, r, r)
    p.setPen(QtGui.QPen(stroke, 2))
    p.setBrush(QtCore.Qt.NoBrush)
    p.drawRoundedRect(x, y, w, h, r, r)


def draw_scanlines(p: QtGui.QPainter, w, h, alpha=12):
    p.setPen(QtCore.Qt.NoPen)
    p.setBrush(QtGui.QBrush(QtGui.QColor(0, 0, 0, alpha)))
    for y in range(0, h, 2):
        p.drawRect(0, y, w, 1)


# GUIアプリ
class MonitorWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ロボットバトル モニター")
        self.setMinimumSize(640, 360)
        self.resize(FRAME_W // 2, FRAME_H // 2)

        self.video_label = QtWidgets.QLabel(alignment=QtCore.Qt.AlignCenter)
        self.info_label = QtWidgets.QLabel(alignment=QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
        self.info_label.setFixedHeight(80)
        self.info_label.setFont(QtGui.QFont("Arial", 11))

        layout = QtWidgets.QVBoxLayout(self)
        layout.addWidget(self.video_label, 1)
        layout.addWidget(self.info_label)

        self.timer = QtCore.QTimer(self, interval=int(1000 / FPS))
        self.timer.timeout.connect(self.update_frame)
        self.timer.start()

        self.theme = HUDTheme("ac_cyan")
        self.fpsm = FPSMeter(90)
        self.flash = DamageFlash(decay=0.9)
        self._last_hit_seen = None

        if RTSP_URL is not None:
            self.cap = cv2.VideoCapture(RTSP_URL)
        else:
            self.cap = None

        self.match_total_sec = 3 * 60
        self.match_start_ts = time.time()
        self.match_running = True

    def closeEvent(self, event: QtGui.QCloseEvent):
        if self.cap:
            self.cap.release()
        event.accept()
        QtCore.QCoreApplication.quit()

    def keyPressEvent(self, e: QtGui.QKeyEvent):
        if e.key() == QtCore.Qt.Key_F11:
            self.setWindowState(self.windowState() ^ QtCore.Qt.WindowFullScreen)
        elif e.key() == QtCore.Qt.Key_Escape:
            self.close()

    def make_black_frame(self):
        return np.zeros((FRAME_H, FRAME_W, 3), dtype=np.uint8)

    def update_frame(self):
        # フレーム取得
        if self.cap:
            ret, frame = self.cap.read()
            if not ret:
                frame = self.make_black_frame()
            else:
                frame = cv2.resize(frame, (FRAME_W, FRAME_H), interpolation=cv2.INTER_LINEAR)
        else:
            frame = self.make_black_frame()

        with state_lock:
            st = dict(state)

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape

        scale = 1  # パフォーマンス重視
        big_w, big_h = w * scale, h * scale

        big_img = QtGui.QImage(big_w, big_h, QtGui.QImage.Format_ARGB32_Premultiplied)
        big_img.fill(QtGui.QColor(0, 0, 0, 255))
        painter = QtGui.QPainter(big_img)
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
        painter.setRenderHint(QtGui.QPainter.TextAntialiasing, True)
        painter.setRenderHint(QtGui.QPainter.SmoothPixmapTransform, True)

        src_qimg = QtGui.QImage(rgb.data, w, h, ch * w, QtGui.QImage.Format_RGB888).copy()
        src_qimg_big = src_qimg.scaled(big_w, big_h, QtCore.Qt.IgnoreAspectRatio, QtCore.Qt.SmoothTransformation)
        painter.drawImage(0, 0, src_qimg_big)

        theme = self.theme
        font_name = FONT_HEAD

        # === HP: 細ピル + 数字オーバーレイ =========================
        max_hp = st.get("max_hp", 100)
        hp = max(0, min(int(st.get("hp", 0)), max_hp))

        bar_w = int(w * 0.40) * scale
        bar_h = int(14 * scale)                  # スリム
        bar_x = 30 * scale
        bar_y = 36 * scale

        # 背面パネル（薄ガラス／枠付き）
        painter.setOpacity(0.95)
        draw_glass_panel(painter, bar_x - 8, bar_y - int(18 * scale), bar_w + 16, int(56 * scale), theme.stroke)
        painter.setOpacity(1.0)

        # 細いHPピル
        hp_ratio = (hp / float(max_hp)) if max_hp > 0 else 0.0
        painter.setPen(QtGui.QPen(QtGui.QColor(200, 200, 200, 220), 1 * scale))
        painter.setBrush(QtGui.QBrush(QtGui.QColor(28, 28, 28, 220)))
        painter.drawRoundedRect(bar_x, bar_y, bar_w, bar_h, 7 * scale, 7 * scale)

        grad = QtGui.QLinearGradient(bar_x, bar_y, bar_x + bar_w, bar_y)
        grad.setColorAt(0.0, QtGui.QColor(int(255 * (1 - hp_ratio)), 70, 70))
        grad.setColorAt(1.0, QtGui.QColor(70, int(255 * hp_ratio), 90))
        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(QtGui.QBrush(grad))
        painter.drawRoundedRect(bar_x, bar_y, int(bar_w * hp_ratio), bar_h, 7 * scale, 7 * scale)

        # 数字オーバーレイ（等幅フォント／中央寄せ）
        num_font = QtGui.QFont(FONT_NUM, int(18 * scale))
        num_font.setStyleStrategy(QtGui.QFont.PreferAntialias)
        painter.setFont(num_font)
        text_rect = QtCore.QRect(bar_x, bar_y - int(3 * scale), bar_w, bar_h + int(6 * scale))
        hp_text = f"HP: {hp}/{max_hp}"
        painter.setPen(QtGui.QPen(QtGui.QColor(0, 0, 0, 160), 2))
        painter.drawText(text_rect, QtCore.Qt.AlignCenter | QtCore.Qt.AlignVCenter, hp_text)
        painter.setPen(QtGui.QPen(QtGui.QColor(245, 245, 245), 1))
        painter.drawText(text_rect, QtCore.Qt.AlignCenter | QtCore.Qt.AlignVCenter, hp_text)

        # === Ammo 行：重なり防止で少し下へ ===
        ammo_text = f"Ammo: {st.get('ammo', 0)}/{st.get('max_ammo', 5)}"
        ammo_font = QtGui.QFont(FONT_HEAD, int(14 * scale))
        painter.setFont(ammo_font)
        ammo_x = bar_x
        ammo_y = bar_y + bar_h + int(22 * scale)
        painter.setPen(QtGui.QPen(QtGui.QColor(240, 240, 240), 1))
        painter.drawText(ammo_x + int(140 * scale), ammo_y, ammo_text)
        draw_ammo_icons(painter, ammo_x, ammo_y - int(14 * scale), int(st.get("ammo", 0)), int(st.get("max_ammo", 5)), theme.ammo, theme.ammo, scale=1.0 * scale)

        # --- 試合時間 / 接続パネル ---
        elapsed = int(time.time() - self.match_start_ts) if self.match_running else 0
        remaining = max(0, self.match_total_sec - elapsed)
        mm, ss = divmod(remaining, 60)
        time_text = f"{mm:02d}:{ss:02d} / {self.match_total_sec // 60:02d}:{self.match_total_sec % 60:02d}"
        conn_text = "Connected" if st.get("connected") else "Disconnected"
        ping_val = st.get("ping_ms")
        ping_text = f"PING {ping_val}ms" if ping_val is not None else "PING --ms"

        right_w = int(440 * scale)
        right_h = int(86 * scale)
        right_x = big_w - right_w - 30 * scale
        right_y = 22 * scale
        panel = QtCore.QRect(right_x, right_y, right_w, right_h)
        painter.setOpacity(0.92)
        painter.setPen(QtGui.QPen(theme.stroke, 1))
        painter.setBrush(QtGui.QColor(10, 12, 18, 200))
        painter.drawRoundedRect(panel, 8 * scale, 8 * scale)
        draw_corner_brackets(painter, panel.x(), panel.y(), panel.width(), panel.height(), theme.stroke, len_px=14 * scale, thick=2)
        painter.setOpacity(1.0)
        painter.setPen(theme.stroke)
        painter.setFont(theme.font_small)
        painter.drawText(panel.x() + 16 * scale, panel.y() + 26 * scale, "LINK:")
        painter.drawText(panel.x() + 16 * scale, panel.y() + 50 * scale, "TIME:")
        painter.setFont(theme.font_num)
        painter.drawText(panel.x() + 72 * scale, panel.y() + 26 * scale, f"{conn_text}   {ping_text}")
        painter.drawText(panel.x() + 72 * scale, panel.y() + 50 * scale, time_text)
        draw_dotted_divider(painter, panel.x() + 12 * scale, panel.y() + panel.height() - 12 * scale, panel.x() + panel.width() - 12 * scale, theme.stroke, dash=4, gap=4, thick=1)

        # --- 中央: 照準 + センサーリング + ティックリング ---
        cx, cy = (w // 2) * scale, (h // 2) * scale
        draw_crosshair(painter, cx, cy, int(22 * scale), theme.crosshair, thick=2 * scale)
        pen_tick = QtGui.QPen(theme.accent, 1 * scale, QtCore.Qt.SolidLine, QtCore.Qt.SquareCap)
        painter.setPen(pen_tick)
        r_tick = int(min(w, h) * 0.25) * scale
        for i in range(24):
            a = 2 * np.pi * i / 24
            x1 = cx + int((r_tick - 8 * scale) * np.cos(a))
            y1 = cy + int((r_tick - 8 * scale) * np.sin(a))
            x2 = cx + int(r_tick * np.cos(a))
            y2 = cy + int(r_tick * np.sin(a))
            painter.drawLine(x1, y1, x2, y2)
        draw_sensor_ring(painter, cx, cy, int(min(w, h) * 0.23) * scale, st.get("last_hit"), theme.accent, theme.warn)

        # --- 被弾パルス ---
        if st.get("last_hit") is not None and st.get("last_hit") != self._last_hit_seen:
            self._last_hit_seen = st.get("last_hit")
            self.flash.trigger()
        a = self.flash.alpha()
        if a > 0.0:
            painter.setOpacity(0.35 * a)
            painter.setBrush(QtGui.QBrush(theme.warn))
            painter.setPen(QtCore.Qt.NoPen)
            margin = int(8 * scale)
            painter.drawRect(0, 0, big_w, margin)
            painter.drawRect(0, big_h - margin, big_w, margin)
            painter.drawRect(0, 0, margin, big_h)
            painter.drawRect(big_w - margin, 0, margin, big_h)
            painter.setOpacity(1.0)

        # --- スキャンライン & FPS ---
        draw_scanlines(painter, big_w, big_h, alpha=12)
        self.fpsm.tick()
        painter.setPen(theme.pen_stroke)
        painter.setFont(QtGui.QFont(FONT_NUM, int(14 * scale)))
        painter.drawText(24 * scale, big_h - 18 * scale, f"{self.fpsm.fps():.0f} FPS")

        painter.end()

        # 表示
        pix = QtGui.QPixmap.fromImage(big_img)
        try:
            dpr = self.devicePixelRatioF()
        except Exception:
            dpr = 1.0
        pix.setDevicePixelRatio(dpr)
        final_pix = pix.scaled(int(self.video_label.width() * dpr), int(self.video_label.height() * dpr), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)
        self.video_label.setPixmap(final_pix)

        # 下のインフォ行
        conn_short = "OK" if st.get("connected") else "NO"
        ping_str = f"{st.get('ping_ms')}ms" if st.get("ping_ms") is not None else "--"
        self.info_label.setText(
            f"<b>HP</b>: {st.get('hp')}/{st.get('max_hp')} &nbsp;&nbsp;"
            f"<b>Ammo</b>: {st.get('ammo')}/{st.get('max_ammo')} &nbsp;&nbsp;"
            f"<b>Conn</b>: {conn_short} ({ping_str}) &nbsp;&nbsp;"
            f"<b>LastHit</b>: {st.get('last_hit')}"
        )


def main():
    # DPI属性はQApplication作成前に
    QtCore.QCoreApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
    QtCore.QCoreApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps, True)

    # スレッド起動
    threading.Thread(target=udp_listener, args=(UDP_IP, UDP_PORT), daemon=True).start()
    if LOG_CSV:
        threading.Thread(target=logger_worker, args=(LOG_FILENAME,), daemon=True).start()
        print(f"[LOG] writing to {LOG_FILENAME}")

    app = QtWidgets.QApplication(sys.argv)
    win = MonitorWindow()
    win.show()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
