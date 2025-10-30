# robot_monitor.py — PyQt AC-HUD 完成版（ULTRA HUD: 極細HP＋右上テイスト揃え）
# 依存: PyQt5, numpy, opencv-python-headless
# 起動: python robot_monitor.py（または UI_proto.py として使う場合は同名に保存）

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

# --- Qt plugin path auto-fix (before importing PyQt5) -----------------
import os, sys, glob, importlib.util

def _fix_qt_plugin_path():
    # 既存の邪魔な環境変数は無効化
    os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)
    os.environ.pop("QT_PLUGIN_PATH", None)

    # PyQt5 のパッケージパスを import なしで取得
    spec = importlib.util.find_spec("PyQt5")
    pkg_dir = None
    if spec and spec.submodule_search_locations:
        pkg_dir = spec.submodule_search_locations[0]
    elif spec and spec.origin:
        pkg_dir = os.path.dirname(spec.origin)
    if not pkg_dir:
        return  # PyQt5 未インストール時などは何もしない

    # qwindows.dll を総当たりで探索
    cands = glob.glob(os.path.join(pkg_dir, "**", "platforms", "qwindows.dll"), recursive=True)
    if not cands:
        return

    plat_dir = os.path.dirname(cands[0])               # .../plugins/platforms
    qt_root  = os.path.dirname(os.path.dirname(plat_dir))  # .../Qt5 or .../Qt

    # 起動に必要な2点をProcess環境に設定
    os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = plat_dir

    bin_dir = os.path.join(qt_root, "bin")
    if os.path.isdir(bin_dir):
        os.environ["PATH"] = bin_dir + os.pathsep + os.environ.get("PATH", "")

_fix_qt_plugin_path()
# ----------------------------------------------------------------------


# --- Qt plugin path (触らなくてもOK。wheel同梱Qtが拾える想定) ---
from PyQt5 import QtCore, QtGui, QtWidgets

# --- Safe font loader (Qt5×可変フォント落ち対策) ---
def safe_add_font(path: str):
    try:
        if not os.path.exists(path):
            return None
        # Qt5 は VariableFont で落ちやすいので一旦スキップ
        base = os.path.basename(path).lower()
        if "variablefont" in base:
            return None
        if os.path.getsize(path) < 1024:
            return None
        fid = QtGui.QFontDatabase.addApplicationFont(path)
        if fid == -1:
            return None
        fams = QtGui.QFontDatabase.applicationFontFamilies(fid)
        return fams[0] if fams else None
    except Exception:
        return None

BASE_DIR = os.path.dirname(__file__)

# ← ここを「パス配列 + フォールバック名」に変える
FONT_HEAD = "Arial"      # 家族名は後で差し替え
FONT_NUM  = "Consolas"

HEAD_FONT_CANDIDATES = [
    os.path.join(BASE_DIR, "assets", "fonts", "Orbitron-Regular.ttf"),
    os.path.join(BASE_DIR, "assets", "fonts", "Orbitron-Bold.ttf"),
]
# どこかに置いた init_fonts() の直上あたり
NUM_FONT_CANDIDATES = [
    os.path.join(BASE_DIR, "assets", "fonts", "Orbitron-Regular.ttf"),
]


def init_fonts():
    """QApplication生成『後』に呼ぶ。成功したら FONT_HEAD / FONT_NUM を家族名に置き換える。"""
    global FONT_HEAD, FONT_NUM
    for p in HEAD_FONT_CANDIDATES:
        fam = safe_add_font(p)
        if fam:
            FONT_HEAD = fam
            break
    for p in NUM_FONT_CANDIDATES:
        fam = safe_add_font(p)
        if fam:
            FONT_NUM = fam
            break
    print("[font] head =", FONT_HEAD, "/ num =", FONT_NUM)  # 起動時ログ

# ---------------------- 設定 ----------------------
UDP_IP = "0.0.0.0";
UDP_PORT = 5005
LOG_CSV = True
LOG_FILENAME = f"match_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
RTSP_URL =  None #WebカメラのURL "http://10.133.5.170:8080/stream"   # None=黒背景, 0=Webカメラ, "rtsp://..."=RTSP
FRAME_W, FRAME_H = 1280, 720
FPS = 30
# -------------------------------------------------

# === UI表示フラグ（上パネルを一時的にオフ） ===
SHOW_TOP_LEFT  = False
SHOW_TOP_RIGHT = False

# 共有ステート
state_lock = threading.Lock()
state = {
    "hp": 1000, "max_hp": 1000,
    "ammo": 5, "max_ammo": 5,
    "sensors": [0]*8,
    "last_hit": None,
    "ping_ms": None,
    "connected": False,
    "timestamp": None,
}
log_queue = deque(maxlen=1000)

# ---------------- UDP 受信 ----------------

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
                print("[UDP] json error:", e)
                continue
            with state_lock:
                for k in ("hp","max_hp","ammo","max_ammo","sensors","last_hit"):
                    if k in obj: state[k] = obj[k]
                state["connected"] = True
                state["timestamp"] = t_recv
                state["ping_ms"] = int((t_recv - float(obj.get("sent_ts", t_recv)))*1000) if obj.get("sent_ts") else None
                log_queue.append({"t": datetime.fromtimestamp(t_recv).isoformat(timespec="seconds"),
                                  "hp": state.get("hp"),
                                  "ammo": state.get("ammo"),
                                  "last_hit": state.get("last_hit")})
        except socket.timeout:
            with state_lock:
                if state["timestamp"] is None or time.time()-state["timestamp"]>1.5:
                    state["connected"] = False
        except Exception as e:
            print("[UDP] error:", e)
            time.sleep(0.2)

# ---------------- ロガー ----------------

def logger_worker(filename: str):
    if not LOG_CSV: return
    with open(filename, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=["t","hp","ammo","last_hit"])
        w.writeheader()
        while True:
            while log_queue:
                w.writerow(log_queue.popleft()); f.flush()
            time.sleep(0.2)

# ---------------- テーマ/描画 ----------------
class HUDTheme:
    def __init__(self, name="ac_cyan"):
        self.bg      = QtGui.QColor(0,0,0,255)
        self.panel   = QtGui.QColor(12,14,18,210)
        self.stroke  = QtGui.QColor(190,210,230,220)
        self.hp_ok   = QtGui.QColor(80,220,140)
        self.hp_low  = QtGui.QColor(255,88,72)
        self.ammo    = QtGui.QColor(240,240,240)
        self.warn    = QtGui.QColor(255,120,72)
        self.accent  = QtGui.QColor(140,190,255)
        self.glow    = QtGui.QColor(120,170,255,100)
        self.shadow  = QtGui.QColor(0,0,0,160)
        self.crosshair = QtGui.QColor(222,232,245,220)
        # フォント
        self.font_title = QtGui.QFont(FONT_HEAD, 18); self.font_title.setWeight(600)
        self.font_body  = QtGui.QFont(FONT_HEAD, 14)
        self.font_num   = QtGui.QFont(FONT_NUM, 16)
        self.font_small = QtGui.QFont(FONT_HEAD, 12)
        self.font_body.setLetterSpacing(QtGui.QFont.AbsoluteSpacing, 0.5)
        self.font_small.setLetterSpacing(QtGui.QFont.AbsoluteSpacing, 0.5)

        # ペン
        self.pen_stroke = QtGui.QPen(self.stroke)
        self.pen_accent = QtGui.QPen(self.accent, 3, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap)


def draw_corner_brackets(p: QtGui.QPainter, x, y, w, h, color: QtGui.QColor, len_px=14, thick=2):
    pen = QtGui.QPen(color, thick, QtCore.Qt.SolidLine, QtCore.Qt.SquareCap)
    p.setPen(pen); p.setBrush(QtCore.Qt.NoBrush)
    p.drawLine(x, y, x+len_px, y); p.drawLine(x, y, x, y+len_px)
    p.drawLine(x+w-len_px, y, x+w, y); p.drawLine(x+w, y, x+w, y+len_px)
    p.drawLine(x, y+h-len_px, x, y+h); p.drawLine(x, y+h, x+len_px, y+h)
    p.drawLine(x+w-len_px, y+h, x+w, y+h); p.drawLine(x+w, y+h-len_px, x+w, y+h)

def draw_dotted_divider(p: QtGui.QPainter, x1, y, x2, color: QtGui.QColor, dash=4, gap=4, thick=1):
    pen = QtGui.QPen(color, thick); pen.setStyle(QtCore.Qt.CustomDashLine); pen.setDashPattern([dash,gap])
    p.setPen(pen); p.drawLine(x1, y, x2, y)

def draw_crosshair(p: QtGui.QPainter, cx, cy, size, color: QtGui.QColor, thick=2):
    p.setRenderHint(QtGui.QPainter.Antialiasing, True)
    p.setPen(QtGui.QPen(color, thick, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap))
    gap = int(size*0.35)
    for dx,dy in [(1,0),(-1,0),(0,1),(0,-1)]:
        x1 = cx + dx*gap; y1 = cy + dy*gap; x2 = cx + dx*size; y2 = cy + dy*size
        p.drawLine(x1,y1,x2,y2)

def draw_sensor_ring(p: QtGui.QPainter, cx, cy, r, hit_idx, color_base, color_hit):
    p.setRenderHint(QtGui.QPainter.Antialiasing, True)
    segs=8; base_pen = QtGui.QPen(color_base, 3, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap)
    p.setPen(base_pen); RAD2QT = 180.0*16.0/np.pi
    for i in range(segs):
        a0 = (2*np.pi)*(i/segs); a1 = (2*np.pi)*((i+0.7)/segs)
        start16 = int(-a0*RAD2QT); span16 = int(-(a1-a0)*RAD2QT)
        p.setPen(base_pen); p.drawArc(cx-r, cy-r, 2*r, 2*r, start16, span16)
        if hit_idx is not None and i == hit_idx % segs:
            glow = QtGui.QPen(color_hit, 6, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap)
            p.setPen(glow); p.drawArc(cx-r, cy-r, 2*r, 2*r, start16, span16)

def draw_ammo_dots(p: QtGui.QPainter, x, y, count, max_count, color_fill, color_line, scale=1.0):
    size = int(12*scale); gap = int(6*scale)
    for i in range(max_count):
        rect = QtCore.QRect(x + i*(size+gap), y, size, size)
        if i < count:
            p.setBrush(QtGui.QBrush(color_fill)); p.setPen(QtGui.QPen(QtGui.QColor(0,0,0,160), 1))
            p.drawEllipse(rect)
        else:
            p.setBrush(QtCore.Qt.NoBrush); p.setPen(QtGui.QPen(color_line, 1))
            p.drawEllipse(rect)

def draw_scanlines(p: QtGui.QPainter, w, h, alpha=12):
    p.setPen(QtCore.Qt.NoPen); p.setBrush(QtGui.QBrush(QtGui.QColor(0,0,0,alpha)))
    for Y in range(0, h, 2): p.drawRect(0, Y, w, 1)

# ---------------- メインウィンドウ ----------------
class MonitorWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ロボットバトル モニター")
        self.setMinimumSize(640, 360)
        self.resize(FRAME_W//2, FRAME_H//2)

        self.video_label = QtWidgets.QLabel(alignment=QtCore.Qt.AlignCenter)
        self.info_label  = QtWidgets.QLabel(alignment=QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
        self.info_label.setFixedHeight(80); self.info_label.setFont(QtGui.QFont("Arial", 11))
        # Bottom panels と重ならないよう非表示（四隅レイアウト時）
        self.info_label.setVisible(False)

        v = QtWidgets.QVBoxLayout(self); v.addWidget(self.video_label,1); v.addWidget(self.info_label)

        self.timer = QtCore.QTimer(self, interval=int(1000/FPS)); self.timer.timeout.connect(self.update_frame); self.timer.start()
        self.theme = HUDTheme(); self._last_hit_seen=None

        self.cap = cv2.VideoCapture(RTSP_URL) if RTSP_URL is not None else None
        self.match_total_sec = 3*60; self.match_start_ts = time.time(); self.match_running=True

    def closeEvent(self, e: QtGui.QCloseEvent):
        if self.cap: self.cap.release(); e.accept(); QtCore.QCoreApplication.quit()

    def keyPressEvent(self, e: QtGui.QKeyEvent):
        if e.key()==QtCore.Qt.Key_F11: self.setWindowState(self.windowState() ^ QtCore.Qt.WindowFullScreen)
        elif e.key()==QtCore.Qt.Key_Escape: self.close()

    def make_black_frame(self):
        return np.zeros((FRAME_H, FRAME_W, 3), dtype=np.uint8)

    def update_frame(self):
        # 背景取得
        if self.cap:
            ret, frame = self.cap.read(); frame = self.make_black_frame() if not ret else cv2.resize(frame,(FRAME_W,FRAME_H))
        else:
            frame = self.make_black_frame()
        with state_lock: st = dict(state)

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB); h,w,ch = rgb.shape
        scale = 1; big_w, big_h = w*scale, h*scale
        big_img = QtGui.QImage(big_w, big_h, QtGui.QImage.Format_ARGB32_Premultiplied); big_img.fill(QtGui.QColor(0,0,0,255))
        p = QtGui.QPainter(big_img); p.setRenderHint(QtGui.QPainter.Antialiasing, True); p.setRenderHint(QtGui.QPainter.TextAntialiasing, True)
        src_qimg = QtGui.QImage(rgb.data, w, h, ch*w, QtGui.QImage.Format_RGB888).copy(); src_qimg_big = src_qimg.scaled(big_w,big_h,QtCore.Qt.IgnoreAspectRatio,QtCore.Qt.SmoothTransformation); p.drawImage(0,0,src_qimg_big)

        t = self.theme

        # (disabled) 左上パネルは非表示にしました。
# if SHOW_TOP_LEFT:
#     # ← ここに左上パネル描画（無効化中）
#     pass

# (disabled) 右上パネルは非表示にしました。
# if SHOW_TOP_RIGHT:
#     # ← ここに右上パネル描画（無効化中）
#     pass

        # --- 四隅展開：左下・右下にも同デザインを配置 ---
        margin = int(26 * scale)
        left_size  = QtCore.QSize(int(w * 0.42) * scale, int(64 * scale))
        right_size = QtCore.QSize(int(440 * scale),     int(86 * scale))

        rect_bl_left  = QtCore.QRect(margin, big_h - margin - left_size.height(),
                                     left_size.width(), left_size.height())
        rect_br_right = QtCore.QRect(big_w - margin - right_size.width(),
                                     big_h - margin - right_size.height(),
                                     right_size.width(), right_size.height())

        render_left_status(p, rect_bl_left,  st, t, scale)
        render_right_info(p, rect_br_right, st, t, scale, self.match_total_sec, self.match_start_ts, self.match_running)
        # 右下パネル内にFPSを表示
        p.setPen(t.pen_stroke)
        p.setFont(QtGui.QFont(FONT_NUM, int(14*scale)))
        fps_rect = QtCore.QRect(rect_br_right.x()+12*scale,
                                 rect_br_right.y()+rect_br_right.height()-int(28*scale),
                                 rect_br_right.width()-24*scale,
                                 int(22*scale))
        p.drawText(fps_rect, QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter, f"{int(FPS)} FPS")

        # 未接続トースト
        if not st.get("connected"):
            msg = "LINK LOST — RETRYING…"; tw=int(360*scale); th=int(36*scale); tx=(big_w-tw)//2; ty=int(12*scale)
            phase=(time.time()*1.25)%1.0; op = 0.35 + 0.35*(1.0 if phase<0.5 else 0.0)
            p.setOpacity(op); p.setPen(QtGui.QPen(t.stroke,1)); p.setBrush(QtGui.QColor(10,12,18,210))
            p.drawRoundedRect(tx,ty,tw,th,6*scale,6*scale)
            draw_dotted_divider(p, tx+8*scale, ty+th-8*scale, tx+tw-8*scale, t.stroke, dash=4, gap=4, thick=1)
            p.setOpacity(1.0); p.setFont(t.font_num); p.setPen(t.stroke)
            p.drawText(QtCore.QRect(tx,ty,tw,th), QtCore.Qt.AlignCenter, msg)

        # ---------- 中央: 照準 & リング ----------
        cx,cy=(w//2)*scale,(h//2)*scale; draw_crosshair(p, cx,cy, int(22*scale), t.crosshair, thick=2*scale)
        pen_tick = QtGui.QPen(t.accent, 1*scale, QtCore.Qt.SolidLine, QtCore.Qt.SquareCap); p.setPen(pen_tick)
        r_tick = int(min(w,h)*0.25)*scale
        for i in range(24):
            a = 2*np.pi*i/24; x1=cx+int((r_tick-8*scale)*np.cos(a)); y1=cy+int((r_tick-8*scale)*np.sin(a))
            x2=cx+int(r_tick*np.cos(a)); y2=cy+int(r_tick*np.sin(a)); p.drawLine(x1,y1,x2,y2)
        draw_sensor_ring(p, cx,cy, int(min(w,h)*0.23)*scale, st.get("last_hit"), t.accent, t.warn)

        # スキャンライン & 右下FPS
        draw_scanlines(p, big_w, big_h, alpha=12)
        p.setPen(t.pen_stroke); p.setFont(QtGui.QFont(FONT_NUM, int(14*scale)))
        
        p.end()
        pix = QtGui.QPixmap.fromImage(big_img)
        try: dpr = self.devicePixelRatioF()
        except Exception: dpr = 1.0
        pix.setDevicePixelRatio(dpr)
        final_pix = pix.scaled(int(self.video_label.width()*dpr), int(self.video_label.height()*dpr), QtCore.Qt.KeepAspectRatio, QtCore.Qt.SmoothTransformation)
        self.video_label.setPixmap(final_pix)

        # 下のインフォ行（簡易）
        conn_short = "OK" if st.get("connected") else "NO"; ping_str = f"{st.get('ping_ms')}ms" if st.get("ping_ms") is not None else "--"
        self.info_label.setText(f"<b>HP</b>: {st.get('hp')}/{st.get('max_hp')} &nbsp;&nbsp; <b>Ammo</b>: {st.get('ammo')}/{st.get('max_ammo')} &nbsp;&nbsp; <b>Conn</b>: {conn_short} ({ping_str}) &nbsp;&nbsp; <b>LastHit</b>: {st.get('last_hit')}")

# ── AC風 左パネル（APゲージ＋AMMO）を描く ─────────────────
def render_left_status(p, rect: QtCore.QRect, st, t, scale):
    # 下地＋角金具
    p.setOpacity(0.92)
    p.setPen(QtGui.QPen(t.stroke, 1)); p.setBrush(QtGui.QColor(10,12,18,200))
    p.drawRoundedRect(rect, 8*scale, 8*scale)
    draw_corner_brackets(p, rect.x(), rect.y(), rect.width(), rect.height(), t.stroke, len_px=14*scale, thick=2)
    p.setOpacity(1.0)

    # HP/比率
    max_hp = st.get("max_hp", 1000)
    hp     = max(0, min(int(st.get("hp", 0)), max_hp))
    ratio  = (hp/float(max_hp)) if max_hp>0 else 0.0

    # 位置決め（“APすれすれ/数字は少し上”のやつ）
    bar_y   = rect.y() + int(28 * scale)
    label_y = bar_y - int(1 * scale)
    num_y   = bar_y - int(20 * scale)

    # APラベル
    p.setFont(QtGui.QFont(FONT_HEAD, int(12 * scale))); p.setPen(t.stroke)
    p.drawText(rect.x()+int(12*scale), label_y, "AP")

    # 5桁ゼロ詰めの数値（右端寄せ）
    ap_font = QtGui.QFont(FONT_NUM, int(22*scale)); ap_font.setStyleStrategy(QtGui.QFont.PreferAntialias)
    p.setFont(ap_font)
    ap_text = f"{hp:05d}"
    rnum = QtCore.QRect(rect.x()+rect.width()-int(120*scale), num_y, int(110*scale), int(24*scale))
    p.setPen(QtGui.QPen(QtGui.QColor(0,0,0,160), 2)); p.drawText(rnum, QtCore.Qt.AlignRight|QtCore.Qt.AlignVCenter, ap_text)
    p.setPen(QtGui.QPen(QtGui.QColor(230,240,255), 1)); p.drawText(rnum, QtCore.Qt.AlignRight|QtCore.Qt.AlignVCenter, ap_text)

    # 極細ラインバー（トラック→グロー→本体）
    bar_x = rect.x()+int(12*scale); bar_w = rect.width()-int(24*scale); y_mid = bar_y+int(8*scale)
    p.setPen(QtGui.QPen(QtGui.QColor(160,170,180,140), 2*scale, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap))
    p.drawLine(bar_x, y_mid, bar_x+bar_w, y_mid)

    prog = int(bar_w*ratio)
    p.setPen(QtGui.QPen(QtGui.QColor(120,170,255,70), 6*scale, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap))
    p.drawLine(bar_x, y_mid, bar_x+prog, y_mid)
    p.setPen(QtGui.QPen(QtGui.QColor(220,240,255), 3*scale, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap))
    p.drawLine(bar_x, y_mid, bar_x+prog, y_mid)

    # 低HPでアンバー点滅
    if max_hp>0 and hp/max_hp<0.30:
        phase=(time.time()*2.0)%1.0; alpha=0.55+0.45*(1.0 if phase<0.5 else 0.0)
        p.setPen(QtGui.QPen(QtGui.QColor(255,160,90,int(60*alpha)), 6*scale, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap))
        p.drawLine(bar_x, y_mid, bar_x+prog, y_mid)
        p.setPen(QtGui.QPen(QtGui.QColor(255,200,150,int(255*alpha)), 3*scale, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap))
        p.drawLine(bar_x, y_mid, bar_x+prog, y_mid)

    ''' AMMO（右上テイスト）
    p.setFont(t.font_small); p.setPen(t.stroke)
    tY = rect.y()+int(36*scale)
    p.drawText(rect.x()+int(12*scale), tY, "AMMO:")
    draw_ammo_dots(p, rect.x()+int(70*scale), tY-int(12*scale),
                   int(st.get("ammo",0)), int(st.get("max_ammo",5)), t.ammo, t.stroke, scale=1.0*scale)

    '''
    # 下辺ドット仕切り
    draw_dotted_divider(p, rect.x()+int(12*scale), rect.y()+rect.height()-int(10*scale),
                        rect.x()+rect.width()-int(12*scale), t.stroke, dash=4, gap=4, thick=1)


# ── 右パネル（LINK/TIME）を描く ───────────────────────────
def render_right_info(p, rect: QtCore.QRect, st, t, scale, match_total_sec, match_start_ts, running):
    # 下地＋角金具
    p.setOpacity(0.92)
    p.setPen(QtGui.QPen(t.stroke,1)); p.setBrush(QtGui.QColor(10,12,18,200))
    p.drawRoundedRect(rect, 8*scale, 8*scale)
    draw_corner_brackets(p, rect.x(), rect.y(), rect.width(), rect.height(), t.stroke, len_px=14*scale, thick=2)
    p.setOpacity(1.0)

    # テキスト
    elapsed = int(time.time()-match_start_ts) if running else 0
    rem = max(0, match_total_sec - elapsed); mm, ss = divmod(rem, 60)
    time_text = f"{mm:02d}:{ss:02d} / {match_total_sec//60:02d}:{match_total_sec%60:02d}"
    conn_text = "Connected" if st.get("connected") else "Disconnected"
    ping = st.get("ping_ms"); ping_text = f"PING {ping}ms" if ping is not None else "PING --ms"

    p.setPen(t.stroke); p.setFont(t.font_small)
    p.drawText(rect.x()+16*scale, rect.y()+26*scale, "LINK:")
    p.drawText(rect.x()+16*scale, rect.y()+50*scale, "TIME:")
    p.setFont(t.font_num)
    p.drawText(rect.x()+72*scale, rect.y()+26*scale, f"{conn_text}   {ping_text}")
    p.drawText(rect.x()+72*scale, rect.y()+50*scale, time_text)

    draw_dotted_divider(p, rect.x()+12*scale, rect.y()+rect.height()-12*scale,
                        rect.x()+rect.width()-12*scale, t.stroke, dash=4, gap=4, thick=1)


def main():
    QtCore.QCoreApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
    QtCore.QCoreApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps, True)

    threading.Thread(target=udp_listener, args=(UDP_IP, UDP_PORT), daemon=True).start()
    if LOG_CSV:
        threading.Thread(target=logger_worker, args=(LOG_FILENAME,), daemon=True).start()
        print(f"[LOG] writing to {LOG_FILENAME}")

    app = QtWidgets.QApplication(sys.argv)

    # ★ ここでフォント登録（＝QApplication の後）
    init_fonts()

    win = MonitorWindow()
    win.show()
    sys.exit(app.exec_())

print("[font] head =", FONT_HEAD, "/ num =", FONT_NUM)

if __name__ == "__main__":
    main()


