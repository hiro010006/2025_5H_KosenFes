# robot_monitor.py
# 要件: 映像（黒背景可）にHPバー・弾数・接続状態・試合時間・最後の被弾センサ等を重畳表示
# UDPでJSON状態パケットを受信する（非同期スレッド）／ログをCSV出力

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

from PyQt5 import QtCore, QtGui, QtWidgets

# ------------- 設定 -------------
UDP_IP = "0.0.0.0"
UDP_PORT = 5005
LOG_CSV = True
LOG_FILENAME = f"match_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
# カメラ/ストリームの指定。Noneにすると黒背景（720p）を使う。
RTSP_URL = None  # 例: "rtsp://robot_camera/stream"
FRAME_W = 1280
FRAME_H = 720
FPS = 30
# ---------------------------------

# デフォルトステート
state_lock = threading.Lock()
state = {
    "hp": 100,
    "max_hp": 100,
    "ammo": 5,
    "max_ammo": 5,
    "sensors": [0]*8,
    "last_hit": None,
    "ping_ms": None,
    "connected": False,
    "timestamp": None
}

# ログ用キュー（軽量）
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
            data, addr = sock.recvfrom(4096)
            t_recv = time.time()
            try:
                text = data.decode("utf-8")
                obj = json.loads(text)
            except Exception as e:
                print("[UDP] json decode error:", e)
                continue
            with state_lock:
                # 例として期待するフィールドをマージ
                for k in ("hp", "ammo", "sensors", "last_hit"):
                    if k in obj:
                        state[k] = obj[k]
                state["connected"] = True
                state["timestamp"] = t_recv
                # ping推定（オプション: ロボット側が送信時刻を付ければ正確化可能）
                if "sent_ts" in obj:
                    state["ping_ms"] = int((t_recv - float(obj["sent_ts"])) * 1000)
                else:
                    state["ping_ms"] = None
                # ログ登録
                log_item = {
                    "t": datetime.fromtimestamp(t_recv).isoformat(),
                    "hp": state.get("hp"),
                    "ammo": state.get("ammo"),
                    "last_hit": state.get("last_hit")
                }
                log_queue.append(log_item)
        except socket.timeout:
            # タイムアウト → 接続状態の低下を示す
            with state_lock:
                # 接続切れ検出：最後の受信から1.5秒以上なら切断扱い
                if state["timestamp"] is None or time.time() - state["timestamp"] > 1.5:
                    state["connected"] = False
            continue
        except Exception as e:
            print("[UDP] listener error:", e)
            time.sleep(0.5)

# ログ出力スレッド
def logger_worker(filename):
    if not LOG_CSV:
        return
    with open(filename, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=["t","hp","ammo","last_hit"])
        writer.writeheader()
        while True:
            while log_queue:
                item = log_queue.popleft()
                writer.writerow(item)
                f.flush()
            time.sleep(0.2)

# GUIアプリ
class MonitorWindow(QtWidgets.QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("ロボットバトル モニター")
        self.setMinimumSize(1280//2, 720//2)
        self.resize(FRAME_W//2, FRAME_H//2)

        # ビデオ表示用ラベル
        self.video_label = QtWidgets.QLabel()
        self.video_label.setAlignment(QtCore.Qt.AlignCenter)

        # 情報領域（下）
        self.info_label = QtWidgets.QLabel()
        self.info_label.setFixedHeight(80)
        self.info_label.setAlignment(QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter)
        font = QtGui.QFont("Arial", 11)
        self.info_label.setFont(font)

        # レイアウト
        vbox = QtWidgets.QVBoxLayout()
        vbox.addWidget(self.video_label, stretch=1)
        vbox.addWidget(self.info_label)
        self.setLayout(vbox)

        # タイマーでフレーム更新
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.update_frame)
        self.timer.start(int(1000 / FPS))

        # カメラ/ストリーム初期化（OpenCV）
        if RTSP_URL:
            self.cap = cv2.VideoCapture(RTSP_URL)
        else:
            self.cap = None  # 黒背景で描画

        # 試合時間カウンタ（例: 3分）
        self.match_total_sec = 3 * 60
        self.match_start_ts = time.time()
        self.match_running = True

    def closeEvent(self, event):
        # アプリ終了時処理
        if self.cap:
            self.cap.release()
        event.accept()
        QtCore.QCoreApplication.quit()

    def make_black_frame(self):
        return np.zeros((FRAME_H, FRAME_W, 3), dtype=np.uint8)

    def overlay_ui(self, frame, st):
        # frame: BGR numpy array
        h, w = frame.shape[:2]
        # --- HPバー ---
        max_hp = st.get("max_hp", 100)
        hp = max(0, min(max_hp, int(st.get("hp", 0))))
        bar_w = int(w * 0.4)
        bar_h = 24
        bar_x = 20
        bar_y = 20

        # 背景長方形
        cv2.rectangle(frame, (bar_x, bar_y), (bar_x+bar_w, bar_y+bar_h), (50,50,50), -1)
        # HP比率
        hp_ratio = hp / float(max_hp) if max_hp>0 else 0.0
        fill_w = int(bar_w * hp_ratio)
        # 塗り（赤→緑的グラデーションを簡易で表現）
        green = int(255 * hp_ratio)
        red = int(255 * (1 - hp_ratio))
        cv2.rectangle(frame, (bar_x, bar_y), (bar_x+fill_w, bar_y+bar_h), (0, green, red), -1)
        # 枠線・テキスト
        cv2.rectangle(frame, (bar_x, bar_y), (bar_x+bar_w, bar_y+bar_h), (200,200,200), 2)
        cv2.putText(frame, f"HP: {hp}/{max_hp}", (bar_x+6, bar_y+bar_h-6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2, cv2.LINE_AA)

        # --- 弾数 ---
        ammo = st.get("ammo", 0)
        max_ammo = st.get("max_ammo", 5)
        ammo_text = f"Ammo: {ammo}/{max_ammo}"
        cv2.putText(frame, ammo_text, (bar_x, bar_y+bar_h+30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (240,240,200), 2, cv2.LINE_AA)

        # --- 接続状態／Ping ---
        conn_text = "Connected" if st.get("connected") else "Disconnected"
        ping = st.get("ping_ms")
        ping_text = f"Ping: {ping} ms" if ping is not None else ""
        cv2.putText(frame, f"{conn_text}  {ping_text}", (w-420, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200,200,200), 2, cv2.LINE_AA)

        # --- 試合時間 ---
        elapsed = int(time.time() - self.match_start_ts) if self.match_running else 0
        remaining = max(0, self.match_total_sec - elapsed)
        mm = remaining // 60
        ss = remaining % 60
        time_text = f"Time: {mm:02d}:{ss:02d} / {self.match_total_sec//60:02d}:{self.match_total_sec%60:02d}"
        cv2.putText(frame, time_text, (w-420, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200,200,200), 2, cv2.LINE_AA)

        # --- 最後の被弾センサを一瞬表示（赤い閃光） ---
        last_hit = st.get("last_hit")
        if last_hit is not None:
            # センサ番号を画面周りの位置にマッピングする（例）
            # sensor positions: 8 sensors around robot
            cx, cy = w//2, h//2
            radius = min(w,h)//3
            angle = (2 * np.pi) * (last_hit / 8.0)
            sx = int(cx + radius * np.cos(angle))
            sy = int(cy + radius * np.sin(angle))
            # フラッシュ（円）
            cv2.circle(frame, (sx, sy), 40, (0,0,255), -1)
            cv2.putText(frame, f"Hit #{last_hit}", (sx-40, sy-50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2, cv2.LINE_AA)

        # --- フッタのログ簡易表示 ---
        log_text = f"Last log: {st.get('timestamp') and datetime.fromtimestamp(st.get('timestamp')).strftime('%H:%M:%S')}"
        cv2.putText(frame, log_text, (20, h-20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (180,180,180), 1, cv2.LINE_AA)

        return frame

    def update_frame(self):
        # --- フレーム取得 ---
        if self.cap:
            ret, frame = self.cap.read()
            if not ret:
                frame = self.make_black_frame()
            else:
                frame = cv2.resize(frame, (FRAME_W, FRAME_H), interpolation=cv2.INTER_LINEAR)
        else:
            frame = self.make_black_frame()

        # --- 状態コピー ---
        with state_lock:
            st_copy = dict(state)

        # --- RGB -> QImage 元画像 ---
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape

        # --- スーパーサンプリング倍率（2または3を試す） ---
        scale_factor = 2  # 2でまず試し、重ければ1に。よりシャープなら3を試す
        big_w, big_h = w * scale_factor, h * scale_factor

        # --- 作業用 QImage（高品質フォーマット） ---
        # ARGB32_Premultiplied を使うことで合成品質が良くなる
        big_img = QtGui.QImage(big_w, big_h, QtGui.QImage.Format_ARGB32_Premultiplied)
        big_img.fill(QtGui.QColor(0, 0, 0, 255))  # 黒背景

        painter = QtGui.QPainter(big_img)
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
        painter.setRenderHint(QtGui.QPainter.TextAntialiasing, True)
        painter.setRenderHint(QtGui.QPainter.SmoothPixmapTransform, True)

        # --- 映像を大きなサイズにスムース描画 ---
        # まず QImage を作る（コピーしておくと安全）
        src_qimg = QtGui.QImage(rgb.data, w, h, ch * w, QtGui.QImage.Format_RGB888).copy()
        src_qimg_big = src_qimg.scaled(big_w, big_h, QtCore.Qt.IgnoreAspectRatio, QtCore.Qt.SmoothTransformation)
        painter.drawImage(0, 0, src_qimg_big)

        # --- ベクタ描画設定（フォント） ---
        # 環境に合わせてフォント名を変更。埋め込みが望ましければ QFontDatabase を使う。
        font_name = "Noto Sans"  # 必要に応じて "Meiryo" や "Segoe UI" 等へ
        hp_font_size = int(28 * scale_factor)
        ammo_font_size = int(20 * scale_factor)

        font_hp = QtGui.QFont(font_name, hp_font_size)
        font_hp.setStyleStrategy(QtGui.QFont.PreferAntialias)
        painter.setFont(font_hp)

        # --- HPバー（ベクタ） ---
        max_hp = st_copy.get("max_hp", 100)
        hp = max(0, min(max_hp, int(st_copy.get("hp", 0))))
        bar_w = int(w * 0.4) * scale_factor
        bar_h = 34 * scale_factor
        bar_x = 30 * scale_factor
        bar_y = 30 * scale_factor

        painter.setPen(QtGui.QPen(QtGui.QColor(200,200,200,220), 2 * scale_factor))
        painter.setBrush(QtGui.QBrush(QtGui.QColor(30,30,30,200)))
        painter.drawRoundedRect(bar_x, bar_y, bar_w, bar_h, 8 * scale_factor, 8 * scale_factor)

        hp_ratio = (hp / float(max_hp)) if max_hp>0 else 0.0
        grad = QtGui.QLinearGradient(bar_x, bar_y, bar_x + bar_w, bar_y)
        grad.setColorAt(0.0, QtGui.QColor(int(255*(1-hp_ratio)), 60, 60))
        grad.setColorAt(1.0, QtGui.QColor(60, int(255*hp_ratio), 60))
        painter.setBrush(QtGui.QBrush(grad))
        painter.setPen(QtCore.Qt.NoPen)
        painter.drawRoundedRect(bar_x, bar_y, int(bar_w * hp_ratio), bar_h, 8 * scale_factor, 8 * scale_factor)

        # --- HP テキストを QPainterPath でベクタ描画（アウトライン）---
        hp_text = f"HP: {hp}/{max_hp}"
        path = QtGui.QPainterPath()
        # addText の座標はベースライン位置（左下ベース）なので注意
        text_x = bar_x + 12 * scale_factor
        text_y = bar_y + int(bar_h * 0.72)
        path.addText(text_x, text_y, font_hp, hp_text)

        # 黒アウトライン（stroke）
        pen_width = max(2 * scale_factor, 3)
        painter.setPen(QtGui.QPen(QtGui.QColor(0,0,0), pen_width, QtCore.Qt.SolidLine, QtCore.Qt.RoundCap, QtCore.Qt.RoundJoin))
        painter.setBrush(QtCore.Qt.NoBrush)
        painter.drawPath(path)

        # 白塗り本体
        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(QtGui.QBrush(QtGui.QColor(255,255,255)))
        painter.drawPath(path)

        # --- Ammo（少し小さいフォント）---
        font_ammo = QtGui.QFont(font_name, ammo_font_size)
        font_ammo.setStyleStrategy(QtGui.QFont.PreferAntialias)
        painter.setFont(font_ammo)
        ammo_text = f"Ammo: {st_copy.get('ammo',0)}/{st_copy.get('max_ammo',5)}"
        ammo_x = bar_x
        ammo_y = bar_y + bar_h + 36 * scale_factor
        # draw via path for crispness
        path2 = QtGui.QPainterPath()
        path2.addText(ammo_x, ammo_y, font_ammo, ammo_text)
        painter.setPen(QtGui.QPen(QtGui.QColor(0,0,0), max(2, scale_factor)))
        painter.drawPath(path2)
        painter.setBrush(QtGui.QBrush(QtGui.QColor(240,240,240)))
        painter.setPen(QtCore.Qt.NoPen)
        painter.drawPath(path2)

        # --- 接続状態・Ping（右上）---
        painter.setFont(QtGui.QFont(font_name, int(16 * scale_factor)))
        conn_text = "Connected" if st_copy.get("connected") else "Disconnected"
        ping_val = st_copy.get("ping_ms")
        ping_text = f"Ping: {ping_val} ms" if ping_val is not None else "Ping: --"
        painter.setPen(QtGui.QPen(QtGui.QColor(220,220,220)))
        painter.drawText(big_w - int(460 * scale_factor), int(40 * scale_factor), f"{conn_text}  {ping_text}")

        # --- 最後の被弾表示（円）---
        last_hit = st_copy.get("last_hit")
        if last_hit is not None:
            cx, cy = (w // 2) * scale_factor, (h // 2) * scale_factor
            radius = int(min(w,h) * 0.22) * scale_factor
            angle = (2 * np.pi) * (last_hit / 8.0)
            sx = int(cx + radius * np.cos(angle))
            sy = int(cy + radius * np.sin(angle))
            hit_color = QtGui.QColor(255, 60, 60, 200)
            painter.setBrush(hit_color)
            painter.setPen(QtCore.Qt.NoPen)
            painter.drawEllipse(QtCore.QPoint(sx, sy), int(30 * scale_factor), int(30 * scale_factor))
            # small white label
            painter.setPen(QtGui.QPen(QtGui.QColor(255,255,255)))
            painter.setFont(QtGui.QFont(font_name, int(14 * scale_factor)))
            painter.drawText(sx - 36 * scale_factor, sy - 44 * scale_factor, f"Hit #{last_hit}")

        painter.end()

        # --- QImage -> QPixmap -> ラベルへ（HiDPI対応） ---
        pix = QtGui.QPixmap.fromImage(big_img)
        # Device pixel ratio をセット（UI DPI に合わせる）
        try:
            dpr = self.devicePixelRatioF()
        except Exception:
            dpr = 1.0
        pix.setDevicePixelRatio(dpr)

        final_pix = pix.scaled(
            int(self.video_label.width() * dpr),
            int(self.video_label.height() * dpr),
            QtCore.Qt.KeepAspectRatio,
            QtCore.Qt.SmoothTransformation
        )

        # setPixmap は自動で device pixel ratio を扱う環境が多いが念のため
        self.video_label.setPixmap(final_pix)

        # --- info_label 更新（元の HTML 表示維持） ---
        conn_text_short = "OK" if st_copy.get("connected") else "NO"
        ping_str = f"{ping_val}ms" if ping_val is not None else "--"
        info_html = (f"<b>HP</b>: {st_copy.get('hp')}/{st_copy.get('max_hp')} &nbsp;&nbsp;"
                    f"<b>Ammo</b>: {st_copy.get('ammo')}/{st_copy.get('max_ammo')} &nbsp;&nbsp;"
                    f"<b>Conn</b>: {conn_text_short} ({ping_str}) &nbsp;&nbsp;"
                    f"<b>LastHit</b>: {last_hit}")
        self.info_label.setText(info_html)


def main():
    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_EnableHighDpiScaling, True)
    QtWidgets.QApplication.setAttribute(QtCore.Qt.AA_UseHighDpiPixmaps, True)

    # UDPリスナースレッド開始
    t = threading.Thread(target=udp_listener, args=(UDP_IP, UDP_PORT), daemon=True)
    t.start()

    # ロガースレッド開始
    if LOG_CSV:
        tlog = threading.Thread(target=logger_worker, args=(LOG_FILENAME,), daemon=True)
        tlog.start()
        print(f"[LOG] writing to {LOG_FILENAME}")

    app = QtWidgets.QApplication(sys.argv)
    win = MonitorWindow()
    win.show()
    sys.exit(app.exec_())

if __name__ == "__main__":
    main()
