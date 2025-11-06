import socket, json, time

TARGET_IP = "127.0.0.1"   # UIを動かしてるPCのIP。別PCならそのIPに。
TARGET_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

hp = 100
ammo = 5
while True:
    msg = {
        "hp": hp, "max_hp": 100,
        "ammo": ammo, "max_ammo": 5,
        "connected": True, "ping_ms": 23
    }
    sock.sendto(json.dumps(msg).encode("utf-8"), (TARGET_IP, TARGET_PORT))
    time.sleep(0.5)

    # デモ：ちょっとずつダメージ＆弾消費
    hp = max(0, hp - 3)
    if ammo > 0 and hp % 2 == 0:
        ammo -= 1
