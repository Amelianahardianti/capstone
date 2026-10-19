import cv2
import os
import json
import argparse
import numpy as np
import paho.mqtt.client as mqtt
from ultralytics import YOLO
import threading
from video_source import open_source

# =================== 0. ARGUMEN INPUT ===================
# Contoh:
#   python test_video2.py                          -> aman.mp4 (default lama)
#   python test_video2.py --source vid.mp4
#   python test_video2.py --source "rtsp://..."    -> live stream
#   set CV_SOURCE=rtsp://...  lalu  python test_video2.py
parser = argparse.ArgumentParser()
parser.add_argument("--source", default=os.environ.get("CV_SOURCE", "aman.mp4"),
                    help="Path video file atau URL RTSP (default: env CV_SOURCE / aman.mp4)")
parser.add_argument("--rtsp-transport", default="tcp", choices=["tcp", "udp"])
args = parser.parse_args()

# ===============================================================
# SMART SAFE WALKING PATH — Final Edge CV Module v6
# Metode kecepatan : Virtual Line State-Crossing
# Arsitektur       : Centralized Intelligence + UI Countdown Simulasi
# ===============================================================

# =================== 1. KONFIGURASI MQTT LOCAL EDGE ===================
MQTT_BROKER = "10.76.111.208"
MQTT_PORT   = 1883
MQTT_TOPIC_PUB = "jalan/status"
MQTT_TOPIC_SUB = "jalan/trigger"

# Variabel bantuan untuk menjembatani data dari thread MQTT ke loop utama video
trigger_lock = threading.Lock()
trigger_dari_esp = False

# Fungsi callback saat Laptop menerima data dari ESP32
def on_message(client, userdata, msg):
    global trigger_dari_esp
    try:
        payload = msg.payload.decode('utf-8')
        if payload == "ADA_ORANG":
            print("\n[MQTT RECEIVED] Sinyal PIR dari ESP32 Terdeteksi!")
            with trigger_lock:        # ← TAMBAH INI
                trigger_dari_esp = True
    except Exception as e:
        print(f"[MQTT ERROR] {e}")

client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION1)
client.on_message = on_message # Pasang fungsi callback dengerin pesan

try:
    client.connect(MQTT_BROKER, MQTT_PORT, 60)
    client.subscribe(MQTT_TOPIC_SUB) # Laptop resmi subscribe topik trigger ESP32
    client.loop_start()
    mqtt_connected = True
    print("[MQTT] Local Edge Broker terhubung!")
except Exception as e:
    mqtt_connected = False
    print(f"[MQTT] Broker tidak ditemukan → Mode Simulasi GUI. ({e})")

# =================== 2. LOAD MODEL YOLO ===================
print("[CV] Memuat model YOLOv8 Nano...")
model = YOLO("yolov8n.pt")

# =================== 3. LOAD SUMBER VIDEO (FILE / RTSP) ===================
try:
    source = open_source(args.source, transport=args.rtsp_transport)
except IOError as e:
    print(f"[ERROR] {e}")
    exit()

if source.is_live:
    print("[INFO] Mode LIVE stream (timestamp = waktu nyata)")
else:
    print(f"[INFO] Video FPS: {source.fps:.1f}")
last_reconnect_count = 0
WINDOW_NAME = "Smart Safe Walking Path - Edge Analytics"

# =================== 4. PARAMETER KECEPATAN ===================
JARAK_ANTAR_GARIS_METER = 7.5  
SPEED_THRESHOLD_KMH     = 40    

LINE_A_RATIO = 0.75  
LINE_B_RATIO = 0.40  

# =================== 5. STRUKTUR DATA ===================
track_data  = {}
speed_cache = {}  

# Debounce NGEBUT
NGEBUT_DEBOUNCE_FRAMES = 8
ngebut_counter         = 0
frame_count            = 0

# Variabel Simulasi Countdown CV
is_crossing       = False
countdown_val     = 0
last_tick_time    = 0.0

# =================== 6. LOOP UTAMA ===================
while True:
    # current_time_s: file -> waktu video (sinkron meski ngelag), RTSP -> waktu nyata
    ret, original_frame, current_time_s = source.read()
    if not ret:
        if source.is_live:
            # Belum ada frame baru / sedang reconnect: jaga window tetap responsif
            status = source.status_text()
            if status:
                placeholder = np.zeros((360, 640, 3), dtype=np.uint8)
                cv2.putText(placeholder, status, (20, 180),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)
                cv2.imshow(WINDOW_NAME, placeholder)
            if cv2.waitKey(5) & 0xFF == ord('q'):
                break
            continue
        source.rewind()
        track_data.clear()
        speed_cache.clear()
        ngebut_counter = 0
        frame_count    = 0
        is_crossing    = False
        continue

    # Setelah reconnect ada jeda waktu: buang state crossing agar
    # kecepatan tidak dihitung melintasi gap (time_A sebelum putus, time_B sesudah)
    if source.reconnect_count != last_reconnect_count:
        last_reconnect_count = source.reconnect_count
        track_data.clear()
        speed_cache.clear()
        print("[RTSP] Reconnect terdeteksi -> state tracking kecepatan di-reset")

    frame_count += 1

    # Frame skipping hanya untuk file. Pada RTSP, reader thread sudah
    # otomatis membuang frame lama sehingga yang diproses selalu frame terbaru.
    if not source.is_live and frame_count % 2 == 0:
        continue

    # ---- ROI ----
    h_orig, w_orig = original_frame.shape[:2]
    roi_top  = int(h_orig * 0.25)
    roi_left = int(w_orig * 0.25)
    frame    = original_frame[roi_top:h_orig, roi_left:w_orig]
    h_roi, w_roi = frame.shape[:2]

    line_a_y = int(h_roi * LINE_A_RATIO)
    line_b_y = int(h_roi * LINE_B_RATIO)

    # ---- YOLO Tracking ----
    results = model.track(
        frame,
        persist = True,
        tracker = "bytetrack.yaml",
        classes = [0, 1, 2, 3, 5, 7], 
        conf    = 0.25,
        iou     = 0.5,
        imgsz   = 640,
        verbose = False
    )

    kepadatan  = 0
    ada_ngebut = False

    if results[0].boxes.id is not None:
        boxes     = results[0].boxes.xywh.cpu()
        track_ids = results[0].boxes.id.int().cpu().tolist()
        kepadatan = len(track_ids)

        for box, track_id in zip(boxes, track_ids):
            cx, cy = float(box[0]), float(box[1])
            bw, bh = float(box[2]), float(box[3])
            
            # Ground Plane Tracking (Contact Patch)
            bottom_y = cy + (bh / 2)

            if track_id not in track_data:
                TOLERANSI_SPAWN = 100
                spawned_below_a = bottom_y > (line_a_y - TOLERANSI_SPAWN)
                
                track_data[track_id] = {
                    "prev_y"     : bottom_y,
                    "crossed_A"  : False,
                    "crossed_B"  : False,
                    "time_A"     : None,
                    "time_B"     : None,
                    "valid_entry": spawned_below_a,
                }
                speed_cache[track_id] = 0

            td     = track_data[track_id]
            prev_y = td["prev_y"]

            if td["valid_entry"]:
                # State-crossing LINE_A
                if not td["crossed_A"] and prev_y > line_a_y >= bottom_y:
                    td["crossed_A"] = True
                    td["time_A"]    = current_time_s

                # State-crossing LINE_B
                if td["crossed_A"] and not td["crossed_B"] and prev_y > line_b_y >= bottom_y:
                    td["crossed_B"] = True
                    td["time_B"]    = current_time_s

                    delta_t = td["time_B"] - td["time_A"]
                    if delta_t > 0.05:
                        speed_ms  = JARAK_ANTAR_GARIS_METER / delta_t
                        speed_kmh = int(speed_ms * 3.6)
                        speed_cache[track_id] = speed_kmh

                    td["crossed_A"] = False
                    td["crossed_B"] = False
                    td["time_A"]    = None
                    td["time_B"]    = None

            td["prev_y"] = bottom_y

            kecepatan_kmh = speed_cache.get(track_id, 0)
            is_ngebut     = kecepatan_kmh > SPEED_THRESHOLD_KMH
            if is_ngebut:
                ada_ngebut = True

            color = (0, 0, 255) if is_ngebut else (0, 255, 0)
            if not td["valid_entry"]:
                color = (150, 150, 150)

            x1, y1 = int(cx - bw / 2), int(cy - bh / 2)
            x2, y2 = int(cx + bw / 2), int(cy + bh / 2)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)

            spd_label = f"{kecepatan_kmh} km/h" if td["valid_entry"] else "N/A"
            cv2.putText(frame, f"ID:{track_id} | {spd_label}",
                        (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

            if is_ngebut:
                cv2.putText(frame, "NGEBUT!", (x1, y1 - 24),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    # ---- Debounce NGEBUT ----
    ngebut_counter = min(ngebut_counter + 2, NGEBUT_DEBOUNCE_FRAMES + 2) if ada_ngebut \
                     else max(ngebut_counter - 1, 0)
    status_jalan   = "NGEBUT" if ngebut_counter >= NGEBUT_DEBOUNCE_FRAMES else "AMAN"

    # =========================================================
    # 7. LOGIKA KEPUTUSAN TARGET TIMER
    # =========================================================
    if status_jalan == "NGEBUT":
        target_timer = "HOLD"
    elif kepadatan < 2:
        target_timer = 9
    elif kepadatan <= 3:
        target_timer = 7
    else:
        target_timer = 5

    # =========================================================
    # 7.5 SIMULASI COUNTDOWN VIRTUAL
    # =========================================================
    with trigger_lock:
        lokal_trigger = trigger_dari_esp
        if lokal_trigger:
            trigger_dari_esp = False  # reset di dalam lock

    if lokal_trigger and not is_crossing:
        if target_timer == "HOLD":
            print("[WARNING] Jalanan NGEBUT! Trigger ESP32 diabaikan.")
        else:
            is_crossing = True
            countdown_val = target_timer
            last_tick_time = current_time_s
            print(f"[AUTO-TRIGGER] Countdown: {countdown_val}s")

    if is_crossing:
        # Kurangi angka setiap 1 detik waktu video
        if current_time_s - last_tick_time >= 1.0:
            countdown_val -= 1
            last_tick_time = current_time_s
            
        # Selesai nyeberang
        if countdown_val <= 0:
            is_crossing = False

    # ---- Cleanup memori ----
    if results[0].boxes.id is not None:
        active_ids = set(results[0].boxes.id.int().cpu().tolist())
    else:
        active_ids = set()

    if len(track_data) > 300:
        for tid in [t for t in track_data if t not in active_ids]:
            track_data.pop(tid, None)
            speed_cache.pop(tid, None)

    # =================== 8. PUBLISH MQTT ===================
    if mqtt_connected:
        # Menyelaraskan key JSON menjadi "target_timer" agar klop dengan main.cpp ESP32
        client.publish(MQTT_TOPIC_PUB, json.dumps({
            "kepadatan": kepadatan,
            "status"   : status_jalan,
            "timer"    : target_timer   # Kirim target timer ke ESP32
        }))

    # =================== 9. UI ===================
    cv2.line(frame, (0, line_a_y), (w_roi, line_a_y), (255, 120, 0), 1)
    cv2.putText(frame, "LINE A (start)", (5, line_a_y - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 120, 0), 1)

    cv2.line(frame, (0, line_b_y), (w_roi, line_b_y), (0, 210, 210), 1)
    cv2.putText(frame, "LINE B (stop)", (5, line_b_y - 5),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 210, 210), 1)

    # UI Kendaraan & Status
    cv2.putText(frame, f"Kendaraan: {kepadatan}", (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
    
    status_color = (0, 0, 255) if status_jalan == "NGEBUT" else (0, 255, 0)
    cv2.putText(frame, f"Status: {status_jalan}", (20, 70),
                cv2.FONT_HERSHEY_SIMPLEX, 1, status_color, 3)

    # UI Simulasi Timer
    if is_crossing:
        # Layar saat sedang nyeberang (Countdown Jalan)
        cv2.putText(frame, f"NYEBERANG: {countdown_val}s", (int(w_roi/2) - 150, 100),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 255), 4)
    else:
        # Layar Standby
        timer_color = (0, 165, 255) # Orange
        cv2.putText(frame, f"Target Timer: {target_timer}", (20, 110),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, timer_color, 3)
        cv2.putText(frame, "[TEKAN SPASI UTK NYEBERANG]", (20, 140),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)

    cv2.putText(frame,
                f"Jarak garis: {JARAK_ANTAR_GARIS_METER}m | Threshold: {SPEED_THRESHOLD_KMH}km/h",
                (10, h_roi - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (180, 180, 180), 1)

    cv2.imshow(WINDOW_NAME, frame)
    
    # Deteksi Keyboard
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord(' '):  # Tombol SPASI ditekan
        if not is_crossing:
            if target_timer == "HOLD":
                print("[WARNING] Jalanan NGEBUT! Belum boleh nyeberang.")
            else:
                is_crossing = True
                countdown_val = target_timer
                last_tick_time = current_time_s
                print(f"[SIMULASI] Menyeberang dengan jatah waktu {countdown_val} detik.")

# =================== 10. CLEANUP ===================
source.release()
cv2.destroyAllWindows()
if mqtt_connected:
    client.loop_stop()
    client.disconnect()
    print("[MQTT] Koneksi ditutup.")