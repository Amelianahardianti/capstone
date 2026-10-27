import os
import sys
import cv2
import json
import queue
import threading
import time
from collections import deque

import numpy as np
import torch
import paho.mqtt.client as mqtt
from ultralytics import YOLO

# ===============================================================
# SMART SAFE WALKING PATH — Final Edge CV Module v8 (perbaikan dari v7)
# Metode kecepatan : Homografi (piksel → meter di permukaan jalan) + regresi posisi-waktu
# Arsitektur       : Centralized Intelligence + UI Countdown Simulasi
#
# Perubahan dari v7:
#  1. Kecepatan diukur SEJAK kendaraan pertama kali terlacak, tidak menunggu lewat LINE A lalu LINE B.
#     Titik bawah bounding box diubah ke meter lewat homografi (kalibrasi 4 titik), lalu
#     kecepatan = kemiringan regresi posisi terhadap waktu selama JENDELA_S terakhir.
#     Kendaraan yang mulai terlacak di tengah jalan / ID-nya berganti tetap terukur.
#  2. Hanya kendaraan di dalam jalur yang dikalibrasi yang dihitung → mobil parkir di jalan sebelah /
#     orang di trotoar tidak lagi menambah kepadatan.
#  3. Kecepatan maksimum tiap kendaraan dicetak saat kendaraan keluar layar (untuk validasi).
#
# Tetap dari v7: device GPU eksplisit, bytetrack_fixed.yaml, pembaca RTSP di thread terpisah,
# timestamp kamera (PTS).
#
# test_finetune: sama dengan test_nino (v8), tapi memakai yolo26m hasil fine-tuning dataset Roboflow
# (best_e50.pt, class 0 mobil, 1 bus, 2 motor, 3 truk; tidak ada class person) + nama class di label box.
# Butuh ultralytics 8.4.188 (versi yang dipakai saat training di server).
#
# Pemakaian: python testvid_fixed.py            → RTSP kamera
#            python testvid_fixed.py video.mp4  → file video (untuk uji ulang)
#            python kalibrasi.py                → klik ulang titik kalibrasi kalau kamera bergeser
# ===============================================================

BASE_DIR = os.path.dirname(os.path.abspath(__file__))  # file model/yaml dicari di folder script ini

# =================== 1. KONFIGURASI MQTT LOCAL EDGE ===================
# Laptop = Windows Mobile hotspot "cameliaa" (ESP32 menyambung ke sini), IP laptop di hotspot
# selalu 192.168.137.1. Broker Mosquitto jalan di laptop yang sama.
# ESP32 memicu siklus sendiri (push button / piezo + ultrasonik); script ini hanya mengirim
# hasil CV ke ESP lewat jalan/status: panjang countdown (detik) + status NGEBUT/AMAN.
MQTT_BROKER = "192.168.137.1"
MQTT_PORT   = 1883
MQTT_TOPIC_PUB = "jalan/status"
PUBLISH_INTERVAL_S = 0.5  # ESP hanya butuh data terbaru, tidak perlu tiap frame


def on_connect(client, userdata, *args):
    print(f"[MQTT] Terhubung ke broker {MQTT_BROKER}:{MQTT_PORT}")


def on_disconnect(client, userdata, *args):
    print("[MQTT] Terputus dari broker, mencoba menyambung ulang...")


if hasattr(mqtt, "CallbackAPIVersion"):  # paho-mqtt >= 2.0
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
else:  # paho-mqtt 1.x
    client = mqtt.Client()
client.on_connect    = on_connect
client.on_disconnect = on_disconnect

# connect_async: tidak memblokir kalau broker belum nyala, dan terus mencoba menyambung.
# Selama belum tersambung CV tetap jalan (mode simulasi GUI), data baru terkirim setelah tersambung.
client.connect_async(MQTT_BROKER, MQTT_PORT, 60)
client.loop_start()
print(f"[MQTT] Menyambung ke broker {MQTT_BROKER}:{MQTT_PORT}...")

# =================== 2. LOAD MODEL YOLO ===================
# Tanpa argumen device, Ultralytics DIAM-DIAM pakai CPU kalau PyTorch yang ter-install versi CPU
# (contoh: torch 2.12.0+cpu). Jadi device dipilih eksplisit dan statusnya dicetak.
USE_GPU = torch.cuda.is_available()
DEVICE  = 0 if USE_GPU else "cpu"
IMGSZ   = 960 if USE_GPU else 640  # GPU kuat → input lebih besar → kendaraan jauh/kecil lebih terdeteksi
MODEL_FILE = "best_e50.pt"  # yolo26m fine-tuning (runs/yolo26m_e50 di server)

if USE_GPU:
    print(f"[CV] GPU aktif: {torch.cuda.get_device_name(0)}")
else:
    print(f"[CV] PERINGATAN: torch {torch.__version__} tidak bisa memakai CUDA → YOLO berjalan di CPU!")

print(f"[CV] Memuat model {MODEL_FILE}...")
model = YOLO(os.path.join(BASE_DIR, MODEL_FILE))
TRACKER_CFG = os.path.join(BASE_DIR, "bytetrack_fixed.yaml")

# =================== 3. SUMBER VIDEO (RTSP / FILE) ===================
# URL RTSP (berisi username/password) dibaca dari rtsp_url.txt agar tidak tertulis di kode.
# Format: rtsps://USERNAME:PASSWORD@IP_ADDRESS:PORT/...
RTSP_FILE = os.path.join(BASE_DIR, "rtsp_url.txt")
rtsp_url  = ""
if os.path.exists(RTSP_FILE):
    with open(RTSP_FILE) as f:
        rtsp_url = f.read().strip()
if len(sys.argv) <= 1 and not rtsp_url:
    print(f"[ERROR] Tidak ada sumber video. Isi {RTSP_FILE} dengan URL RTSP, atau jalankan: python testvid_fixed.py video.mp4")
    exit()
source   = sys.argv[1] if len(sys.argv) > 1 else rtsp_url
is_live  = source.lower().startswith(("rtsp://", "rtsps://", "http://", "https://"))


class FrameReader:
    """Thread khusus pembaca video.

    - Live (RTSP): hanya frame TERBARU yang disimpan. Kalau YOLO lebih lambat dari kamera,
      frame lama dibuang di sini, jadi antrean/lag tidak menumpuk.
      (cap.set(CAP_PROP_BUFFERSIZE, 1) tidak berpengaruh di backend FFmpeg.)
    - File: semua frame diteruskan satu per satu.
    Setiap frame membawa timestamp kamera (PTS stream) untuk perhitungan kecepatan.
    """

    def __init__(self, src, live):
        self.src, self.live = src, live
        self.cap = cv2.VideoCapture(src)
        self.fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.q = queue.Queue(maxsize=1)
        self.epoch    = 0     # naik setiap timeline waktu putus (reconnect / PTS melompat)
        self.use_pts  = True  # False → fallback ke waktu frame DITERIMA (tetap bukan waktu diproses)
        self.bad_pts  = 0
        self.last_pts = None
        self.running  = True
        self.thread = threading.Thread(target=self._loop, daemon=True)

    def start(self):
        self.thread.start()
        return self

    def _loop(self):
        while self.running:
            if not self.cap.grab():
                if not self.live:
                    break  # file habis
                print("[WARNING] Sinyal RTSP terputus! Menyambung ulang...")
                self.cap.release()
                time.sleep(1)
                self.cap = cv2.VideoCapture(self.src)
                self.epoch += 1
                self.last_pts = None
                continue

            t_terima = time.time()
            pts = self.cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0
            ok, frame = self.cap.retrieve()
            if not ok:
                continue

            # PTS harus selalu naik dengan selisih wajar; kalau tidak, timeline dianggap putus
            if self.use_pts and self.last_pts is not None and not (0 < pts - self.last_pts < 1.0):
                self.bad_pts += 1
                self.epoch   += 1
                if self.bad_pts > 5:
                    self.use_pts = False
                    print("[TIME] PTS stream tidak valid → pakai waktu frame diterima.")
            self.last_pts = pts
            item = (frame, pts if self.use_pts else t_terima, self.epoch)

            if self.live:
                try:
                    self.q.get_nowait()  # buang frame lama yang belum sempat diproses
                except queue.Empty:
                    pass
            while self.running:
                try:
                    self.q.put(item, timeout=0.5)
                    break
                except queue.Full:
                    continue

        while self.running:  # sinyal akhir video ke loop utama
            try:
                self.q.put(None, timeout=0.5)
                break
            except queue.Full:
                continue

    def stop(self):
        self.running = False
        self.thread.join(timeout=2)
        if not self.thread.is_alive():  # jangan release saat thread masih di dalam grab()
            self.cap.release()


print(f"[CV] Mencoba terhubung ke {source}...")
reader = FrameReader(source, is_live)

if not reader.cap.isOpened():
    print("[ERROR] Gagal membuka RTSP. Pastikan terkoneksi Wi-Fi Teknik!")
    exit()

video_fps = reader.fps if 0 < reader.fps < 240 else 30.0
frame_w   = int(reader.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_h   = int(reader.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
print(f"[INFO] Video FPS: {video_fps:.1f} | Resolusi: {frame_w}x{frame_h}")
reader.start()

# =================== 4. KALIBRASI PERSPEKTIF ===================
# 4 titik di TEPI jalur yang dipantau, koordinat frame ASLI (bukan ROI), urutan:
#   A-kiri, A-kanan, B-kanan, B-kiri
# A dan B = dua garis melintang jalan dengan jarak nyata JARAK_KALIBRASI_METER.
# Kiri = tepi median, kanan = tepi kanstin luar (dua garis yang sejajar arah jalan).
# Default diambil dari frame kamera 20-10-2026: A = baris y 252, B = baris y 468
# (posisi LINE A/LINE B v7 yang jaraknya diukur 13.5 m).
# Kamera bergeser / pindah lokasi → jalankan: python kalibrasi.py  (hasilnya kalibrasi.json)
KALIBRASI_TITIK        = [(673, 252), (805, 252), (974, 468), (681, 468)]
KALIBRASI_UKURAN_FRAME = (1280, 720)
JARAK_KALIBRASI_METER  = 13.5

KALIBRASI_FILE = os.path.join(BASE_DIR, "kalibrasi.json")
if os.path.exists(KALIBRASI_FILE):
    with open(KALIBRASI_FILE) as f:
        kal = json.load(f)
    KALIBRASI_TITIK        = [tuple(p) for p in kal["titik"]]
    KALIBRASI_UKURAN_FRAME = tuple(kal["ukuran_frame"])
    JARAK_KALIBRASI_METER  = kal["jarak_meter"]
    print(f"[KALIBRASI] Memakai {KALIBRASI_FILE}")
else:
    print("[KALIBRASI] kalibrasi.json tidak ada → memakai titik default di script.")


def buat_homografi(w, h):
    """Homografi frame asli → koordinat jalan.

    X = posisi melintang dalam satuan lebar jalur (0 = tepi kiri, 1 = tepi kanan).
    Y = meter sepanjang jalan (0 = garis A, bertambah ke arah garis B / mendekati kamera).
    Lebar jalur nyata tidak perlu diketahui: sumbu Y tidak bergantung pada skala sumbu X.
    """
    sx, sy = w / KALIBRASI_UKURAN_FRAME[0], h / KALIBRASI_UKURAN_FRAME[1]
    src = np.float32([(x * sx, y * sy) for x, y in KALIBRASI_TITIK])
    L   = JARAK_KALIBRASI_METER
    dst = np.float32([(0, 0), (1, 0), (1, L), (0, L)])
    return cv2.getPerspectiveTransform(src, dst)


H     = buat_homografi(frame_w or KALIBRASI_UKURAN_FRAME[0], frame_h or KALIBRASI_UKURAN_FRAME[1])
H_inv = np.linalg.inv(H)
H_inv = H_inv / (H_inv @ (0, 0, 1.0))[2]  # skala homogen garis A > 0 → w > 0 berarti di depan kamera


def ke_frame(x, y):
    """Koordinat jalan (X, Y) → piksel frame asli. None kalau titik itu di belakang kamera."""
    u, v, w = H_inv @ (x, y, 1.0)
    return (int(u / w), int(v / w)) if w > 0 else None


# Penggaris kalibrasi (koordinat frame asli): garis melintang tiap 5 m dari 50 m sebelum garis A.
# Cocokkan dengan patokan nyata (jarak pohon, tiang, kanstin) untuk mengecek kalibrasi.
penggaris = [(ke_frame(0, j), ke_frame(1, j)) for j in range(-50, 31, 5)]
penggaris = [(p, q) for p, q in penggaris if p and q and 0 <= p[1] < (frame_h or KALIBRASI_UKURAN_FRAME[1])]
garis_A   = (ke_frame(0, 0), ke_frame(1, 0))
garis_B   = (ke_frame(0, JARAK_KALIBRASI_METER), ke_frame(1, JARAK_KALIBRASI_METER))

# =================== 5. PARAMETER KECEPATAN ===================
SPEED_THRESHOLD_KMH = 40

JENDELA_S        = 1.0   # kecepatan = regresi posisi selama 1 detik terakhir (meredam goyangan box)
MIN_DURASI_S     = 0.5   # minimal 0.5 detik riwayat sebelum kecepatan ditampilkan
MIN_TITIK        = 6
BATAS_LUAR_JALUR = 0.15  # toleransi di luar tepi jalur (fraksi lebar jalur) masih dianggap di jalan
HAPUS_TRACK_S    = 3.0   # track tidak terlihat selama ini → laporkan kecepatan maks, lalu hapus

# Kategori kepadatan = jumlah kendaraan di jalur yang dikalibrasi.
# Panjang countdown penyeberang (detik) ditentukan di sini dan dikirim ke ESP32 lewat jalan/status;
# ESP32 menampilkannya di 7-segment 2 digit (maks 99). NGEBUT tidak mengubah angka ini: ESP32
# menahan fase PREPARING (buzzer panjang) selama status NGEBUT, lalu countdown berjalan normal.
PADAT_SEDANG_MIN    = 2    # kendaraan >= 2  → SEDANG   (0-1 = SEPI)
PADAT_PADAT_MIN     = 4    # kendaraan >= 4  → PADAT
TIMER_SEPI          = 15   # detik
TIMER_SEDANG        = 20
TIMER_PADAT         = 25
KEPADATAN_JENDELA_S = 3.0  # kepadatan = nilai TERBESAR dalam 3 detik terakhir (meredam kedip deteksi)

# =================== 6. STRUKTUR DATA ===================
track_data = {}

# Debounce NGEBUT
NGEBUT_DEBOUNCE_FRAMES = 8
ngebut_counter         = 0

riwayat_padat   = deque()  # (waktu, jumlah kendaraan) untuk kepadatan yang dihaluskan
terakhir_publish = 0.0

# Simulasi countdown lokal (tombol SPASI) hanya untuk melihat angka di layar laptop;
# countdown sebenarnya jalan di ESP32.
is_crossing       = False
countdown_val     = 0
last_tick_time    = 0.0

# Variabel kalkulasi real-time FPS
prev_frame_time = 0
new_frame_time  = 0


def kecepatan_regresi(riwayat):
    """Kemiringan least-squares posisi (m) terhadap waktu (s) → km/jam.
    Memakai semua titik di jendela, jadi jauh lebih tahan goyangan box daripada selisih 2 titik."""
    t = np.array([p[0] for p in riwayat])
    y = np.array([p[1] for p in riwayat])
    t = t - t.mean()
    return abs(float((t * (y - y.mean())).sum() / (t * t).sum())) * 3.6


# =================== 7. LOOP UTAMA ===================
while True:
    try:
        item = reader.q.get(timeout=5)
    except queue.Empty:
        print("[WARNING] Belum ada frame baru dari kamera...")
        continue
    if item is None:
        break  # file video habis

    # frame_ts: waktu kamera (untuk kecepatan). now: jam dinding (untuk countdown & FPS UI).
    original_frame, frame_ts, epoch = item
    now = time.time()

    # ---- ROI (Region of Interest) ----
    h_orig, w_orig = original_frame.shape[:2]

    # 1. Atas tidak dipotong sama sekali (Y = 0) agar kendaraan jauh tetap terlihat
    roi_top = 0

    # 2. Potong Kiri (SANGAT PENTING): Membuang lajur kiri dan pembatas jalan tengah.
    # Kita potong 50% sisi kiri layar, sehingga AI hanya mulai melihat dari tengah jalan ke kanan.
    roi_left = int(w_orig * 0.50)

    # 3. Potong Kanan: Membuang sedikit trotoar/pohon di ujung paling kanan (menyisakan 95% layar).
    roi_right = int(w_orig * 0.95)

    # Frame di-crop: Atas ke Bawah UTUH, Kiri ke Kanan fokus di LAJUR KANAN (arah mendekat)
    frame = original_frame[roi_top:h_orig, roi_left:roi_right]

    h_roi, w_roi = frame.shape[:2]

    # ---- YOLO Tracking ----
    results = model.track(
        frame,
        persist = True,
        tracker = TRACKER_CFG,
        classes = [0, 1, 2, 3],  # mobil, bus, motor, truk (semua class model fine-tuning)
        conf    = 0.12,
        iou     = 0.5,
        imgsz   = IMGSZ,
        device  = DEVICE,
        quantize = 16 if USE_GPU else None,  # FP16 di GPU (ultralytics 8.4.188: pengganti half=True)
        verbose = False
    )

    kepadatan  = 0
    ada_ngebut = False

    if results[0].boxes.id is not None:
        boxes     = results[0].boxes.xywh.cpu().numpy()
        track_ids = results[0].boxes.id.int().cpu().tolist()
        classes   = results[0].boxes.cls.int().cpu().tolist()

        # Ground Plane Tracking (Contact Patch): titik bawah-tengah box → koordinat jalan (X, Y meter)
        kaki = np.stack([boxes[:, 0] + roi_left, boxes[:, 1] + boxes[:, 3] / 2 + roi_top], axis=1)
        posisi_jalan = cv2.perspectiveTransform(
            kaki.reshape(-1, 1, 2).astype(np.float32), H).reshape(-1, 2)

        for box, track_id, cls, (pos_x, pos_y) in zip(boxes, track_ids, classes, posisi_jalan):
            # Di luar jalur yang dikalibrasi (parkir di jalan sebelah, trotoar) → abaikan
            if not (-BATAS_LUAR_JALUR <= pos_x <= 1 + BATAS_LUAR_JALUR):
                continue
            kepadatan += 1

            cx, cy, bw, bh = (float(v) for v in box)
            bottom_y = cy + (bh / 2)

            td = track_data.get(track_id)
            if td is None or td["epoch"] != epoch:
                # Track baru, atau timeline putus (reconnect) → riwayat waktu lama tidak bisa dipakai
                td = track_data[track_id] = {
                    "riwayat"  : deque(),   # (timestamp kamera, posisi Y meter)
                    "epoch"    : epoch,
                    "kecepatan": None,
                    "maks"     : None,
                    "kelas"    : model.names[cls],
                }
            td["terlihat"] = now

            # Box terpotong tepi bawah frame → titik bawahnya bukan lagi kontak ban, jangan dipakai
            riwayat = td["riwayat"]
            if bottom_y < h_roi - 2:
                riwayat.append((frame_ts, float(pos_y)))
            while riwayat and frame_ts - riwayat[0][0] > JENDELA_S:
                riwayat.popleft()

            if len(riwayat) >= MIN_TITIK and riwayat[-1][0] - riwayat[0][0] >= MIN_DURASI_S:
                td["kecepatan"] = round(kecepatan_regresi(riwayat))
                td["maks"]      = max(td["maks"] or 0, td["kecepatan"])

            kecepatan_kmh = td["kecepatan"]
            is_ngebut     = kecepatan_kmh is not None and kecepatan_kmh > SPEED_THRESHOLD_KMH
            if is_ngebut:
                ada_ngebut = True

            if kecepatan_kmh is None:
                color, spd_label = (150, 150, 150), "mengukur..."
            else:
                color     = (0, 0, 255) if is_ngebut else (0, 255, 0)
                spd_label = f"{kecepatan_kmh} km/h"

            x1, y1 = int(cx - bw / 2), int(cy - bh / 2)
            x2, y2 = int(cx + bw / 2), int(cy + bh / 2)
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, f"ID:{track_id} {model.names[cls]} | {spd_label}",
                        (x1, y1 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

            if is_ngebut:
                cv2.putText(frame, "NGEBUT!", (x1, y1 - 24),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    # ---- Debounce NGEBUT ----
    ngebut_counter = min(ngebut_counter + 2, NGEBUT_DEBOUNCE_FRAMES + 2) if ada_ngebut \
                     else max(ngebut_counter - 1, 0)
    status_jalan   = "NGEBUT" if ngebut_counter >= NGEBUT_DEBOUNCE_FRAMES else "AMAN"

    # =========================================================
    # 8. LOGIKA KEPUTUSAN PANJANG COUNTDOWN
    # =========================================================
    riwayat_padat.append((now, kepadatan))
    while riwayat_padat and now - riwayat_padat[0][0] > KEPADATAN_JENDELA_S:
        riwayat_padat.popleft()
    kepadatan_halus = max(k for _, k in riwayat_padat)

    if kepadatan_halus >= PADAT_PADAT_MIN:
        kategori, target_timer = "PADAT", TIMER_PADAT
    elif kepadatan_halus >= PADAT_SEDANG_MIN:
        kategori, target_timer = "SEDANG", TIMER_SEDANG
    else:
        kategori, target_timer = "SEPI", TIMER_SEPI

    # =========================================================
    # 8.5 SIMULASI COUNTDOWN LOKAL (tombol SPASI)
    # =========================================================
    if is_crossing:
        # Kurangi angka setiap 1 detik waktu nyata
        if now - last_tick_time >= 1.0:
            countdown_val -= 1
            last_tick_time = now

        # Selesai nyeberang
        if countdown_val <= 0:
            is_crossing = False

    # ---- Laporan kecepatan + cleanup memori ----
    for tid in [t for t, td in track_data.items() if now - td["terlihat"] > HAPUS_TRACK_S]:
        td = track_data.pop(tid)
        if td["maks"] is not None:
            print(f"[SPEED] ID {tid} ({td['kelas']}): maks {td['maks']} km/h")

    # =================== 9. PUBLISH MQTT ===================
    # Format dibaca mqttCallback() di main.cpp: "timer" (detik) & "status" (NGEBUT/AMAN)
    if client.is_connected() and now - terakhir_publish >= PUBLISH_INTERVAL_S:
        terakhir_publish = now
        client.publish(MQTT_TOPIC_PUB, json.dumps({
            "kepadatan": kepadatan_halus,
            "kategori" : kategori,
            "status"   : status_jalan,
            "timer"    : target_timer
        }))

    # =================== 10. UI ===================
    # --- KALKULASI FPS REAL-TIME ---
    new_frame_time = time.time()
    if prev_frame_time != 0:
        fps_realtime = int(1 / max(new_frame_time - prev_frame_time, 1e-6))
    else:
        fps_realtime = 0
    prev_frame_time = new_frame_time

    # 1. Menampilkan FPS asli dari aliran RTSP (warna putih)
    cv2.putText(frame, f"RTSP: {int(video_fps)} FPS", (w_roi - 200, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    # 2. Menampilkan FPS pemrosesan AI di laptop (warna kuning)
    cv2.putText(frame, f"Proses: {fps_realtime} FPS", (w_roi - 200, 60),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

    # 3. Device & sumber waktu yang benar-benar dipakai
    cv2.putText(frame, f"{'GPU' if USE_GPU else 'CPU!'} | {'PTS kamera' if reader.use_pts else 'waktu terima'}",
                (w_roi - 200, 85), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

    # Penggaris kalibrasi (tiap 5 m) + tepi jalur + garis kalibrasi A/B
    geser = lambda p: (p[0] - roi_left, p[1] - roi_top)  # frame asli → frame ROI
    for p, q in penggaris:
        cv2.line(frame, geser(p), geser(q), (90, 90, 90), 1)
    if penggaris:
        cv2.line(frame, geser(penggaris[0][0]), geser(penggaris[-1][0]), (90, 90, 90), 1)
        cv2.line(frame, geser(penggaris[0][1]), geser(penggaris[-1][1]), (90, 90, 90), 1)

    for (p, q), nama, warna in [(garis_A, "A", (255, 120, 0)), (garis_B, "B", (0, 210, 210))]:
        cv2.line(frame, geser(p), geser(q), warna, 1)
        cv2.putText(frame, nama, (q[0] - roi_left + 5, q[1] - roi_top + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, warna, 1)

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
        cv2.putText(frame, f"Countdown: {target_timer}s ({kategori})", (20, 110),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, timer_color, 3)
        cv2.putText(frame, f"MQTT: {'OK' if client.is_connected() else 'belum tersambung'} | [SPASI] simulasi",
                    (20, 140), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 2)

    cv2.putText(frame,
                f"Kalibrasi A-B: {JARAK_KALIBRASI_METER}m | Garis abu: tiap 5m | Threshold: {SPEED_THRESHOLD_KMH}km/h",
                (10, h_roi - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (180, 180, 180), 1)

    cv2.imshow("Smart Safe Walking Path - Edge Analytics", frame)

    # Deteksi Keyboard
    key = cv2.waitKey(1) & 0xFF
    if key == ord('q'):
        break
    elif key == ord(' '):  # Tombol SPASI ditekan (simulasi lokal)
        if not is_crossing:
            is_crossing = True
            countdown_val = target_timer
            last_tick_time = now
            print(f"[SIMULASI] Menyeberang dengan jatah waktu {countdown_val} detik.")

# =================== 11. CLEANUP ===================
for tid, td in track_data.items():
    if td["maks"] is not None:
        print(f"[SPEED] ID {tid} ({td['kelas']}): maks {td['maks']} km/h")
reader.stop()
cv2.destroyAllWindows()
client.loop_stop()
client.disconnect()
print("[MQTT] Koneksi ditutup.")
