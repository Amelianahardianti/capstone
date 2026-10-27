import os
import sys
import cv2
import json

# ===============================================================
# KALIBRASI PERSPEKTIF untuk testvid_fixed.py
# Klik 4 titik di TEPI jalur yang dipantau (frame kamera asli), urutan:
#   1. A-kiri   2. A-kanan   3. B-kanan   4. B-kiri
# A dan B = dua garis melintang jalan yang jarak nyatanya diketahui (diukur pakai meteran).
# Kiri/kanan = dua tepi jalan yang SEJAJAR arah jalan (mis. tepi median dan kanstin luar).
#
# Klik kiri = tambah titik | klik kanan = hapus titik terakhir | S/Enter = simpan | Q/Esc = batal
# Hasil disimpan ke kalibrasi.json (otomatis dipakai testvid_fixed.py).
#
# Pemakaian: python kalibrasi.py            → ambil frame dari RTSP kamera
#            python kalibrasi.py video.mp4  → ambil frame dari file video
# ===============================================================

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
OUT_FILE = os.path.join(BASE_DIR, "kalibrasi.json")

# URL RTSP dibaca dari rtsp_url.txt (sama dengan testvid_fixed.py)
RTSP_FILE = os.path.join(BASE_DIR, "rtsp_url.txt")
rtsp_url  = open(RTSP_FILE).read().strip() if os.path.exists(RTSP_FILE) else ""
if len(sys.argv) <= 1 and not rtsp_url:
    print(f"[ERROR] Isi {RTSP_FILE} dengan URL RTSP, atau jalankan: python kalibrasi.py video.mp4")
    exit()
source   = sys.argv[1] if len(sys.argv) > 1 else rtsp_url

cap = cv2.VideoCapture(source)
ok, frame = cap.read()
cap.release()
if not ok:
    print(f"[ERROR] Gagal mengambil frame dari {source}")
    exit()

h, w = frame.shape[:2]
NAMA  = ["A-kiri", "A-kanan", "B-kanan", "B-kiri"]
titik = []


def klik(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN and len(titik) < 4:
        titik.append((x, y))
    elif event == cv2.EVENT_RBUTTONDOWN and titik:
        titik.pop()


WIN = "Kalibrasi - klik A-kiri, A-kanan, B-kanan, B-kiri"
cv2.namedWindow(WIN)
cv2.setMouseCallback(WIN, klik)

while True:
    img = frame.copy()
    for i, (x, y) in enumerate(titik):
        cv2.circle(img, (x, y), 5, (0, 0, 255), -1)
        cv2.putText(img, NAMA[i], (x + 8, y - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    if len(titik) > 1:
        for p, q in zip(titik, titik[1:] + (titik[:1] if len(titik) == 4 else [])):
            cv2.line(img, p, q, (0, 255, 255), 1)

    info = f"Klik titik: {NAMA[len(titik)]}" if len(titik) < 4 else "S/Enter = simpan"
    cv2.putText(img, f"{info} | klik kanan = undo | Q = batal", (10, h - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.imshow(WIN, img)

    key = cv2.waitKey(30) & 0xFF
    if key in (ord('q'), 27):
        print("[KALIBRASI] Dibatalkan, tidak ada yang disimpan.")
        cv2.destroyAllWindows()
        exit()
    if key in (ord('s'), 13) and len(titik) == 4:
        break

cv2.destroyAllWindows()

masukan = input("Jarak nyata garis A ke garis B (meter) [13.5]: ").strip()
jarak   = float(masukan) if masukan else 13.5

with open(OUT_FILE, "w") as f:
    json.dump({"titik": titik, "ukuran_frame": [w, h], "jarak_meter": jarak}, f, indent=2)
print(f"[KALIBRASI] Tersimpan di {OUT_FILE}: {titik}, A-B = {jarak} m")
