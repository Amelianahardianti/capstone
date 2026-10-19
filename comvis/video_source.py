import os
import time
import threading
from urllib.parse import urlsplit, urlunsplit, parse_qsl, urlencode
import cv2

# ===============================================================
# Sumber video untuk pipeline CV: file lokal atau live RTSP stream.
#
#   src = open_source("aman.mp4")          # file (perilaku lama)
#   src = open_source("rtsp://...")        # live stream
#   ok, frame, t = src.read()              # t = timestamp frame (detik)
#
# File  : t = posisi video (CAP_PROP_POS_MSEC), EOF -> ok=False, panggil rewind()
# RTSP  : t = waktu dunia nyata (monotonic) saat frame diterima.
#         Thread latar membaca terus & hanya menyimpan frame terbaru
#         (latency rendah). read() -> ok=False jika belum ada frame baru.
#         Putus / gagal baca -> reconnect otomatis dengan backoff.
# ===============================================================

LIVE_PREFIXES = ("rtsp://", "rtsps://", "rtmp://", "http://", "https://")


def is_live_source(source):
    return str(source).lower().startswith(LIVE_PREFIXES)


def open_source(source, **kwargs):
    if is_live_source(source):
        return RTSPSource(source, **kwargs)
    return FileSource(source)


def mask_url(url):
    """Sembunyikan path & query (token / kredensial) agar tidak tercetak di log."""
    p = urlsplit(url)
    host = p.hostname or ""
    if p.port:
        host += f":{p.port}"
    return f"{p.scheme}://{host}/***"


def normalize_url(url):
    """Buang parameter ?enableSrtp (UniFi Protect). FFmpeg tidak mendukung SRTP,
    stream tetap terenkripsi lewat TLS (rtsps) tanpa parameter ini."""
    p = urlsplit(url)
    query = [(k, v) for k, v in parse_qsl(p.query, keep_blank_values=True)
             if k.lower() != "enablesrtp"]
    return urlunsplit(p._replace(query=urlencode(query)))


def candidate_urls(url):
    """Daftar URL yang dicoba bergiliran. Untuk UniFi Protect (rtsps:7441)
    ditambah cadangan RTSP tanpa enkripsi di port 7447 dengan path yang sama."""
    clean = normalize_url(url)
    urls = [clean]
    p = urlsplit(clean)
    if p.scheme.lower() == "rtsps" and p.port == 7441:
        netloc = p.netloc.rsplit(":", 1)[0] + ":7447"
        urls.append(urlunsplit(p._replace(scheme="rtsp", netloc=netloc)))
    return urls


class FileSource:
    is_live = False

    def __init__(self, path):
        self.path = path
        self.cap = cv2.VideoCapture(path)
        if not self.cap.isOpened():
            raise IOError(f"Gagal membuka video: {path}")
        self.fps = self.cap.get(cv2.CAP_PROP_FPS)
        if self.fps <= 0:
            self.fps = 30.0
        self.reconnect_count = 0

    def read(self):
        ret, frame = self.cap.read()
        if not ret:
            return False, None, None
        return True, frame, self.cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0

    def rewind(self):
        self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)

    def status_text(self):
        return ""

    def release(self):
        self.cap.release()


class RTSPSource:
    is_live = True

    def __init__(self, url, transport="tcp", open_timeout_s=10,
                 read_timeout_s=5, stall_timeout_s=8, max_backoff_s=30):
        self.urls = candidate_urls(url)
        self.url = self.urls[0]       # URL yang sedang dipakai
        if url != self.url:
            print("[RTSP] Parameter enableSrtp dibuang (tidak didukung FFmpeg)")
        # RTSPS = RTSP di atas TLS, hanya bisa lewat TCP
        if url.lower().startswith("rtsps://") and transport != "tcp":
            print("[RTSP] rtsps:// butuh transport TCP, --rtsp-transport diabaikan")
            transport = "tcp"
        self.transport = transport
        self.open_timeout_ms = int(open_timeout_s * 1000)
        self.read_timeout_ms = int(read_timeout_s * 1000)
        self.stall_timeout_s = stall_timeout_s
        self.max_backoff_s = max_backoff_s

        self.fps = 30.0
        self.reconnect_count = 0      # naik setiap koneksi ulang berhasil
        self.connected = False
        self.last_error = ""

        self._lock = threading.Lock()
        self._frame = None
        self._frame_time = None
        self._frame_seq = 0
        self._read_seq = 0
        self._last_frame_wall = 0.0
        self._stopped = False

        # Transport TCP lebih stabil (tanpa packet loss / artefak) dibanding UDP
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = f"rtsp_transport;{transport}"

        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    # ---------------- thread pembaca ----------------
    def _open(self):
        params = [cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, self.open_timeout_ms,
                  cv2.CAP_PROP_READ_TIMEOUT_MSEC, self.read_timeout_ms]
        cap = cv2.VideoCapture(self.url, cv2.CAP_FFMPEG, params)
        if not cap.isOpened():
            cap.release()
            return None
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        fps = cap.get(cv2.CAP_PROP_FPS)
        if 0 < fps <= 120:
            self.fps = fps
        return cap

    def _run(self):
        backoff = 1.0
        first = True
        idx = 0
        while not self._stopped:
            self.url = self.urls[idx]
            print(f"[RTSP] Menghubungkan ke {mask_url(self.url)} ...")
            cap = self._open()
            if cap is None:
                self.last_error = "connect gagal"
                idx = (idx + 1) % len(self.urls)
                if idx != 0:
                    # Masih ada URL cadangan: langsung coba tanpa menunggu
                    print(f"[RTSP] Gagal connect. Coba URL cadangan {mask_url(self.urls[idx])}")
                    continue
                print(f"[RTSP] Gagal connect. Coba lagi dalam {backoff:.0f}s")
                self._sleep(backoff)
                backoff = min(backoff * 2, self.max_backoff_s)
                continue

            self.connected = True
            self.last_error = ""
            backoff = 1.0
            if not first:
                self.reconnect_count += 1
            first = False
            self._last_frame_wall = time.monotonic()
            print(f"[RTSP] Terhubung ({cap.get(3):.0f}x{cap.get(4):.0f} @ {self.fps:.1f} FPS)")

            fail_streak = 0
            while not self._stopped:
                ok, frame = cap.read()
                now = time.monotonic()
                if ok and frame is not None:
                    fail_streak = 0
                    with self._lock:
                        self._frame = frame
                        self._frame_time = now
                        self._frame_seq += 1
                    self._last_frame_wall = now
                    continue

                fail_streak += 1
                stalled = now - self._last_frame_wall > self.stall_timeout_s
                if fail_streak >= 30 or stalled:
                    self.last_error = "stream terputus"
                    print(f"[RTSP] Stream terputus ({fail_streak} gagal baca, "
                          f"{now - self._last_frame_wall:.1f}s tanpa frame). Reconnect...")
                    break
                time.sleep(0.05)

            self.connected = False
            cap.release()
            if not self._stopped:
                self._sleep(backoff)

    def _sleep(self, seconds):
        end = time.monotonic() + seconds
        while not self._stopped and time.monotonic() < end:
            time.sleep(0.1)

    # ---------------- API untuk loop utama ----------------
    def read(self):
        """(True, frame, t) hanya jika ada frame BARU sejak read() terakhir."""
        with self._lock:
            if self._frame is None or self._frame_seq == self._read_seq:
                return False, None, None
            self._read_seq = self._frame_seq
            return True, self._frame, self._frame_time

    def status_text(self):
        if self.connected:
            return ""
        return f"RTSP: menghubungkan... {self.last_error}".strip()

    def release(self):
        self._stopped = True
        self._thread.join(timeout=self.read_timeout_ms / 1000 + 2)
