# Uji Sensor Piezo — ESP32-C3 Super Mini

Firmware untuk membaca sensor piezoelectric (deteksi pijakan pejalan kaki) pada proyek **Smart Safe Walking Path**. Firmware ini membaca ADC 12-bit, lalu menampilkan gelombangnya secara real-time. Selain itu firmware menghitung **baseline**, **peak**, **threshold**, dan **jumlah ketukan**.

> ⚠️ **Rangkaian sensor BELUM terverifikasi aman.** Kode yang berhasil di-build **tidak** berarti wiring aman. Ikuti tahapan pengujian di bawah **secara berurutan**, dan jangan memasang piezo ke ESP32 sebelum Tahap 3 lolos.

> Nilai yang ditampilkan adalah **satuan ADC (0–4095)**, bukan Newton atau kg. Konversi ke satuan fisik membutuhkan data kalibrasi.

---

## 1. Identitas board

| Item | Nilai |
|---|---|
| Board | ESP32-C3 Super Mini |
| Chip | ESP32-C3 (RISC-V 160 MHz, 4 MB flash) |
| Board PlatformIO | `esp32-c3-devkitm-1` (chip sama, kompatibel) |
| USB | **USB bawaan chip** (tanpa CH340/CP2102). Di Windows muncul sebagai *USB JTAG/serial debug unit* atau *USB Serial Device*, VID:PID `303A:1001` |
| LED bawaan | GPIO8 (menyala saat LOW) |
| Tombol | BOOT = GPIO9, RST = reset |
| Tegangan pin maks | **0 – 3,3 V**. Batas mutlak datasheet −0,3 V s/d 3,6 V |

### Pinout (cocokkan dengan tulisan di board kamu!)
Posisi: USB-C di atas, chip menghadap kamu. Susunan ini umum dipakai di Super Mini, tapi **clone bisa berbeda**. Yang jadi patokan adalah **tulisan yang tercetak di board**.

```
          [ USB-C ]
   5V  ●             ● 5
   G   ●             ● 6
   3.3 ●             ● 7
   4   ●             ● 8   (LED)
   3   ●             ● 9   (BOOT)
   2   ●             ● 10
   1   ●             ● 20  (RX)
   0   ●  <- ADC     ● 21  (TX)
```

- **GPIO0 = ADC1_CH0** (menurut datasheet Espressif) → dipakai untuk sensor.
- Pin ADC lain yang aman: GPIO1–GPIO4 (ADC1).
- **Hindari**: GPIO5 (ADC2, tidak andal), GPIO2/8/9 (pin *strapping* yang memengaruhi boot), GPIO18/19 (jalur USB).
- **Jangan pernah** menyambungkan pin **5V** ke rangkaian sensor.

### Cara memastikan chip-nya benar-benar ESP32-C3
Colokkan board, lalu jalankan di terminal PlatformIO. Perintah ini **hanya membaca**, tidak meng-upload apa pun:
```
pio device list
pio pkg exec -p tool-esptoolpy -- esptool.py --chip auto chip_id
```
Yang diharapkan: `Chip is ESP32-C3 ...` dan `Detected flash size: 4MB`.

---

## 2. Instalasi (sekali saja)

1. Install **VS Code**.
2. Install extension **PlatformIO IDE** (sudah terpasang di laptop ini).
3. Install extension **Teleplot** (`alexnesnes.teleplot`) untuk grafik real-time.
4. Buka folder `mikon/ESP32-C3-piezo` lewat **File → Open Folder**. Buka folder ini sendiri, **bukan** folder `mikon`, supaya PlatformIO mengenalinya.
5. Tunggu PlatformIO selesai menyiapkan proyek (lihat status bar di bawah). Unduhan pertama bisa memakan waktu beberapa menit.

---

## 3. Build, upload, dan monitor

Tombol PlatformIO ada di status bar bawah VS Code:

| Ikon | Fungsi |
|---|---|
| ✓ (centang) | **Build**: compile saja, aman |
| → (panah) | **Upload**: tulis firmware ke ESP32 |
| 🔌 (colokan) | **Serial Monitor**: lihat teks dari ESP32 |

Lewat terminal:
```
pio run                 # build
pio run -t upload       # upload
pio device monitor      # serial monitor (keluar: Ctrl+C)
```

### Memilih port COM
1. Cabut board, lalu buka **Device Manager → Ports (COM & LPT)**.
2. Colokkan board. Port yang **baru muncul** adalah port ESP32-C3, misalnya `USB Serial Device (COM8)`.
3. Laptop ini punya banyak port **"Standard Serial over Bluetooth link"** (COM3–COM14). **Itu bukan ESP32**, jangan dipilih.
4. Kalau PlatformIO salah memilih port, buka `platformio.ini` dan isi:
   ```ini
   upload_port = COM8
   monitor_port = COM8
   ```

### Kalau upload gagal ("Failed to connect", "No serial data received")
Masuk ke mode download secara manual:
1. **Tahan** tombol **BOOT**.
2. Tekan lalu lepas tombol **RST**, sambil BOOT tetap ditahan.
3. Lepas BOOT, lalu klik Upload lagi.
4. Setelah upload selesai, tekan **RST** sekali agar program berjalan.

---

## 4. Grafik real-time

### Pilihan A: Teleplot di VS Code (disarankan)
1. **Tutup Serial Monitor PlatformIO terlebih dahulu**, karena satu port hanya bisa dibuka satu aplikasi.
2. Tekan `Ctrl+Shift+P`, lalu pilih **Teleplot: Start**.
3. Pilih port COM ESP32 dan baud `115200`, lalu klik **Open**.
4. Grafik `raw`, `smooth`, `envelope`, `baseline`, `threshold`, `taps`, dan `peak` akan muncul.
   Tarik `threshold` ke grafik `envelope` supaya keduanya tampil di satu grafik.
5. Perintah bisa dikirim lewat kotak teks Teleplot, misalnya `r` lalu Enter.

### Pilihan B: Arduino IDE 2 Serial Plotter
1. Kirim perintah `p` (lewat Serial Monitor) untuk mengubah format ke **Arduino**.
2. Tutup Serial Monitor, lalu buka Arduino IDE 2 → **Tools → Serial Plotter**, port yang sama, baud 115200.

### Perintah serial (ketik huruf lalu Enter)
| Perintah | Fungsi |
|---|---|
| `r` | Tampilkan **RAW**: ADC asli, tanpa filter |
| `s` | Tampilkan **SMOOTH**: moving average |
| `b` | Tampilkan **BOTH**: keduanya sekaligus, untuk perbandingan (default) |
| `+` / `-` | Naikkan / turunkan threshold |
| `c` | Reset jumlah ketukan & peak |
| `k` | Kalibrasi ulang baseline (jangan sentuh sensor) |
| `i` | Info konfigurasi + 1 pembacaan tegangan (mV) |
| `q` | Hentikan / lanjutkan data plot (agar pesan teks mudah dibaca) |
| `p` | Ganti format: Teleplot ↔ Arduino Serial Plotter |
| `h` | Bantuan |

### Cara membaca grafik
```
ADC
4095 ┤                                  ← jenuh (sinyal terlalu besar)
     │        ╭╮
thr  ┤- - - - ││- - - - - - -╭╮- - - -  ← garis threshold = baseline + threshold
     │        ││    ╭╮       ││
base ┤────────╯╰────╯╰───────╯╰──────   ← baseline (sensor diam)
   0 ┼──────────────────────────────▶ waktu
              ↑      ↑       ↑
         terhitung  TIDAK   terhitung
                  (di bawah threshold)
```
- **raw** (garis kasar): nilai ADC apa adanya, termasuk noise. Grafik menerima 1 titik per `PLOT_EVERY_N` sampel, berisi nilai maksimum dari sampel-sampel itu, jadi puncak ketukan tidak hilang.
- **envelope**: tinggi sinyal yang langsung naik saat ada ketukan lalu turun perlahan. Ketukan yang sangat singkat terlihat sebagai "bukit" lebar. Kalau bukitnya melewati garis threshold, ketukan itu terhitung.
- **smooth** (garis halus): rata-rata beberapa sampel terakhir. Noise berkurang, tapi puncak sedikit lebih rendah dan sedikit terlambat.
- **baseline**: nilai saat sensor diam. Seharusnya rata dan dekat 0, karena piezo hanya menghasilkan tegangan positif saat ditekan dan rangkaian memotong sisi negatifnya.
- Deteksi ketukan selalu memakai sinyal **raw** (lihat `DETECT_ON_SMOOTH`). Mode `r`/`s`/`b` hanya mengubah garis yang ditampilkan.
- **threshold**: batas deteksi. Sinyal yang melewati garis ini dihitung sebagai 1 ketukan, dan `taps` bertambah 1.
- **peak**: tinggi puncak setiap ketukan, diukur dari baseline.
- Garis menempel di **4095** berarti sinyal terlalu besar (jenuh), jadi kekuatan ketukan tidak bisa dibedakan lagi.

---

## 5. Tahapan pengujian (WAJIB berurutan)

### Tahap 1: ESP32 saja, tanpa sensor
Tujuan: memastikan upload dan Serial berfungsi.
1. **Tidak ada** kabel apa pun di pin board, hanya USB.
2. Upload firmware, lalu buka Serial Monitor.
3. ✅ Lolos jika: muncul banner `Smart Safe Walking Path - Uji Sensor Piezo`, daftar perintah, dan `[KALIBRASI] Baseline = ...`.
4. Ketik `i`. Pembacaan `raw` akan acak, karena pin GPIO0 mengambang. Ini **normal**.

### Tahap 2: Uji ADC dengan tegangan DC yang aman
Tujuan: memastikan ADC membaca tegangan dengan benar sebelum memakai piezo.

Rangkaian pembagi tegangan (cabut USB saat merangkai):
```
3.3 ──[10kΩ]──┬──[10kΩ]── G
              │
              └──[1kΩ]── GPIO0
```
1. Colok USB, lalu ketik `i`.
2. Ukur tegangan titik tengah dengan multimeter, harusnya sekitar **1,65 V**.
3. ✅ Lolos jika `tegangan=... mV` di Serial selisihnya kurang dari ±100 mV dari multimeter.
4. Opsional: pakai **potensiometer 10 kΩ** (kaki ujung ke 3.3 dan G, kaki tengah lewat 1 kΩ ke GPIO0). Putar, lalu lihat grafik `raw` naik turun dengan mulus.

> Catatan ESP32-C3: pembacaan akurat sampai sekitar **2,5 V**. Di atas itu nilai mendekati 4095 (jenuh). Ini normal.

### Tahap 3: Verifikasi rangkaian piezo dengan multimeter (TANPA ESP32)
**Jangan sambungkan Node B ke GPIO0 dulu.** Baca juga [docs/analisis-rangkaian.md](docs/analisis-rangkaian.md), terutama soal proteksi tegangan negatif yang **perlu ditambah**.

Ukur dengan semua catu daya terlepas:

| Ukur | Mode multimeter | Perkiraan |
|---|---|---|
| Node A ↔ GND | Ohm | ≈ 99 kΩ (R1 ∥ (R2+R3)) |
| Node B ↔ GND (merah di B) | Ohm | ≈ 9,9 kΩ (R3 ∥ (R2+R1)) |
| Node A ↔ Node B | Ohm | ≈ 91 kΩ |
| Diode: merah di GND, hitam di Node B | Diode | ≈ 0,6–0,7 V (zener arah maju). ≈ 0,2–0,3 V jika Schottky sudah dipasang |
| Diode: merah di Node B, hitam di GND | Diode | "OL" / tidak terbaca (arah balik) |
| 3.3 ↔ GND di ESP32 | Ohm / buzzer | **Tidak boleh** bunyi atau 0 Ω |

Angka bisa sedikit bergeser karena toleransi resistor dan kapasitor C1 yang sedang terisi. Tunggu beberapa detik sampai angkanya stabil.

Uji tegangan, masih **tanpa ESP32**:
1. Hubungkan multimeter (mode DC V) antara **Node B** dan **GND**.
2. Ketuk piezo **pelan**.
3. ✅ Lolos jika tegangan di Node B **tidak pernah lebih dari 3,3 V** dan **tidak lebih negatif dari −0,3 V**. Multimeter mungkin terlalu lambat untuk menangkap puncak yang singkat. Kalau ada, osiloskop jauh lebih baik.
4. Ketuk lebih keras secara bertahap dan amati.

### Tahap 4: Piezo + ESP32, ketukan ringan
**Hanya setelah Tahap 3 lolos.**
1. Cabut USB, lalu sambungkan Node B ke GPIO0 (disarankan lewat resistor 1 kΩ) dan GND rangkaian ke **G** ESP32.
2. Colok USB, lalu buka Teleplot.
3. Ketuk sensor **ringan** dengan jari.
4. ✅ Lolos jika: baseline stabil dekat 0, ada lonjakan saat diketuk, LED biru berkedip, dan `taps` bertambah.
5. Bandingkan bentuk gelombang di mode `r` (RAW) dan `s` (SMOOTH).

### Tahap 5: Eksperimen threshold & kalibrasi
1. Catat `peak` untuk beberapa jenis input: ketukan jari, injakan pelan, injakan normal, dan getaran lain (benda jatuh, langkah orang di dekat sensor).
2. Atur threshold dengan `+`/`-` sampai injakan terhitung tetapi getaran kecil tidak.
3. Catat semua hasilnya di tabel pada [docs/dokumentasi-laporan.md](docs/dokumentasi-laporan.md).
4. Setelah nilai bagus ditemukan, ubah `THRESHOLD_DEFAULT` di `src/main.cpp`.

---

## 6. Parameter yang bisa diubah (`src/main.cpp`, bagian 1)

| Parameter | Default | Arti |
|---|---|---|
| `ADC_PIN` | 0 | Pin sensor |
| `SAMPLE_RATE_HZ` | 200 | Sampel per detik |
| `PLOT_EVERY_N` | 4 | Kirim 1 titik plot per N sampel (200 Hz / 4 = 50 titik/detik). Lebih besar = grafik lebih lambat |
| `ENVELOPE_DECAY` | 0.98 | Kecepatan envelope turun setelah puncak. Makin dekat 1 = makin lambat |
| `SMOOTH_WINDOW` | 8 | Jumlah sampel moving average (tampilan) |
| `DETECT_ON_SMOOTH` | false | Deteksi ketukan memakai RAW. `true` = pakai SMOOTH (puncak teredam, ketukan singkat bisa terlewat) |
| `BASELINE_ALPHA` | 0.01 | Kecepatan baseline mengikuti perubahan lambat |
| `BASELINE_CALIB_MS` | 1000 | Lama kalibrasi baseline saat start |
| `THRESHOLD_DEFAULT` | 200 | Batas deteksi (satuan ADC di atas baseline). **Tebakan awal** |
| `RELEASE_RATIO` | 0.5 | Ketukan selesai saat sinyal di bawah 50% threshold |
| `MIN_TAP_INTERVAL_MS` | 150 | Jeda minimum antar ketukan |
| `USE_LED` | true | LED berkedip saat ketukan |

---

## 7. Troubleshooting

| Masalah | Penyebab & solusi |
|---|---|
| Build gagal `riscv32-esp-elf-g++: error: CreateProcess: No such file or directory` atau `Access is denied` | Antivirus (laptop ini punya McAfee + Reason + Defender) menahan compiler. **Klik Build sekali lagi**, biasanya berhasil. Solusi permanen: tambahkan folder `C:\Users\ASUS\.platformio` ke daftar pengecualian (exclusion) antivirus |
| Port ESP32 tidak muncul | Kabel USB hanya untuk charging, jadi ganti kabel data. Atau masuk mode download (tahan BOOT + tekan RST) |
| Upload: `Failed to connect to ESP32-C3` | Lakukan langkah BOOT + RST (bagian 3) |
| Upload: `could not open port ... Access is denied` | Port sedang dipakai Serial Monitor atau Teleplot. Tutup dulu |
| Serial Monitor kosong | Tekan **RST**. Pastikan `ARDUINO_USB_CDC_ON_BOOT=1` ada di `platformio.ini` |
| Huruf acak di Serial Monitor | Baud rate harus 115200 |
| Port COM berganti setelah upload / reset | Normal untuk USB bawaan ESP32-C3. Pilih ulang port |
| Grafik Teleplot tidak muncul | Format harus Teleplot (kirim `p` untuk mengganti). Pastikan Serial Monitor PlatformIO sudah ditutup |
| `raw` acak padahal sensor terpasang | Kabel GND tidak tersambung, atau sambungan longgar |
| Baseline naik turun terus | Noise, kabel terlalu panjang, atau GND kurang baik. Coba mode SMOOTH dan perpendek kabel |
| Satu ketukan terhitung 2× atau lebih | Naikkan `MIN_TAP_INTERVAL_MS` atau threshold |
| Ketukan ringan tidak terhitung | Turunkan threshold. Atau puncaknya terlalu singkat untuk 200 Hz: coba `SAMPLE_RATE_HZ = 500`. Pastikan `DETECT_ON_SMOOTH = false` |
| Grafik menempel di 4095 | Sinyal jenuh. Ketuk lebih pelan, atau perbesar pembagi tegangan (R2 lebih besar) |
| Board panas / bau | **Cabut USB segera.** Periksa korsleting 3.3 ↔ GND |
