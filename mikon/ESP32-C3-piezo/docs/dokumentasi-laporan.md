# Dokumentasi Modul Sensor Pijakan Piezoelectric

*Bahan untuk laporan Capstone — Smart Safe Walking Path (Tim B04)*

## 1. Tujuan
Modul ini mendeteksi pijakan pejalan kaki di area penyeberangan dengan sensor piezoelectric. Keluarannya berupa kejadian **"ada pijakan"** beserta amplitudo puncaknya. Informasi ini nantinya melengkapi deteksi pejalan kaki pada sistem.

## 2. Perangkat
| Komponen | Spesifikasi | Fungsi |
|---|---|---|
| Mikrokontroler | ESP32-C3 Super Mini (RISC-V 160 MHz, ADC 12-bit) | Akuisisi & pengolahan sinyal |
| Sensor | Piezoelectric disk | Mengubah tekanan/getaran menjadi tegangan |
| Pengkondisi sinyal | R1 1 MΩ, R2 100 kΩ, R3 10 kΩ, C1 100 nF, zener 3,3 V (+ usulan Schottky & R 1 kΩ) | Menurunkan tegangan, filter, proteksi pin |
| Antarmuka | USB CDC 115200 baud | Pemantauan & visualisasi |

Analisis rangkaian lengkap: [analisis-rangkaian.md](analisis-rangkaian.md).

## 3. Alur sistem
```
Piezo → Pembagi tegangan + filter + proteksi → ADC GPIO0 (12-bit, 200 Hz)
      → Baseline → Deteksi threshold (hysteresis) pada sinyal RAW → Hitung ketukan
      → Moving average (hanya untuk visualisasi/perbandingan)
      → Serial (grafik real-time + kejadian ketukan)
```

## 4. Metode pengolahan sinyal

### 4.1 Akuisisi
- Resolusi 12 bit (0–4095), atenuasi 11 dB (rentang akurat ±0–2,5 V).
- Sampling periodik **f_s = 200 Hz** (periode 5 ms), dijadwalkan dengan timer mikrodetik agar interval tetap.

### 4.2 Moving average (mode SMOOTH)
$$y[n] = \frac{1}{N}\sum_{k=0}^{N-1} x[n-k], \quad N = 8$$
Moving average menekan noise frekuensi tinggi, tetapi menurunkan tinggi puncak dan menambah keterlambatan sekitar (N−1)/2 sampel ≈ 17,5 ms. Mode **RAW** menampilkan x[n] tanpa filter untuk pembanding.

**Keputusan desain:** deteksi memakai sinyal **RAW**, bukan moving average. Pada uji simulasi (pulsa ketukan dengan konstanta waktu peluruhan 12 ms), moving average 8 sampel meredam puncak sekitar 3×, sehingga 1 dari 4 ketukan terlewat. Dengan sinyal RAW, 4/4 ketukan terdeteksi, sementara ketukan lemah dan pantulan tetap diabaikan. Noise sudah diatasi oleh threshold, hysteresis, dan jeda minimum, sehingga filter tidak diperlukan untuk deteksi.

### 4.3 Baseline
Baseline awal adalah rata-rata 1 detik pertama (sensor tidak disentuh). Selanjutnya baseline diperbarui dengan *exponential moving average*:
$$b[n] = b[n-1] + \alpha\,(x[n] - b[n-1]), \quad \alpha = 0{,}01$$
Pembaruan **hanya dilakukan saat sensor diam** (|x − b| < threshold), sehingga pijakan tidak menggeser baseline.

### 4.4 Deteksi pijakan
Amplitudo dihitung sebagai a[n] = x[n] − b[n] (x = sinyal RAW).
- **Mulai pijakan:** a[n] > T **dan** sudah lewat ≥ 150 ms sejak pijakan sebelumnya, lalu hitungan bertambah 1.
- **Selama pijakan:** catat nilai maksimum a[n] sebagai *peak*.
- **Akhir pijakan:** a[n] < 0,5·T (*hysteresis*).

Hysteresis dan jeda minimum mencegah satu pijakan terhitung berkali-kali akibat getaran sisa.

### 4.5 Satuan
Semua nilai dinyatakan dalam **satuan ADC**. Konversi ke gaya (N) atau massa (kg) **tidak dilakukan**, karena:
1. belum ada data kalibrasi beban–tegangan;
2. keluaran piezo bergantung pada **laju perubahan** gaya, bukan besar gaya statis;
3. zener 3,3 V membuat respons tidak linear pada amplitudo tinggi.

## 5. Parameter
| Parameter | Nilai | Dasar pemilihan |
|---|---|---|
| Sampling rate | 200 Hz | Pijakan berlangsung puluhan–ratusan ms, sehingga 200 Hz cukup |
| Jendela moving average | 8 sampel (40 ms) | Hanya untuk visualisasi |
| Sinyal deteksi | RAW | Moving average meredam puncak ketukan singkat (hasil simulasi) |
| α baseline | 0,01 | Konstanta waktu ≈ 100 sampel ≈ 0,5 s |
| Threshold T | 200 (awal) | **Ditentukan dari eksperimen Tahap 5** |
| Rasio release | 0,5 | Hysteresis 50% |
| Jeda minimum | 150 ms | Batas atas ≈ 6–7 pijakan/detik, jauh di atas laju langkah manusia (≈ 2/detik) |

## 6. Prosedur pengujian
| Tahap | Pengujian | Kriteria lolos |
|---|---|---|
| 1 | ESP32 tanpa sensor | Banner & kalibrasi tampil di Serial |
| 2 | Pembagi tegangan DC 1,65 V | Selisih ADC (mV) vs multimeter < ±100 mV |
| 3 | Rangkaian piezo diukur multimeter, tanpa ESP32 | Resistansi sesuai, Node B selalu antara −0,3 V dan 3,3 V |
| 4 | Piezo + ESP32, ketukan ringan | Lonjakan terlihat, hitungan bertambah |
| 5 | Kalibrasi threshold | Pijakan terdeteksi, gangguan tidak terdeteksi |

## 7. Tabel hasil (diisi saat pengujian)

### Tahap 2: Akurasi ADC
| No | Tegangan multimeter (mV) | Pembacaan ESP32 (mV) | Raw ADC | Selisih (mV) |
|---|---|---|---|---|
| 1 | | | | |
| 2 | | | | |
| 3 | | | | |

### Tahap 3: Verifikasi rangkaian
| Titik ukur | Perkiraan | Hasil ukur | OK? |
|---|---|---|---|
| Node A – GND | ≈ 99 kΩ | | |
| Node B – GND | ≈ 9,9 kΩ | | |
| Node A – Node B | ≈ 91 kΩ | | |
| Dioda GND→B | 0,2–0,7 V | | |
| V Node B maks saat diketuk | < 3,3 V | | |
| V Node B min saat dilepas | > −0,3 V | | |

### Tahap 5: Karakterisasi peak
| Jenis input | Percobaan | Peak (ADC) rata-rata | Peak min | Peak maks | Terdeteksi (T = …) |
|---|---|---|---|---|---|
| Ketukan jari ringan | 10× | | | | |
| Injakan pelan | 10× | | | | |
| Injakan normal | 10× | | | | |
| Hentakan keras | 10× | | | | |
| Gangguan (benda jatuh di dekat sensor) | 10× | | | | |
| Diam / tanpa input (noise) | 30 s | | | | |

Threshold dipilih di antara peak maksimum gangguan dan peak minimum injakan pelan. Akurasi deteksi = (pijakan terdeteksi benar) / (total pijakan).

## 8. Batasan
- Piezo **tidak mendeteksi orang yang berdiri diam**, hanya perubahan tekanan.
- Nilai peak dipengaruhi kecepatan dan posisi injakan, sehingga tidak bisa langsung dipakai sebagai ukuran berat.
- Rangkaian belum memotong tegangan negatif di bawah −0,3 V tanpa Schottky tambahan (lihat analisis rangkaian).
- Sampling 200 Hz bisa melewatkan ketukan jari yang sangat singkat. Untuk pijakan kaki hal ini tidak masalah.
