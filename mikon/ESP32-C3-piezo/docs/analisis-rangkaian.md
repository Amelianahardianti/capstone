# Analisis Rangkaian Pengkondisi Sinyal Piezo

> Status: **rancangan awal, belum diverifikasi di hardware.** Analisis ini memakai perhitungan teoretis. Pastikan dengan multimeter (dan osiloskop jika ada) sesuai Tahap 3 di README.

## 1. Rangkaian yang direncanakan

```
              Node A        R2 100kΩ        Node B
  Piezo (+) ────●──────────/\/\/\──────────●──────────────► GPIO0 (ADC)
                │                          │
              R1 1MΩ           ┌───────────┼────────────┐
                │              │           │            │
  Piezo (−) ────●            R3 10kΩ    D1 Zener     C1 100nF
                │              │        3,3V (K di B)    │
               GND            GND          GND          GND
```

## 2. Perhitungan

### Pembagi tegangan
Tegangan di Node B ≈ tegangan piezo × R3 / (R2 + R3) = × 10k / 110k ≈ **0,091** (sekitar 1/11).

| Tegangan piezo (Node A) | Node B (tanpa zener) | Keterangan |
|---|---|---|
| 5 V | 0,45 V | Ketukan ringan |
| 20 V | 1,8 V | Masih dalam rentang ADC |
| 27 V | 2,5 V | Batas atas ADC akurat (ESP32-C3, 11 dB) |
| 40 V | 3,6 V | **Melewati batas mutlak.** Zener harus bekerja |
| 100 V | 9 V | Hentakan keras. Zener wajib menahan |

Piezo disk yang diinjak bisa menghasilkan puluhan volt. Karena itu pembagi tegangan saja **tidak cukup**, dan proteksi harus benar-benar bekerja.

### Arus yang masuk ke proteksi
R2 (100 kΩ) membatasi arus: pada 100 V, arus ≈ 100 / 100k = **1 mA**. Ini kecil dan aman untuk zener maupun dioda. Jadi R2 sudah merupakan **pembatas arus yang baik**.

### Filter (C1)
C1 bersama R2 ∥ R3 (≈ 9,1 kΩ) membentuk low-pass filter:
- τ = 9,1 kΩ × 100 nF ≈ **0,91 ms**
- f₋₃dB ≈ 1 / (2π·τ) ≈ **175 Hz**

Akibatnya:
- Puncak ketukan yang sangat singkat (< 1 ms) ikut diredam. Ini justru baik untuk menekan noise dan lonjakan.
- Dengan sampling 200 Hz (frekuensi Nyquist 100 Hz), sebagian komponen 100–175 Hz bisa ikut terbaca sebagai frekuensi lain (*aliasing*), dan puncak yang sempit bisa terlewat di antara dua sampel. Untuk **injakan kaki** (puluhan sampai ratusan ms) hal ini tidak masalah. Untuk **ketukan jari** yang sangat singkat, pertimbangkan sampling 500–1000 Hz.

### Beban piezo dan R1
Beban total piezo = R1 ∥ (R2 + R3) = 1 MΩ ∥ 110 kΩ ≈ **99 kΩ**. Artinya jalur R2+R3 yang mendominasi, sedangkan R1 hampir tidak berpengaruh. R1 tetap berguna untuk membuang muatan piezo saat Node B belum tersambung.

Piezo bersifat kapasitif (umumnya 10–50 nF), jadi bersama beban ~99 kΩ ia membentuk high-pass filter (sekitar 30–160 Hz, tergantung kapasitansi piezo). Konsekuensi penting:

> **Piezo mendeteksi PERUBAHAN tekanan, bukan tekanan diam.** Saat kaki menginjak muncul lonjakan, dan saat kaki diangkat muncul lonjakan arah berlawanan (negatif). Orang yang **berdiri diam** di atas sensor **tidak** terdeteksi. Ini sifat fisik piezo, bukan bug firmware.

## 3. Risiko yang ditemukan ⚠️

### Risiko 1: tegangan NEGATIF (prioritas tinggi)
Saat kaki diangkat, piezo menghasilkan tegangan negatif. Zener yang dipasang dengan katoda di B dan anoda di GND akan **bekerja maju** dan menahan Node B di sekitar **−0,6 sampai −0,7 V**.

Batas mutlak pin ESP32-C3 adalah **−0,3 V**. Jadi **−0,7 V melanggar spesifikasi**. Arusnya memang kecil (dibatasi R2, < 1 mA), dan dioda ESD internal chip ikut menahan, sehingga kemungkinan besar chip tidak langsung rusak. Tetapi kondisi ini **di luar spesifikasi** dan bisa:
- membuat pembacaan ADC kanal lain kacau,
- menurunkan umur pin bila terjadi berulang-ulang (ribuan langkah per hari).

**Rekomendasi:** tambahkan **dioda Schottky** (BAT54, BAT85, 1N5819, atau BAT43) dari GND ke Node B, dengan **anoda di GND dan katoda di Node B**. Schottky mulai menghantar di sekitar 0,2–0,3 V, sehingga Node B tidak turun di bawah kira-kira −0,3 V.

### Risiko 2: zener 3,3 V "lunak"
Zener tegangan rendah (3,3 V) pada arus kecil tidak memotong tajam. Zener ini mulai bocor sejak sekitar 2 V. Akibatnya:
- puncak sinyal di atas ±2 V terbaca **lebih rendah** dari sebenarnya (terkompresi), sehingga hubungan "ketukan lebih keras = peak lebih tinggi" tidak linear di daerah atas,
- toleransi zener (±5%, yaitu 3,1–3,5 V) masih di bawah 3,6 V. Jadi untuk **proteksi** zener masih cukup.

**Rekomendasi (opsional, lebih baik):** ganti atau lengkapi dengan **dua Schottky ke rel catu**:
- Schottky 1: anoda Node B, katoda **3,3 V**, untuk menahan tegangan positif.
- Schottky 2: anoda GND, katoda Node B, untuk menahan tegangan negatif.

Dengan begitu tegangan Node B terkunci di antara −0,3 V dan 3,6 V dengan pemotongan yang tajam.

### Risiko 3: tidak ada resistor seri ke GPIO
**Rekomendasi:** pasang resistor **1 kΩ** antara Node B dan GPIO0. Resistor ini membatasi arus ke dioda internal chip jika proteksi luar gagal atau terlambat. Efeknya ke pembacaan ADC tidak berarti.

### Risiko 4: sinyal negatif hilang
ADC hanya bisa membaca 0–3,3 V. Setengah gelombang negatif dipotong (tampil sebagai 0). Untuk **deteksi pijakan** ini tidak masalah, karena lonjakan positif saat menginjak sudah cukup. Kalau nanti dibutuhkan bentuk gelombang penuh, sinyal perlu digeser ke tengah (bias ±1,65 V). Itu rancangan berbeda yang perlu dianalisis tersendiri.

## 4. Rangkaian yang disarankan

```
              Node A        R2 100kΩ        Node B        R4 1kΩ
  Piezo (+) ────●──────────/\/\/\──────────●─────────────/\/\/\────► GPIO0
                │                          │
              R1 1MΩ        ┌──────────┬───┴─────┬──────────┐
                │           │          │         │          │
  Piezo (−) ────●         R3 10kΩ   D1 Zener   D2 Schottky C1 100nF
                │           │       3,3V       (BAT54)      │
               GND         GND      (K di B)   (K di B)    GND
                                      GND        GND
```

| Komponen | Status | Fungsi |
|---|---|---|
| R1 1 MΩ | sudah ada | Membuang muatan piezo |
| R2 100 kΩ | sudah ada | Pembagi tegangan + pembatas arus |
| R3 10 kΩ | sudah ada | Pembagi tegangan |
| C1 100 nF | sudah ada | Low-pass filter (~175 Hz) |
| D1 Zener 3,3 V | sudah ada | Proteksi tegangan positif |
| **D2 Schottky** | **tambahan** | Proteksi tegangan negatif (< −0,3 V) |
| **R4 1 kΩ** | **tambahan** | Pembatas arus ke pin GPIO |

## 5. Hal yang belum diketahui (perlu diukur)
- Tegangan maksimum piezo saat diinjak orang dewasa. Ukur dengan osiloskop, tanpa ESP32.
- Kapasitansi piezo (dengan LCR meter atau multimeter berfitur kapasitansi).
- Jenis dan nomor seri zener yang dipakai, untuk memeriksa kurva bocornya di datasheet.
