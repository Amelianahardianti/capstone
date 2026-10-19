// ===============================================================
// Smart Safe Walking Path — Firmware Uji Sensor Piezoelectric
// Board  : ESP32-C3 Super Mini (PlatformIO: esp32-c3-devkitm-1)
// Fungsi : membaca ADC 12-bit, menampilkan gelombang real-time,
//          menghitung baseline, puncak (peak), dan jumlah ketukan.
//
// CATATAN: nilai yang ditampilkan adalah SATUAN ADC (0–4095),
// BUKAN gaya (Newton) atau berat (kg). Konversi ke satuan fisik
// membutuhkan data kalibrasi yang belum ada.
// ===============================================================

#include <Arduino.h>

// =================== 1. KONFIGURASI (boleh diubah) ===================

// Pin ADC. GPIO0 = ADC1_CH0 pada chip ESP32-C3 (sesuai datasheet Espressif).
// Pada Super Mini, pin bertuliskan "0" di board. Pin ADC aman lainnya: 1, 2, 3, 4.
// JANGAN pakai GPIO5 (ADC2, tidak andal di ESP32-C3 saat dipakai bersama WiFi).
const uint8_t ADC_PIN = 0;

// Frekuensi sampling (Hz). 100–200 Hz untuk tahap awal.
const uint32_t SAMPLE_RATE_HZ = 200;

// Data plot dikirim setiap N sampel (200 Hz / 4 = 50 titik per detik) agar grafik
// bergerak lebih lambat. Nilai raw yang dikirim adalah MAKSIMUM dari N sampel itu,
// sehingga puncak ketukan yang singkat tidak hilang. Deteksi tetap memakai semua sampel.
const uint8_t PLOT_EVERY_N = 4;

// Envelope: tinggi sinyal yang turun perlahan setelah puncak. Ketukan yang hanya
// 1–2 sampel tampil sebagai "bukit" lebar (±0,5 detik) di grafik.
// Per sampel; makin dekat ke 1 = makin lambat turun.
const float ENVELOPE_DECAY = 0.98f;

// Jumlah sampel untuk moving average (mode SMOOTH). Lebih besar = lebih halus
// tetapi puncak ketukan ikut "tumpul" dan sedikit terlambat.
const uint8_t SMOOTH_WINDOW = 8;

// Sinyal untuk DETEKSI ketukan. false = RAW (disarankan): moving average
// meredam puncak ketukan yang singkat (±3× pada uji simulasi) sehingga
// ketukan sedang bisa terlewat. Mode r/s/b hanya mengatur TAMPILAN grafik.
const bool DETECT_ON_SMOOTH = false;

// Baseline (nilai diam/tanpa tekanan) dihitung dengan rata-rata bergerak
// eksponensial. Nilai kecil = baseline berubah sangat lambat.
const float BASELINE_ALPHA = 0.01f;

// Lama kalibrasi baseline saat start (ms). Jangan sentuh sensor selama ini.
const uint32_t BASELINE_CALIB_MS = 1000;

// Threshold deteksi ketukan, dalam satuan ADC DI ATAS baseline.
// Nilai awal ini hanya tebakan — WAJIB disesuaikan lewat eksperimen (Tahap 5).
const int THRESHOLD_DEFAULT = 40;
const int THRESHOLD_STEP    = 20;    // perubahan per tombol '+' / '-'

// Ketukan dianggap selesai jika sinyal turun di bawah (threshold × rasio ini).
// Selisih antara batas naik dan turun (hysteresis) mencegah 1 ketukan
// terhitung berkali-kali karena sinyal bergetar di sekitar threshold.
const float RELEASE_RATIO = 0.5f;

// Jeda minimum antar ketukan (ms) supaya getaran sisa tidak dihitung lagi.
const uint32_t MIN_TAP_INTERVAL_MS = 150;

// LED bawaan Super Mini di GPIO8 (menyala saat LOW). Menyala saat ada ketukan.
const bool    USE_LED        = true;
const uint8_t LED_PIN        = 8;
const bool    LED_ACTIVE_LOW = true;

// =================== 2. STATE PROGRAM ===================

enum Mode { MODE_RAW, MODE_SMOOTH, MODE_BOTH };
enum PlotFormat { FORMAT_TELEPLOT, FORMAT_ARDUINO };

Mode       mode       = MODE_BOTH;
PlotFormat plotFormat = FORMAT_TELEPLOT;
bool       plotOn     = true;   // false = data plot dihentikan, hanya pesan teks

int threshold = THRESHOLD_DEFAULT;

// Buffer moving average
uint16_t smoothBuf[SMOOTH_WINDOW];
uint8_t  smoothIdx = 0;
uint32_t smoothSum = 0;

float    baseline     = 0;
bool     inTap        = false;
uint32_t tapCount     = 0;
int      tapPeak      = 0;   // puncak ketukan yang sedang berlangsung
int      lastTapPeak  = 0;   // puncak ketukan terakhir yang selesai
int      maxPeak      = 0;   // puncak terbesar sejak reset
uint32_t lastTapMs    = 0;

float    envelope     = 0;   // di atas baseline, turun perlahan (lihat ENVELOPE_DECAY)
int      plotRawMax   = 0;   // raw maksimum sejak titik plot terakhir
uint8_t  plotCounter  = 0;

const uint32_t SAMPLE_PERIOD_US = 1000000UL / SAMPLE_RATE_HZ;
uint32_t nextSampleUs = 0;

// =================== 3. FUNGSI BANTU ===================

void setLed(bool on) {
  if (!USE_LED) return;
  digitalWrite(LED_PIN, (on ^ LED_ACTIVE_LOW) ? HIGH : LOW);
}

const char* modeName() {
  switch (mode) {
    case MODE_RAW:    return "RAW";
    case MODE_SMOOTH: return "SMOOTH";
    default:          return "BOTH";
  }
}

// Pesan teks hanya dicetak di format Teleplot: Serial Plotter Arduino
// akan salah membaca teks biasa sebagai data.
bool textAllowed() { return plotFormat == FORMAT_TELEPLOT || !plotOn; }

void printHelp() {
  Serial.println();
  Serial.println("=== PERINTAH (ketik huruf lalu Enter) ===");
  Serial.println(" r : tampilkan RAW    (ADC asli, tanpa filter)");
  Serial.println(" s : tampilkan SMOOTH (moving average)");
  Serial.println(" b : tampilkan BOTH   (RAW + SMOOTH bersamaan)");
  Serial.println(" + : naikkan threshold   - : turunkan threshold");
  Serial.println(" c : reset jumlah ketukan & peak");
  Serial.println(" k : kalibrasi ulang baseline (jangan sentuh sensor)");
  Serial.println(" i : info konfigurasi + 1 pembacaan tegangan (mV)");
  Serial.println(" p : ganti format plot (Teleplot <-> Arduino Serial Plotter)");
  Serial.println(" q : hentikan/lanjutkan data plot (agar pesan [KETUKAN] mudah dibaca)");
  Serial.println(" h : tampilkan bantuan ini");
  Serial.println();
}

void printInfo() {
  int raw = analogRead(ADC_PIN);
  int mv  = analogReadMilliVolts(ADC_PIN);
  Serial.println();
  Serial.println("=== INFO ===");
  Serial.printf(" ADC pin        : GPIO%u\n", ADC_PIN);
  Serial.printf(" Sampling       : %lu Hz\n", (unsigned long)SAMPLE_RATE_HZ);
  Serial.printf(" Titik plot     : %lu per detik\n", (unsigned long)(SAMPLE_RATE_HZ / PLOT_EVERY_N));
  Serial.printf(" Mode           : %s\n", modeName());
  Serial.printf(" Data plot      : %s\n", plotOn ? "ON" : "OFF (tekan q)");
  Serial.printf(" Smooth window  : %u sampel\n", SMOOTH_WINDOW);
  Serial.printf(" Deteksi pakai  : %s\n", DETECT_ON_SMOOTH ? "SMOOTH" : "RAW");
  Serial.printf(" Baseline       : %.1f\n", baseline);
  Serial.printf(" Threshold      : %d (di atas baseline)\n", threshold);
  Serial.printf(" Jumlah ketukan : %lu\n", (unsigned long)tapCount);
  Serial.printf(" Peak terakhir  : %d   Peak maks: %d\n", lastTapPeak, maxPeak);
  Serial.printf(" Pembacaan kini : raw=%d  tegangan=%d mV\n", raw, mv);
  Serial.println();
}

// Kalibrasi baseline: rata-rata pembacaan selama BASELINE_CALIB_MS.
void calibrateBaseline() {
  if (textAllowed()) {
    Serial.printf("[KALIBRASI] Mengukur baseline %lu ms, JANGAN sentuh sensor...\n",
                  (unsigned long)BASELINE_CALIB_MS);
  }
  uint32_t sum = 0, n = 0;
  uint32_t start = millis();
  while (millis() - start < BASELINE_CALIB_MS) {
    sum += analogRead(ADC_PIN);
    n++;
    delay(2);
  }
  baseline = (float)sum / n;

  // Isi buffer moving average dengan baseline agar tidak mulai dari 0
  smoothSum = 0;
  for (uint8_t i = 0; i < SMOOTH_WINDOW; i++) {
    smoothBuf[i] = (uint16_t)baseline;
    smoothSum += smoothBuf[i];
  }
  inTap      = false;
  envelope   = 0;
  plotRawMax = 0;
  setLed(false);
  if (textAllowed()) {
    Serial.printf("[KALIBRASI] Baseline = %.1f (dari %lu sampel)\n", baseline, (unsigned long)n);
  }
}

void handleCommand(char c) {
  switch (c) {
    case 'r': mode = MODE_RAW;    break;
    case 's': mode = MODE_SMOOTH; break;
    case 'b': mode = MODE_BOTH;   break;
    case '+': threshold += THRESHOLD_STEP; break;
    case '-': threshold = max(THRESHOLD_STEP, threshold - THRESHOLD_STEP); break;
    case 'c': tapCount = 0; lastTapPeak = 0; maxPeak = 0; break;
    case 'k': calibrateBaseline(); return;
    case 'i': printInfo(); return;
    case 'h': printHelp(); return;
    case 'q': plotOn = !plotOn; break;
    case 'p':
      plotFormat = (plotFormat == FORMAT_TELEPLOT) ? FORMAT_ARDUINO : FORMAT_TELEPLOT;
      break;
    default: return;   // abaikan Enter, spasi, dll.
  }
  if (textAllowed()) {
    Serial.printf("[SET] mode=%s threshold=%d format=%s ketukan=%lu\n", modeName(), threshold,
                  plotFormat == FORMAT_TELEPLOT ? "Teleplot" : "Arduino", (unsigned long)tapCount);
  }
}

// Kirim satu titik data ke plotter
void plotSample(int raw, int smooth) {
  int base = (int)baseline;
  int thr  = base + threshold;
  int env  = base + (int)envelope;   // satuan sama dengan threshold, bisa digabung 1 grafik

  if (plotFormat == FORMAT_TELEPLOT) {
    // Format Teleplot: satu variabel per baris, diawali '>'
    if (mode != MODE_SMOOTH) Serial.printf(">raw:%d\n", raw);
    if (mode != MODE_RAW)    Serial.printf(">smooth:%d\n", smooth);
    Serial.printf(">envelope:%d\n>baseline:%d\n>threshold:%d\n>taps:%lu\n",
                  env, base, thr, (unsigned long)tapCount);
  } else {
    // Format Arduino Serial Plotter: "nama:nilai" dipisah koma, satu baris per sampel
    if (mode != MODE_SMOOTH) Serial.printf("raw:%d,", raw);
    if (mode != MODE_RAW)    Serial.printf("smooth:%d,", smooth);
    Serial.printf("envelope:%d,baseline:%d,threshold:%d\n", env, base, thr);
  }
}

// =================== 4. SETUP ===================

void setup() {
  Serial.begin(115200);
  // USB CDC butuh waktu untuk dikenali PC. Tunggu maks 3 detik,
  // tetapi tetap jalan walaupun Serial Monitor belum dibuka.
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);

  if (USE_LED) {
    pinMode(LED_PIN, OUTPUT);
    setLed(false);
  }

  // ADC 12-bit (0–4095). Atenuasi 11 dB: rentang ukur akurat ±0–2500 mV
  // pada ESP32-C3; di atas itu nilai mendekati 4095 (jenuh).
  analogReadResolution(12);
  analogSetPinAttenuation(ADC_PIN, ADC_11db);

  Serial.println();
  Serial.println("===============================================");
  Serial.println(" Smart Safe Walking Path - Uji Sensor Piezo");
  Serial.printf (" ESP32-C3 | ADC GPIO%u | %lu Hz | 12-bit\n", ADC_PIN, (unsigned long)SAMPLE_RATE_HZ);
  Serial.println(" Nilai = satuan ADC (0-4095), BUKAN Newton/kg");
  Serial.println("===============================================");
  printHelp();

  calibrateBaseline();
  nextSampleUs = micros();
}

// =================== 5. LOOP UTAMA ===================

void loop() {
  // ---- Perintah dari Serial ----
  while (Serial.available()) handleCommand((char)Serial.read());

  // ---- Sampling dengan periode tetap ----
  uint32_t nowUs = micros();
  if ((int32_t)(nowUs - nextSampleUs) < 0) return;
  nextSampleUs += SAMPLE_PERIOD_US;
  // Jika tertinggal jauh (mis. Serial macet), sinkronkan ulang jadwalnya
  if ((int32_t)(nowUs - nextSampleUs) > (int32_t)(SAMPLE_PERIOD_US * 10)) nextSampleUs = nowUs;

  // ---- Baca ADC ----
  int raw = analogRead(ADC_PIN);

  // ---- Moving average ----
  smoothSum -= smoothBuf[smoothIdx];
  smoothBuf[smoothIdx] = (uint16_t)raw;
  smoothSum += raw;
  smoothIdx = (smoothIdx + 1) % SMOOTH_WINDOW;
  int smooth = (int)(smoothSum / SMOOTH_WINDOW);

  // Sinyal untuk deteksi (lihat DETECT_ON_SMOOTH), tidak tergantung mode tampilan
  int signal    = DETECT_ON_SMOOTH ? smooth : raw;
  int amplitude = signal - (int)baseline;   // tinggi sinyal di atas baseline
  uint32_t nowMs = millis();

  // ---- Envelope: naik langsung ke puncak, turun perlahan ----
  envelope = max((float)amplitude, envelope * ENVELOPE_DECAY);

  // ---- Deteksi ketukan (dengan hysteresis + jeda minimum) ----
  if (!inTap) {
    if (amplitude > threshold && nowMs - lastTapMs >= MIN_TAP_INTERVAL_MS) {
      inTap     = true;
      tapPeak   = amplitude;
      lastTapMs = nowMs;
      tapCount++;
      setLed(true);
    } else if (abs(amplitude) < threshold) {
      // Baseline hanya diperbarui saat sensor diam, agar tidak ikut naik saat ditekan
      baseline += BASELINE_ALPHA * (signal - baseline);
    }
  } else {
    if (amplitude > tapPeak) tapPeak = amplitude;
    if (amplitude < threshold * RELEASE_RATIO) {
      inTap       = false;
      lastTapPeak = tapPeak;
      if (tapPeak > maxPeak) maxPeak = tapPeak;
      setLed(false);
      if (textAllowed()) {
        Serial.printf(">peak:%d\n", tapPeak);
        Serial.printf("[KETUKAN] #%lu  peak=%d  (threshold=%d)\n",
                      (unsigned long)tapCount, tapPeak, threshold);
      }
    }
  }

  // ---- Kirim ke plotter (1 titik per PLOT_EVERY_N sampel) ----
  if (raw > plotRawMax) plotRawMax = raw;
  if (++plotCounter >= PLOT_EVERY_N) {
    plotCounter = 0;
    if (plotOn) plotSample(plotRawMax, smooth);
    plotRawMax = 0;
  }
}
