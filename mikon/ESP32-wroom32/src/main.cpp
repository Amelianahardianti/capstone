#include <Arduino.h>

// ===============================================================
// Smart Safe Walking Path — Traffic Light + PJU + Buzzer + 7-Segment
//
// Siklus sistem:
//   1. IDLE  : hanya Kuning kedip. Merah & Hijau mati, PJU mati, layar mati.
//              Menunggu push button.
//   2. TIMER : (setelah push button) Merah nyala, PILOT nyala,
//              7-segment hitung mundur 15 -> 0, buzzer bunyi.
//              LDR dibaca SAAT tombol ditekan: gelap -> PJU nyala,
//              terang -> PJU mati.
//   3. HIJAU : (setelah countdown habis) Hijau nyala 7 detik, Kuning mati.
//   -> kembali ke IDLE (PJU dimatikan, kuning kedip lagi).
//
// Semua proses berbasis millis() (tanpa delay).
// ===============================================================

void tampilkanAngka(int angka);
void matikanSevenSegment();
void resetSevenSegment();
void resetVariabel();
bool tombolDitekan();
void mulaiSiklus(unsigned long sekarang);
void updateLampu(unsigned long sekarang);
void updateBuzzer(unsigned long sekarang);
void setBuzzer(bool bunyi);
void tesRelay();

// ================= DEFINISI PIN ================= //
// Pin Input
const int PIN_BUTTON = 32;
const int PIN_LDR = 34; // Dihubungkan ke pin DO pada modul LDR

// Pin Output Lampu (modul relay)
const int PIN_PJU = 2;
const int LAMPU_KUNING = 26; // Kedip saat idle
const int LAMPU_MERAH = 27;  // Menyala saat timer jalan
const int LAMPU_HIJAU = 15;  // Menyala 7 detik setelah timer habis

const int PIN_PILOT  = 4;    // HIGH selama timer jalan (sama seperti kode tes)
const int PIN_BUZZER = 13;   // Buzzer 5V: OUTPUT+LOW = bunyi, INPUT = mati

// Pin Dekoder 1 (Puluhan)
const int DEC1_A = 19;
const int DEC1_B = 16;
const int DEC1_C = 17;
const int DEC1_D = 18;

// Pin Dekoder 2 (Satuan)
const int DEC2_A = 25;
const int DEC2_B = 21;
const int DEC2_C = 22;
const int DEC2_D = 33;

// ================= PENGATURAN ================= //
// Modul relay ACTIVE-HIGH: HIGH = ON, LOW = OFF.
// Jika ternyata relay menyala saat LOW, cukup tukar dua nilai ini.
const int RELAY_ON  = HIGH;
const int RELAY_OFF = LOW;

const int DURASI_TIMER = 15;                    // Hitung mundur 15 -> 0 (detik)
const unsigned long DURASI_HIJAU_MS = 7000;     // Hijau nyala 7 detik setelah countdown
const unsigned long KEDIP_KUNING_MS = 500;      // Kuning ON 0,5 s / OFF 0,5 s
const unsigned long DEBOUNCE_MS = 50;           // Debounce tombol

// Level pin DO modul LDR saat kondisi GELAP.
// Modul LDR umumnya: gelap = HIGH, terang = LOW.
// Jika PJU malah nyala saat terang, ganti menjadi LOW.
const int LDR_GELAP = HIGH;

// true = saat ESP32 menyala, tiap relay dinyalakan 1 detik bergantian
// (MERAH -> KUNING -> HIJAU -> PJU) untuk mengecek wiring relay.
const bool TES_RELAY_SAAT_BOOT = true;

// ================= VARIABEL SISTEM ================= //
enum FaseSistem { FASE_IDLE, FASE_TIMER, FASE_HIJAU };
FaseSistem fase = FASE_IDLE;

int sisaWaktu = 0;
unsigned long waktuDetikTerakhir = 0;
unsigned long waktuMulaiHijau = 0;

// Variabel untuk kedip kuning tanpa delay
unsigned long waktuKedipTerakhir = 0;
bool statusLampuKuning = false;

// ================= SETUP ================= //
void setup() {
  Serial.begin(115200);

  // Pin 32 memiliki pull-up internal bawaan pabrik
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_LDR, INPUT); // Mode digital untuk DO

  pinMode(PIN_PJU, OUTPUT);
  pinMode(LAMPU_MERAH, OUTPUT);
  pinMode(LAMPU_KUNING, OUTPUT);
  pinMode(LAMPU_HIJAU, OUTPUT);
  pinMode(PIN_PILOT, OUTPUT);

  // TRIK BUZZER: Jadikan INPUT agar jalur terputus dan buzzer 5V benar-benar mati
  pinMode(PIN_BUZZER, INPUT);

  // Tulis 0 ke register output dekoder SEBELUM pin dijadikan OUTPUT,
  // agar saat pin aktif levelnya pasti LOW (bukan acak)
  tampilkanAngka(0);

  // Set semua pin dekoder sebagai OUTPUT
  pinMode(DEC1_A, OUTPUT); pinMode(DEC1_B, OUTPUT);
  pinMode(DEC1_C, OUTPUT); pinMode(DEC1_D, OUTPUT);

  pinMode(DEC2_A, OUTPUT); pinMode(DEC2_B, OUTPUT);
  pinMode(DEC2_C, OUTPUT); pinMode(DEC2_D, OUTPUT);

  // Kondisi awal: semua relay OFF, pilot OFF, variabel & 7-segment di-reset
  digitalWrite(PIN_PJU, RELAY_OFF);
  digitalWrite(LAMPU_MERAH, RELAY_OFF);
  digitalWrite(LAMPU_KUNING, RELAY_OFF);
  digitalWrite(LAMPU_HIJAU, RELAY_OFF);
  digitalWrite(PIN_PILOT, LOW);
  resetVariabel();
  resetSevenSegment();

  Serial.println("\n=== ESP32 START ===");
  if (TES_RELAY_SAAT_BOOT) tesRelay();

  Serial.println("=== SISTEM SIAP: IDLE (KUNING KEDIP) ===");
}

// ================= LOOP UTAMA ================= //
void loop() {
  unsigned long sekarang = millis();
  bool ditekan = tombolDitekan(); // dibaca tiap loop agar debounce tetap akurat

  switch (fase) {
    // --- IDLE: tunggu push button ---
    case FASE_IDLE:
      if (ditekan) mulaiSiklus(sekarang);
      break;

    // --- TIMER: hitung mundur tiap 1 detik ---
    case FASE_TIMER:
      if (sekarang - waktuDetikTerakhir >= 1000) {
        waktuDetikTerakhir += 1000;
        sisaWaktu--;

        if (sisaWaktu >= 0) {
          tampilkanAngka(sisaWaktu);
          Serial.printf("COUNTDOWN: %d\n", sisaWaktu);
        } else {
          // Angka 0 sudah tampil 1 detik -> masuk fase hijau
          fase = FASE_HIJAU;
          waktuMulaiHijau = sekarang;
          digitalWrite(PIN_PILOT, LOW);
          resetSevenSegment();
          Serial.println("TIMER SELESAI -> HIJAU 7 DETIK");
        }
      }
      break;

    // --- HIJAU: nyala 7 detik, lalu kembali idle ---
    case FASE_HIJAU:
      if (sekarang - waktuMulaiHijau >= DURASI_HIJAU_MS) {
        digitalWrite(PIN_PJU, RELAY_OFF); // PJU mati, siap untuk penyeberang berikutnya
        resetVariabel();                  // fase = IDLE, kedip kuning mulai dari awal
        resetSevenSegment();
        Serial.println("HIJAU SELESAI -> PJU OFF -> IDLE (KUNING KEDIP)");
      }
      break;
  }

  updateBuzzer(sekarang);
  updateLampu(sekarang);
}

// ================= FUNGSI BANTUAN ================= //

// Dipanggil saat push button ditekan di fase IDLE
void mulaiSiklus(unsigned long sekarang) {
  // LDR hanya dibaca saat tombol ditekan
  bool gelap = (digitalRead(PIN_LDR) == LDR_GELAP);
  digitalWrite(PIN_PJU, gelap ? RELAY_ON : RELAY_OFF);

  // Mulai dari kondisi yang sama setiap siklus
  resetVariabel();
  resetSevenSegment();

  fase = FASE_TIMER;
  sisaWaktu = DURASI_TIMER;
  waktuDetikTerakhir = sekarang;
  digitalWrite(PIN_PILOT, HIGH);
  tampilkanAngka(sisaWaktu);

  Serial.println("\nTOMBOL DITEKAN -> TIMER MULAI -> MERAH");
  Serial.printf("LDR DO: %s -> %s -> PJU: %s\n",
                digitalRead(PIN_LDR) == HIGH ? "HIGH" : "LOW",
                gelap ? "GELAP" : "TERANG",
                gelap ? "ON" : "OFF");
  Serial.printf("COUNTDOWN: %d\n", sisaWaktu);
}

// Lampu traffic light sesuai fase:
// IDLE  = kuning kedip | TIMER = merah | HIJAU = hijau
void updateLampu(unsigned long sekarang) {
  if (fase == FASE_IDLE) {
    if (sekarang - waktuKedipTerakhir >= KEDIP_KUNING_MS) {
      statusLampuKuning = !statusLampuKuning;
      waktuKedipTerakhir = sekarang;
    }
  } else {
    statusLampuKuning = false;
  }

  digitalWrite(LAMPU_MERAH,  fase == FASE_TIMER ? RELAY_ON : RELAY_OFF);
  digitalWrite(LAMPU_HIJAU,  fase == FASE_HIJAU ? RELAY_ON : RELAY_OFF);
  digitalWrite(LAMPU_KUNING, statusLampuKuning  ? RELAY_ON : RELAY_OFF);
}

// Pola buzzer sama seperti kode tes (hanya saat timer jalan):
// sisa 15-4: bunyi 500 ms, diam 500 ms (1 beep per detik)
// sisa 3-0 : 4 beep cepat per detik (bunyi 125 ms, diam 125 ms)
void updateBuzzer(unsigned long sekarang) {
  bool bunyi = false;
  if (fase == FASE_TIMER) {
    unsigned long posisi = sekarang - waktuDetikTerakhir; // 0-999 ms di detik ini
    if (sisaWaktu >= 4) {
      bunyi = (posisi < 500);
    } else {
      bunyi = (posisi % 250) < 125;
    }
  }
  setBuzzer(bunyi);
}

// Pinmode hanya diubah saat status berubah
void setBuzzer(bool bunyi) {
  static bool statusTerakhir = false;
  if (bunyi == statusTerakhir) return;
  statusTerakhir = bunyi;

  if (bunyi) {
    // Bunyi: Jadikan Output lalu tarik ke LOW
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);
  } else {
    // Mati: Lepas jalur dengan menjadikannya INPUT
    pinMode(PIN_BUZZER, INPUT);
  }
}

// true satu kali setiap tombol ditekan (tepi turun, sudah di-debounce).
// Menahan tombol tidak akan memulai siklus berulang-ulang.
bool tombolDitekan() {
  static int bacaanTerakhir = HIGH;
  static int statusStabil = HIGH;
  static unsigned long waktuBerubah = 0;

  int bacaan = digitalRead(PIN_BUTTON);
  if (bacaan != bacaanTerakhir) {
    bacaanTerakhir = bacaan;
    waktuBerubah = millis();
  }

  if (millis() - waktuBerubah >= DEBOUNCE_MS && bacaan != statusStabil) {
    statusStabil = bacaan;
    if (statusStabil == LOW) return true; // INPUT_PULLUP: ditekan = LOW
  }
  return false;
}

// Nyalakan tiap relay 1 detik bergantian, lihat Serial Monitor
void tesRelay() {
  const int pinRelay[]    = { LAMPU_MERAH, LAMPU_KUNING, LAMPU_HIJAU, PIN_PJU };
  const char* namaRelay[] = { "MERAH",     "KUNING",     "HIJAU",     "PJU"   };

  Serial.println("=== TES RELAY (tiap relay ON 1 detik) ===");
  for (int i = 0; i < 4; i++) {
    Serial.printf("TES RELAY: %-6s (GPIO%d) ON\n", namaRelay[i], pinRelay[i]);
    digitalWrite(pinRelay[i], RELAY_ON);
    delay(1000);
    digitalWrite(pinRelay[i], RELAY_OFF);
    delay(300);
  }
  Serial.println("=== TES RELAY SELESAI ===");
}

void tampilkanAngka(int angka) {
  int puluhan = angka / 10;
  int satuan  = angka % 10;

  digitalWrite(DEC1_A, bitRead(puluhan, 0));
  digitalWrite(DEC1_B, bitRead(puluhan, 1));
  digitalWrite(DEC1_C, bitRead(puluhan, 2));
  digitalWrite(DEC1_D, bitRead(puluhan, 3));

  digitalWrite(DEC2_A, bitRead(satuan, 0));
  digitalWrite(DEC2_B, bitRead(satuan, 1));
  digitalWrite(DEC2_C, bitRead(satuan, 2));
  digitalWrite(DEC2_D, bitRead(satuan, 3));
}

// Fungsi Trik Biner untuk mematikan 7-Segment
void matikanSevenSegment() {
  // Kirim sinyal biner 1111 (Angka 15) agar IC 74LS47 melakukan "BLANK"
  digitalWrite(DEC1_A, HIGH); digitalWrite(DEC1_B, HIGH);
  digitalWrite(DEC1_C, HIGH); digitalWrite(DEC1_D, HIGH);

  digitalWrite(DEC2_A, HIGH); digitalWrite(DEC2_B, HIGH);
  digitalWrite(DEC2_C, HIGH); digitalWrite(DEC2_D, HIGH);
}

// State awal 7-segment yang selalu sama:
// 1. tulis 0 (0000) ke register kedua dekoder
// 2. lalu BLANK (1111)
void resetSevenSegment() {
  tampilkanAngka(0);
  matikanSevenSegment();
}

// Semua variabel sistem kembali ke nilai awal (fase IDLE)
void resetVariabel() {
  fase = FASE_IDLE;
  sisaWaktu = 0;
  waktuDetikTerakhir = 0;
  waktuMulaiHijau = 0;
  waktuKedipTerakhir = 0;
  statusLampuKuning = false;
}
