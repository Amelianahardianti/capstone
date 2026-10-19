#include <Arduino.h>

// ===============================================================
// TES HARDWARE SEVEN SEGMENT (2x 74LS47)
// Tekan tombol -> 3 fase berurutan:
//   FASE 1: Seven segment 1 (DEC1/puluhan) hitung mundur 9-0, seven segment 2 mati
//   FASE 2: Seven segment 2 (DEC2/satuan)  hitung mundur 9-0, seven segment 1 mati
//   FASE 3: Kedua seven segment bersamaan (99, 88, ... 00)
// Buka Serial Monitor (115200) untuk melihat bit BCD yang dikirim tiap langkah,
// lalu cocokkan dengan angka yang tampil di display.
// Program traffic light disimpan di backup/main_traffic_light.cpp
// ===============================================================

void tampilkanAngka(int angka);
void tulisDekoder(int pinA, int pinB, int pinC, int pinD, int nilai);
void jedaDenganBuzzer(int waktu);
void cetakStatus(int nilaiDec1, int nilaiDec2);

// ================= DEFINISI PIN ================= //
// PINDAHKAN KABEL TOMBOL KE PIN 32!
const int PIN_BUTTON = 32;

const int PIN_PILOT  = 4;
const int PIN_BUZZER = 13;

const int DEC1_A = 19;
const int DEC1_B = 16;
const int DEC1_C = 17;
const int DEC1_D = 18;

const int DEC2_A = 25;
const int DEC2_B = 21;
const int DEC2_C = 22;
const int DEC2_D = 33;

// Input biner 1111 (15) membuat IC 74LS47 mematikan semua segmen (BLANK)
const int BLANK = 15;

// ================= SETUP ================= //
void setup() {
  Serial.begin(115200);

  // Pin 32 memiliki pull-up internal bawaan pabrik
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  pinMode(PIN_PILOT, OUTPUT);

  // TRIK BUZZER: Jadikan INPUT agar jalur terputus dan buzzer 5V benar-benar mati
  pinMode(PIN_BUZZER, INPUT);

  pinMode(DEC1_A, OUTPUT); pinMode(DEC1_B, OUTPUT);
  pinMode(DEC1_C, OUTPUT); pinMode(DEC1_D, OUTPUT);

  pinMode(DEC2_A, OUTPUT); pinMode(DEC2_B, OUTPUT);
  pinMode(DEC2_C, OUTPUT); pinMode(DEC2_D, OUTPUT);

  digitalWrite(PIN_PILOT, LOW);
  tampilkanAngka(0);

  Serial.println();
  Serial.println("=== TES SEVEN SEGMENT SIAP ===");
  Serial.println("DEC1 (puluhan): A=GPIO19 B=GPIO16 C=GPIO17 D=GPIO18");
  Serial.println("DEC2 (satuan) : A=GPIO25 B=GPIO21 C=GPIO22 D=GPIO33");
  Serial.println("Tampilan awal harus '00'. Tekan tombol untuk mulai.");
}

// ================= LOOP UTAMA ================= //
void loop() {
  if (digitalRead(PIN_BUTTON) == LOW) {

    digitalWrite(PIN_PILOT, HIGH);

    // FASE 1: Hanya seven segment 1 (puluhan), seven segment 2 dimatikan
    Serial.println("\n=== FASE 1: SEVEN SEGMENT 1 (DEC1) 9 -> 0, DEC2 MATI ===");
    for (int waktu = 9; waktu >= 0; waktu--) {
      tulisDekoder(DEC1_A, DEC1_B, DEC1_C, DEC1_D, waktu);
      tulisDekoder(DEC2_A, DEC2_B, DEC2_C, DEC2_D, BLANK);
      cetakStatus(waktu, BLANK);
      jedaDenganBuzzer(waktu);
    }

    // FASE 2: Hanya seven segment 2 (satuan), seven segment 1 dimatikan
    Serial.println("\n=== FASE 2: SEVEN SEGMENT 2 (DEC2) 9 -> 0, DEC1 MATI ===");
    for (int waktu = 9; waktu >= 0; waktu--) {
      tulisDekoder(DEC1_A, DEC1_B, DEC1_C, DEC1_D, BLANK);
      tulisDekoder(DEC2_A, DEC2_B, DEC2_C, DEC2_D, waktu);
      cetakStatus(BLANK, waktu);
      jedaDenganBuzzer(waktu);
    }

    // FASE 3: Kedua seven segment bersamaan (99, 88, 77, ... 00)
    Serial.println("\n=== FASE 3: KEDUA SEVEN SEGMENT 99 -> 00 ===");
    for (int waktu = 9; waktu >= 0; waktu--) {
      tampilkanAngka(waktu * 11);
      cetakStatus(waktu, waktu);
      jedaDenganBuzzer(waktu);
    }

    digitalWrite(PIN_PILOT, LOW);
    pinMode(PIN_BUZZER, INPUT); // Pastikan mati
    tampilkanAngka(0);
    Serial.println("\n=== TES SELESAI, tampilan kembali '00' ===");

    delay(500);
  }
}

// ================= FUNGSI BANTUAN ================= //
void tampilkanAngka(int angka) {
  int puluhan = angka / 10;
  int satuan  = angka % 10;

  tulisDekoder(DEC1_A, DEC1_B, DEC1_C, DEC1_D, puluhan);
  tulisDekoder(DEC2_A, DEC2_B, DEC2_C, DEC2_D, satuan);
}

// Kirim nilai 0-15 dalam bentuk BCD ke satu IC 74LS47
void tulisDekoder(int pinA, int pinB, int pinC, int pinD, int nilai) {
  digitalWrite(pinA, bitRead(nilai, 0));
  digitalWrite(pinB, bitRead(nilai, 1));
  digitalWrite(pinC, bitRead(nilai, 2));
  digitalWrite(pinD, bitRead(nilai, 3));
}

// LOGIKA BUZZER ASLI (TIDAK DIUBAH), total 1 detik per langkah
void jedaDenganBuzzer(int waktu) {
  if (waktu >= 4) {
    // Bunyi: Jadikan Output lalu tarik ke LOW
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);
    delay(500);
    // Mati: Lepas jalur dengan menjadikannya INPUT
    pinMode(PIN_BUZZER, INPUT);
    delay(500);
  } else {
    // Beep Cepat di 4 detik terakhir
    for (int b = 0; b < 4; b++) {
      pinMode(PIN_BUZZER, OUTPUT);
      digitalWrite(PIN_BUZZER, LOW);
      delay(125);
      pinMode(PIN_BUZZER, INPUT);
      delay(125);
    }
  }
}

// Cetak nilai + bit BCD (urutan D C B A) yang dikirim ke tiap dekoder
void cetakStatus(int nilaiDec1, int nilaiDec2) {
  Serial.printf("DEC1=%-5s [D C B A = %d %d %d %d]  |  DEC2=%-5s [D C B A = %d %d %d %d]\n",
                nilaiDec1 == BLANK ? "MATI" : String(nilaiDec1).c_str(),
                bitRead(nilaiDec1, 3), bitRead(nilaiDec1, 2), bitRead(nilaiDec1, 1), bitRead(nilaiDec1, 0),
                nilaiDec2 == BLANK ? "MATI" : String(nilaiDec2).c_str(),
                bitRead(nilaiDec2, 3), bitRead(nilaiDec2, 2), bitRead(nilaiDec2, 1), bitRead(nilaiDec2, 0));
}
