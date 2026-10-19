#include <Arduino.h> 

void tampilkanAngka(int angka);
void matikanSevenSegment();

// ================= DEFINISI PIN ================= //
// Pin Input
const int PIN_BUTTON = 32;
const int PIN_LDR = 34; // Dihubungkan ke pin DO pada modul LDR

// Pin Output Lampu
const int PIN_PJU = 2;
const int LAMPU_KUNING = 26; // Blink saat standby
const int LAMPU_MERAH = 27;  // Menyala saat timer jalan
const int LAMPU_HIJAU = 15;  // Menyala 5 detik setelah timer habis

// Pin Dekoder 1 (Puluhan)
const int DEC1_A = 19; 
const int DEC1_B = 16;
const int DEC1_C = 17;
const int DEC1_D = 18;

// Pin Dekoder 2 (Satuan) - Sudah di Pin 33
const int DEC2_A = 25;
const int DEC2_B = 21;
const int DEC2_C = 22;
const int DEC2_D = 33; 

// ================= VARIABEL SISTEM ================= //
// Variabel untuk kedip kuning tanpa delay
unsigned long waktuKedipTerakhir = 0;
bool statusLampuKuning = false;

// ================= SETUP ================= //
void setup() {
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_LDR, INPUT); // Mode digital untuk DO
  
  pinMode(PIN_PJU, OUTPUT);
  pinMode(LAMPU_MERAH, OUTPUT);
  pinMode(LAMPU_KUNING, OUTPUT);
  pinMode(LAMPU_HIJAU, OUTPUT);

  // Set semua pin dekoder sebagai OUTPUT
  pinMode(DEC1_A, OUTPUT); pinMode(DEC1_B, OUTPUT); 
  pinMode(DEC1_C, OUTPUT); pinMode(DEC1_D, OUTPUT);
  
  pinMode(DEC2_A, OUTPUT); pinMode(DEC2_B, OUTPUT); 
  pinMode(DEC2_C, OUTPUT); pinMode(DEC2_D, OUTPUT);

  // Kondisi Awal Sistem (Standby)
  matikanSevenSegment(); // Layar mati
  digitalWrite(PIN_PJU, LOW);
  digitalWrite(LAMPU_MERAH, LOW);
  digitalWrite(LAMPU_KUNING, LOW);
  digitalWrite(LAMPU_HIJAU, LOW);
}

// ================= LOOP UTAMA ================= //
void loop() {
  
  // --- MODE STANDBY: Kedip Kuning Terus Menerus ---
  if (millis() - waktuKedipTerakhir >= 500) { // Kedip tiap 0.5 detik
    statusLampuKuning = !statusLampuKuning;
    digitalWrite(LAMPU_KUNING, statusLampuKuning);
    waktuKedipTerakhir = millis();
  }

  // --- CEK TRIGGER: Jika Tombol Ditekan ---
  if (digitalRead(PIN_BUTTON) == LOW) {
    
    // 1. Matikan lampu kuning (Keluar dari Standby)
    digitalWrite(LAMPU_KUNING, LOW);
    
    // 2. Baca LDR (DO). Jika HIGH (Gelap), nyalakan PJU
    // Catatan: Jika sifat modulmu terbalik (PJU nyala saat siang), 
    // cukup ubah HIGH menjadi LOW pada baris di bawah ini.
    if (digitalRead(PIN_LDR) == HIGH) { 
      digitalWrite(PIN_PJU, HIGH);
    }
    
    // 3. Nyalakan Lampu Merah
    digitalWrite(LAMPU_MERAH, HIGH);

    // 4. Hitung mundur 7-Segment dari 15 ke 0
    for (int waktu = 15; waktu >= 0; waktu--) {
      tampilkanAngka(waktu);
      delay(1000); // Jeda persis 1 detik
    }
    
    // 5. Timer selesai: Matikan 7-Segment dan Lampu Merah
    matikanSevenSegment();
    digitalWrite(LAMPU_MERAH, LOW);
    
    // 6. Nyalakan Lampu Hijau selama 5 detik
    digitalWrite(LAMPU_HIJAU, HIGH);
    delay(5000);
    digitalWrite(LAMPU_HIJAU, LOW);
    
    // 7. Matikan PJU (Reset untuk penyeberang berikutnya)
    digitalWrite(PIN_PJU, LOW);
    
    // Jeda 1 detik pendinginan sistem sebelum kembali ke standby
    delay(1000); 
  }
}

// ================= FUNGSI BANTUAN ================= //
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