#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Preferences.h>

// ===============================================================
// Smart Safe Walking Path — Traffic Light + PJU + Buzzer + 7-Segment
// + Sinkronisasi 2 node (dua sisi penyeberangan) lewat MQTT + ESP-NOW
//
// Siklus sistem:
//   1. IDLE  : hanya Kuning kedip. Merah & Hijau mati, PJU mati, layar mati.
//              Menunggu push button / sensor pejalan kaki (piezo + ultrasonik)
//              ATAU pesan START dari node lain.
//   2. TIMER : Merah nyala, PILOT nyala, 7-segment hitung mundur 15 -> 0,
//              buzzer bunyi. LDR dibaca oleh node yang tombolnya ditekan,
//              hasilnya (PJU on/off) dibagikan ke node lain.
//   3. HIJAU : (setelah countdown habis) Hijau nyala 7 detik, Kuning mati.
//   -> kembali ke IDLE (PJU dimatikan, kuning kedip lagi).
//
// Sinkronisasi:
//   - Satu siklus = idSiklus (acak) + waktuMulaiSiklus. Seluruh fase dihitung
//     dari "posisi" = millis() - waktuMulaiSiklus, jadi untuk menyamakan dua
//     node cukup menyamakan posisi siklusnya.
//   - Node yang tombolnya ditekan = pemilik siklus. Ia mengirim START lalu
//     heartbeat STATE tiap 1 detik. Node lain ikut / mengoreksi drift.
//   - Tombol ditekan hampir bersamaan di kedua sisi -> idSiklus terkecil menang.
//   - Pesan dikirim lewat DUA jalur sekaligus: MQTT (WiFi + broker laptop)
//     dan ESP-NOW (langsung ESP ke ESP). WiFi/broker mati -> ESP-NOW tetap
//     menyinkronkan. Dua-duanya mati -> tiap node tetap jalan sendiri.
//
// Kedua ESP memakai kode yang SAMA. Bedanya hanya NODE_ID (lihat platformio.ini).
// Semua proses lampu berbasis millis() (tanpa delay), jaringan di task core 0.
// ===============================================================

// ================= IDENTITAS NODE ================= //
// Diisi dari build_flags di platformio.ini (env node_a / node_b).
// Jika kosong, ID diambil otomatis dari MAC address.
#ifndef NODE_ID
#define NODE_ID ""
#endif

// ================= KONFIGURASI WIFI & MQTT ================= //
const char* WIFI_SSID = "veroo";            // GANTI DENGAN WIFI HP/ROUTER
const char* WIFI_PASS = "veronica";         // GANTI DENGAN PASS WIFI
const char* MQTT_HOST = "10.130.117.191";   // IP laptop (Mosquitto)
const uint16_t MQTT_PORT = 1883;

// Topik:
//   penyeberangan/sync               -> pesan sinkron antar node (dan nanti CV)
//   penyeberangan/node/<id>/status   -> status terakhir node (retained, untuk monitor)
//   penyeberangan/node/<id>/online   -> "online"/"offline" (LWT, retained)
const char* TOPIC_SYNC = "penyeberangan/sync";

struct PesanSync;
void tampilkanAngka(int angka);
void matikanSevenSegment();
void resetSevenSegment();
bool tombolDitekan();
bool pejalanKakiTerdeteksi(unsigned long sekarang);
void IRAM_ATTR isrEcho();
void kalibrasiSensor();
void kirimLog(const char* sub, bool retain, const char* format, ...);
void mulaiSiklusLokal(unsigned long sekarang, const char* pemicu);
void aturSiklus(uint32_t id, const char* pemilik, bool pju, uint32_t posisi, unsigned long sekarang);
void updateSiklus(unsigned long sekarang);
void prosesPesanPeer(const PesanSync& p);
void kirimState(const char* evt, unsigned long sekarang);
void updateLampu(unsigned long sekarang);
void updateBuzzer(unsigned long sekarang);
void setBuzzer(bool bunyi);
void tesRelay();
void taskJaringan(void* pvParameters);

// ================= DEFINISI PIN ================= //
// Pin Input
const int PIN_BUTTON = 32;
const int PIN_LDR = 34; // Dihubungkan ke pin DO pada modul LDR

// Sensor pejalan kaki (pemicu selain push button)
// GPIO34-39 hanya bisa INPUT, jadi TRIG (output) di 23 dan ECHO (input) di 39.
// ECHO SRF05 bertegangan 5V -> pakai pembagi tegangan ke 3,3V!
const int PIN_PIEZO = 35; // ADC1_CH7 (ADC1 tetap bisa dipakai saat WiFi aktif)
const int PIN_TRIG  = 23;
const int PIN_ECHO  = 39;

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

// Sensor pejalan kaki: siklus mulai jika piezo DAN ultrasonik sama-sama mendeteksi.
// Saat boot, sensor dikalibrasi otomatis (KALIBRASI_MS) -> jangan injak piezo
// dan jangan berdiri di depan ultrasonik selama kalibrasi.
const unsigned long KALIBRASI_MS = 3000;

// Piezo: dianggap "ada injakan" jika puncak sinyal melebihi baseline (nilai diam)
// sebanyak ambang. Ambang = nilai terbesar dari 3 hal di bawah, jadi otomatis
// ikut naik kalau noise-nya besar. Makin besar angkanya = makin TIDAK sensitif.
const int   PIEZO_DELTA_MIN    = 150;   // ambang minimum di atas baseline (ADC 0-4095)
const float PIEZO_FAKTOR_NOISE = 5.0;   // ambang >= faktor x rata-rata noise
const float PIEZO_FAKTOR_KALIB = 1.5;   // ambang >= 1,5 x lonjakan terbesar saat kalibrasi
const float PIEZO_ALPHA        = 0.01;  // kecepatan baseline/noise menyesuaikan (per 10 ms)
const unsigned long PIEZO_HOLD_MS = 1000; // injakan "diingat" 1 s, menunggu ultrasonik

// Ultrasonik: jarak latar (lantai/dinding di depan sensor) diukur saat kalibrasi.
// Ada orang jika jarak < min(BATAS_JARAK_CM, jarak latar - MARGIN_LATAR_CM)
// selama ULTRA_KONFIRMASI bacaan berturut-turut.
const float BATAS_JARAK_CM  = 150.0;    // batas maksimum, walau latar lebih jauh
const float MARGIN_LATAR_CM = 20.0;     // harus lebih dekat 20 cm dari latar
const int   ULTRA_KONFIRMASI = 3;       // 3 x 100 ms berturut-turut -> bukan noise sesaat

const unsigned long INTERVAL_ULTRA_MS = 100;    // trigger SRF05 tiap 100 ms
const unsigned long INTERVAL_PIEZO_MS = 10;     // evaluasi puncak piezo tiap 10 ms
const bool LOG_SENSOR = false;                  // true = kirim data sensor (format Teleplot)

// Pemantauan sensor lewat MQTT (tanpa kabel USB), lihat di laptop:
//   penyeberangan/node/<id>/kalibrasi   -> hasil kalibrasi saat boot (retained)
//   penyeberangan/node/<id>/sensor      -> tiap kali sensor LOLOS memicu siklus (retained)
//   penyeberangan/node/<id>/sensor_live -> data sensor berkala (JSON, untuk grafik)
const bool LOG_SENSOR_MQTT = true;              // false = matikan sensor_live
const unsigned long INTERVAL_SENSOR_LIVE_MS = 500;

// Angka 15..0 masing-masing tampil 1 detik -> fase TIMER = 16 detik
const unsigned long MS_TIMER  = (DURASI_TIMER + 1) * 1000UL;
const unsigned long MS_SIKLUS = MS_TIMER + DURASI_HIJAU_MS;

// Sinkronisasi
const unsigned long HEARTBEAT_AKTIF_MS = 500;   // kirim STATE tiap 0,5 s saat siklus jalan
const unsigned long HEARTBEAT_IDLE_MS  = 2000;  // tanda "masih hidup" saat IDLE
const long TOLERANSI_SYNC_MS = 200;             // selisih > ini -> posisi dikoreksi

// WiFi & ESP-NOW
const uint8_t CHANNEL_DEFAULT = 1;              // dipakai jika belum pernah tersambung hotspot
const unsigned long WIFI_TIMEOUT_MS = 10000;    // batas satu percobaan sambung WiFi
const unsigned long WIFI_RETRY_MS   = 30000;    // jeda sambung ulang saat offline & ESP-NOW lancar
const unsigned long WIFI_RETRY_CEPAT_MS = 8000; // jeda sambung ulang saat node lain tidak terdengar
const unsigned long MQTT_RETRY_MS   = 5000;     // jeda percobaan sambung ke broker
const unsigned long ESPNOW_SEGAR_MS = 3000;     // ESP-NOW dianggap aktif jika ada paket < 3 s lalu
const unsigned long PEER_SENYAP_MS  = 5000;     // tidak ada paket ESP-NOW > 5 s -> beda channel
const unsigned long LOG_LINK_MS     = 10000;    // ringkasan status link di Serial

// Level pin DO modul LDR saat kondisi GELAP.
// Modul LDR umumnya: gelap = HIGH, terang = LOW.
// Jika PJU malah nyala saat terang, ganti menjadi LOW.
const int LDR_GELAP = HIGH;

// true = saat ESP32 menyala, tiap relay dinyalakan 1 detik bergantian
// (MERAH -> KUNING -> HIJAU -> PJU) untuk mengecek wiring relay.
const bool TES_RELAY_SAAT_BOOT = true;

// ================= PESAN SINKRON ================= //
enum JalurPesan : uint8_t { JALUR_LOKAL, JALUR_MQTT, JALUR_ESPNOW };
const char* NAMA_JALUR[] = { "LOKAL", "MQTT", "ESP-NOW" };

struct PesanSync {
  char     asal[16];   // NODE_ID pengirim
  char     evt[8];     // START / STATE / FASE (informasi untuk log/monitor)
  uint32_t idSiklus;   // 0 = pengirim sedang IDLE
  uint32_t posisiMs;   // ms sejak siklus dimulai
  uint8_t  fase;       // FaseSistem pengirim
  bool     pju;        // keputusan PJU dari pemilik siklus
  uint32_t stempel;    // millis() lokal saat posisiMs dicatat (kompensasi antrian)
  uint8_t  jalur;      // JalurPesan: lewat mana pesan ini diterima
};

const uint8_t ALAMAT_BROADCAST[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
const uint32_t MAGIC_PAKET = 0x50594231; // "PYB1", menyaring paket ESP-NOW lain

// Format paket ESP-NOW (biner, ukuran tetap)
struct __attribute__((packed)) PaketNow {
  uint32_t magic;
  char     asal[16];
  char     evt[8];
  uint32_t idSiklus;
  uint32_t posisiMs;
  uint8_t  fase;
  uint8_t  pju;
};

QueueHandle_t qKirim  = NULL;  // loop() -> taskJaringan (khusus MQTT)
QueueHandle_t qTerima = NULL;  // taskJaringan -> loop()

// Log teks ke MQTT (loop() -> taskJaringan). Disimpan di antrian sampai MQTT
// tersambung, jadi hasil kalibrasi saat boot tetap terkirim walau WiFi telat.
struct PesanLog {
  char sub[12];     // akhiran topik: penyeberangan/node/<id>/<sub>
  bool retain;
  char isi[200];
};
QueueHandle_t qLog  = NULL;
QueueHandle_t qLive = NULL;    // panjang 1, selalu ditimpa data sensor terbaru

WiFiClient espClient;
PubSubClient mqtt(espClient);
char idNode[16];
char clientId[32];
char topicStatus[48];
char topicOnline[48];

// Dibaca lintas core (taskJaringan <-> loop)
volatile bool sedangSiklus = false;            // true = siklus berjalan, jangan scan WiFi
volatile bool espNowSiap = false;
volatile unsigned long waktuNowTerakhir = 0;   // millis() paket ESP-NOW terakhir dari node lain
char peerNowTerakhir[16] = "";

// ================= VARIABEL SISTEM ================= //
enum FaseSistem { FASE_IDLE, FASE_TIMER, FASE_HIJAU };
FaseSistem fase = FASE_IDLE;
const char* NAMA_FASE[] = { "IDLE", "TIMER", "HIJAU" };

uint32_t idSiklus = 0;                 // 0 = tidak ada siklus berjalan
uint32_t idSiklusSelesai = 0;          // siklus terakhir yang sudah selesai
char pemilikSiklus[16] = "";
unsigned long waktuMulaiSiklus = 0;
bool statusPJU = false;
int angkaTampil = -1;                  // angka yang sedang tampil di 7-segment
unsigned long waktuHeartbeat = 0;

// Variabel untuk kedip kuning tanpa delay
unsigned long waktuKedipTerakhir = 0;
bool statusLampuKuning = false;

// Sensor pejalan kaki
// Lebar pulsa ECHO diukur lewat interrupt (bukan pulseIn) agar loop tidak
// tertahan sampai 30 ms dan lampu/buzzer tetap tepat waktu.
volatile unsigned long echoMulaiUs = 0;
volatile unsigned long echoDurasiUs = 0;
unsigned long echoTerakhirUs = 0;      // lebar pulsa ECHO terakhir (untuk diagnosa)
volatile uint32_t echoIsrHitung = 0;   // jumlah interrupt ECHO (untuk diagnosa)
volatile bool echoBaru = false;
unsigned long waktuUltraTerakhir = 0;
unsigned long waktuPiezoTerakhir = 0;
unsigned long waktuKetukan = 0;
int piezoPuncak = 0;
int piezoKetukan = 0;                  // nilai puncak injakan terakhir (untuk log)
int piezoLiveMaks = 0;                 // puncak piezo sejak sensor_live terakhir
float piezoBaseline = 0;               // nilai piezo saat diam (menyesuaikan pelan-pelan)
float piezoNoise = 0;                  // rata-rata simpangan saat diam
float piezoMarginKalib = 0;            // lonjakan terbesar saat kalibrasi x faktor
float jarakCm = 400.0;
float jarakLatar = 400.0;              // jarak ke lantai/dinding saat tidak ada orang
float batasJarak = BATAS_JARAK_CM;     // batas efektif setelah kalibrasi
int ultraHitung = 0;                   // berapa bacaan berturut-turut di bawah batas
bool ultraDeteksi = false;
bool piezoAktif = false;

// ================= SETUP ================= //
void setup() {
  Serial.begin(115200);

  // Pin 32 memiliki pull-up internal bawaan pabrik
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  pinMode(PIN_LDR, INPUT); // Mode digital untuk DO

  // Sensor pejalan kaki
  pinMode(PIN_TRIG, OUTPUT);
  digitalWrite(PIN_TRIG, LOW);
  pinMode(PIN_ECHO, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_ECHO), isrEcho, CHANGE);
  analogReadResolution(12);                     // 0 - 4095
  analogSetPinAttenuation(PIN_PIEZO, ADC_11db); // rentang ~0 - 3,1 V

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

  // Kondisi awal: semua relay OFF, pilot OFF, 7-segment di-reset
  digitalWrite(PIN_PJU, RELAY_OFF);
  digitalWrite(LAMPU_MERAH, RELAY_OFF);
  digitalWrite(LAMPU_KUNING, RELAY_OFF);
  digitalWrite(LAMPU_HIJAU, RELAY_OFF);
  digitalWrite(PIN_PILOT, LOW);
  resetSevenSegment();

  // Identitas node: dari build flag, atau 6 digit terakhir MAC
  uint32_t mac = (uint32_t)(ESP.getEfuseMac() >> 24) & 0xFFFFFF;
  if (strlen(NODE_ID) > 0) strlcpy(idNode, NODE_ID, sizeof(idNode));
  else snprintf(idNode, sizeof(idNode), "%06X", mac);
  snprintf(clientId, sizeof(clientId), "penyeberangan-%s-%06X", idNode, mac);
  snprintf(topicStatus, sizeof(topicStatus), "penyeberangan/node/%s/status", idNode);
  snprintf(topicOnline, sizeof(topicOnline), "penyeberangan/node/%s/online", idNode);

  Serial.printf("\n=== ESP32 START — NODE %s ===\n", idNode);

  qKirim  = xQueueCreate(10, sizeof(PesanSync));
  qTerima = xQueueCreate(10, sizeof(PesanSync));
  qLog    = xQueueCreate(8, sizeof(PesanLog));
  qLive   = xQueueCreate(1, sizeof(PesanLog));
  if (qKirim == NULL || qTerima == NULL || qLog == NULL || qLive == NULL) {
    Serial.println("FATAL: QUEUE GAGAL DIBUAT");
    while (1) delay(1000);
  }

  // Jaringan di core 0, logika lampu tetap di loop() (core 1)
  xTaskCreatePinnedToCore(taskJaringan, "Task_Jaringan", 6144, NULL, 1, NULL, 0);

  if (TES_RELAY_SAAT_BOOT) tesRelay();
  kalibrasiSensor();

  waktuKedipTerakhir = millis();
  Serial.println("=== SISTEM SIAP: IDLE (KUNING KEDIP) ===");
}

// ================= LOOP UTAMA ================= //
void loop() {
  bool ditekan = tombolDitekan(); // dibaca tiap loop agar debounce tetap akurat
  bool adaPejalan = pejalanKakiTerdeteksi(millis()); // sampling piezo secepat mungkin

  // Pesan dari node lain
  PesanSync masuk;
  while (xQueueReceive(qTerima, &masuk, 0) == pdTRUE) {
    prosesPesanPeer(masuk);
  }

  unsigned long sekarang = millis();

  // Tombol / sensor hanya memulai siklus saat IDLE
  if (fase == FASE_IDLE) {
    if (ditekan) mulaiSiklusLokal(sekarang, "TOMBOL DITEKAN");
    else if (adaPejalan) mulaiSiklusLokal(sekarang, "SENSOR: PEJALAN KAKI TERDETEKSI (PIEZO + ULTRASONIK)");
  }

  updateSiklus(sekarang);
  sedangSiklus = (idSiklus != 0);

  unsigned long intervalHB = idSiklus ? HEARTBEAT_AKTIF_MS : HEARTBEAT_IDLE_MS;
  if (sekarang - waktuHeartbeat >= intervalHB) kirimState("STATE", sekarang);

  updateBuzzer(sekarang);
  updateLampu(sekarang);
}

// ================= SIKLUS & SINKRONISASI ================= //

// Dipanggil saat push button ditekan / sensor mendeteksi pejalan kaki di fase IDLE
void mulaiSiklusLokal(unsigned long sekarang, const char* pemicu) {
  // LDR hanya dibaca saat siklus dipicu, hasilnya dibagikan ke node lain
  int bacaanLDR = digitalRead(PIN_LDR);
  bool gelap = (bacaanLDR == LDR_GELAP);

  Serial.printf("\n%s -> TIMER MULAI -> MERAH\n", pemicu);
  Serial.printf("LDR DO: %s -> %s -> PJU: %s\n",
                bacaanLDR == HIGH ? "HIGH" : "LOW",
                gelap ? "GELAP" : "TERANG",
                gelap ? "ON" : "OFF");

  aturSiklus(esp_random() | 1, idNode, gelap, 0, sekarang); // |1 agar id tidak pernah 0
  kirimState("START", sekarang);
}

// Pasang siklus (milik sendiri atau ikut node lain) pada posisi tertentu
void aturSiklus(uint32_t id, const char* pemilik, bool pju, uint32_t posisi, unsigned long sekarang) {
  idSiklus = id;
  strlcpy(pemilikSiklus, pemilik, sizeof(pemilikSiklus));
  statusPJU = pju;
  digitalWrite(PIN_PJU, pju ? RELAY_ON : RELAY_OFF);
  waktuMulaiSiklus = sekarang - posisi;
  angkaTampil = -1; // paksa 7-segment ditulis ulang
  updateSiklus(sekarang);
}

void gantiFase(FaseSistem baru, unsigned long sekarang) {
  fase = baru;
  switch (baru) {
    case FASE_TIMER:
      digitalWrite(PIN_PILOT, HIGH);
      Serial.printf("FASE: MERAH (siklus %08lX, pemilik %s)\n",
                    (unsigned long)idSiklus, pemilikSiklus);
      break;

    case FASE_HIJAU:
      digitalWrite(PIN_PILOT, LOW);
      resetSevenSegment();
      angkaTampil = -1;
      Serial.println("TIMER SELESAI -> HIJAU 7 DETIK");
      break;

    case FASE_IDLE:
      digitalWrite(PIN_PILOT, LOW);
      digitalWrite(PIN_PJU, RELAY_OFF); // PJU mati, siap untuk penyeberang berikutnya
      statusPJU = false;
      resetSevenSegment();
      angkaTampil = -1;
      idSiklusSelesai = idSiklus;
      idSiklus = 0;
      pemilikSiklus[0] = '\0';
      waktuKedipTerakhir = sekarang; // kedip kuning mulai dari awal
      statusLampuKuning = false;
      Serial.println("HIJAU SELESAI -> PJU OFF -> IDLE (KUNING KEDIP)");
      break;
  }
  kirimState("FASE", sekarang);
}

// Fase & angka countdown dihitung dari posisi siklus
void updateSiklus(unsigned long sekarang) {
  if (idSiklus == 0) return;

  unsigned long posisi = sekarang - waktuMulaiSiklus;
  FaseSistem target = (posisi < MS_TIMER)  ? FASE_TIMER
                    : (posisi < MS_SIKLUS) ? FASE_HIJAU
                    : FASE_IDLE;
  if (target != fase) gantiFase(target, sekarang);

  if (fase == FASE_TIMER) {
    int angka = DURASI_TIMER - (int)(posisi / 1000);
    if (angka != angkaTampil) {
      angkaTampil = angka;
      tampilkanAngka(angka);
      Serial.printf("COUNTDOWN: %d\n", angka);
    }
  }
}

// Aturan saat menerima pesan dari node lain
void prosesPesanPeer(const PesanSync& p) {
  if (p.idSiklus == 0) return; // node lain IDLE: tidak memaksa apa pun

  unsigned long sekarang = millis();
  uint32_t posisi = p.posisiMs + (sekarang - p.stempel);
  if (posisi >= MS_SIKLUS) return; // siklus node lain sudah selesai

  if (idSiklus == 0) {
    // Heartbeat telat dari siklus yang baru saja kita selesaikan -> abaikan
    if (p.idSiklus == idSiklusSelesai) return;

    Serial.printf("\nSYNC: IKUT SIKLUS %08lX DARI NODE %s via %s (%s, posisi %lu ms)\n",
                  (unsigned long)p.idSiklus, p.asal, NAMA_JALUR[p.jalur], p.evt,
                  (unsigned long)posisi);
    aturSiklus(p.idSiklus, p.asal, p.pju, posisi, sekarang);

  } else if (p.idSiklus == idSiklus) {
    // Siklus sama: hanya pemilik siklus yang jadi acuan koreksi drift
    if (strcmp(p.asal, pemilikSiklus) != 0) return;
    // Latensi MQTT lebih besar & tidak stabil; selama ESP-NOW aktif, pakai ESP-NOW saja
    if (p.jalur == JALUR_MQTT && sekarang - waktuNowTerakhir < ESPNOW_SEGAR_MS) return;

    long selisih = (long)(sekarang - waktuMulaiSiklus) - (long)posisi;
    if (labs(selisih) > TOLERANSI_SYNC_MS) {
      Serial.printf("SYNC: KOREKSI DRIFT %ld ms\n", selisih);
      waktuMulaiSiklus = sekarang - posisi;
      updateSiklus(sekarang);
    }

  } else if (p.idSiklus < idSiklus) {
    // Tombol ditekan hampir bersamaan di kedua sisi: id terkecil menang
    Serial.printf("SYNC: KONFLIK SIKLUS, IKUT %08lX DARI NODE %s via %s\n",
                  (unsigned long)p.idSiklus, p.asal, NAMA_JALUR[p.jalur]);
    aturSiklus(p.idSiklus, p.asal, p.pju, posisi, sekarang);
  }
  // p.idSiklus > idSiklus: node lain yang akan mengikuti siklus kita
}

// Kirim status saat ini:
//   - ESP-NOW langsung dari sini (tidak ikut tertahan kalau MQTT sedang blocking)
//   - MQTT lewat antrian ke taskJaringan
void kirimState(const char* evt, unsigned long sekarang) {
  PesanSync p = {};
  strlcpy(p.asal, idNode, sizeof(p.asal));
  strlcpy(p.evt, evt, sizeof(p.evt));
  p.idSiklus = idSiklus;
  p.posisiMs = idSiklus ? (sekarang - waktuMulaiSiklus) : 0;
  p.fase = fase;
  p.pju = statusPJU;
  p.stempel = sekarang;

  if (espNowSiap) {
    PaketNow pk = {};
    pk.magic = MAGIC_PAKET;
    strlcpy(pk.asal, p.asal, sizeof(pk.asal));
    strlcpy(pk.evt, p.evt, sizeof(pk.evt));
    pk.idSiklus = p.idSiklus;
    pk.posisiMs = p.posisiMs;
    pk.fase = p.fase;
    pk.pju = p.pju;
    esp_now_send(ALAMAT_BROADCAST, (const uint8_t*)&pk, sizeof(pk));
  }

  xQueueSend(qKirim, &p, 0); // antrian penuh -> pesan dibuang, heartbeat berikutnya menyusul
  waktuHeartbeat = sekarang;
}

// ================= JARINGAN (CORE 0) ================= //
// Dua jalur dipakai bersamaan:
//   - MQTT    : WiFi + broker di laptop (juga untuk monitor / CV nanti)
//   - ESP-NOW : broadcast langsung ESP ke ESP, tetap jalan walau WiFi mati
// Pesan yang datang dobel dari dua jalur aman, karena logika sinkron
// berbasis idSiklus + posisi.
//
// ESP-NOW hanya sampai jika kedua ESP berada di channel yang sama:
//   - WiFi tersambung : channel ikut hotspot (dicatat ke flash)
//   - WiFi putus      : auto-reconnect dimatikan, radio dikunci di channel terakhir
//   - Sambung ulang   : scan WiFi membuat radio pindah-pindah channel, jadi:
//       * node lain masih terdengar via ESP-NOW -> coba tiap 30 s, hanya saat IDLE
//       * node lain senyap (hotspot nyala lagi di channel lain / node lain sudah
//         online duluan) -> coba tiap 8 s, boleh saat siklus jalan, karena
//         ESP-NOW memang sedang tidak sampai
// PubSubClient TIDAK thread-safe, jadi hanya dipakai di task ini.

enum StatusWifi { ST_MENYAMBUNG, ST_ONLINE, ST_OFFLINE };
const char* NAMA_STATUS_WIFI[] = { "MENYAMBUNG", "ONLINE", "OFFLINE" };

Preferences prefs;

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, payload, length)) return;

  const char* asal = doc["src"] | "";
  if (asal[0] == '\0' || strcmp(asal, idNode) == 0) return; // pesan sendiri

  PesanSync p = {};
  strlcpy(p.asal, asal, sizeof(p.asal));
  strlcpy(p.evt, doc["evt"] | "", sizeof(p.evt));
  p.idSiklus = doc["cyc"] | 0UL;
  p.posisiMs = doc["pos"] | 0UL;
  p.pju = doc["pju"] | false;
  p.stempel = millis();
  p.jalur = JALUR_MQTT;
  xQueueSend(qTerima, &p, 0);
}

// Dipanggil dari task WiFi internal ESP32 saat ada paket ESP-NOW masuk
#if ESP_ARDUINO_VERSION_MAJOR >= 3
void espNowTerima(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
#else
void espNowTerima(const uint8_t* mac, const uint8_t* data, int len) {
#endif
  if (len != sizeof(PaketNow)) return;
  PaketNow pk;
  memcpy(&pk, data, sizeof(pk));
  if (pk.magic != MAGIC_PAKET) return;
  pk.asal[sizeof(pk.asal) - 1] = '\0';
  pk.evt[sizeof(pk.evt) - 1] = '\0';
  if (pk.asal[0] == '\0' || strcmp(pk.asal, idNode) == 0) return;

  PesanSync p = {};
  strlcpy(p.asal, pk.asal, sizeof(p.asal));
  strlcpy(p.evt, pk.evt, sizeof(p.evt));
  p.idSiklus = pk.idSiklus;
  p.posisiMs = pk.posisiMs;
  p.fase = pk.fase;
  p.pju = pk.pju;
  p.stempel = millis();
  p.jalur = JALUR_ESPNOW;

  waktuNowTerakhir = p.stempel;
  strlcpy(peerNowTerakhir, pk.asal, sizeof(peerNowTerakhir));
  xQueueSend(qTerima, &p, 0);
}

bool mulaiEspNow() {
  if (esp_now_init() != ESP_OK) return false;
  esp_now_register_recv_cb(espNowTerima);

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, ALAMAT_BROADCAST, 6);
  peer.channel = 0;            // 0 = ikut channel radio saat ini
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  return esp_now_add_peer(&peer) == ESP_OK;
}

// Paksa radio diam di channel tertentu (hanya boleh saat STA tidak tersambung)
void kunciChannel(uint8_t channel) {
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  esp_wifi_set_promiscuous(false);
}

// Kirim satu pesan ke broker MQTT
void publishMqtt(const PesanSync& p) {
  // Tambahkan waktu yang dihabiskan pesan di antrian
  uint32_t posisi = p.idSiklus ? p.posisiMs + (millis() - p.stempel) : 0;

  StaticJsonDocument<256> doc;
  doc["src"]  = p.asal;
  doc["evt"]  = p.evt;
  doc["cyc"]  = p.idSiklus;
  doc["pos"]  = posisi;
  doc["fase"] = NAMA_FASE[p.fase];
  doc["pju"]  = p.pju;
  if (p.fase == FASE_TIMER) doc["sisa"] = DURASI_TIMER - (int)(posisi / 1000);

  char buf[200];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  mqtt.publish(TOPIC_SYNC, (const uint8_t*)buf, n, false);
  mqtt.publish(topicStatus, (const uint8_t*)buf, n, true);
}

void hubungkanMQTT() {
  Serial.print("MQTT: menghubungkan... ");
  if (mqtt.connect(clientId, nullptr, nullptr, topicOnline, 1, true, "offline")) {
    Serial.println("TERHUBUNG");
    mqtt.publish(topicOnline, "online", true);
    mqtt.subscribe(TOPIC_SYNC);
  } else {
    Serial.printf("GAGAL (rc=%d)\n", mqtt.state());
  }
}

void taskJaringan(void* pvParameters) {
  // Channel terakhir hotspot, agar ESP-NOW tetap satu channel walau boot tanpa WiFi
  prefs.begin("penyeberangan", false);
  uint8_t channel = prefs.getUChar("ch", CHANNEL_DEFAULT);

  WiFi.persistent(false);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);          // modem sleep membuat paket ESP-NOW sering hilang
  WiFi.setAutoReconnect(false);  // sambung ulang diatur sendiri (lihat atas)
  kunciChannel(channel);

  if (mulaiEspNow()) {
    espNowSiap = true;
    Serial.printf("ESP-NOW: SIAP (channel %u)\n", channel);
  } else {
    Serial.println("ESP-NOW: GAGAL INIT");
  }

  espClient.setTimeout(1);   // connect TCP ke broker maks ~1 s (default 3 s)
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  mqtt.setKeepAlive(5);      // broker cepat tahu kalau node mati (LWT)
  mqtt.setSocketTimeout(2);

  Serial.println("WIFI: menyambung...");
  WiFi.begin(WIFI_SSID, WIFI_PASS, channel);
  StatusWifi status = ST_MENYAMBUNG;
  unsigned long waktuStatus = millis();   // kapan masuk status sekarang
  unsigned long waktuCobaMQTT = 0;
  unsigned long waktuLog = 0;
  PesanSync p;

  for (;;) {
    unsigned long sekarang = millis();
    bool terhubung = (WiFi.status() == WL_CONNECTED);
    bool peerSenyap = (sekarang - waktuNowTerakhir > PEER_SENYAP_MS);

    switch (status) {
      case ST_MENYAMBUNG:
        if (terhubung) {
          channel = WiFi.channel();
          if (prefs.getUChar("ch", 0) != channel) prefs.putUChar("ch", channel);
          Serial.printf("WIFI: TERHUBUNG, IP %s, channel %u\n",
                        WiFi.localIP().toString().c_str(), channel);
          status = ST_ONLINE;
          waktuCobaMQTT = sekarang - MQTT_RETRY_MS; // langsung coba MQTT
        } else if (sekarang - waktuStatus >= WIFI_TIMEOUT_MS || (sedangSiklus && !peerSenyap)) {
          // Siklus mulai & node lain masih terdengar -> jangan ganggu ESP-NOW dengan scan
          Serial.println(sedangSiklus && !peerSenyap ? "WIFI: percobaan dibatalkan (siklus berjalan)"
                                                     : "WIFI: gagal tersambung");
          WiFi.disconnect();
          kunciChannel(channel);
          Serial.printf("WIFI: OFFLINE -> sinkron lewat ESP-NOW saja (channel %u)\n", channel);
          status = ST_OFFLINE;
          waktuStatus = sekarang;
        }
        break;

      case ST_ONLINE:
        if (!terhubung) {
          mqtt.disconnect();
          WiFi.disconnect();
          kunciChannel(channel);
          Serial.printf("WIFI: TERPUTUS -> sinkron lewat ESP-NOW saja (channel %u)\n", channel);
          status = ST_OFFLINE;
          waktuStatus = sekarang;
        }
        break;

      case ST_OFFLINE:
        if (peerSenyap ? (sekarang - waktuStatus >= WIFI_RETRY_CEPAT_MS)
                       : (!sedangSiklus && sekarang - waktuStatus >= WIFI_RETRY_MS)) {
          Serial.printf("WIFI: mencoba sambung ulang%s...\n",
                        peerSenyap ? " (node lain tidak terdengar via ESP-NOW)" : "");
          WiFi.begin(WIFI_SSID, WIFI_PASS, channel);
          status = ST_MENYAMBUNG;
          waktuStatus = sekarang;
        }
        break;
    }

    bool mqttOk = (status == ST_ONLINE) && mqtt.connected();
    if (status == ST_ONLINE && !mqttOk && sekarang - waktuCobaMQTT >= MQTT_RETRY_MS) {
      waktuCobaMQTT = sekarang;
      hubungkanMQTT(); // boleh blocking: lampu & ESP-NOW tetap jalan dari core 1
      mqttOk = mqtt.connected();
    }
    if (mqttOk) mqtt.loop();

    while (xQueueReceive(qKirim, &p, 0) == pdTRUE) {
      if (mqttOk) publishMqtt(p); // tanpa MQTT: dibuang, ESP-NOW sudah terkirim dari loop()
    }

    // Log sensor: tetap di antrian sampai MQTT tersambung
    PesanLog lg;
    while (mqttOk && (xQueueReceive(qLog, &lg, 0) == pdTRUE || xQueueReceive(qLive, &lg, 0) == pdTRUE)) {
      char topik[64];
      snprintf(topik, sizeof(topik), "penyeberangan/node/%s/%s", idNode, lg.sub);
      mqtt.publish(topik, lg.isi, lg.retain);
    }

    // Ringkasan link berkala, untuk memantau saat uji WiFi putus/nyambung
    if (sekarang - waktuLog >= LOG_LINK_MS) {
      waktuLog = sekarang;
      unsigned long terakhir = waktuNowTerakhir;
      Serial.printf("LINK: WiFi=%s ch=%u MQTT=%s | ESP-NOW: ",
                    NAMA_STATUS_WIFI[status], channel, mqttOk ? "OK" : "-");
      if (terakhir == 0) Serial.println("belum ada paket dari node lain");
      else Serial.printf("node %s, %.1f s lalu\n", peerNowTerakhir,
                         (millis() - terakhir) / 1000.0f);
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// ================= FUNGSI BANTUAN ================= //

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
// Ikut posisi siklus, jadi bunyi kedua node juga sinkron.
void updateBuzzer(unsigned long sekarang) {
  bool bunyi = false;
  if (fase == FASE_TIMER) {
    unsigned long posisi = (sekarang - waktuMulaiSiklus) % 1000; // 0-999 ms di detik ini
    if (angkaTampil >= 4) {
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

// Interrupt ECHO: catat waktu naik, hitung lebar pulsa saat turun.
// Bug hardware ESP32: tiap analogRead() (piezo), GPIO36/39 bisa dapat kedipan
// LOW ~80 ns yang ikut memicu interrupt. Waktu mulai hanya dicatat saat pulsa
// benar-benar baru naik, agar kedipan di tengah pulsa tidak memotong durasi.
void IRAM_ATTR isrEcho() {
  static bool echoTinggi = false;
  unsigned long t = micros();
  echoIsrHitung++;
  if (digitalRead(PIN_ECHO) == HIGH) {
    if (!echoTinggi) {
      echoTinggi = true;
      echoMulaiUs = t;
    }
  } else if (echoTinggi) {
    echoTinggi = false;
    echoDurasiUs = t - echoMulaiUs;
    echoBaru = true;
  }
}

// Tiap 100 ms: ambil hasil echo dari trigger sebelumnya, lalu kirim trigger baru
void updateUltrasonik(unsigned long sekarang) {
  if (sekarang - waktuUltraTerakhir < INTERVAL_ULTRA_MS) return;
  waktuUltraTerakhir = sekarang;

  noInterrupts();
  bool ada = echoBaru;
  unsigned long durasi = echoDurasiUs;
  echoBaru = false;
  interrupts();

  // Diagnosa: 0 = tidak ada pulsa ECHO sama sekali (cek wiring/daya SRF05),
  // >= 30000 = ada pulsa tapi tidak ada benda dalam jangkauan
  echoTerakhirUs = ada ? durasi : 0;

  // Tanpa echo / di luar jangkauan dianggap jauh
  jarakCm = (ada && durasi > 0 && durasi < 30000) ? durasi * 0.0343f / 2.0f : 400.0f;

  // Harus beberapa bacaan berturut-turut, agar pantulan nyasar tidak ikut memicu
  if (jarakCm < batasJarak) {
    if (ultraHitung < ULTRA_KONFIRMASI) ultraHitung++;
  } else {
    ultraHitung = 0;
  }
  ultraDeteksi = (ultraHitung >= ULTRA_KONFIRMASI);

  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  if (LOG_SENSOR) {
    Serial.printf(">jarak_cm:%.1f\n", jarakCm);
    Serial.printf(">jarak_batas:%.1f\n", batasJarak);
    Serial.printf(">ultra_deteksi:%d\n", ultraDeteksi ? 1 : 0);
  }
}

// Ambang piezo (di atas baseline) saat ini
float ambangPiezo() {
  return max((float)PIEZO_DELTA_MIN, max(PIEZO_FAKTOR_NOISE * piezoNoise, piezoMarginKalib));
}

// Piezo di-sampling tiap loop, puncaknya dievaluasi tiap 10 ms
void updatePiezo(unsigned long sekarang) {
  int nilai = analogRead(PIN_PIEZO);
  if (nilai > piezoPuncak) piezoPuncak = nilai;

  if (sekarang - waktuPiezoTerakhir < INTERVAL_PIEZO_MS) return;
  waktuPiezoTerakhir = sekarang;

  float simpangan = piezoPuncak - piezoBaseline;
  if (simpangan > ambangPiezo()) {
    waktuKetukan = sekarang;
    piezoKetukan = piezoPuncak;
  } else {
    // Hanya belajar dari sinyal "diam", injakan tidak ikut menggeser baseline
    piezoBaseline += PIEZO_ALPHA * simpangan;
    piezoNoise    += PIEZO_ALPHA * (fabsf(simpangan) - piezoNoise);
  }
  piezoAktif = (waktuKetukan != 0) && (sekarang - waktuKetukan < PIEZO_HOLD_MS);
  if (piezoPuncak > piezoLiveMaks) piezoLiveMaks = piezoPuncak;

  if (LOG_SENSOR) {
    Serial.printf(">piezo:%d\n", piezoPuncak);
    Serial.printf(">piezo_ambang:%.0f\n", piezoBaseline + ambangPiezo());
    Serial.printf(">piezo_aktif:%d\n", piezoAktif ? 1 : 0);
  }
  piezoPuncak = 0;
}

// Dipanggil sekali saat boot: ukur kondisi "tidak ada orang"
//   - piezo     : baseline, rata-rata noise, dan lonjakan terbesar
//   - ultrasonik: jarak latar (median, tahan terhadap bacaan nyasar)
void kalibrasiSensor() {
  static int puncakWindow[KALIBRASI_MS / INTERVAL_PIEZO_MS + 10];
  float daftarJarak[KALIBRASI_MS / INTERVAL_ULTRA_MS + 10];
  int nPiezo = 0, nJarak = 0, puncak = 0;
  bool bacaanPertama = true;

  Serial.printf("=== KALIBRASI SENSOR %lu DETIK: jangan injak piezo / berdiri di depan sensor ===\n",
                KALIBRASI_MS / 1000);

  unsigned long mulai = millis();
  unsigned long waktuWindow = mulai;
  while (millis() - mulai < KALIBRASI_MS) {
    int nilai = analogRead(PIN_PIEZO);
    if (nilai > puncak) puncak = nilai;

    unsigned long now = millis();
    if (now - waktuWindow >= INTERVAL_PIEZO_MS) {
      waktuWindow = now;
      if (nPiezo < (int)(sizeof(puncakWindow) / sizeof(int))) puncakWindow[nPiezo++] = puncak;
      puncak = 0;
    }
    if (now - waktuUltraTerakhir >= INTERVAL_ULTRA_MS) {
      updateUltrasonik(now);
      // Bacaan pertama belum punya echo (trigger baru saja dikirim) -> lewati
      if (!bacaanPertama && nJarak < (int)(sizeof(daftarJarak) / sizeof(float)))
        daftarJarak[nJarak++] = jarakCm;
      bacaanPertama = false;
    }
  }

  // Piezo
  float jumlah = 0;
  int maks = 0;
  for (int i = 0; i < nPiezo; i++) {
    jumlah += puncakWindow[i];
    if (puncakWindow[i] > maks) maks = puncakWindow[i];
  }
  piezoBaseline = nPiezo ? jumlah / nPiezo : 0;
  float jumlahSimpangan = 0;
  for (int i = 0; i < nPiezo; i++) jumlahSimpangan += fabsf(puncakWindow[i] - piezoBaseline);
  piezoNoise = nPiezo ? jumlahSimpangan / nPiezo : 0;
  piezoMarginKalib = (maks - piezoBaseline) * PIEZO_FAKTOR_KALIB;

  // Ultrasonik: median
  for (int i = 1; i < nJarak; i++) {
    float x = daftarJarak[i];
    int j = i - 1;
    while (j >= 0 && daftarJarak[j] > x) { daftarJarak[j + 1] = daftarJarak[j]; j--; }
    daftarJarak[j + 1] = x;
  }
  jarakLatar = nJarak ? daftarJarak[nJarak / 2] : 400.0f;
  batasJarak = min(BATAS_JARAK_CM, jarakLatar - MARGIN_LATAR_CM);
  ultraHitung = 0;
  ultraDeteksi = false;

  kirimLog("kalibrasi", true,
           "PIEZO: baseline %.0f, noise rata-rata %.1f, lonjakan maks %d -> ambang %.0f | "
           "ULTRASONIK: jarak latar %.1f cm -> terdeteksi jika < %.1f cm%s",
           piezoBaseline, piezoNoise, maks, piezoBaseline + ambangPiezo(),
           jarakLatar, batasJarak,
           batasJarak < 10 ? " | PERINGATAN: ada benda terlalu dekat dengan ultrasonik saat kalibrasi!" : "");
}

// Tulis ke Serial (kalau USB tersambung) DAN kirim ke MQTT
void kirimLog(const char* sub, bool retain, const char* format, ...) {
  PesanLog lg = {};
  strlcpy(lg.sub, sub, sizeof(lg.sub));
  lg.retain = retain;
  va_list args;
  va_start(args, format);
  vsnprintf(lg.isi, sizeof(lg.isi), format, args);
  va_end(args);

  Serial.printf("[%s] %s\n", sub, lg.isi);
  xQueueSend(qLog, &lg, 0); // antrian penuh -> dibuang
}

// Data sensor berkala untuk grafik di MQTT Explorer (hanya yang terbaru yang disimpan)
void kirimSensorLive(unsigned long sekarang) {
  static unsigned long waktuTerakhir = 0;
  if (!LOG_SENSOR_MQTT || sekarang - waktuTerakhir < INTERVAL_SENSOR_LIVE_MS) return;
  waktuTerakhir = sekarang;

  PesanLog lg = {};
  strlcpy(lg.sub, "sensor_live", sizeof(lg.sub));
  snprintf(lg.isi, sizeof(lg.isi),
           "{\"piezo\":%d,\"piezo_ambang\":%.0f,\"piezo_aktif\":%d,"
           "\"jarak\":%.1f,\"jarak_batas\":%.1f,\"ultra_aktif\":%d,\"echo_us\":%lu,"
           "\"echo_isr\":%lu,\"echo_pin\":%d}",
           piezoLiveMaks, piezoBaseline + ambangPiezo(), piezoAktif ? 1 : 0,
           jarakCm, batasJarak, ultraDeteksi ? 1 : 0, echoTerakhirUs,
           (unsigned long)echoIsrHitung, digitalRead(PIN_ECHO));
  echoIsrHitung = 0;
  xQueueOverwrite(qLive, &lg);
  piezoLiveMaks = 0;
}

// true satu kali saat piezo DAN ultrasonik mulai mendeteksi bersamaan
// (mirip tepi tombol: orang yang terus berdiri tidak memicu siklus berulang).
bool pejalanKakiTerdeteksi(unsigned long sekarang) {
  static bool sebelumnya = false;

  updatePiezo(sekarang);
  updateUltrasonik(sekarang);
  kirimSensorLive(sekarang);

  bool deteksi = piezoAktif && ultraDeteksi;
  bool tepiNaik = deteksi && !sebelumnya;
  sebelumnya = deteksi;

  if (tepiNaik && fase == FASE_IDLE) {
    kirimLog("sensor", true, "LOLOS: piezo %d (ambang %.0f), jarak %.1f cm (batas %.1f cm)",
             piezoKetukan, piezoBaseline + ambangPiezo(), jarakCm, batasJarak);
  }
  return tepiNaik;
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