#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Preferences.h>

// ================= LOG DIAGNOSTIK ================= //
// Format: [millis][TAG] pesan. millis() tiap ESP berbeda, jadi untuk
// membandingkan dua Serial Monitor pakai timestamp PC (monitor_filters = time).
#define LOG(tag, fmt, ...) Serial.printf("[%7lu][%s] " fmt "\n", millis(), tag, ##__VA_ARGS__)

// ===============================================================
// Smart Safe Walking Path — Traffic Light + PJU + Buzzer + 7-Segment
// + Sinkronisasi 2 node (dua sisi penyeberangan) lewat MQTT + ESP-NOW
//
// Siklus sistem:
//   1. IDLE  : hanya Kuning kedip. Merah & Hijau mati, PJU mati, layar mati.
//              Menunggu push button (lokal) ATAU pesan START dari node lain.
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
void mulaiSiklusLokal(unsigned long sekarang);
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

// Angka 15..0 masing-masing tampil 1 detik -> fase TIMER = 16 detik
const unsigned long MS_TIMER  = (DURASI_TIMER + 1) * 1000UL;
const unsigned long MS_SIKLUS = MS_TIMER + DURASI_HIJAU_MS;

// Sinkronisasi
const unsigned long HEARTBEAT_AKTIF_MS = 500;   // kirim STATE tiap 0,5 s saat siklus jalan
const unsigned long HEARTBEAT_IDLE_MS  = 2000;  // tanda "masih hidup" saat IDLE
const long TOLERANSI_SYNC_MS = 200;             // selisih > ini -> posisi dikoreksi

// WiFi & ESP-NOW
const uint8_t CHANNEL_DEFAULT = 1;                // dipakai jika belum pernah tersambung hotspot
const unsigned long WIFI_CONNECT_TIMEOUT_MS = 8000; // batas sambung setelah hotspot terlihat di scan
const unsigned long WIFI_RETRY_MS        = 15000; // jeda percobaan saat offline (hanya saat IDLE)
const unsigned long WIFI_RETRY_SENYAP_MS = 8000;  // jeda saat node lain tidak terdengar via ESP-NOW
const unsigned long WIFI_RETRY_JITTER_MS = 3000;  // acak, agar dua node tidak scan bersamaan
const uint8_t  SCAN_PENUH_TIAP  = 4;              // tiap N percobaan scan SEMUA channel
const uint32_t SCAN_DWELL_MS    = 120;            // lama scan per channel
const unsigned long SCAN_BATAS_MS = 5000;         // pengaman jika scan tidak pernah selesai
const unsigned long MQTT_RETRY_MS   = 5000;       // jeda percobaan sambung ke broker
const unsigned long ESPNOW_SEGAR_MS = 3000;       // ESP-NOW dianggap aktif jika ada paket < 3 s lalu
const unsigned long PEER_SENYAP_MS  = 6000;       // tidak ada paket ESP-NOW > 6 s -> mungkin beda channel
const unsigned long LOG_LINK_MS     = 10000;      // ringkasan status link di Serial

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

WiFiClient espClient;
PubSubClient mqtt(espClient);
char idNode[16];
char clientId[32];
char topicStatus[48];
char topicOnline[48];

// Dibaca lintas core (taskJaringan <-> loop)
volatile bool sedangSiklus = false;            // true = siklus berjalan, jangan scan WiFi
volatile bool espNowSiap = false;
volatile bool mqttSiap = false;                // WiFi ONLINE + broker tersambung
volatile unsigned long waktuNowTerakhir = 0;   // millis() paket ESP-NOW terakhir dari node lain
char peerNowTerakhir[16] = "";

// Diagnostik (diisi callback WiFi/ESP-NOW, dicetak di taskJaringan)
volatile uint32_t nowKirimOk = 0;      // send callback: frame terpancar
volatile uint32_t nowKirimGagal = 0;   // send callback / esp_now_send gagal
volatile uint32_t nowTerima = 0;       // paket ESP-NOW valid dari node lain
volatile uint8_t  alasanPutusWifi = 0; // reason code STA_DISCONNECTED terakhir
volatile bool     adaPutusWifi = false;

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
  if (qKirim == NULL || qTerima == NULL) {
    Serial.println("FATAL: QUEUE GAGAL DIBUAT");
    while (1) delay(1000);
  }

  // Jaringan di core 0, logika lampu tetap di loop() (core 1)
  xTaskCreatePinnedToCore(taskJaringan, "Task_Jaringan", 6144, NULL, 1, NULL, 0);

  if (TES_RELAY_SAAT_BOOT) tesRelay();

  waktuKedipTerakhir = millis();
  Serial.println("=== SISTEM SIAP: IDLE (KUNING KEDIP) ===");
}

// ================= LOOP UTAMA ================= //
void loop() {
  bool ditekan = tombolDitekan(); // dibaca tiap loop agar debounce tetap akurat

  // Pesan dari node lain
  PesanSync masuk;
  while (xQueueReceive(qTerima, &masuk, 0) == pdTRUE) {
    prosesPesanPeer(masuk);
  }

  unsigned long sekarang = millis();

  // Tombol hanya memulai siklus saat IDLE
  if (ditekan && fase == FASE_IDLE) mulaiSiklusLokal(sekarang);

  updateSiklus(sekarang);
  sedangSiklus = (idSiklus != 0);

  unsigned long intervalHB = idSiklus ? HEARTBEAT_AKTIF_MS : HEARTBEAT_IDLE_MS;
  if (sekarang - waktuHeartbeat >= intervalHB) kirimState("STATE", sekarang);

  updateBuzzer(sekarang);
  updateLampu(sekarang);
}

// ================= SIKLUS & SINKRONISASI ================= //

// Dipanggil saat push button ditekan di fase IDLE
void mulaiSiklusLokal(unsigned long sekarang) {
  // LDR hanya dibaca saat tombol ditekan, hasilnya dibagikan ke node lain
  int bacaanLDR = digitalRead(PIN_LDR);
  bool gelap = (bacaanLDR == LDR_GELAP);

  LOG("BTN", "TOMBOL DITEKAN -> TIMER MULAI -> MERAH");
  LOG("BTN", "LDR DO: %s -> %s -> PJU: %s",
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
      LOG("FASE", "MERAH (siklus %08lX, pemilik %s)", (unsigned long)idSiklus, pemilikSiklus);
      break;

    case FASE_HIJAU:
      digitalWrite(PIN_PILOT, LOW);
      resetSevenSegment();
      angkaTampil = -1;
      LOG("FASE", "TIMER SELESAI -> HIJAU 7 DETIK");
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
      LOG("FASE", "HIJAU SELESAI -> PJU OFF -> IDLE (KUNING KEDIP)");
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
      LOG("CD", "COUNTDOWN: %d", angka);
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

    // posisi > ~600 ms berarti paket START / heartbeat awal tidak sampai (radio tuli?)
    LOG("SYNC", "IKUT SIKLUS %08lX DARI NODE %s via %s (%s, posisi %lu ms, antri %lu ms)",
        (unsigned long)p.idSiklus, p.asal, NAMA_JALUR[p.jalur], p.evt,
        (unsigned long)posisi, sekarang - p.stempel);
    aturSiklus(p.idSiklus, p.asal, p.pju, posisi, sekarang);

  } else if (p.idSiklus == idSiklus) {
    // Siklus sama: hanya pemilik siklus yang jadi acuan koreksi drift
    if (strcmp(p.asal, pemilikSiklus) != 0) return;
    // Latensi MQTT lebih besar & tidak stabil; selama ESP-NOW aktif, pakai ESP-NOW saja
    if (p.jalur == JALUR_MQTT && sekarang - waktuNowTerakhir < ESPNOW_SEGAR_MS) return;

    long selisih = (long)(sekarang - waktuMulaiSiklus) - (long)posisi;
    if (labs(selisih) > TOLERANSI_SYNC_MS) {
      LOG("SYNC", "KOREKSI DRIFT %ld ms via %s", selisih, NAMA_JALUR[p.jalur]);
      waktuMulaiSiklus = sekarang - posisi;
      updateSiklus(sekarang);
    }

  } else if (p.idSiklus < idSiklus) {
    // Tombol ditekan hampir bersamaan di kedua sisi: id terkecil menang
    LOG("SYNC", "KONFLIK SIKLUS, IKUT %08lX DARI NODE %s via %s",
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
    esp_err_t r = esp_now_send(ALAMAT_BROADCAST, (const uint8_t*)&pk, sizeof(pk));
    if (r != ESP_OK) {
      nowKirimGagal++;
      LOG("NOW", "esp_now_send %s GAGAL: %s", evt, esp_err_to_name(r));
    } else if (strcmp(evt, "STATE") != 0) {
      // Heartbeat tidak dicetak agar log tidak banjir
      LOG("NOW", "TX %s siklus %08lX posisi %lu ms", evt,
          (unsigned long)p.idSiklus, (unsigned long)p.posisiMs);
    }
  } else if (strcmp(evt, "START") == 0) {
    LOG("NOW", "TX START dilewati: ESP-NOW belum siap");
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
// ESP-NOW hanya sampai jika kedua ESP berada di channel yang sama (radio cuma satu):
//   - WiFi tersambung : channel ikut hotspot (dicatat ke flash)
//   - WiFi putus      : auto-reconnect dimatikan, radio dikunci di channel terakhir
//   - Sambung ulang   : HANYA saat IDLE (tidak pernah saat PREPARING/COUNTDOWN/HIJAU):
//       1. scan async. Biasanya hanya channel tersimpan (~0,1 s, radio tetap di
//          channel itu). Scan semua channel (~2 s) tiap SCAN_PENUH_TIAP percobaan
//          atau saat node lain tidak terdengar (hotspot mungkin pindah channel).
//       2. setelah scan radio LANGSUNG dikunci lagi ke channel ESP-NOW.
//       3. WiFi.begin() hanya jika hotspot terlihat, langsung ke channel + BSSID-nya.
//   - Siklus mulai saat scan / menyambung -> dibatalkan, radio dikunci lagi.
// PubSubClient TIDAK thread-safe, jadi hanya dipakai di task ini.

enum StatusWifi { ST_OFFLINE, ST_SCAN, ST_MENYAMBUNG, ST_ONLINE };
const char* NAMA_STATUS_WIFI[] = { "OFFLINE", "SCAN", "MENYAMBUNG", "ONLINE" };

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
  nowTerima++;
  strlcpy(peerNowTerakhir, pk.asal, sizeof(peerNowTerakhir));
  xQueueSend(qTerima, &p, 0);
}

// Broadcast tidak punya ACK di level MAC: SUCCESS hanya berarti frame terpancar,
// BUKAN berarti node lain menerima / memproses pesan.
void espNowTerkirim(const uint8_t* mac, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) nowKirimOk++;
  else nowKirimGagal++;
}

// Dipanggil task event WiFi: hanya simpan, dicetak oleh taskJaringan
void wifiEvent(WiFiEvent_t event, WiFiEventInfo_t info) {
  if (event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
    alasanPutusWifi = info.wifi_sta_disconnected.reason;
    adaPutusWifi = true;
  }
}

// Channel radio yang benar-benar aktif saat ini
uint8_t channelRadio() {
  uint8_t primary = 0;
  wifi_second_chan_t second;
  esp_wifi_get_channel(&primary, &second);
  return primary;
}

bool mulaiEspNow() {
  if (esp_now_init() != ESP_OK) return false;
  esp_now_register_recv_cb(espNowTerima);
  esp_now_register_send_cb(espNowTerkirim);

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

// rc PubSubClient: -4 timeout, -2 TCP gagal (IP salah / firewall / listener
// Mosquitto hanya localhost), 2 client ID ditolak, 5 tidak diizinkan (allow_anonymous)
void hubungkanMQTT() {
  LOG("MQTT", "menghubungkan ke %s:%u sebagai %s ...", MQTT_HOST, MQTT_PORT, clientId);
  unsigned long t0 = millis();
  if (mqtt.connect(clientId, nullptr, nullptr, topicOnline, 1, true, "offline")) {
    LOG("MQTT", "TERHUBUNG (%lu ms)", millis() - t0);
    mqtt.publish(topicOnline, "online", true);
    mqtt.subscribe(TOPIC_SYNC);
  } else {
    LOG("MQTT", "GAGAL rc=%d (%lu ms)", mqtt.state(), millis() - t0);
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

  LOG("NET", "MAC STA %s, channel tersimpan %u, channel radio %u",
      WiFi.macAddress().c_str(), channel, channelRadio());
  WiFi.onEvent(wifiEvent);

  if (mulaiEspNow()) {
    espNowSiap = true;
    LOG("NOW", "SIAP (channel radio %u)", channelRadio());
  } else {
    LOG("NOW", "GAGAL INIT");
  }

  espClient.setTimeout(1);   // connect TCP ke broker maks ~1 s (default 3 s)
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(mqttCallback);
  mqtt.setKeepAlive(5);      // broker cepat tahu kalau node mati (LWT)
  mqtt.setSocketTimeout(2);

  StatusWifi status = ST_OFFLINE;
  unsigned long waktuStatus = millis();      // kapan masuk status sekarang
  unsigned long waktuCobaBerikut = millis(); // boot = IDLE -> langsung coba
  unsigned long waktuCobaMQTT = 0;
  unsigned long waktuLog = 0;
  uint8_t percobaan = 0;                     // menentukan kapan scan semua channel
  bool scanPenuh = false;
  PesanSync p;

  // Jadwal percobaan berikutnya (+ jitter acak)
  auto jadwalUlang = [&](unsigned long sekarang, bool peerSenyap) {
    unsigned long jeda = (peerSenyap ? WIFI_RETRY_SENYAP_MS : WIFI_RETRY_MS) + esp_random() % WIFI_RETRY_JITTER_MS;
    waktuCobaBerikut = sekarang + jeda;
  };
  // Kembali OFFLINE dan pastikan radio di channel ESP-NOW
  auto keOffline = [&](unsigned long sekarang, bool peerSenyap, const char* alasan) {
    WiFi.disconnect();
    kunciChannel(channel);
    status = ST_OFFLINE;
    waktuStatus = sekarang;
    jadwalUlang(sekarang, peerSenyap);
    LOG("WIFI", "OFFLINE (%s) -> radio dikunci ch %u (terbaca %u), coba lagi %lu s",
        alasan, channel, channelRadio(), (waktuCobaBerikut - sekarang) / 1000);
  };

  for (;;) {
    unsigned long sekarang = millis();
    bool terhubung = (WiFi.status() == WL_CONNECTED);
    bool peerSenyap = (sekarang - waktuNowTerakhir > PEER_SENYAP_MS);
    bool gagalCepat = false;

    if (adaPutusWifi) {
      adaPutusWifi = false;
      uint8_t alasan = alasanPutusWifi;
      // 201 NO_AP_FOUND: SSID tidak terlihat (hotspot mati / 5 GHz)
      // 15/202/204 : password salah / handshake gagal
      LOG("WIFI", "STA_DISCONNECTED reason %u (%s), status=%s, channel radio %u",
          alasan, WiFi.disconnectReasonName((wifi_err_reason_t)alasan),
          NAMA_STATUS_WIFI[status], channelRadio());
      gagalCepat = (alasan == WIFI_REASON_NO_AP_FOUND || alasan == WIFI_REASON_AUTH_FAIL ||
                    alasan == WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT || alasan == WIFI_REASON_HANDSHAKE_TIMEOUT);
    }

    switch (status) {
      case ST_OFFLINE:
        if (terhubung) {
          // Retry bawaan framework (sekali setelah boot) ternyata berhasil
          status = ST_MENYAMBUNG;
          break;
        }
        if (!sedangSiklus && (long)(sekarang - waktuCobaBerikut) >= 0) {
          scanPenuh = peerSenyap || (percobaan % SCAN_PENUH_TIAP == 0);
          percobaan++;
          int16_t r = scanPenuh ? WiFi.scanNetworks(true, false, false, SCAN_DWELL_MS)
                                : WiFi.scanNetworks(true, false, false, SCAN_DWELL_MS, channel);
          if (r == WIFI_SCAN_FAILED) {
            LOG("WIFI", "scan gagal dimulai");
            keOffline(sekarang, peerSenyap, "scan gagal");
          } else {
            LOG("WIFI", "scan %s mencari \"%s\"%s...", scanPenuh ? "SEMUA channel" : "channel tersimpan",
                WIFI_SSID, peerSenyap ? " (node lain tidak terdengar via ESP-NOW)" : "");
            status = ST_SCAN;
            waktuStatus = sekarang;
          }
        }
        break;

      case ST_SCAN: {
        int16_t n = WiFi.scanComplete();
        if (n == WIFI_SCAN_RUNNING) {
          if (sedangSiklus) {
            esp_wifi_scan_stop();
            WiFi.scanDelete();
            keOffline(sekarang, peerSenyap, "scan dibatalkan, siklus mulai");
          } else if (sekarang - waktuStatus > SCAN_BATAS_MS) {
            esp_wifi_scan_stop();
            WiFi.scanDelete();
            keOffline(sekarang, peerSenyap, "scan tidak selesai");
          }
          break;
        }

        // Cari hotspot dengan sinyal terkuat
        int terbaik = -1;
        for (int i = 0; i < n; i++) {
          if (WiFi.SSID(i) == WIFI_SSID && (terbaik < 0 || WiFi.RSSI(i) > WiFi.RSSI(terbaik))) terbaik = i;
        }
        uint8_t chAp = 0;
        uint8_t bssid[6];
        int32_t rssi = 0;
        if (terbaik >= 0) {
          chAp = WiFi.channel(terbaik);
          rssi = WiFi.RSSI(terbaik);
          memcpy(bssid, WiFi.BSSID(terbaik), 6);
        }
        WiFi.scanDelete();
        LOG("WIFI", "scan selesai %lu ms, %d jaringan, channel radio %u",
            sekarang - waktuStatus, (int)max(n, (int16_t)0), channelRadio());

        if (terbaik < 0 || sedangSiklus) {
          keOffline(sekarang, peerSenyap, terbaik < 0 ? "hotspot tidak terlihat" : "siklus mulai");
          break;
        }
        if (chAp != channel) {
          LOG("WIFI", "PERHATIAN: hotspot di ch %u, ESP-NOW sekarang ch %u -> ESP-NOW ikut pindah", chAp, channel);
        }
        LOG("WIFI", "hotspot terlihat ch %u RSSI %ld dBm -> menyambung", chAp, (long)rssi);
        adaPutusWifi = false;
        WiFi.begin(WIFI_SSID, WIFI_PASS, chAp, bssid);
        status = ST_MENYAMBUNG;
        waktuStatus = sekarang;
        break;
      }

      case ST_MENYAMBUNG:
        if (terhubung) {
          channel = WiFi.channel();
          if (prefs.getUChar("ch", 0) != channel) prefs.putUChar("ch", channel);
          LOG("WIFI", "TERHUBUNG, IP %s, channel %u (%lu ms)",
              WiFi.localIP().toString().c_str(), channel, sekarang - waktuStatus);
          status = ST_ONLINE;
          percobaan = 0;
          waktuCobaMQTT = sekarang - MQTT_RETRY_MS; // langsung coba MQTT
        } else if (sedangSiklus) {
          keOffline(sekarang, peerSenyap, "menyambung dibatalkan, siklus mulai");
        } else if (gagalCepat) {
          keOffline(sekarang, peerSenyap, "hotspot menolak / tidak ditemukan");
        } else if (sekarang - waktuStatus >= WIFI_CONNECT_TIMEOUT_MS) {
          keOffline(sekarang, peerSenyap, "timeout menyambung");
        }
        break;

      case ST_ONLINE:
        if (!terhubung) {
          mqtt.disconnect();
          keOffline(sekarang, peerSenyap, "WiFi terputus, sinkron lewat ESP-NOW");
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
    if (mqttSiap != mqttOk) {
      mqttSiap = mqttOk;
      if (!mqttOk) LOG("MQTT", "TIDAK SIAP (WiFi=%s, rc=%d)", NAMA_STATUS_WIFI[status], mqtt.state());
    }

    while (xQueueReceive(qKirim, &p, 0) == pdTRUE) {
      if (mqttOk) publishMqtt(p); // tanpa MQTT: dibuang, ESP-NOW sudah terkirim dari loop()
    }

    // Ringkasan link berkala, untuk memantau saat uji WiFi putus/nyambung
    if (sekarang - waktuLog >= LOG_LINK_MS) {
      waktuLog = sekarang;
      unsigned long terakhir = waktuNowTerakhir;
      char peer[48];
      if (terakhir == 0) strlcpy(peer, "belum ada paket dari node lain", sizeof(peer));
      else snprintf(peer, sizeof(peer), "node %s %.1f s lalu", peerNowTerakhir,
                    (millis() - terakhir) / 1000.0f);
      LOG("LINK", "WiFi=%s ch_simpan=%u ch_radio=%u MQTT=%s | ESP-NOW: %s | tx ok=%lu gagal=%lu rx=%lu",
          NAMA_STATUS_WIFI[status], channel, channelRadio(), mqttOk ? "OK" : "-", peer,
          (unsigned long)nowKirimOk, (unsigned long)nowKirimGagal, (unsigned long)nowTerima);
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