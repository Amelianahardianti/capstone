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
// Siklus sistem (posisi = ms sejak tombol ditekan, sama di kedua node):
//   1. IDLE      : Kuning kedip. Merah & Hijau mati, PJU mati, layar mati.
//   2. PREPARING : 0 .. DURASI_PREPARING_MS. Kuning TETAP kedip, buzzer Node A
//                  bunyi panjang, countdown belum jalan. Di sini kedua node
//                  sinkronisasi: START -> ACK -> COMMIT -> ACK.
//   3. COUNTDOWN : Merah nyala, PILOT nyala, PJU sesuai LDR, 7-segment 15 -> 0,
//                  buzzer pola lama. HANYA jika sinkronisasi terkonfirmasi.
//   4. HIJAU     : Hijau nyala 7 detik.
//   -> IDLE (PJU dimatikan, kuning kedip lagi).
//   FAILSAFE     : sinkronisasi tidak terkonfirmasi sampai PREPARING habis.
//                  Countdown TIDAK dimulai, kuning kedip, Node A bunyi 3x pendek,
//                  lalu IDLE lagi (boleh coba ulang).
//                  CATATAN: ini perilaku PROTOTIPE. Untuk jalan sungguhan, kondisi
//                  aman perlu dirancang ulang (mis. kunci fase penyeberangan +
//                  alarm perawatan), bukan sekadar kembali ke standby.
//
// Sinkronisasi (arsitektur simetris, tidak ada master/slave tetap):
//   - Node yang tombolnya ditekan = koordinator (pemilik siklus), node lain = pengikut.
//   - Tombol ditekan hampir bersamaan -> idSiklus (cycleId) terkecil menang.
//   - Jalur: MQTT (utama) jika broker siap & node lain online di MQTT.
//     Tanpa ACK dalam MQTT_ACK_TIMEOUT_MS / MQTT putus -> fallback ESP-NOW.
//     Pesan siklus TIDAK dikirim lewat dua jalur sekaligus (kecuali ABORT).
//   - Setelah COMMIT, timeline kedua node deterministik dari titik nol yang sama,
//     jadi pergantian / putusnya jalur tidak mengubah siklus yang sudah disepakati.
//   - Pesan duplikat (senderId + sequenceId) dibuang; START basi / siklus lama diabaikan.
//
// Kedua ESP memakai kode yang SAMA. Bedanya hanya NODE_ID & ADA_BUZZER (platformio.ini).
// Semua proses lampu berbasis millis() (tanpa delay), jaringan di task core 0.
// ===============================================================

// ================= IDENTITAS NODE ================= //
// Diisi dari build_flags di platformio.ini (env node_a / node_b).
// Jika kosong, ID diambil otomatis dari MAC address.
#ifndef NODE_ID
#define NODE_ID ""
#endif

// 1 = node ini punya buzzer di PIN_BUZZER (Node A). 0 = GPIO buzzer tidak disentuh.
#ifndef ADA_BUZZER
#define ADA_BUZZER 0
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
const char* TOPIC_ONLINE_SEMUA = "penyeberangan/node/+/online";

struct PesanSync;
void tampilkanAngka(int angka);
void matikanSevenSegment();
void resetSevenSegment();
bool tombolDitekan();
void mulaiSiklusLokal(unsigned long sekarang);
void aturSiklus(uint32_t id, const char* pemilik, bool pemilikSaya, bool pju, uint32_t posisi, unsigned long sekarang);
void updateSiklus(unsigned long sekarang);
void jalankanHandshake(unsigned long sekarang);
void masukFailsafe(unsigned long sekarang, const char* alasan);
void prosesPesan(const PesanSync& p);
uint32_t kirimPesan(uint8_t tipe, uint8_t jalur, unsigned long sekarang, uint8_t ackTipe = 0, uint32_t ackSeq = 0);
void kirimHeartbeat(unsigned long sekarang);
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

const unsigned long DURASI_PREPARING_MS = 4000; // PREPARING sebelum countdown (3000-5000 ms)
const int DURASI_TIMER = 15;                    // Hitung mundur 15 -> 0 (detik)
const unsigned long DURASI_HIJAU_MS = 7000;     // Hijau nyala 7 detik setelah countdown
const unsigned long DURASI_FAILSAFE_MS = 3000;  // lama FAILSAFE sebelum kembali IDLE
const unsigned long KEDIP_KUNING_MS = 500;      // Kuning ON 0,5 s / OFF 0,5 s
const unsigned long DEBOUNCE_MS = 50;           // Debounce tombol
const unsigned long BUZZER_JEDA_PREP_MS = 200;  // bunyi panjang berhenti sesaat sebelum countdown

static_assert(DURASI_PREPARING_MS >= 1500 && DURASI_PREPARING_MS <= 10000,
              "DURASI_PREPARING_MS harus cukup untuk handshake");

// Timeline siklus. Angka 15..0 masing-masing tampil 1 detik -> COUNTDOWN 16 detik
const unsigned long MS_PREP      = DURASI_PREPARING_MS;          // countdown mulai (scheduledStart)
const unsigned long MS_TIMER     = (DURASI_TIMER + 1) * 1000UL;
const unsigned long MS_TIMER_END = MS_PREP + MS_TIMER;
const unsigned long MS_SIKLUS    = MS_TIMER_END + DURASI_HIJAU_MS;

// Handshake START -> ACK(START) -> COMMIT -> ACK(COMMIT)
const unsigned long MQTT_ACK_TIMEOUT_MS = 1000;  // tanpa ACK via MQTT -> fallback ESP-NOW
const unsigned long KIRIM_ULANG_MQTT_MS = 250;   // kirim ulang START/COMMIT via MQTT
const unsigned long KIRIM_ULANG_NOW_MS  = 150;   // kirim ulang START/COMMIT via ESP-NOW
const unsigned long BATAS_COMMIT_MS        = MS_PREP - 500; // ACK START setelah ini -> tidak di-COMMIT
const unsigned long BATAS_KIRIM_COMMIT_MS  = MS_PREP - 250; // koordinator berhenti kirim COMMIT
const unsigned long BATAS_TERIMA_COMMIT_MS = MS_PREP - 200; // pengikut menolak COMMIT setelah ini

// Heartbeat
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
enum TipePesan : uint8_t { MSG_STATE, MSG_START, MSG_ACK, MSG_COMMIT, MSG_ABORT };
const char* NAMA_TIPE[] = { "STATE", "START", "ACK", "COMMIT", "ABORT" };
const uint8_t JUMLAH_TIPE = 5;

enum JalurPesan : uint8_t { JALUR_LOKAL, JALUR_MQTT, JALUR_ESPNOW };
const char* NAMA_JALUR[] = { "LOKAL", "MQTT", "ESP-NOW" };

enum TujuanMqtt : uint8_t { KE_SYNC, KE_STATUS };

struct PesanSync {
  uint8_t  tipe;        // messageType (TipePesan)
  uint8_t  fase;        // state pengirim (FaseSistem)
  uint8_t  pju;         // keputusan PJU dari pemilik siklus
  uint8_t  ackTipe;     // ACK: tipe pesan yang dibalas (START / COMMIT)
  uint8_t  channel;     // channel radio pengirim (validasi ESP-NOW)
  uint8_t  jalur;       // JalurPesan: lewat mana dikirim / diterima
  uint8_t  tujuanMqtt;  // khusus antrian MQTT: topik sync atau status
  char     asal[16];    // senderId (NODE_ID)
  uint32_t seq;         // sequenceId, naik tiap pesan dari pengirim
  uint32_t idSiklus;    // cycleId, 0 = pengirim tidak sedang siklus
  uint32_t posisiMs;    // ms sejak siklus dimulai (saat pesan dibuat)
  uint32_t ackSeq;      // ACK: sequenceId pesan yang dibalas
  uint16_t rttSetengah; // COMMIT: estimasi latensi satu arah (ms)
  uint16_t jadwalMulai; // scheduledStart: posisi mulai countdown (ms)
  uint16_t durTimer;    // duration countdown (detik)
  uint16_t durHijau;    // duration hijau (ms)
  uint32_t stempel;     // millis() lokal saat posisiMs dicatat (kompensasi antrian)
};

const uint8_t ALAMAT_BROADCAST[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
const uint32_t MAGIC_PAKET = 0x50594232; // "PYB2", menyaring paket ESP-NOW lain / versi lama
const uint8_t VERSI_PROTOKOL = 2;

// Format paket ESP-NOW (biner, ukuran tetap)
struct __attribute__((packed)) PaketNow {
  uint32_t magic;
  uint8_t  versi;
  uint8_t  tipe;
  uint8_t  fase;
  uint8_t  pju;
  uint8_t  ackTipe;
  uint8_t  channel;
  char     asal[16];
  uint32_t seq;
  uint32_t idSiklus;
  uint32_t posisiMs;
  uint32_t ackSeq;
  uint16_t rttSetengah;
  uint16_t jadwalMulai;
  uint16_t durTimer;
  uint16_t durHijau;
};

QueueHandle_t qKirim  = NULL;  // loop() -> taskJaringan (khusus MQTT)
QueueHandle_t qTerima = NULL;  // taskJaringan / callback ESP-NOW -> loop()

WiFiClient espClient;
PubSubClient mqtt(espClient);
char idNode[16];
char clientId[32];
char topicStatus[48];
char topicOnline[48];

// Dibaca lintas core (taskJaringan <-> loop)
volatile bool sedangSiklus = false;            // true = bukan IDLE, jangan scan / sambung WiFi
volatile bool espNowSiap = false;
volatile bool mqttSiap = false;                // WiFi ONLINE + broker tersambung
volatile bool peerMqttOnline = false;          // node lain "online" di MQTT (topik LWT)
volatile uint8_t channelSekarang = 0;          // channel radio, diperbarui taskJaringan
volatile unsigned long waktuNowTerakhir = 0;   // millis() paket ESP-NOW terakhir dari node lain
char peerNowTerakhir[16] = "";

// Diagnostik (diisi callback WiFi/ESP-NOW, dicetak di taskJaringan)
volatile uint32_t nowKirimOk = 0;      // send callback: frame terpancar
volatile uint32_t nowKirimGagal = 0;   // send callback / esp_now_send gagal
volatile uint32_t nowTerima = 0;       // paket ESP-NOW valid dari node lain
volatile uint32_t antrianPenuh = 0;    // pesan masuk dibuang karena qTerima penuh
volatile uint8_t  alasanPutusWifi = 0; // reason code STA_DISCONNECTED terakhir
volatile bool     adaPutusWifi = false;

// ================= VARIABEL SISTEM ================= //
// Urutan penting: fase hanya boleh maju (lihat updateSiklus)
enum FaseSistem { FASE_IDLE, FASE_PREP, FASE_TIMER, FASE_HIJAU, FASE_FAILSAFE };
FaseSistem fase = FASE_IDLE;
const char* NAMA_FASE[] = { "IDLE", "PREPARING", "COUNTDOWN", "HIJAU", "FAILSAFE" };
const uint8_t JUMLAH_FASE = 5;

uint32_t idSiklus = 0;                 // 0 = tidak ada siklus berjalan
uint32_t idSiklusSelesai = 0;          // siklus terakhir yang selesai normal
uint32_t idSiklusGagal = 0;            // siklus terakhir yang berakhir FAILSAFE
char pemilikSiklus[16] = "";
bool sayaPemilik = false;              // true = node ini koordinator siklus
unsigned long waktuMulaiSiklus = 0;
bool pjuSiklus = false;                // keputusan PJU (LDR koordinator), dinyalakan saat COUNTDOWN
bool statusPJU = false;                // PJU sedang menyala
bool syncOK = false;                   // koordinator: ACK(COMMIT) diterima | pengikut: COMMIT diterima
bool commitTerkirim = false;           // koordinator: ACK(START) diterima, COMMIT sedang dikirim
uint8_t jalurSiklus = JALUR_ESPNOW;    // jalur yang dipakai siklus ini
unsigned long waktuJalurMulai = 0;     // kapan langkah handshake sekarang mulai di jalur ini
unsigned long waktuKirimUlang = 0;
bool kirimSegera = false;
uint16_t rttSetengah = 0;              // dikirim di COMMIT
uint16_t rttJalur[3] = { 0, 0, 0 };    // RTT/2 terukur per jalur
unsigned long waktuFailsafe = 0;
int angkaTampil = -1;                  // angka yang sedang tampil di 7-segment
unsigned long waktuHeartbeat = 0;
uint32_t seqKirim = 0;                 // sequenceId terakhir yang dikirim
unsigned long waktuPeringatan = 0;     // pembatas log peringatan channel / durasi

// Waktu kirim START (untuk mengukur RTT dari ACK)
struct CatatanKirim { uint32_t seq; unsigned long waktu; };
CatatanKirim startTerkirim[8];
uint8_t idxStart = 0;

// Deduplikasi: (senderId, sequenceId) yang baru diproses
struct IdPesan { char asal[16]; uint32_t seq; };
IdPesan pesanTerakhir[16];
uint8_t idxPesan = 0;

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
  seqKirim = esp_random();

  Serial.printf("\n=== ESP32 START — NODE %s ===\n", idNode);
  LOG("BOOT", "buzzer: %s | PREPARING %lu ms, countdown %d s, hijau %lu ms",
      ADA_BUZZER ? "ADA (GPIO13)" : "tidak ada", MS_PREP, DURASI_TIMER, DURASI_HIJAU_MS);

  qKirim  = xQueueCreate(10, sizeof(PesanSync));
  qTerima = xQueueCreate(16, sizeof(PesanSync));
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
    prosesPesan(masuk);
  }

  unsigned long sekarang = millis();

  // Tombol hanya memulai siklus saat IDLE (ditekan berulang tidak membuat siklus ganda)
  if (ditekan) {
    if (fase == FASE_IDLE) mulaiSiklusLokal(sekarang);
    else LOG("BTN", "tombol diabaikan (fase %s)", NAMA_FASE[fase]);
  }

  jalankanHandshake(sekarang);
  updateSiklus(sekarang);
  sedangSiklus = (fase != FASE_IDLE);

  unsigned long intervalHB = idSiklus ? HEARTBEAT_AKTIF_MS : HEARTBEAT_IDLE_MS;
  if (sekarang - waktuHeartbeat >= intervalHB) kirimHeartbeat(sekarang);

  updateBuzzer(sekarang);
  updateLampu(sekarang);
}

// ================= SIKLUS & STATE MACHINE ================= //

// Dipanggil saat push button ditekan di fase IDLE -> node ini koordinator
void mulaiSiklusLokal(unsigned long sekarang) {
  // LDR hanya dibaca saat tombol ditekan, hasilnya dibagikan ke node lain
  int bacaanLDR = digitalRead(PIN_LDR);
  bool gelap = (bacaanLDR == LDR_GELAP);

  LOG("BTN", "TOMBOL DITEKAN -> PREPARING %lu ms", MS_PREP);
  LOG("BTN", "LDR DO: %s -> %s -> PJU saat COUNTDOWN: %s",
      bacaanLDR == HIGH ? "HIGH" : "LOW",
      gelap ? "GELAP" : "TERANG",
      gelap ? "ON" : "OFF");

  aturSiklus(esp_random() | 1, idNode, true, gelap, 0, sekarang); // |1 agar id tidak pernah 0

  // MQTT jalur utama, tapi hanya jika node lain juga terlihat online di broker
  jalurSiklus = (mqttSiap && peerMqttOnline) ? JALUR_MQTT : JALUR_ESPNOW;
  LOG("SYNC", "KOORDINATOR siklus %08lX, jalur awal %s (%s)", (unsigned long)idSiklus,
      NAMA_JALUR[jalurSiklus],
      !mqttSiap ? "MQTT tidak siap" : (peerMqttOnline ? "MQTT siap" : "node lain offline di MQTT"));
  updateSiklus(sekarang);  // IDLE -> PREPARING
  jalankanHandshake(sekarang);  // START pertama langsung dikirim
}

// Pasang siklus (milik sendiri atau ikut node lain) pada posisi tertentu
void aturSiklus(uint32_t id, const char* pemilik, bool pemilikSaya, bool pju, uint32_t posisi, unsigned long sekarang) {
  idSiklus = id;
  strlcpy(pemilikSiklus, pemilik, sizeof(pemilikSiklus));
  sayaPemilik = pemilikSaya;
  pjuSiklus = pju;
  waktuMulaiSiklus = sekarang - posisi;
  syncOK = false;
  commitTerkirim = false;
  rttSetengah = 0;
  waktuJalurMulai = sekarang;
  kirimSegera = true;
  angkaTampil = -1; // paksa 7-segment ditulis ulang
}

void gantiFase(FaseSistem baru, unsigned long sekarang) {
  FaseSistem lama = fase;
  fase = baru;
  switch (baru) {
    case FASE_PREP:
      // Lampu & relay tidak berubah: kuning tetap kedip, merah/hijau/PJU mati
      LOG("FASE", "PREPARING (siklus %08lX, pemilik %s%s) -> countdown di posisi %lu ms",
          (unsigned long)idSiklus, pemilikSiklus, sayaPemilik ? ", node ini koordinator" : "", MS_PREP);
      break;

    case FASE_TIMER:
    case FASE_HIJAU:
      // PJU dinyalakan saat penyeberangan benar-benar dimulai (bukan saat PREPARING)
      if (statusPJU != pjuSiklus) {
        statusPJU = pjuSiklus;
        digitalWrite(PIN_PJU, statusPJU ? RELAY_ON : RELAY_OFF);
      }
      if (baru == FASE_TIMER) {
        digitalWrite(PIN_PILOT, HIGH);
        LOG("FASE", "COUNTDOWN MULAI -> MERAH (siklus %08lX, pemilik %s, jalur %s, PJU %s)",
            (unsigned long)idSiklus, pemilikSiklus, NAMA_JALUR[jalurSiklus], statusPJU ? "ON" : "OFF");
      } else {
        digitalWrite(PIN_PILOT, LOW);
        resetSevenSegment();
        angkaTampil = -1;
        LOG("FASE", "TIMER SELESAI -> HIJAU %lu ms", DURASI_HIJAU_MS);
      }
      break;

    case FASE_IDLE:
      digitalWrite(PIN_PILOT, LOW);
      digitalWrite(PIN_PJU, RELAY_OFF); // PJU mati, siap untuk penyeberang berikutnya
      statusPJU = false;
      resetSevenSegment();
      angkaTampil = -1;
      if (idSiklus != 0) idSiklusSelesai = idSiklus;
      idSiklus = 0;
      sayaPemilik = false;
      pemilikSiklus[0] = '\0';
      if (lama == FASE_HIJAU) {
        // Kuning sempat mati selama siklus: kedip mulai dari awal.
        // Dari FAILSAFE kuning sudah kedip, jangan diputus.
        waktuKedipTerakhir = sekarang;
        statusLampuKuning = false;
      }
      LOG("FASE", "%s -> PJU OFF -> IDLE (KUNING KEDIP)", NAMA_FASE[lama]);
      break;

    case FASE_FAILSAFE:
      // Kondisi aman prototipe: tidak ada fase penyeberangan, kuning kedip
      digitalWrite(PIN_PILOT, LOW);
      digitalWrite(PIN_PJU, RELAY_OFF);
      statusPJU = false;
      resetSevenSegment();
      angkaTampil = -1;
      waktuFailsafe = sekarang;
      idSiklusGagal = idSiklus;
      idSiklus = 0;
      sayaPemilik = false;
      pemilikSiklus[0] = '\0';
      break;
  }
  kirimHeartbeat(sekarang); // beri tahu node lain fase terbaru
}

// Fase & angka countdown dihitung dari posisi siklus
void updateSiklus(unsigned long sekarang) {
  if (fase == FASE_FAILSAFE) {
    if (sekarang - waktuFailsafe >= DURASI_FAILSAFE_MS) gantiFase(FASE_IDLE, sekarang);
    return;
  }
  if (idSiklus == 0) return;

  unsigned long posisi = sekarang - waktuMulaiSiklus;
  FaseSistem target;
  if (posisi < MS_PREP) {
    target = FASE_PREP;
  } else if (!syncOK) {
    // Jangan pernah mulai countdown sepihak
    masukFailsafe(sekarang, commitTerkirim ? "ACK COMMIT tidak diterima"
                          : sayaPemilik    ? "tidak ada ACK START dari node lain"
                                           : "COMMIT tidak diterima");
    return;
  } else if (posisi < MS_TIMER_END) {
    target = FASE_TIMER;
  } else if (posisi < MS_SIKLUS) {
    target = FASE_HIJAU;
  } else {
    target = FASE_IDLE;
  }
  // Fase hanya maju; koreksi drift kecil tidak boleh membuat fase mundur
  if (target != fase && (target > fase || target == FASE_IDLE)) gantiFase(target, sekarang);

  if (fase == FASE_TIMER) {
    int angka = DURASI_TIMER - (int)((posisi - MS_PREP) / 1000);
    if (angka < 0) angka = 0;
    if (angka != angkaTampil) {
      angkaTampil = angka;
      tampilkanAngka(angka);
      LOG("CD", "COUNTDOWN: %d", angka);
    }
  }
}

void masukFailsafe(unsigned long sekarang, const char* alasan) {
  LOG("FAIL", "SIKLUS %08lX GAGAL SINKRON: %s (jalur %s) -> FAILSAFE %lu ms, countdown TIDAK dimulai",
      (unsigned long)idSiklus, alasan, NAMA_JALUR[jalurSiklus], DURASI_FAILSAFE_MS);
  if (sayaPemilik) {
    // ABORT = pesan keselamatan, satu-satunya yang dikirim lewat dua jalur
    kirimPesan(MSG_ABORT, JALUR_ESPNOW, sekarang);
    if (mqttSiap) kirimPesan(MSG_ABORT, JALUR_MQTT, sekarang);
  }
  gantiFase(FASE_FAILSAFE, sekarang);
}

// ================= HANDSHAKE (KOORDINATOR) ================= //
// START dikirim ulang sampai ACK(START), lalu COMMIT dikirim ulang sampai ACK(COMMIT).
// Jalur MQTT tanpa ACK dalam MQTT_ACK_TIMEOUT_MS (atau MQTT putus) -> pindah ke ESP-NOW
// dan tetap di ESP-NOW sampai siklus selesai.
void jalankanHandshake(unsigned long sekarang) {
  if (!sayaPemilik || fase != FASE_PREP || syncOK) return;
  unsigned long posisi = sekarang - waktuMulaiSiklus;

  if (jalurSiklus == JALUR_MQTT && (!mqttSiap || sekarang - waktuJalurMulai >= MQTT_ACK_TIMEOUT_MS)) {
    LOG("SYNC", "FALLBACK MQTT -> ESP-NOW: %s saat menunggu ACK %s (posisi %lu ms)",
        mqttSiap ? "ACK timeout" : "MQTT terputus", commitTerkirim ? "COMMIT" : "START", posisi);
    jalurSiklus = JALUR_ESPNOW;
    waktuJalurMulai = sekarang;
    kirimSegera = true;
  }

  unsigned long interval = (jalurSiklus == JALUR_MQTT) ? KIRIM_ULANG_MQTT_MS : KIRIM_ULANG_NOW_MS;
  if (!kirimSegera && sekarang - waktuKirimUlang < interval) return;

  if (!commitTerkirim) {
    if (posisi > BATAS_COMMIT_MS) return; // sudah terlambat, FAILSAFE di akhir PREPARING
    uint32_t seq = kirimPesan(MSG_START, jalurSiklus, sekarang);
    startTerkirim[idxStart] = { seq, sekarang };
    idxStart = (idxStart + 1) % 8;
  } else {
    if (posisi > BATAS_KIRIM_COMMIT_MS) return;
    rttSetengah = rttJalur[jalurSiklus];
    kirimPesan(MSG_COMMIT, jalurSiklus, sekarang);
  }
  waktuKirimUlang = sekarang;
  kirimSegera = false;
}

// ================= PEMROSESAN PESAN MASUK ================= //

// true jika (senderId, sequenceId) sudah pernah diproses
bool sudahDiproses(const char* asal, uint32_t seq) {
  for (const IdPesan& e : pesanTerakhir) {
    if (e.seq == seq && strcmp(e.asal, asal) == 0) return true;
  }
  strlcpy(pesanTerakhir[idxPesan].asal, asal, sizeof(pesanTerakhir[idxPesan].asal));
  pesanTerakhir[idxPesan].seq = seq;
  idxPesan = (idxPesan + 1) % 16;
  return false;
}

// Posisi siklus pengirim saat ini (posisi di pesan + lama pesan di antrian lokal)
unsigned long posisiPesan(const PesanSync& p, unsigned long sekarang) {
  return p.posisiMs + (sekarang - p.stempel);
}

// ACK dikirim lewat jalur yang sama dengan pesan yang dibalas
void kirimAck(const PesanSync& p, unsigned long sekarang) {
  uint8_t jalur = p.jalur;
  if (jalur == JALUR_MQTT && !mqttSiap) jalur = JALUR_ESPNOW;
  kirimPesan(MSG_ACK, jalur, sekarang, p.tipe, p.seq);
}

void tanganiStart(const PesanSync& p, unsigned long sekarang) {
  unsigned long posisi = posisiPesan(p, sekarang);
  if (posisi > BATAS_COMMIT_MS) {
    LOG("SYNC", "START %08lX basi (posisi %lu ms) diabaikan", (unsigned long)p.idSiklus, posisi);
    return;
  }
  if (p.idSiklus == idSiklusSelesai || p.idSiklus == idSiklusGagal) return; // siklus lama

  if (fase == FASE_IDLE) {
    aturSiklus(p.idSiklus, p.asal, false, p.pju, posisi, sekarang);
    jalurSiklus = p.jalur;
    LOG("SYNC", "PENGIKUT siklus %08lX dari %s via %s (posisi %lu ms)",
        (unsigned long)p.idSiklus, p.asal, NAMA_JALUR[p.jalur], posisi);
    updateSiklus(sekarang);  // IDLE -> PREPARING
    kirimAck(p, sekarang);

  } else if (p.idSiklus == idSiklus) {
    if (sayaPemilik) return;
    // START ulang / lewat jalur lain: selaraskan lagi selama belum COMMIT
    if (fase == FASE_PREP && !syncOK) waktuMulaiSiklus = sekarang - posisi;
    jalurSiklus = p.jalur;
    kirimAck(p, sekarang);

  } else if (fase == FASE_PREP && !syncOK && p.idSiklus < idSiklus) {
    // Dua tombol ditekan hampir bersamaan: cycleId terkecil menang
    LOG("SYNC", "KONFLIK: siklus %08lX (%s) < %08lX -> node ini jadi PENGIKUT",
        (unsigned long)p.idSiklus, p.asal, (unsigned long)idSiklus);
    aturSiklus(p.idSiklus, p.asal, false, p.pju, posisi, sekarang);
    jalurSiklus = p.jalur;
    kirimAck(p, sekarang);

  } else {
    LOG("SYNC", "START %08lX dari %s diabaikan (fase %s, siklus %08lX)",
        (unsigned long)p.idSiklus, p.asal, NAMA_FASE[fase], (unsigned long)idSiklus);
  }
}

void tanganiAck(const PesanSync& p, unsigned long sekarang) {
  if (!sayaPemilik || p.idSiklus != idSiklus || fase != FASE_PREP) return;
  unsigned long posisi = sekarang - waktuMulaiSiklus;

  if (p.ackTipe == MSG_START) {
    if (commitTerkirim) return; // ACK START tambahan, sudah ditangani

    // RTT dari START yang dibalas
    unsigned long rtt = 0;
    for (const CatatanKirim& c : startTerkirim) {
      if (c.seq == p.ackSeq && c.waktu != 0) { rtt = sekarang - c.waktu; break; }
    }
    rttJalur[p.jalur] = (uint16_t)min(rtt / 2, 1000UL);

    if (posisi > BATAS_COMMIT_MS) {
      LOG("SYNC", "ACK START terlambat (posisi %lu > %lu ms) -> tidak di-COMMIT", posisi, BATAS_COMMIT_MS);
      return;
    }
    commitTerkirim = true;
    jalurSiklus = p.jalur;
    waktuJalurMulai = sekarang;
    kirimSegera = true;
    LOG("SYNC", "ACK START dari %s via %s, RTT %lu ms -> kirim COMMIT", p.asal, NAMA_JALUR[p.jalur], rtt);
    jalankanHandshake(sekarang);

  } else if (p.ackTipe == MSG_COMMIT) {
    if (!commitTerkirim || syncOK) return;
    syncOK = true;
    LOG("SYNC", "SINKRON OK: ACK COMMIT dari %s via %s (posisi %lu ms) -> COUNTDOWN di posisi %lu ms",
        p.asal, NAMA_JALUR[p.jalur], posisi, MS_PREP);
  }
}

void tanganiCommit(const PesanSync& p, unsigned long sekarang) {
  if (sayaPemilik || p.idSiklus != idSiklus) return;

  if (fase == FASE_PREP) {
    // Posisi koordinator = posisi di pesan + antrian lokal + latensi jaringan (RTT/2)
    unsigned long posisi = posisiPesan(p, sekarang) + p.rttSetengah;
    if (!syncOK) {
      if (posisi > BATAS_TERIMA_COMMIT_MS) {
        LOG("SYNC", "COMMIT terlambat (posisi %lu > %lu ms) -> ditolak", posisi, BATAS_TERIMA_COMMIT_MS);
        return;
      }
      long koreksi = (long)(sekarang - waktuMulaiSiklus) - (long)posisi;
      waktuMulaiSiklus = sekarang - posisi;
      syncOK = true;
      jalurSiklus = p.jalur;
      LOG("SYNC", "SINKRON OK: COMMIT dari %s via %s (latensi %u ms, koreksi %ld ms) -> COUNTDOWN di posisi %lu ms",
          p.asal, NAMA_JALUR[p.jalur], p.rttSetengah, koreksi, MS_PREP);
    }
    kirimAck(p, sekarang);
  } else if (fase == FASE_TIMER || fase == FASE_HIJAU) {
    kirimAck(p, sekarang); // ACK sebelumnya hilang, koordinator masih menunggu
  }
}

void tanganiState(const PesanSync& p, unsigned long sekarang) {
  if (p.idSiklus == 0) return;
  unsigned long posisi = posisiPesan(p, sekarang);
  if (posisi >= MS_SIKLUS) return;

  if (p.idSiklus == idSiklus) {
    // Koreksi drift hanya dari pemilik siklus & via ESP-NOW (latensi MQTT tidak stabil)
    if (!sayaPemilik && strcmp(p.asal, pemilikSiklus) == 0 && p.jalur == JALUR_ESPNOW &&
        (fase == FASE_TIMER || fase == FASE_HIJAU)) {
      long selisih = (long)(sekarang - waktuMulaiSiklus) - (long)posisi;
      if (labs(selisih) > TOLERANSI_SYNC_MS) {
        LOG("SYNC", "KOREKSI DRIFT %ld ms via %s", selisih, NAMA_JALUR[p.jalur]);
        waktuMulaiSiklus = sekarang - posisi;
      }
    }
    return;
  }

  // Pengaman: node lain sudah COUNTDOWN/HIJAU untuk siklus yang tidak kita jalankan
  // (mis. ACK COMMIT hilang atau node ini reboot). Merah di dua sisi lebih aman
  // daripada merah di satu sisi saja.
  if ((p.fase == FASE_TIMER || p.fase == FASE_HIJAU) && p.idSiklus != idSiklusSelesai &&
      (fase == FASE_IDLE || fase == FASE_FAILSAFE)) {
    LOG("SYNC", "IKUT TERLAMBAT: node %s sudah %s siklus %08lX (posisi %lu ms) via %s",
        p.asal, NAMA_FASE[p.fase], (unsigned long)p.idSiklus, posisi, NAMA_JALUR[p.jalur]);
    fase = FASE_IDLE; // output FAILSAFE = IDLE (kuning kedip), aman dilompati
    aturSiklus(p.idSiklus, p.asal, false, p.pju, posisi, sekarang);
    syncOK = true;
    jalurSiklus = p.jalur;
    updateSiklus(sekarang);
  }
}

void tanganiAbort(const PesanSync& p, unsigned long sekarang) {
  if (p.idSiklus != idSiklus || fase != FASE_PREP) return;
  masukFailsafe(sekarang, "dibatalkan koordinator (ABORT)");
}

void prosesPesan(const PesanSync& p) {
  if (sudahDiproses(p.asal, p.seq)) {
    if (p.tipe != MSG_STATE)
      LOG("RX", "DUPLIKAT %s seq %lu dari %s via %s diabaikan",
          NAMA_TIPE[p.tipe], (unsigned long)p.seq, p.asal, NAMA_JALUR[p.jalur]);
    return;
  }
  unsigned long sekarang = millis();

  if (p.tipe != MSG_STATE) {
    LOG("RX", "%s%s%s dari %s via %s seq %lu siklus %08lX posisi %lu ms (antri %lu ms)",
        NAMA_TIPE[p.tipe], p.tipe == MSG_ACK ? "/" : "", p.tipe == MSG_ACK ? NAMA_TIPE[p.ackTipe] : "",
        p.asal, NAMA_JALUR[p.jalur], (unsigned long)p.seq, (unsigned long)p.idSiklus,
        (unsigned long)p.posisiMs, sekarang - p.stempel);
  }

  bool perluPeringatan = (sekarang - waktuPeringatan > 10000);
  if (p.channel != 0 && channelSekarang != 0 && p.channel != channelSekarang && perluPeringatan) {
    waktuPeringatan = sekarang;
    LOG("CH", "PERINGATAN: node %s di channel %u, node ini channel %u -> ESP-NOW tidak akan sampai",
        p.asal, p.channel, channelSekarang);
  }
  if (p.idSiklus != 0 &&
      (p.jadwalMulai != MS_PREP || p.durTimer != DURASI_TIMER || p.durHijau != DURASI_HIJAU_MS)) {
    if (perluPeringatan) {
      waktuPeringatan = sekarang;
      LOG("SYNC", "PERINGATAN: durasi node %s beda (prep %u, cd %u, hijau %u) -> firmware tidak sama, pesan diabaikan",
          p.asal, p.jadwalMulai, p.durTimer, p.durHijau);
    }
    return;
  }

  switch (p.tipe) {
    case MSG_START:  tanganiStart(p, sekarang);  break;
    case MSG_ACK:    tanganiAck(p, sekarang);    break;
    case MSG_COMMIT: tanganiCommit(p, sekarang); break;
    case MSG_STATE:  tanganiState(p, sekarang);  break;
    case MSG_ABORT:  tanganiAbort(p, sekarang);  break;
  }
}

// ================= PENGIRIMAN ================= //

void kirimNow(const PesanSync& p) {
  if (!espNowSiap) {
    nowKirimGagal++;
    if (p.tipe != MSG_STATE) LOG("NOW", "TX %s gagal: ESP-NOW belum siap", NAMA_TIPE[p.tipe]);
    return;
  }
  PaketNow pk = {};
  pk.magic = MAGIC_PAKET;
  pk.versi = VERSI_PROTOKOL;
  pk.tipe = p.tipe;
  pk.fase = p.fase;
  pk.pju = p.pju;
  pk.ackTipe = p.ackTipe;
  pk.channel = p.channel;
  strlcpy(pk.asal, p.asal, sizeof(pk.asal));
  pk.seq = p.seq;
  pk.idSiklus = p.idSiklus;
  pk.posisiMs = p.posisiMs;
  pk.ackSeq = p.ackSeq;
  pk.rttSetengah = p.rttSetengah;
  pk.jadwalMulai = p.jadwalMulai;
  pk.durTimer = p.durTimer;
  pk.durHijau = p.durHijau;
  esp_err_t r = esp_now_send(ALAMAT_BROADCAST, (const uint8_t*)&pk, sizeof(pk));
  if (r != ESP_OK) {
    nowKirimGagal++;
    LOG("NOW", "esp_now_send %s GAGAL: %s", NAMA_TIPE[p.tipe], esp_err_to_name(r));
  }
}

// Kirim satu pesan lewat satu jalur. Mengembalikan sequenceId-nya.
//   - ESP-NOW langsung dari sini (tidak ikut tertahan kalau MQTT sedang blocking)
//   - MQTT lewat antrian ke taskJaringan
uint32_t kirimPesan(uint8_t tipe, uint8_t jalur, unsigned long sekarang, uint8_t ackTipe, uint32_t ackSeq) {
  PesanSync p = {};
  p.tipe = tipe;
  p.fase = fase;
  p.pju = pjuSiklus;
  p.ackTipe = ackTipe;
  p.channel = channelSekarang;
  p.jalur = jalur;
  p.tujuanMqtt = KE_SYNC;
  strlcpy(p.asal, idNode, sizeof(p.asal));
  p.seq = ++seqKirim;
  p.idSiklus = idSiklus;
  p.posisiMs = idSiklus ? (sekarang - waktuMulaiSiklus) : 0;
  p.ackSeq = ackSeq;
  p.rttSetengah = rttSetengah;
  p.jadwalMulai = MS_PREP;
  p.durTimer = DURASI_TIMER;
  p.durHijau = DURASI_HIJAU_MS;
  p.stempel = sekarang;

  if (jalur == JALUR_MQTT) {
    if (xQueueSend(qKirim, &p, 0) != pdTRUE && tipe != MSG_STATE) {
      LOG("MQTT", "antrian kirim penuh, %s dibuang", NAMA_TIPE[tipe]);
    }
  } else {
    kirimNow(p);
  }

  if (tipe != MSG_STATE) {
    LOG("TX", "%s%s%s via %s seq %lu siklus %08lX posisi %lu ms",
        NAMA_TIPE[tipe], tipe == MSG_ACK ? "/" : "", tipe == MSG_ACK ? NAMA_TIPE[ackTipe] : "",
        NAMA_JALUR[jalur], (unsigned long)p.seq, (unsigned long)p.idSiklus, (unsigned long)p.posisiMs);
  }
  return p.seq;
}

// Heartbeat STATE:
//   - saat siklus  : lewat jalur siklus (untuk koreksi drift & pengaman ikut-terlambat)
//   - saat IDLE    : lewat ESP-NOW, untuk memantau jalur fallback (channel) tetap hidup
//   - status MQTT  : retained ke topik status (monitor laptop), jika broker siap
void kirimHeartbeat(unsigned long sekarang) {
  uint8_t jalur = JALUR_ESPNOW;
  if (idSiklus && jalurSiklus == JALUR_MQTT && mqttSiap) jalur = JALUR_MQTT;
  kirimPesan(MSG_STATE, jalur, sekarang);

  if (mqttSiap) {
    PesanSync s = {};
    s.tipe = MSG_STATE;
    s.fase = fase;
    s.pju = statusPJU;
    s.channel = channelSekarang;
    s.jalur = JALUR_MQTT;
    s.tujuanMqtt = KE_STATUS;
    strlcpy(s.asal, idNode, sizeof(s.asal));
    s.idSiklus = idSiklus;
    s.posisiMs = idSiklus ? (sekarang - waktuMulaiSiklus) : 0;
    s.jadwalMulai = MS_PREP;
    s.durTimer = DURASI_TIMER;
    s.durHijau = DURASI_HIJAU_MS;
    s.stempel = sekarang;
    xQueueSend(qKirim, &s, 0);
  }
  waktuHeartbeat = sekarang;
}

// ================= JARINGAN (CORE 0) ================= //
// MQTT    : WiFi + broker di laptop (jalur utama + monitor / CV nanti)
// ESP-NOW : broadcast langsung ESP ke ESP (fallback), tetap jalan walau WiFi mati
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

uint8_t indeksNama(const char* nama, const char* const daftar[], uint8_t jumlah) {
  for (uint8_t i = 0; i < jumlah; i++) {
    if (strcmp(nama, daftar[i]) == 0) return i;
  }
  return 255;
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  // penyeberangan/node/<id>/online -> pantau apakah node lain online di MQTT
  const char* awalan = "penyeberangan/node/";
  size_t panjangAwalan = strlen(awalan);
  size_t panjangTopik = strlen(topic);
  if (strncmp(topic, awalan, panjangAwalan) == 0 && panjangTopik > panjangAwalan + 7 &&
      strcmp(topic + panjangTopik - 7, "/online") == 0) {
    char id[16];
    size_t panjangId = min(panjangTopik - panjangAwalan - 7, sizeof(id) - 1);
    memcpy(id, topic + panjangAwalan, panjangId);
    id[panjangId] = '\0';
    if (strcmp(id, idNode) == 0) return;
    bool online = (length == 6 && memcmp(payload, "online", 6) == 0);
    if (online != peerMqttOnline) LOG("MQTT", "node %s %s di MQTT", id, online ? "ONLINE" : "OFFLINE");
    peerMqttOnline = online;
    return;
  }

  StaticJsonDocument<384> doc;
  if (deserializeJson(doc, payload, length)) return;

  const char* asal = doc["src"] | "";
  if (asal[0] == '\0' || strcmp(asal, idNode) == 0) return; // pesan sendiri

  uint8_t tipe = indeksNama(doc["type"] | "", NAMA_TIPE, JUMLAH_TIPE);
  uint8_t faseAsal = indeksNama(doc["fase"] | "", NAMA_FASE, JUMLAH_FASE);
  if (tipe == 255 || faseAsal == 255) return; // format lama / tidak dikenal

  PesanSync p = {};
  p.tipe = tipe;
  p.fase = faseAsal;
  p.pju = doc["pju"] | false;
  p.ackTipe = indeksNama(doc["ackt"] | "STATE", NAMA_TIPE, JUMLAH_TIPE);
  p.channel = doc["ch"] | 0;
  strlcpy(p.asal, asal, sizeof(p.asal));
  p.seq = doc["seq"] | 0UL;
  p.idSiklus = doc["cyc"] | 0UL;
  p.posisiMs = doc["pos"] | 0UL;
  p.ackSeq = doc["ack"] | 0UL;
  p.rttSetengah = doc["rtt"] | 0;
  p.jadwalMulai = doc["mulai"] | 0;
  p.durTimer = doc["cd"] | 0;
  p.durHijau = doc["hijau"] | 0;
  p.stempel = millis();
  p.jalur = JALUR_MQTT;
  if (xQueueSend(qTerima, &p, 0) != pdTRUE) antrianPenuh++;
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
  if (pk.magic != MAGIC_PAKET || pk.versi != VERSI_PROTOKOL) return;
  if (pk.tipe >= JUMLAH_TIPE || pk.fase >= JUMLAH_FASE) return;
  pk.asal[sizeof(pk.asal) - 1] = '\0';
  if (pk.asal[0] == '\0' || strcmp(pk.asal, idNode) == 0) return;

  PesanSync p = {};
  p.tipe = pk.tipe;
  p.fase = pk.fase;
  p.pju = pk.pju;
  p.ackTipe = pk.ackTipe;
  p.channel = pk.channel;
  strlcpy(p.asal, pk.asal, sizeof(p.asal));
  p.seq = pk.seq;
  p.idSiklus = pk.idSiklus;
  p.posisiMs = pk.posisiMs;
  p.ackSeq = pk.ackSeq;
  p.rttSetengah = pk.rttSetengah;
  p.jadwalMulai = pk.jadwalMulai;
  p.durTimer = pk.durTimer;
  p.durHijau = pk.durHijau;
  p.stempel = millis();
  p.jalur = JALUR_ESPNOW;

  waktuNowTerakhir = p.stempel;
  nowTerima++;
  strlcpy(peerNowTerakhir, pk.asal, sizeof(peerNowTerakhir));
  if (xQueueSend(qTerima, &p, 0) != pdTRUE) antrianPenuh++;
}

// Broadcast tidak punya ACK di level MAC: SUCCESS hanya berarti frame terpancar,
// BUKAN berarti node lain menerima / memproses pesan (itu tugas ACK aplikasi).
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

  StaticJsonDocument<384> doc;
  doc["type"]  = NAMA_TIPE[p.tipe];
  doc["seq"]   = p.seq;
  doc["src"]   = p.asal;
  doc["cyc"]   = p.idSiklus;
  doc["pos"]   = posisi;
  doc["fase"]  = NAMA_FASE[p.fase];
  doc["pju"]   = p.pju;
  doc["ch"]    = p.channel;
  doc["mulai"] = p.jadwalMulai;
  doc["cd"]    = p.durTimer;
  doc["hijau"] = p.durHijau;
  if (p.tipe == MSG_ACK) {
    doc["ack"]  = p.ackSeq;
    doc["ackt"] = NAMA_TIPE[p.ackTipe];
  }
  if (p.tipe == MSG_COMMIT) doc["rtt"] = p.rttSetengah;
  if (p.fase == FASE_TIMER && posisi >= MS_PREP) doc["sisa"] = DURASI_TIMER - (int)((posisi - MS_PREP) / 1000);

  char buf[320];
  size_t n = serializeJson(doc, buf, sizeof(buf));
  bool ok = (p.tujuanMqtt == KE_STATUS) ? mqtt.publish(topicStatus, (const uint8_t*)buf, n, true)
                                        : mqtt.publish(TOPIC_SYNC, (const uint8_t*)buf, n, false);
  if (!ok && p.tipe != MSG_STATE) LOG("MQTT", "publish %s GAGAL", NAMA_TIPE[p.tipe]);
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
    mqtt.subscribe(TOPIC_ONLINE_SEMUA);
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
  channelSekarang = channelRadio();

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
  mqtt.setBufferSize(512);   // pesan JSON v2 lebih panjang dari default 256 byte

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
    channelSekarang = channelRadio();

    bool mqttOk = (status == ST_ONLINE) && mqtt.connected();
    if (status == ST_ONLINE && !mqttOk && sekarang - waktuCobaMQTT >= MQTT_RETRY_MS) {
      waktuCobaMQTT = sekarang;
      hubungkanMQTT(); // boleh blocking: lampu & ESP-NOW tetap jalan dari core 1
      mqttOk = mqtt.connected();
    }
    if (mqttOk) mqtt.loop();
    if (mqttSiap != mqttOk) {
      mqttSiap = mqttOk;
      if (!mqttOk) {
        peerMqttOnline = false; // tidak bisa tahu lagi sampai tersambung ulang
        LOG("MQTT", "TIDAK SIAP (WiFi=%s, rc=%d)", NAMA_STATUS_WIFI[status], mqtt.state());
      }
    }

    while (xQueueReceive(qKirim, &p, 0) == pdTRUE) {
      if (mqttOk) publishMqtt(p);
      else if (p.tipe != MSG_STATE) LOG("MQTT", "%s dibuang: MQTT tidak siap", NAMA_TIPE[p.tipe]);
    }

    // Ringkasan link berkala, untuk memantau saat uji WiFi putus/nyambung
    if (sekarang - waktuLog >= LOG_LINK_MS) {
      waktuLog = sekarang;
      unsigned long terakhir = waktuNowTerakhir;
      char peer[48];
      if (terakhir == 0) strlcpy(peer, "belum ada paket dari node lain", sizeof(peer));
      else snprintf(peer, sizeof(peer), "node %s %.1f s lalu", peerNowTerakhir,
                    (millis() - terakhir) / 1000.0f);
      LOG("LINK", "WiFi=%s ch_simpan=%u ch_radio=%u MQTT=%s peerMQTT=%s | ESP-NOW: %s | tx ok=%lu gagal=%lu rx=%lu drop=%lu",
          NAMA_STATUS_WIFI[status], channel, channelRadio(), mqttOk ? "OK" : "-",
          peerMqttOnline ? "online" : "-", peer,
          (unsigned long)nowKirimOk, (unsigned long)nowKirimGagal, (unsigned long)nowTerima,
          (unsigned long)antrianPenuh);
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// ================= FUNGSI BANTUAN ================= //

// Lampu traffic light sesuai fase:
// IDLE / PREPARING / FAILSAFE = kuning kedip | COUNTDOWN = merah | HIJAU = hijau
void updateLampu(unsigned long sekarang) {
  if (fase == FASE_IDLE || fase == FASE_PREP || fase == FASE_FAILSAFE) {
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

// Hanya node dengan buzzer (ADA_BUZZER = 1, Node A):
//   PREPARING : bunyi panjang, berhenti BUZZER_JEDA_PREP_MS sebelum countdown
//   COUNTDOWN : pola kode tes. sisa 15-4: bunyi 500 ms / diam 500 ms,
//               sisa 3-0: 4 beep cepat per detik (bunyi 125 ms, diam 125 ms)
//   FAILSAFE  : 3 bunyi pendek (150 ms bunyi, 150 ms diam)
// Ikut posisi siklus, jadi bunyi selaras dengan countdown kedua node.
void updateBuzzer(unsigned long sekarang) {
  if (!ADA_BUZZER) return; // GPIO buzzer tetap INPUT seperti di setup()

  bool bunyi = false;
  if (fase == FASE_PREP) {
    bunyi = (sekarang - waktuMulaiSiklus) < (MS_PREP - BUZZER_JEDA_PREP_MS);
  } else if (fase == FASE_TIMER) {
    unsigned long posisi = (sekarang - waktuMulaiSiklus - MS_PREP) % 1000; // 0-999 ms di detik ini
    if (angkaTampil >= 4) {
      bunyi = (posisi < 500);
    } else {
      bunyi = (posisi % 250) < 125;
    }
  } else if (fase == FASE_FAILSAFE) {
    unsigned long t = sekarang - waktuFailsafe;
    bunyi = (t < 900) && (t % 300) < 150;
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
