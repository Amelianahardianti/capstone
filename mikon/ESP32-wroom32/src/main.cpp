#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <TM1637Display.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// ================= KONFIGURASI WIFI & MQTT =================
// (TIDAK DIUBAH — kompatibilitas dengan Computer Vision dipertahankan)
const char* ssid = "veroo";         // GANTI DENGAN WIFI HP/ROUTER
const char* password = "veronica"; // GANTI DENGAN PASS WIFI
const char* mqtt_server = "10.76.111.208";  // IP Laptop kamu (adapter Wi-Fi, satu jaringan dgn ESP32)

WiFiClient espClient;
PubSubClient client(espClient);

// ============================================================
// KONFIGURASI PIN
// ============================================================
// --- Input / Sensor ---
const int PIR_PIN         = 13;
const int LDR_PIN         = 34;
const int BUTTON_PIN      = 18;

// --- Output: Lampu Kendaraan ---
const int VEHICLE_RED_PIN    = 23;
const int VEHICLE_YELLOW_PIN = 22;
const int VEHICLE_GREEN_PIN  = 21;

// --- Output: Lampu Penyeberang ---
const int HUMAN_RED_PIN   = 19;
const int HUMAN_GREEN_PIN = 32;

// --- Output: PJU ---
const int PJU_WHITE_PIN = 33;

// --- Output: Buzzer (ACTIVE buzzer — hanya ON/OFF, tidak pakai tone()/PWM) ---
const int BUZZER_PIN = 27;

// --- Output: 7-Segment (modul DIO+CLK berbasis chip TM1637, lihat catatan di bawah) ---
const int SEVENSEG_DIO_PIN = 4;
const int SEVENSEG_CLK_PIN = 26;

// ============================================================
// KONSTANTA LOGIC
// ============================================================
const int LDR_THRESHOLD   = 400;   // Diwarisi dari kode lama, dipakai ulang untuk PJU. Kalibrasi ulang di lapangan jika perlu.
const int LDR_SAMPLES     = 10;
const unsigned long MOTION_TIMEOUT_MS = 15000UL;   // Timeout anti-stuck (dipertahankan dari kode lama)
const unsigned long LDR_POLL_INTERVAL_MS = 1000UL; // Interval baca LDR untuk PJU — dipercepat dari 3000ms karena
                                                     // PJU sekarang juga bergantung pada event PIR yang bisa
                                                     // mulai/selesai kapan saja (butuh respons lebih cepat)

// Debounce push button (mekanis, beda karakter dari PIR — bukan asumsi baru,
// hanya nilai umum untuk tombol mekanis biasa; dokumentasikan agar mudah dikalibrasi)
const unsigned long BUTTON_DEBOUNCE_MS = 250UL;

// Tidak ada fase transisi kuning 3 detik maupun all-red clearance 1,5 detik —
// begitu PIR/Button trigger, langsung Vehicle RED + Human GREEN (lihat vTaskControl).
const unsigned long VEHICLE_GREEN_HOLD_MS          = 5000UL; // Vehicle GREEN tetap tepat 5 detik pasca-crossing
const unsigned long VEHICLE_YELLOW_BLINK_PERIOD_MS = 1000UL; // Fase C (idle): kedip kuning, 500ms ON / 500ms OFF
const unsigned long IDLE_POLL_INTERVAL_MS          = 100UL;  // resolusi polling saat idle (blink + respons cepat ke event baru)

// ============================================================
// STATE SISTEM (menggantikan flag boolean tunggal `adaOrang`)
// ============================================================
enum SystemState {
  SYS_STANDBY,            // IDLE: Vehicle YELLOW berkedip, Human RED, menunggu event berikutnya
  SYS_PEDESTRIAN_REQUEST, // Flag "busy" instan saat baru di-trigger (PIR/Button), sebelum Task_Control bangun
  SYS_CHECK_CV,           // Vehicle RED + Human GREEN sudah aktif; menunggu/evaluasi HOLD/timer dari CV
  SYS_HOLD,               // CV bilang jalan tidak aman, tunggu sampai aman/timeout (lampu tetap seperti CHECK_CV)
  SYS_CROSSING,           // Menyeberang aktif: countdown + buzzer berjalan
  SYS_VEHICLE_GREEN_5S    // Kendaraan hijau tetap selama 5 detik pasca-crossing
};

enum VehicleLightState { VEHICLE_RED, VEHICLE_YELLOW, VEHICLE_GREEN };
enum HumanLightState   { HUMAN_RED, HUMAN_GREEN };

// ============================================================
// HANDLE TASK & SINKRONISASI
// ============================================================
TaskHandle_t xTaskPIRHandle     = NULL;
TaskHandle_t xTaskButtonHandle  = NULL;
TaskHandle_t xTaskMQTTHandle    = NULL;
TaskHandle_t xTaskControlHandle = NULL; // dulu xTaskLEDHandle — sekarang state-machine terpusat
TaskHandle_t xTaskEnvHandle     = NULL; // task baru: LDR -> PJU, independen dari crossing

// Mutex untuk state sistem + timestamp gerakan terakhir
// (menggantikan xMutexAdaOrang — pola mutex+variable biasa dipertahankan persis)
SemaphoreHandle_t xMutexState = NULL;
SystemState   currentState   = SYS_STANDBY; // Diakses HANYA lewat mutex
unsigned long lastMotionTime = 0;           // Diakses HANYA lewat mutex

// Mutex untuk data dari CV + flag publish (TIDAK DIUBAH dari kode lama)
SemaphoreHandle_t xMutexTimer = NULL;
int  timerDariCV = 10;               // default fallback 10 detik
bool statusHold  = false;            // true jika CV kirim "HOLD"
bool pendingPublishTrigger = false;  // flag publish, dieksekusi di Core 0 oleh vTaskMQTT

TM1637Display sevenSeg(SEVENSEG_CLK_PIN, SEVENSEG_DIO_PIN);

// ============================================================
// HELPER: Akses currentState & lastMotionTime dengan aman (thread-safe)
// ============================================================
SystemState getSystemState() {
  SystemState nilai;
  if (xSemaphoreTake(xMutexState, pdMS_TO_TICKS(10)) == pdTRUE) {
    nilai = currentState;
    xSemaphoreGive(xMutexState);
  } else {
    nilai = SYS_STANDBY; // Safe default jika mutex timeout
  }
  return nilai;
}

void setSystemState(SystemState nilai) {
  if (xSemaphoreTake(xMutexState, pdMS_TO_TICKS(10)) == pdTRUE) {
    currentState = nilai;
    xSemaphoreGive(xMutexState);
  }
}

void refreshLastMotionTime() {
  if (xSemaphoreTake(xMutexState, pdMS_TO_TICKS(10)) == pdTRUE) {
    lastMotionTime = millis();
    xSemaphoreGive(xMutexState);
  }
}

unsigned long getLastMotionTime() {
  unsigned long t;
  if (xSemaphoreTake(xMutexState, pdMS_TO_TICKS(10)) == pdTRUE) {
    t = lastMotionTime;
    xSemaphoreGive(xMutexState);
  } else {
    t = millis(); // Safe: anggap baru saja ada gerakan
  }
  return t;
}

// ============================================================
// HELPER: Median filter untuk ADC LDR (anti-noise) — TIDAK DIUBAH dari kode lama
// ============================================================
int bacaLDRFiltered() {
  int samples[LDR_SAMPLES];

  for (int i = 0; i < LDR_SAMPLES; i++) {
    samples[i] = analogRead(LDR_PIN);
    vTaskDelay(pdMS_TO_TICKS(5));
  }

  for (int i = 0; i < LDR_SAMPLES - 1; i++) {
    for (int j = 0; j < LDR_SAMPLES - i - 1; j++) {
      if (samples[j] > samples[j + 1]) {
        int tmp = samples[j];
        samples[j] = samples[j + 1];
        samples[j + 1] = tmp;
      }
    }
  }

  return samples[LDR_SAMPLES / 2];
}

// ============================================================
// OUTPUT TERPUSAT — SATU-SATUNYA fungsi yang boleh menulis GPIO
// lampu kendaraan/manusia, supaya kombinasi tidak pernah invalid.
// Hanya dipanggil dari vTaskControl.
// ============================================================
void applyTrafficState(VehicleLightState v, HumanLightState h) {
  // Guard keselamatan: kombinasi berbahaya dipaksa ke kondisi aman (semua berhenti)
  if (v == VEHICLE_GREEN && h == HUMAN_GREEN) {
    Serial.println("SAFETY:BLOCKED_INVALID_COMBO_FORCE_ALL_RED");
    v = VEHICLE_RED;
    h = HUMAN_RED;
  }

  digitalWrite(VEHICLE_RED_PIN,    v == VEHICLE_RED    ? HIGH : LOW);
  digitalWrite(VEHICLE_YELLOW_PIN, v == VEHICLE_YELLOW ? HIGH : LOW);
  digitalWrite(VEHICLE_GREEN_PIN,  v == VEHICLE_GREEN  ? HIGH : LOW);
  digitalWrite(HUMAN_RED_PIN,      h == HUMAN_RED   ? HIGH : LOW);
  digitalWrite(HUMAN_GREEN_PIN,    h == HUMAN_GREEN ? HIGH : LOW);

  static VehicleLightState lastLoggedV = VEHICLE_RED;
  static HumanLightState   lastLoggedH = HUMAN_RED;
  static bool firstCall = true;

  if (firstCall || v != lastLoggedV) {
    Serial.print("VEHICLE:");
    Serial.println(v == VEHICLE_RED ? "RED" : v == VEHICLE_YELLOW ? "YELLOW" : "GREEN");
    lastLoggedV = v;
  }
  if (firstCall || h != lastLoggedH) {
    Serial.print("HUMAN:");
    Serial.println(h == HUMAN_RED ? "RED" : "GREEN");
    lastLoggedH = h;
  }
  firstCall = false;
}

void setBuzzer(bool on) {
  static bool lastOn = false;
  static bool firstCall = true;
  digitalWrite(BUZZER_PIN, on ? HIGH : LOW);
  if (firstCall || on != lastOn) {
    Serial.print("BUZZER:");
    Serial.println(on ? "ON" : "OFF");
    lastOn = on;
    firstCall = false;
  }
}

// ============================================================
// IDLE / VEHICLE_YELLOW_BLINK — Fase C (state SYS_STANDBY)
// Vehicle: hanya YELLOW yang berkedip non-blocking (berbasis millis()),
// RED & GREEN kendaraan mati. Human tetap RED selama idle (aman).
// ============================================================
void enterIdleBlink() {
  digitalWrite(VEHICLE_RED_PIN, LOW);
  digitalWrite(VEHICLE_GREEN_PIN, LOW);
  digitalWrite(HUMAN_RED_PIN, HIGH);
  digitalWrite(HUMAN_GREEN_PIN, LOW);
  Serial.println("VEHICLE:YELLOW_BLINK_IDLE");
  Serial.println("HUMAN:RED");
}

void tickVehicleYellowBlink() {
  bool on = (millis() % VEHICLE_YELLOW_BLINK_PERIOD_MS) < (VEHICLE_YELLOW_BLINK_PERIOD_MS / 2);
  digitalWrite(VEHICLE_YELLOW_PIN, on ? HIGH : LOW);
}

void update7Segment(int value) {
  sevenSeg.showNumberDec(value, false);
}

// ============================================================
// ISR — PIR & PUSH BUTTON (seminimal mungkin, hanya di IRAM)
// ============================================================
void IRAM_ATTR pirInterruptHandler() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(xTaskPIRHandle, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void IRAM_ATTR buttonInterruptHandler() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(xTaskButtonHandle, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// ============================================================
// Titik masuk permintaan menyeberang — dipakai bersama oleh
// vTaskPIR dan vTaskButton supaya kedua sumber trigger konsisten
// (satu event software yang sama: "ada yang mau menyeberang").
// ============================================================
void requestPedestrianCrossing(const char* source) {
  refreshLastMotionTime(); // selalu refresh, dipakai juga untuk timeout HOLD

  if (getSystemState() == SYS_STANDBY) {
    setSystemState(SYS_PEDESTRIAN_REQUEST);

    if (xSemaphoreTake(xMutexTimer, pdMS_TO_TICKS(10)) == pdTRUE) {
      pendingPublishTrigger = true; // dieksekusi vTaskMQTT (hindari race condition PubSubClient antar-core)
      xSemaphoreGive(xMutexTimer);
    }

    Serial.print("STATUS:PEDESTRIAN_REQUEST:");
    Serial.println(source);

    xTaskNotifyGive(xTaskControlHandle);
  } else {
    // Sequence sedang berjalan — jangan restart, cukup refresh timestamp (sudah dilakukan di atas)
    Serial.print("STATUS:REQUEST_IGNORED_BUSY:");
    Serial.println(source);
  }
}

// ============================================================
// TASK: PIR Handler — Core 1, Priority 3 (pola debounce TIDAK DIUBAH)
// ============================================================
void vTaskPIR(void *pvParameters) {
  TickType_t lastTriggerTick = 0;
  const TickType_t debounceTicks = pdMS_TO_TICKS(500);

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    TickType_t sekarang = xTaskGetTickCount();
    if ((sekarang - lastTriggerTick) < debounceTicks) {
      continue;
    }
    lastTriggerTick = sekarang;

    if (digitalRead(PIR_PIN) == HIGH) {
      Serial.println("PIR:TRIGGERED");
      requestPedestrianCrossing("PIR");
    }
  }
}

// ============================================================
// TASK: Push Button Handler — Core 1, Priority 3
// Fungsi: trigger manual untuk pengujian, meniru event PIR tanpa
// harus menunggu sensor gerak fisik terpicu.
// ============================================================
void vTaskButton(void *pvParameters) {
  TickType_t lastTriggerTick = 0;
  const TickType_t debounceTicks = pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS);

  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    TickType_t sekarang = xTaskGetTickCount();
    if ((sekarang - lastTriggerTick) < debounceTicks) {
      continue;
    }
    lastTriggerTick = sekarang;

    // INPUT_PULLUP: tombol ditekan = LOW
    if (digitalRead(BUTTON_PIN) == LOW) {
      Serial.println("BUTTON:PRESSED");
      requestPedestrianCrossing("BUTTON");
    }
  }
}

// ============================================================
// TASK: MQTT — Core 0, Priority 2 (TIDAK DIUBAH dari kode lama)
// ============================================================
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  Serial.print("Data dari CV: ");
  Serial.println(message);

  StaticJsonDocument<128> doc;
  DeserializationError err = deserializeJson(doc, message);
  if (!err) {
    if (xSemaphoreTake(xMutexTimer, pdMS_TO_TICKS(10)) == pdTRUE) {
      if (doc["timer"].is<const char*>() && String(doc["timer"].as<const char*>()) == "HOLD") {
        statusHold = true;
      } else {
        statusHold = false;
        timerDariCV = doc["timer"] | 10;
      }
      xSemaphoreGive(xMutexTimer);
    }
  }
}

void vTaskMQTT(void *pvParameters) {
  for (;;) {
    if (!client.connected()) {
      Serial.print("Koneksi MQTT...");

      if (client.connect("ESP32_Master")) {
        Serial.println("Terhubung!");
        client.subscribe("jalan/status");
      } else {
        Serial.print("MQTT GAGAL, state = ");
        Serial.println(client.state());
        vTaskDelay(pdMS_TO_TICKS(3000));
      }

    } else {
      client.loop();

      if (xSemaphoreTake(xMutexTimer, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (pendingPublishTrigger) {
          client.publish("jalan/trigger", "ADA_ORANG");
          pendingPublishTrigger = false;
        }
        xSemaphoreGive(xMutexTimer);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

// ============================================================
// TASK: Environment (LDR + Event PIR -> PJU) — Core 0, Priority 1
// PJU WHITE menyala HANYA jika: gelap (LDR) DAN ada pedestrian
// event aktif (currentState != SYS_STANDBY). Task ini tetap terpisah
// dari vTaskControl — hanya MEMBACA state lewat getSystemState(),
// tidak pernah menulis GPIO lampu kendaraan/manusia, dan sebaliknya
// vTaskControl tidak pernah menyentuh PJU_WHITE_PIN.
// ============================================================
void vTaskEnv(void *pvParameters) {
  bool pjuOn = false;
  bool firstRun = true;

  for (;;) {
    int nilaiLDR = bacaLDRFiltered();
    bool gelap = (nilaiLDR < LDR_THRESHOLD);          // GELAP jika di bawah threshold
    bool eventAktif = (getSystemState() != SYS_STANDBY); // ada pedestrian event sedang berjalan
    bool shouldBeOn = gelap && eventAktif;

    if (firstRun || shouldBeOn != pjuOn) {
      pjuOn = shouldBeOn;
      digitalWrite(PJU_WHITE_PIN, pjuOn ? HIGH : LOW);
      Serial.print("LDR_VALUE:");
      Serial.println(nilaiLDR);
      Serial.print("PJU:");
      Serial.println(pjuOn ? "ON" : "OFF");
      firstRun = false;
    }

    vTaskDelay(pdMS_TO_TICKS(LDR_POLL_INTERVAL_MS));
  }
}

// ============================================================
// TASK: Control — Core 0, Priority 1
// State-machine terpusat: traffic light, pedestrian light,
// countdown, buzzer, 7-segment. Menggantikan vTaskLED lama.
// ============================================================
void vTaskControl(void *pvParameters) {
  for (;;) {
    // ---------- SYS_STANDBY (IDLE): Vehicle YELLOW berkedip, tunggu event ----------
    setSystemState(SYS_STANDBY);
    enterIdleBlink();

    uint32_t notified;
    do {
      notified = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(IDLE_POLL_INTERVAL_MS));
      if (notified == 0) {
        tickVehicleYellowBlink(); // masih idle, lanjut berkedip
      }
    } while (notified == 0);

    // ---------- SYS_CHECK_CV ----------
    // Begitu PIR/Button trigger, LANGSUNG Vehicle RED + Human GREEN (tanpa fase
    // kuning/transisi) — kendaraan berhenti & manusia dapat sinyal jalan duluan,
    // sambil menunggu CV memberi tahu aman/HOLD dan berapa lama timer-nya.
    setSystemState(SYS_CHECK_CV);
    applyTrafficState(VEHICLE_RED, HUMAN_GREEN);
    vTaskDelay(pdMS_TO_TICKS(200)); // beri waktu MQTT update timerDariCV terbaru (dipertahankan dari kode lama)

    bool sequenceAborted = false;

    for (;;) {
      bool hold = false;

      if (xSemaphoreTake(xMutexTimer, pdMS_TO_TICKS(10)) == pdTRUE) {
        hold = statusHold;
        xSemaphoreGive(xMutexTimer);
      }

      if (!hold) break; // aman & timer tersedia -> lanjut ke crossing

      // ---------- SYS_HOLD ----------
      setSystemState(SYS_HOLD);
      setBuzzer(false); // requirement: selama HOLD, buzzer harus OFF (bukan berdenyut)
      Serial.println("STATUS:HOLD_WAITING");

      vTaskDelay(pdMS_TO_TICKS(1000)); // cek ulang tiap 1 detik

      if ((millis() - getLastMotionTime()) > MOTION_TIMEOUT_MS) {
        Serial.println("SYSTEM:TIMEOUT_STANDBY");
        sequenceAborted = true;
        break;
      }
    }

    if (sequenceAborted) {
      setBuzzer(false);
      continue; // balik ke atas -> SYS_STANDBY (idle blink) lagi
    }

    // ---------- SYS_CROSSING ----------
    // Lampu sudah Vehicle RED + Human GREEN dari fase CHECK_CV, tidak perlu diubah lagi.
    setSystemState(SYS_CROSSING);
    int countdown = 10;
    if (xSemaphoreTake(xMutexTimer, pdMS_TO_TICKS(10)) == pdTRUE) {
      countdown = timerDariCV;
      xSemaphoreGive(xMutexTimer);
    }

    update7Segment(countdown);
    setBuzzer(true); // buzzer ON selama countdown > 0 (tidak ada timer buzzer terpisah)
    Serial.println("STATUS:CROSSING_START");
    Serial.print("COUNTDOWN:");
    Serial.println(countdown);

    unsigned long lastSecondTick = millis();
    while (countdown > 0) {
      vTaskDelay(pdMS_TO_TICKS(100));

      if (millis() - lastSecondTick >= 1000UL) {
        lastSecondTick += 1000UL;
        countdown--;
        update7Segment(countdown);
        Serial.print("COUNTDOWN:");
        Serial.println(countdown);
      }
    }
    setBuzzer(false); // countdown == 0 -> buzzer OFF

    // ---------- COUNTDOWN SELESAI: langsung Vehicle GREEN + Human RED, tanpa all-red gap ----------
    sevenSeg.clear();
    applyTrafficState(VEHICLE_GREEN, HUMAN_RED);

    // ---------- SYS_VEHICLE_GREEN_5S ----------
    setSystemState(SYS_VEHICLE_GREEN_5S);
    Serial.println("SYSTEM:VEHICLE_GREEN_5S_START");
    vTaskDelay(pdMS_TO_TICKS(VEHICLE_GREEN_HOLD_MS)); // non-blocking (vTaskDelay), task lain tetap jalan

    Serial.println("SYSTEM:CYCLE_DONE_RETURN_TO_IDLE");
    // balik ke atas for(;;) -> SYS_STANDBY (Vehicle YELLOW blink idle)
  }
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  // --- Input ---
  pinMode(PIR_PIN, INPUT_PULLDOWN);
  pinMode(LDR_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // --- Output ---
  pinMode(VEHICLE_RED_PIN, OUTPUT);
  pinMode(VEHICLE_YELLOW_PIN, OUTPUT);
  pinMode(VEHICLE_GREEN_PIN, OUTPUT);
  pinMode(HUMAN_RED_PIN, OUTPUT);
  pinMode(HUMAN_GREEN_PIN, OUTPUT);
  pinMode(PJU_WHITE_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(PJU_WHITE_PIN, LOW);

  // Kondisi awal aman sementara (semua kendaraan berhenti) sebelum vTaskControl
  // mengambil alih dan langsung masuk ke idle-blink (SYS_STANDBY).
  applyTrafficState(VEHICLE_RED, HUMAN_RED);

  // --- 7-Segment ---
  sevenSeg.setBrightness(0x0f);
  sevenSeg.clear();

  // --- WiFi & MQTT (TIDAK DIUBAH) ---
  WiFi.begin(ssid, password);
  Serial.print("Konek WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi OK!");

  Serial.print("ESP32 IP: ");
  Serial.println(WiFi.localIP());

  client.setServer(mqtt_server, 1883);
  client.setCallback(mqttCallback);

  // --- Mutex (dibuat sebelum task) ---
  xMutexState = xSemaphoreCreateMutex();
  xMutexTimer = xSemaphoreCreateMutex();

  if (xMutexState == NULL || xMutexTimer == NULL) {
    Serial.println("FATAL:MUTEX_CREATION_FAILED");
    while (1);
  }

  // --- Tasks ---
  xTaskCreatePinnedToCore(vTaskPIR, "Task_PIR", 2048, NULL, 3, &xTaskPIRHandle, 1);
  xTaskCreatePinnedToCore(vTaskButton, "Task_Button", 2048, NULL, 3, &xTaskButtonHandle, 1);
  xTaskCreatePinnedToCore(vTaskMQTT, "Task_MQTT", 4096, NULL, 2, &xTaskMQTTHandle, 0);
  xTaskCreatePinnedToCore(vTaskControl, "Task_Control", 4096, NULL, 1, &xTaskControlHandle, 0);
  xTaskCreatePinnedToCore(vTaskEnv, "Task_Env", 2048, NULL, 1, &xTaskEnvHandle, 0);

  // Warm-up PIR: modul (mis. HC-SR501) butuh ~30-60 detik untuk stabil setelah power-on
  Serial.println("SYSTEM:PIR_WARMUP_START");
  delay(30000);
  Serial.println("SYSTEM:PIR_WARMUP_DONE");

  attachInterrupt(digitalPinToInterrupt(PIR_PIN), pirInterruptHandler, RISING);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonInterruptHandler, FALLING);

  Serial.println("SYSTEM:READY_DUAL_CORE_RTOS");
}

void loop() {
  vTaskDelete(NULL);
}
