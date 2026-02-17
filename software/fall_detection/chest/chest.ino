#include <Wire.h>
#include "esp_timer.h"
#include <Adafruit_LSM6DSOX.h>
#include <math.h>
#include <esp_now.h>
#include <WiFi.h>

// Chest Master script and Wrist data receiver 

Adafruit_LSM6DSOX lsm6ds;

// ESP Now wrist message structure
typedef struct __attribute__((packed)) {
  float ax, ay, az;
  float gx, gy, gz;
} WristMsg;

volatile WristMsg wristData;
volatile int64_t last_wrist_rx_us = 0;

// Pins
#define USER_LED 10
#define SDA_PIN 3
#define SCL_PIN 8

// Sampling Rate
const uint32_t SAMPLE_PERIOD_MS = 10; // ~100 Hz

// Helper Functions 
static inline float mag3f(float x, float y, float z) {
  return sqrtf(x*x + y*y + z*z);
}
static void blink_confirm() {
  digitalWrite(USER_LED, HIGH);
  delay(250);
  digitalWrite(USER_LED, LOW);
}

// Fall Detection State Machine
enum FallState { FD_IDLE, FD_IMPACT_DETECTED };
FallState fall_state = FD_IDLE;

int64_t impact_time_us = 0;
int64_t chest_still_start_us = 0;

const float G = 9.81f;

// Impact trigger thresholds (chest)
const float IMPACT_A_THR = 25.0f;        // m/s^2
const float IMPACT_W_THR = 6.0f;         // rad/s
const float GYRO_IMPACT_A_MIN = 11.0f;   // if gyro triggers, require some accel too

// Chest stillness to qualify as candidate fall
const float CHEST_STILL_W_THR  = 1.0f;   // rad/s
const float CHEST_STILL_A_BAND = 2.0f;   // m/s^2
const int64_t CHEST_STILL_US   = 1500000;

// Guards
const int64_t TIMEOUT_US = 6000000;
const int64_t STARTUP_IGNORE_US = 2000000;

// Debouncing impact
uint8_t impact_hits = 0;
const uint8_t IMPACT_HITS_REQUIRED = 3;

// Verification states  
bool verifying = false;
int64_t verify_start_us = 0;

// Verify timing
const int64_t VERIFY_TIMEOUT_US = 12000000;

// Wrist packet freshness
const int64_t WRIST_RX_TIMEOUT_US = 800000;

// Stillness thresholds (tune this later)
const float CHEST_VERIFY_W_THR  = 2.0f;
const float WRIST_VERIFY_W_THR  = 2.5f;

const float CHEST_VERIFY_A_BAND = 3.5f;
const float WRIST_VERIFY_A_BAND = 4.0f;

// Score accumulator: increments when BOTH still, decays when not
int32_t verify_still_score = 0;
const int32_t VERIFY_SCORE_REQ = 500;   // confirm once we reach this
const int32_t SCORE_INC = 1;            // +1 each sample both still
const int32_t SCORE_DEC = 1;            // -1 each sample not both still
const int32_t SCORE_MAX = 700;          // cap (prevents runaway)

// ESP-NOW receive callback
void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
  if (len == (int)sizeof(WristMsg)) {
    memcpy((void*)&wristData, incomingData, sizeof(WristMsg));
    last_wrist_rx_us = esp_timer_get_time();
  }
}

void setup() {
  pinMode(USER_LED, OUTPUT);
  digitalWrite(USER_LED, LOW);

  Serial.begin(115200);
  delay(500);

  // ESP now init
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (1) { delay(500); }
  }
  esp_now_register_recv_cb(OnDataRecv);
  Serial.println("ESP-NOW receiver ready");

  // Chest IMU init
  Wire.begin(SDA_PIN, SCL_PIN);
  if (!lsm6ds.begin_I2C(LSM6DS_I2CADDR_DEFAULT, &Wire)) {
    Serial.println("Chest IMU not found!");
    while (1) { delay(500); }
  }

  lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
  lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
  lsm6ds.setAccelDataRate(LSM6DS_RATE_104_HZ);
  lsm6ds.setGyroDataRate(LSM6DS_RATE_104_HZ);

  Serial.println("Chest fall detector ready.");
  Serial.println("Flow: impact -> chest still(1.5s) -> VERIFY score (both still ~5s total)");
}

void loop() {
  static uint32_t last_sample_ms = 0;
  static uint32_t last_debug_ms = 0;
  static uint32_t last_verify_dbg_ms = 0;

  uint32_t now_ms = millis();
  if (now_ms - last_sample_ms < SAMPLE_PERIOD_MS) return;
  last_sample_ms += SAMPLE_PERIOD_MS;

  int64_t t_us = esp_timer_get_time();

  // Read the chest IMU
  sensors_event_t accel, gyro, temp;
  lsm6ds.getEvent(&accel, &gyro, &temp);

  float chest_a_mag = mag3f(accel.acceleration.x, accel.acceleration.y, accel.acceleration.z);
  float chest_w_mag = mag3f(gyro.gyro.x, gyro.gyro.y, gyro.gyro.z);

  // Copy the wrist data
  WristMsg w;
  memcpy(&w, (const void*)&wristData, sizeof(WristMsg));
  int64_t wrist_rx_age_us = t_us - last_wrist_rx_us;

  float wrist_a_mag = mag3f(w.ax, w.ay, w.az);
  float wrist_w_mag = mag3f(w.gx, w.gy, w.gz);

  // Ignoring Startup readings for 2 seconds to allow sensor stabilization
  if (t_us < STARTUP_IGNORE_US) {
    fall_state = FD_IDLE;
    verifying = false;
    impact_hits = 0;
    chest_still_start_us = 0;
    verify_still_score = 0;
    return;
  }

  // Chest candidate detection
  if (!verifying) {
    switch (fall_state) {
      case FD_IDLE: {
        bool impact_now =
          (chest_a_mag > IMPACT_A_THR) ||
          ((chest_w_mag > IMPACT_W_THR) && (chest_a_mag > GYRO_IMPACT_A_MIN));

        if (impact_now) impact_hits++;
        else impact_hits = 0;

        if (impact_hits >= IMPACT_HITS_REQUIRED) {
          impact_hits = 0;
          fall_state = FD_IMPACT_DETECTED;
          impact_time_us = t_us;
          chest_still_start_us = 0;
          Serial.println(">>> CHEST IMPACT DETECTED");
        }
        break;
      }

      case FD_IMPACT_DETECTED: {
        bool chest_still =
          (chest_w_mag < CHEST_STILL_W_THR) &&
          (fabsf(chest_a_mag - G) < CHEST_STILL_A_BAND);

        if (chest_still) {
          if (chest_still_start_us == 0) chest_still_start_us = t_us;
        } else {
          chest_still_start_us = 0;
        }

        if (chest_still_start_us != 0 && (t_us - chest_still_start_us) >= CHEST_STILL_US) {
          Serial.println("***** CHEST FALL CANDIDATE -> START VERIFY *****");

          verifying = true;
          verify_start_us = t_us;
          verify_still_score = 0;

          Serial.print("VERIFY START t_us=");
          Serial.println((long long)verify_start_us);

          fall_state = FD_IDLE;
          return;
        }

        if ((t_us - impact_time_us) > TIMEOUT_US) {
          Serial.println("Impact cancelled (chest timeout)");
          fall_state = FD_IDLE;
          chest_still_start_us = 0;
          impact_hits = 0;
          return;
        }

        break;
      }
    }
  }

  // Verification Stage using a score based stillness
  if (verifying) {
    bool wrist_fresh = (wrist_rx_age_us >= 0) && (wrist_rx_age_us < WRIST_RX_TIMEOUT_US);

    bool chest_still_verify =
      (chest_w_mag < CHEST_VERIFY_W_THR) &&
      (fabsf(chest_a_mag - G) < CHEST_VERIFY_A_BAND);

    bool wrist_still_verify =
      wrist_fresh &&
      (wrist_w_mag < WRIST_VERIFY_W_THR) &&
      (fabsf(wrist_a_mag - G) < WRIST_VERIFY_A_BAND);

    bool both_still = chest_still_verify && wrist_still_verify;

    // Score update
    if (both_still) {
      verify_still_score += SCORE_INC;
      if (verify_still_score > SCORE_MAX) verify_still_score = SCORE_MAX;
    } else {
      verify_still_score -= SCORE_DEC;
      if (verify_still_score < 0) verify_still_score = 0;
    }

    // Confirm
    if (verify_still_score >= VERIFY_SCORE_REQ) {
      Serial.print("######## FALL CONFIRMED ######## elapsed_s=");
      Serial.print((t_us - verify_start_us) / 1000000.0, 3);
      Serial.print("  score=");
      Serial.println(verify_still_score);
      blink_confirm();

      verifying = false;
      verify_still_score = 0;
      return;
    }

    // Timeout
    if ((t_us - verify_start_us) > VERIFY_TIMEOUT_US) {
      Serial.print("VERIFY TIMEOUT (not confirmed) -- elapsed_s=");
      Serial.print((t_us - verify_start_us) / 1000000.0, 3);
      Serial.print("  score=");
      Serial.println(verify_still_score);

      verifying = false;
      verify_still_score = 0;
      return;
    }

    // Once a second verify debug using DEVICE TIME
    if (now_ms - last_verify_dbg_ms >= 1000) {
      last_verify_dbg_ms = now_ms;

      Serial.print("[VERIFY] elapsed_s=");
      Serial.print((t_us - verify_start_us) / 1000000.0, 3);
      Serial.print(" score=");
      Serial.print(verify_still_score);
      Serial.print(" chestStill=");
      Serial.print(chest_still_verify ? "Y" : "N");
      Serial.print(" wristStill=");
      Serial.print(wrist_still_verify ? "Y" : "N");
      Serial.print(" wristAgeMs=");
      Serial.print((long)(wrist_rx_age_us / 1000));
      Serial.print(" wrist|a|=");
      Serial.print(wrist_a_mag, 2);
      Serial.print(" wrist|w|=");
      Serial.print(wrist_w_mag, 2);
      Serial.print(" chest|a|=");
      Serial.print(chest_a_mag, 2);
      Serial.print(" chest|w|=");
      Serial.println(chest_w_mag, 2);
    }
  }

  // Regular debug (1 Hz)
  if (now_ms - last_debug_ms >= 1000) {
    last_debug_ms = now_ms;

    Serial.print("Chest |a|=");
    Serial.print(chest_a_mag, 2);
    Serial.print(" |w|=");
    Serial.print(chest_w_mag, 2);

    Serial.print("  Wrist |a|=");
    Serial.print(wrist_a_mag, 2);
    Serial.print(" |w|=");
    Serial.print(wrist_w_mag, 2);

    Serial.print("  verifying=");
    Serial.println(verifying ? "yes" : "no");
  }
}
