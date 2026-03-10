#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>

// Pins
#define USER_LED 10
#define SDA_PIN 3
#define SCL_PIN 8
#define V_BATT_SENSE_PIN 0
#define CURR_SENSE_PIN 1

Adafruit_LSM6DSOX lsm6ds;

// Shash's MAc
uint8_t broadcastAddress[] = {0x80, 0xF1, 0xB2, 0xEE, 0xC1, 0x38};

typedef struct struct_message {
  float ax;
  float ay;
  float az;
  float gx;
  float gy;
  float gz;
  uint32_t batt_mv;
  uint32_t curr_mA;
} struct_message;

struct_message myData;
esp_now_peer_info_t peerInfo;

void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  Serial.print("Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void setup() {
  Serial.begin(115200);
  delay(500);

  analogSetPinAttenuation(V_BATT_SENSE_PIN, ADC_6db);
  analogSetPinAttenuation(CURR_SENSE_PIN, ADC_11db);

  // WiFi for ESP-NOW
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!lsm6ds.begin_I2C()) {
    Serial.println("IMU not found!");
    while (1);
  }

  lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
  lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_2000_DPS);
  lsm6ds.setAccelDataRate(LSM6DS_RATE_104_HZ);
  lsm6ds.setGyroDataRate(LSM6DS_RATE_104_HZ);

  Serial.println("Wrist IMU ready. Sending data...");
}

void loop() {

  sensors_event_t accel, gyro, temp;
  lsm6ds.getEvent(&accel, &gyro, &temp);

  myData.ax = accel.acceleration.x;
  myData.ay = accel.acceleration.y;
  myData.az = accel.acceleration.z;

  myData.gx = gyro.gyro.x;
  myData.gy = gyro.gyro.y;
  myData.gz = gyro.gyro.z;

  uint32_t v_batt_sense_raw_mv = analogReadMilliVolts(V_BATT_SENSE_PIN);
  uint32_t v_batt_mv = v_batt_sense_raw_mv * 3;   // 200k / 100k divider

  uint32_t curr_sense_raw_mv = analogReadMilliVolts(CURR_SENSE_PIN);
  uint32_t curr_sense_mA = curr_sense_raw_mv / 5; // 5 mV per mA

  myData.batt_mv = v_batt_mv;
  myData.curr_mA = curr_sense_mA;

  esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

  delay(10);  // 100 Hz
}
