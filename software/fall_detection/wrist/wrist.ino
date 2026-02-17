#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>

#define USER_LED 10
#define SDA_PIN 3
#define SCL_PIN 8

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
} struct_message;

struct_message myData;
esp_now_peer_info_t peerInfo;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void setup() {
  Serial.begin(115200);
  delay(1000);

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

  esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

  delay(10);  // 100 Hz
}
