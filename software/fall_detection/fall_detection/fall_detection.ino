#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "esp_timer.h"
#include "secrets.h"

#include <Adafruit_LSM6DSOX.h>
Adafruit_LSM6DSOX lsm6ds;

// GPIO 10 is USER_LED
#define USER_LED 10

// Custom I2C pins for ESP32-C3-WROOM-02
#define SDA_PIN 3
#define SCL_PIN 8

// Put your laptop's IP here (same Wi-Fi network)
const uint16_t SERVER_PORT = 9000;

WiFiUDP udp;
const char* DEVICE_ID = "chest";
const uint32_t SAMPLE_PERIOD_MS = 10;  // ~100 Hz

void setup(void) {
  pinMode(USER_LED, OUTPUT);
  // Set LED high
  digitalWrite(USER_LED, HIGH);

  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit LSM6DS test!");

  // Initialize I2C with custom pins
  Wire.begin(SDA_PIN, SCL_PIN);

  bool lsm6ds_success;

  // hardware I2C mode, can pass in address & alt Wire

  lsm6ds_success = lsm6ds.begin_I2C(LSM6DS_I2CADDR_DEFAULT, &Wire);

  if (!lsm6ds_success){
    Serial.println("Failed to find LSM6DS chip");
  }
  if (!lsm6ds_success) {
    while (1) {
      delay(10);
    }
  }

 Serial.println("LSM6DS Found!");

 // lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
 Serial.print("Accelerometer range set to: ");
 switch (lsm6ds.getAccelRange()) {
 case LSM6DS_ACCEL_RANGE_2_G:
   Serial.println("+-2G");
   break;
 case LSM6DS_ACCEL_RANGE_4_G:
   Serial.println("+-4G");
   break;
 case LSM6DS_ACCEL_RANGE_8_G:
   Serial.println("+-8G");
   break;
 case LSM6DS_ACCEL_RANGE_16_G:
   Serial.println("+-16G");
   break;
 }

 // lsm6ds.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
 Serial.print("Accelerometer data rate set to: ");
 switch (lsm6ds.getAccelDataRate()) {
 case LSM6DS_RATE_SHUTDOWN:
   Serial.println("0 Hz");
   break;
 case LSM6DS_RATE_12_5_HZ:
   Serial.println("12.5 Hz");
   break;
 case LSM6DS_RATE_26_HZ:
   Serial.println("26 Hz");
   break;
 case LSM6DS_RATE_52_HZ:
   Serial.println("52 Hz");
   break;
 case LSM6DS_RATE_104_HZ:
   Serial.println("104 Hz");
   break;
 case LSM6DS_RATE_208_HZ:
   Serial.println("208 Hz");
   break;
 case LSM6DS_RATE_416_HZ:
   Serial.println("416 Hz");
   break;
 case LSM6DS_RATE_833_HZ:
   Serial.println("833 Hz");
   break;
 case LSM6DS_RATE_1_66K_HZ:
   Serial.println("1.66 KHz");
   break;
 case LSM6DS_RATE_3_33K_HZ:
   Serial.println("3.33 KHz");
   break;
 case LSM6DS_RATE_6_66K_HZ:
   Serial.println("6.66 KHz");
   break;
 }

 // lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS );
 Serial.print("Gyro range set to: ");
 switch (lsm6ds.getGyroRange()) {
 case LSM6DS_GYRO_RANGE_125_DPS:
   Serial.println("125 degrees/s");
   break;
 case LSM6DS_GYRO_RANGE_250_DPS:
   Serial.println("250 degrees/s");
   break;
 case LSM6DS_GYRO_RANGE_500_DPS:
   Serial.println("500 degrees/s");
   break;
 case LSM6DS_GYRO_RANGE_1000_DPS:
   Serial.println("1000 degrees/s");
   break;
 case LSM6DS_GYRO_RANGE_2000_DPS:
   Serial.println("2000 degrees/s");
   break;
 case ISM330DHCX_GYRO_RANGE_4000_DPS:
   Serial.println("4000 degrees/s");
   break;
 }
 // lsm6ds.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
 Serial.print("Gyro data rate set to: ");
 switch (lsm6ds.getGyroDataRate()) {
 case LSM6DS_RATE_SHUTDOWN:
   Serial.println("0 Hz");
   break;
 case LSM6DS_RATE_12_5_HZ:
   Serial.println("12.5 Hz");
   break;
 case LSM6DS_RATE_26_HZ:
   Serial.println("26 Hz");
   break;
 case LSM6DS_RATE_52_HZ:
   Serial.println("52 Hz");
   break;
 case LSM6DS_RATE_104_HZ:
   Serial.println("104 Hz");
   break;
 case LSM6DS_RATE_208_HZ:
   Serial.println("208 Hz");
   break;
 case LSM6DS_RATE_416_HZ:
   Serial.println("416 Hz");
   break;
 case LSM6DS_RATE_833_HZ:
   Serial.println("833 Hz");
   break;
 case LSM6DS_RATE_1_66K_HZ:
   Serial.println("1.66 KHz");
   break;
 case LSM6DS_RATE_3_33K_HZ:
   Serial.println("3.33 KHz");
   break;
 case LSM6DS_RATE_6_66K_HZ:
   Serial.println("6.66 KHz");
   break;
 }
  
  Serial.print("Device ID: ");
  Serial.println(DEVICE_ID);

  // Connecting to WiFi
  Serial.print("Connecting to WiFi: ");
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    digitalWrite(USER_LED, !digitalRead(USER_LED));
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi connected!");
  Serial.print("ESP32 IP address: ");
  Serial.println(WiFi.localIP());

  digitalWrite(USER_LED, HIGH);

  Serial.print("Sending UDP to ");
  Serial.print(SERVER_IP);
  Serial.print(":");
  Serial.println(SERVER_PORT);

  Serial.println("Format: device_id,t_device_us,ax,ay,az,gx,gy,gz,temp");

}

void loop() {
  static bool led_state = false;
  static uint32_t last_sample_ms = 0;

  static uint32_t last_debug_ms = 0;
  static uint32_t packet_count = 0;

  uint32_t now_ms = millis();

  // Simple scheduler: run at ~100 Hz
  if (now_ms - last_sample_ms < SAMPLE_PERIOD_MS) {
    return;
  }
  last_sample_ms += SAMPLE_PERIOD_MS;

  // Blink LED at sampling rate / 2 (just a heartbeat)
  led_state = !led_state;
  digitalWrite(USER_LED, led_state ? HIGH : LOW);

  sensors_event_t accel, gyro, temp;
  lsm6ds.getEvent(&accel, &gyro, &temp);

  int64_t t_us = esp_timer_get_time();  // microseconds since boot

  // Build UDP message
  char buf[220];
  int n = snprintf(
    buf, sizeof(buf),
    "%s,%lld,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.2f",
    DEVICE_ID,
    (long long)t_us,
    accel.acceleration.x, accel.acceleration.y, accel.acceleration.z,
    gyro.gyro.x, gyro.gyro.y, gyro.gyro.z,
    temp.temperature
  );

  // Send UDP packet
  udp.beginPacket(SERVER_IP, SERVER_PORT);
  udp.write((uint8_t*)buf, n);
  udp.endPacket();

  packet_count++;

  // Debug print once per second (won't destroy timing)
  if (now_ms - last_debug_ms >= 1000) {
    last_debug_ms = now_ms;

    Serial.print("[DEBUG] packets_sent=");
    Serial.print(packet_count);
    Serial.print("  t_us=");
    Serial.print((long long)t_us);
    Serial.print("  ax=");
    Serial.print(accel.acceleration.x, 3);
    Serial.print(" ay=");
    Serial.print(accel.acceleration.y, 3);
    Serial.print(" az=");
    Serial.print(accel.acceleration.z, 3);
    Serial.print("  WiFi=");
    Serial.println(WiFi.status() == WL_CONNECTED ? "OK" : "DISCONNECTED");
  }
}