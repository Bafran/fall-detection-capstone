#include <Adafruit_LSM6DSOX.h>
Adafruit_LSM6DSOX lsm6ds;

// GPIO 10 is USER_LED
#define USER_LED 10

// Custom I2C pins for ESP32-C3-WROOM-02
#define SDA_PIN 3
#define SCL_PIN 8

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
}

void loop() {
  static bool led_state = false;
  if (led_state) {
    digitalWrite(USER_LED, HIGH);
  } else {
    digitalWrite(USER_LED, LOW);
  }
  led_state = !led_state;
  

 sensors_event_t accel, gyro, temp;

  //  /* Get new normalized sensor events */
  lsm6ds.getEvent(&accel, &gyro, &temp);

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("\t\tAccel X: ");
  Serial.print(accel.acceleration.x, 4);
  Serial.print(" \tY: ");
  Serial.print(accel.acceleration.y, 4);
  Serial.print(" \tZ: ");
  Serial.print(accel.acceleration.z, 4);
  Serial.println(" \tm/s^2 ");

  /* Display the results (rotation is measured in rad/s) */
  Serial.print("\t\tGyro  X: ");
  Serial.print(gyro.gyro.x, 4);
  Serial.print(" \tY: ");
  Serial.print(gyro.gyro.y, 4);
  Serial.print(" \tZ: ");
  Serial.print(gyro.gyro.z, 4);
  Serial.println(" \tradians/s ");

  Serial.print("\t\tTemp   :\t\t\t\t\t");
  Serial.print(temp.temperature);
  Serial.println(" \tdeg C");
  Serial.println();
  delay(1000);

}