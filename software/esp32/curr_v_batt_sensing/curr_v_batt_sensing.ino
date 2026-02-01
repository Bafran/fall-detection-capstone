// GPIO 10 is USER_LED
#define USER_LED 10

// Custom I2C pins for ESP32-C3-WROOM-02
#define SDA_PIN 3
#define SCL_PIN 8

#define V_BATT_SENSE_PIN 0
#define CURR_SENSE_PIN 1

void setup(void) {
  pinMode(USER_LED, OUTPUT);
  // Set LED high
  digitalWrite(USER_LED, HIGH);
  
  Serial.begin(115200);

  analogSetPinAttenuation(V_BATT_SENSE_PIN, ADC_6db);
  analogSetPinAttenuation(CURR_SENSE_PIN, ADC_11db); 
}

void loop() {
  static bool led_state = true;
  
  // Toggle LED
  if (led_state) {
    digitalWrite(USER_LED, HIGH);
  } else {
    digitalWrite(USER_LED, LOW);
  }
  led_state = !led_state;

  /* Read Sense ADCs */

  // Battery sense ADC
  // Read through a 200k ohm / 100k ohm voltage divider
  uint32_t v_batt_sense_pin_raw_mv = analogReadMilliVolts(V_BATT_SENSE_PIN);
  uint32_t v_batt_sense_pin_mv = (v_batt_sense_pin_raw_mv * 3); // Scale up to actual battery voltage

  // Current sense ADC
  // Current is read through a 100mOhm shunt resistor and amplified 50x
  // V = I * R where R = 0.1 Ohm and I = 1mA (we desire the gain of mV/mA)
  // Then, V = 0.001 * 0.1 = 1 * 10^(-4) V = 0.1 mV
  // Amplify by 50x => V = 5 mV per mA
  uint32_t curr_sense_raw_mv = analogReadMilliVolts(CURR_SENSE_PIN);
  uint32_t curr_sense_mA = curr_sense_raw_mv / 5; // Convert mV to mA


  Serial.printf("Battery Voltage: %u mV | Current Sense: %u mA\n", v_batt_sense_pin_mv, curr_sense_mA);
  delay(500);
}
