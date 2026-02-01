#define V_BATT_SENSE_PIN 0
#define CURR_SENSE_PIN 1

void setup() {
  Serial.begin(115200);

  analogSetPinAttenuation(V_BATT_SENSE_PIN, ADC_6db);
  analogSetPinAttenuation(CURR_SENSE_PIN, ADC_11db); 
}

void loop() {

  uint32_t v_batt_sense_pin_mv = analogReadMilliVolts(V_BATT_SENSE_PIN);
  uint32_t curr_sense_mv = analogReadMilliVolts(CURR_SENSE_PIN);

  Serial.printf("v_batt_sense: %u mV | curr_sense: %u mV\n", v_batt_sense_pin_mv, curr_sense_mv);
  delay(500);
}
