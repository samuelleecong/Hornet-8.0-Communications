void debug_serial_print(bool debug_on, String msg) {
  if (debug_on) {
    Serial.println(msg);
  }
}

void blink_led(byte LED_pin, int delay_ms) {
  static unsigned long blink_millis = 0;

  if (millis() - blink_millis > delay_ms) {
    digitalWrite(LED_pin, !digitalRead(LED_pin));
    blink_millis = millis();
  }
}

void start_up_led() {
  delay(500);
  digitalWrite(Pink_LED, HIGH);
  delay(100);
  digitalWrite(WarmW_LED, HIGH);
  delay(100);
  digitalWrite(Green_LED, HIGH);
  delay(100);
  digitalWrite(Pink_LED, LOW);
  delay(100);
  digitalWrite(WarmW_LED, LOW);
  delay(100);
  digitalWrite(Green_LED, LOW);
  delay(100);
}

float custom_EEPROM_get_float(uint8_t address) {
  float x;
  EEPROM.get(address, x);
  return x;
}
