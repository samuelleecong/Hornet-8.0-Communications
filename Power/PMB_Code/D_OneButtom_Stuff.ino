void checkTicks() {
  // include all buttons here to be checked
  button.tick(); // just call tick() to check the state.
}

// this function will be called when the button was pressed 1 time only.
// on relay
void singleClick() {
  set_relay();
}   

// this function will be called when the button was pressed 2 times in a short timeframe.
// normal shut down, will call for shutdown
// or cancel shut down, but SBC might shut down if it received the shut down call
void doubleClick() {
  shut_down = !shut_down && relay_state;
}

// this function will be called when the button was pressed multiple times in a short timeframe.
void multiClick() {
  int n = button.getNumberClicks();
  if (n == 3) {
    // calibrate current sensor
    // calibrating current sensor will turn off the relay, dun calibrate with SBC in
    read_current(true);
  } else if (n == 4) {
    // calibrate voltage sensor
    // disconnect usb, connect jst to bench power supply set to 16.8 V, check with DMM
    read_voltage(0, true);
    read_voltage(1, true);
  } else {
    Serial.print("multiClick(");
    Serial.print(n);
    Serial.println(") detected.");
  }
} // multiClick


// this function will be called when the button was held down for 1 second or more.
// force off relay, will not ask for shut down, used for debug
void pressStart() {
  Serial.println("pressStart()");
  reset_relay();
  pressStartTime = millis() - 1000; // as set in setPressTicks()
} // pressStart()


// this function will be called when the button was released after a long hold.
void pressStop() {
  Serial.print("pressStop(");
  Serial.print(millis() - pressStartTime);
  Serial.println(") detected.");
} // pressStop()
  
