void checkTicks() {
  // include all buttons here to be checked
  button.tick(); // just call tick() to check the state.
}

// this function will be called when the button was pressed 1 time only.
void singleClick() {
}

// this function will be called when the button was pressed 2 times in a short timeframe.
void doubleClick() {
}

// this function will be called when the button was pressed multiple times in a short timeframe.
void multiClick() {
  int n = button.getNumberClicks();
  if (n == 3) {
    // calibrate current sensor
    // calibrating current sensor will turn off the relay, dun calibrate with SBC in
    // will only do if debug_mode is true, to avoid misfiring when using the magnet
    if (debug_mode) {
      read_current(true);
    }
  } else if (n == 4) {
    // calibrate voltage sensor
    // disconnect usb, connect jst to bench power supply set to 16.8 V, check with DMM
    // will only do if debug_mode is true, to avoid misfiring when using the magnet
    if (debug_mode) {
      read_voltage(0, true);
      read_voltage(1, true);
    }
  } else {
    Serial.print("multiClick(");
    Serial.print(n);
    Serial.println(") detected.");
  }
} // multiClick

// this function will be called when the button was held down for 1 second or more.
void pressStart() {
  Serial.println("pressStart()");

  // if relay is on, then call for call for shutdown, this is normal shut down
  // if relay is off, then will turn on
  // to force shut down use the other reed swtich
  
  if (relay_state) {
    shut_down = true;
  } else {
    set_relay();
  }

  pressStartTime = millis() - 1000; // as set in setPressTicks()
} // pressStart()

// this function will be called when the button was released after a long hold.
void pressStop() {
  Serial.print("pressStop(");
  Serial.print(millis() - pressStartTime);
  Serial.println(") detected.");
} // pressStop()
