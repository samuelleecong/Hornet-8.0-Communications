#include <OneButton.h>
#include <EEPROM.h>
#include <ACAN2515.h>

// Sensor Stuff //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define max_current_address 0 // address 0  for max current drawn
#define batt_A_calibrated_address 1 // 4 bytes per float
#define batt_B_calibrated_address 5 // 4 bytes per float

#define batt_A_sensor A1
#define batt_B_sensor A0
#define current_sensor A7
// End of Sensor Stuff //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define button_pin 4

#define Pink_LED 3
#define WarmW_LED 5
#define Green_LED 6
#define relay_set A2
#define relay_reset A3

#define relay_set_pulse 120

// MCP Stuff //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static const byte MCP2515_SCK  = 13 ; // SCK input of MCP2515 (adapt to your design)
static const byte MCP2515_MOSI = 11 ; // SDI input of MCP2515 (adapt to your design)
static const byte MCP2515_MISO = 12 ; // SDO output of MCP2515 (adapt to your design)
static const byte MCP2515_CS   = 10 ;  // CS input of MCP2515 (adapt to your design)

ACAN2515 can (MCP2515_CS, SPI, 255) ;

static const uint32_t QUARTZ_FREQUENCY = 8UL * 1000UL * 1000UL ; // 8 MHz
// End of MCP Stuff //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// OneButton Stuff //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// The 2. parameter activeLOW is true, because external wiring sets the button to LOW when pressed.
OneButton button(button_pin, true);

// save the millis when a press has started.
unsigned long pressStartTime;
// End of OneButton stuff //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool debug_mode = false;                  // allows user to turn on even when battery is low
bool debug_print_on = true;              // serial print enable

bool relay_state = LOW;                   // relay state flag

float batt_A_volt = 0;                    // storing average battery A voltage
float batt_B_volt = 0;                    // storing average battery B voltage
float current = 0;                        // storing average current
float max_current = 0;                    // storing peak current drawn, currently no way to clear except for restarting nano

uint8_t counter = 0;

float low_batt = 15.0;                    // gives warning of low battery and would not on if shut offed
float shut_off_voltage = 14.8;            // calls for shut down and shut down
float low_batt_recover = low_batt + 0.3;  // hysteresis so no flickers

float shut_down_current = 0.5;            // check for current less than this before shutting down

bool low_battery = false;                 // low battery flag
bool shut_down = false;                   // shut down everything flag

unsigned long prev_comms_millis;
unsigned long prev_sample_millis;

void setup() {
  Serial.begin(115200);

  pinMode(Pink_LED, OUTPUT);
  pinMode(WarmW_LED, OUTPUT);
  pinMode(Green_LED, OUTPUT);

  pinMode(relay_set, OUTPUT);
  pinMode(relay_reset, OUTPUT);

  pinMode(batt_A_sensor, INPUT);
  pinMode(batt_B_sensor, INPUT);
  pinMode(current_sensor, INPUT);

  digitalWrite(Pink_LED, LOW);
  digitalWrite(WarmW_LED, LOW);
  digitalWrite(Green_LED, LOW);

  digitalWrite(relay_set, LOW);
  digitalWrite(relay_reset, LOW);

  read_current(true); // Calibrate current sensor

  // OneButton Stuff //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  attachInterrupt(digitalPinToInterrupt(button_pin), checkTicks, CHANGE);
  button.attachClick(singleClick);
  button.attachDoubleClick(doubleClick);
  button.attachMultiClick(multiClick);
  button.attachLongPressStart(pressStart);
  button.attachLongPressStop(pressStop);
  // End of OneButton Stuff //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  start_up_led();
  debug_serial_print(debug_print_on, "set up done");
}

void loop() {
  button.tick();
  
  digitalWrite(WarmW_LED, relay_state);

  if (!shut_down || !relay_state || debug_mode) {
    if (millis() - prev_sample_millis > 100) {
      batt_A_volt += read_voltage(0, false);
      batt_B_volt += read_voltage(1, false);
      float instantaneous_current = read_current(false);
      current += instantaneous_current;
      if (instantaneous_current > max_current) {
        max_current = instantaneous_current;
      }
      counter += 1;
      prev_sample_millis = millis();
    }

    if (millis() - prev_comms_millis > 1000 && counter >= 10) {
      batt_A_volt = batt_A_volt / counter;
      batt_B_volt = batt_B_volt / counter;
      current = current / counter;

      if (batt_A_volt > low_batt_recover || batt_B_volt > low_batt_recover) {
        low_battery = false;
        shut_down = false;
        digitalWrite(Pink_LED, low_battery);
        digitalWrite(Green_LED, !shut_down);
      } else if (batt_A_volt > low_batt || batt_B_volt > low_batt) {
        digitalWrite(Pink_LED, low_battery);
        digitalWrite(Green_LED, !shut_down);
      } else if (batt_A_volt > shut_off_voltage || batt_B_volt > shut_off_voltage) {
        low_battery = true;
        digitalWrite(Pink_LED, low_battery);
        digitalWrite(Green_LED, !shut_down);
      } else {
        shut_down = true;
        digitalWrite(Pink_LED, shut_down);
        digitalWrite(Green_LED, !shut_down);
      }

      int batt_A_volt_send = int((batt_A_volt + 0.05) * 10);
      int batt_B_volt_send = int((batt_B_volt + 0.05) * 10);
      int current_send = int((current + 0.05) * 10);
      int max_current_send = int((max_current + 0.05) * 10);

      // mulitple debug print else string too long can't print for some reason
      if (debug_print_on) {
        debug_serial_print(debug_print_on, (String)"Battery A = " + batt_A_volt_send + " dV   " + "Battery B = " + batt_B_volt_send + " dV");
        debug_serial_print(debug_print_on, (String)"Current = " + current_send + " dA   " + "Max Current = " + max_current_send + " dA");
        debug_serial_print(debug_print_on, (String)"Low Battery, resurface = " + low_battery);
        debug_serial_print(debug_print_on, (String)"Shutting down = " + shut_down);
      }

      // only can send if MCP is powered
      if (relay_state) {
        CAN_bus_send(byte(batt_A_volt_send), byte(batt_B_volt_send), byte(current_send), byte(max_current_send), byte(low_battery), byte(shut_down), byte(0), byte(0));
      }

      batt_A_volt = 0;
      batt_B_volt = 0;
      current = 0;
      counter = 0;

      prev_comms_millis = millis();
    }
  } else if (shut_down && relay_state) {
    // call for shutdown
    // start reading can bus
    // if stop getting msg from sbc
    // check current is low enuf before shut down
    // if fail wait 10s then check again
    // off relay

    blink_led(Green_LED, 500);

    if (millis() - prev_comms_millis > 10000) {
      debug_serial_print(debug_print_on, "Shutting down");
      
      CAN_bus_send(byte(0), byte(0), byte(0), byte(0), byte(low_battery), byte(shut_down), byte(0), byte(0));
       
      if (!CAN_bus_SBC_shutdown() && (read_current(false) < shut_down_current)) {
          shut_down = false;
          reset_relay();
          debug_serial_print(debug_print_on, "shut down");
      }
      
      prev_comms_millis = millis();
    }
  }
}
