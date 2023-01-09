# Hornet-v8.0-PMB

BUTTON_____________________________________________________________________________________________

Press once to turn on

Press twice to turn off with shut down call, or to cancel shut down call
- shut down will either be initialized by button or when battery voltage gets too low

Press thrice to calibrate current sensor
- do not calibrate with SBC connected as it will turn it off without shut down call
- disconnect usb

Press four times to calibrate voltage sensor
- disconnect usb, connect jst to bench power supply set to 16.8 V, check with DMM

Press and hold to force shut down



LED INDICATOR________________________________________________________________________________________

Pink flashing/on only for a while = calibrating current/voltage sensor

Pink, Warm White, Green cycle = set up finished

1 = on

0 = off

X = whatever

F = flashing

P W G = Pink, Warm White, Green LEDs

1 0 0 = extremely low battery, power is not on, will not to be able to switch power on in this level without debug_mode == true

1 0 1 = low battery, power is not on, will not to be able to switch power on in this level without debug_mode == true

1 1 0 = extremely low battery, power is on, call for shut down

1 1 1 = low battery, power is on, call for resurface

0 0 1 = normal operation, power not on, can be turn on

0 1 1 = normal operation, power is on, can be turn off (shut down call or force shut down)

X 1 F = shut down call is initialized, SBC will shut down and power will be cut, can be cancelled by double press button but SBC might shut down anyway

0 0 0 = sometimes is like this when voltage transition thru the hysteresis for low voltage flag (idk how to fix)

if any other combination exists means got bug 
