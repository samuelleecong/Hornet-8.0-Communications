1. Open arduino -> tools -> board manager -> search Raspberry Pi Pico/RP2040 (by Earle F. Philhower) and install the board manager
2. Open files -> preferences -> paste: "https://github.com/earlephilhower/arduino-pico/releases/download/global/package_rp2040_index.json" into Additional Boards Manager
3. To upload code to the pico: Tools -> boards -> RP2040 boards -> select 'Raspberry Pi Pico' (default model)
4. Leave all the other options in tools as default, select port (should be labelled as Pico) and upload

DISCLAIMER: Checked to work on version 2.6.4 and below, may not be compatitble to work with newer versions. Do update the version accordingly.
