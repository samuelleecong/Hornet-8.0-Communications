UPDATE 3/5: 19jan_NEW_CAN.ino has been archived, changed to integrate Mahony filter for IMU

From float function from old code (both_sensors_CAN) does not work. Redeployed code to include an addition CAN frame from the pico in order to fit the 
decimal points and not lose precision. Pico code done by YS, SBC done by ZX.

Check again the max range of values that accelerometer and gyroscope can output. Gryoscope range from +- 245 degrees, accelerometer is +- 2g. We are assuming
default setting (245 & 2) with 2dp precision here since the respective pins to control the sensors' range aren't connected.
