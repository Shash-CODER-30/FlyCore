# FlyCore
FlyCore is an experimental open-source project to power Arduino drones.

Disclaimer: Use this firmware at your own risk. The author assumes no liability for damage or injury resulting from its use.

Drone:
Parts:
1. Arduino UNO (any other ATMEGA328P-based board may be used)
2. nRF24L01 module
3. MPU6050
4. ESC (supporting standard RC PWM input (1000 µs - 2000 µs pulse width) [x4]
5. BLDC motor [x4]

Notes:
1. Improving the 3.3V supply to the nRF24L01 using a 3.3V regulator, a decoupling capacitor, or a breakout board is strongly recommended.
2. Keeping the SPI communication lines short between the nRF24L01 and the Arduino is strongly recommended.
3. Ensure a stable power supply appropriate for your battery voltage and current draw. A dedicated voltage regulator for the flight controller     and RF module is recommended.
4. Motor mapping should be as described: front-left: M1; front-right: M2; back-left: M3; back-right: M4.
5. Minor drift may occur from sensor noise and calibration offsets.
6. Switch up two of the phases between the ESC and the BLDC motor if needed, to reverse the direction.
7. M1 should spin CW, M2 CCW, M3 CCW, and M4 CW.
8. Behavior during RF signal loss depends on firmware configuration and may result in motor cutoff.

Controller:
Parts:
1. Arduino UNO (any other ATMEGA328P-based board may be used)
2. nRF24L01 module
3. Joystick [x2]
4. Switch

Notes:
1. The arm switch is used to toggle controls ON or OFF.
2. The joysticks should be placed in such a way that the X-axis sits horizontally (regular layout). For a mode-2 layout, J2 is placed on the       left, and J1 is placed on the right.
3. Improving the 3.3V supply to the nRF24L01 using a 3.3V regulator, a decoupling capacitor, or a breakout board is strongly recommended.
4. Keeping the SPI communication lines short and away from motor lines between the nRF24L01 and the Arduino is strongly recommended.
5. Using the PA+LNA version of the nRF24L01 is strongly recommended to improve range.

Remarks:
1. Feel free to tune the control constants.
2. Feel free to change the nRF24L01 code, while making sure it's the same across the controller and the drone.
3. Feel free to change the nRF24L01 channel, while making sure it's the same across the controller and the drone.
