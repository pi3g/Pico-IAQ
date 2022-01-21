# Pico-IAQ
C Example for IAQ with BME68X and BOSCH BSEC 1.4.8.0 on the Raspberry Pi Pico

This example allows you to track and display the **IAQ** - **I**ndoor **A**ir **Q**uality using your Raspberry Pi Pico and a BME68X sensor.
You can even equip you Pico with three LEDs (Red, Yellow and Green) and a buzzer.
The LEDs will indicate the IAQ.
| LED | IAQ | Meaning |
| --- | --- | ------- |
| GREEN | < 100 | Good |
| YELLOW | 100 - 300 | Average |
| RED | > 300 | Bad |

If the IAQ is in the bad range ( > 300) for longer than 60 seconds, the buzzer will start to buzz.
