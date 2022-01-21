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

Important note
--------------

This example depends on the [BSEC 1.4.8.0 software by BOSCH](https://www.bosch-sensortec.com/software-tools/software/bsec/). Due to copyright we cannot share the precompiled .uf2 application. To run the application you need to

- Clone this repository
- Download the [BSEC](https://www.bosch-sensortec.com/software-tools/software/bsec/)
- Unzip it into the cloned repository (next to this README.md)
  - Browse the BSEC_1.4.8.0_Generic_Release folder and copy the following files next to this README.md
    - All the content in the /BSEC_1.4.8.0_Generic_Release/examples/bsec_iot_example folder
      - bsec_datatypes.h
      - bsec_integration.c
      - bsec_integration.h
      - bsec_interface.h
      - bme680.c
      - bme680.h
      - bme680_defs.h
    - /BSEC_1.4.8.0_Generic_Release/algo/normal_version/bin/RaspberryPi/PiThree_ArmV6/libalgobsec.a
- Build the application using the Pico SDK
- Flash your Pico with the newly created .uf2 file
