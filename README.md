## SPECTRE <sup><sub>[S]tandalone device for [P]TT data [E]xtraction, [C]ompilation, [Tr]ansfer and [E]valuation</sub></sup>

This repository refers to the measuring unit presented in the MDPI article [Measuring Suite for Vascular Response Monitoring during Hyperbaric Oxygen Therapy by Means of Pulse Transit Time (PTT) Analysis](https://www.mdpi.com/1424-8220/22/21/8295).
It contains the measuring unit's PCB designed with Altium Designer 21.4.1 as well as the corresponding microcontroller firmware.

The project aims at providing the necessary data for anyone who wants to recreate the system.


## Instructions for how to use the data.
You can order the PCB at your favourite distributor using the ODB++ files.

The firmware is C based and was developed using the [Arduino IDE](https://www.arduino.cc/en/software). 
It is ready to be flashed onto the system's central microcontroller unit (ESP32).


## Licensing information

This project is published under [Attribution 4.0 International (CC BY 4.0)](https://creativecommons.org/licenses/by/4.0/).


### Include credit and licenses for embedded resources

* [AsyncTCP](https://github.com/me-no-dev/AsyncTCP)
* [Async-MQTT-Client](https://github.com/marvinroger/async-mqtt-client)
* [SPIFlash](https://github.com/kriswiner/SPIFlash)
* [ArduinoOTA](https://lastminuteengineers.com/esp32-ota-updates-arduino-ide/)
