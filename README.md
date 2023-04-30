# Engine Monitor based on SH-ESP32 with Engine Hat, SensESP and SignalK

This repository provides software for the [SH-ESP32 Engine Top Hat add-on board](https://hatlabs.fi/product/sh-esp32-engine-top-hat-kit/) to monitor engine and environmental data. It is based on the [example code/firmware of the SH-ESP32 Engine Hat](https://github.com/hatlabs/SH-ESP32-engine-hat-firmware). Which is based on SensESP, see [Getting Started](https://signalk.org/SensESP/pages/getting_started/) instructions). The device sends it data to [signalk-server](https://github.com/SignalK/signalk-server), which makes it available through [signalk-to-nmea2000](https://github.com/SignalK/signalk-to-nmea2000) plugin on the NMEA2000 / STNG bus.

I have the solution connected to a Volvo Penta MD2020 engine, this is what it looks like:
![Solution based on SH-ESP32 and Engine Hat](SH-ESP32-engine-hat.png)


## Mapping of sensors on SH-ESP32 and Engine Hat
Table below shows the connected sensors.


| Connector       | Sensor                                          | Pin       | Jumper | Signalk path                                 | NMEA2000 PGN  |
| --------------- | ----------------------------------------------- | --------- | ------ | -------------------------------------------- | ------------- |
| 6. Power           | 12V Power + | 12V+      | N/A    | N/A                                          |               |
| 6. Power           | Ground         | GND       | N/A    | N/A                                          |               |
| 5. CAN             | Connect [four wires](http://docs.hatlabs.fi/sh-esp32/pages/tutorials/nmea2000-gateway/)                                 | CAN-block | N/A    | N/A                                          |               |
| 4. 1-wire #1       | Temp oil carter                                 | 1-wire #1 | N/A    | propulsion.1.oilTemperature                  | 127489        |
| 4. 1-wire #1       | Temp engine room                                | 1-wire #1 | N/A    | environment.inside.engineRoom.temperature    |               |
| 4. 1-wire #1       | Temp cabin                                      | 1-wire #1 | N/A    | environment.outside.temperature              | 130310        |
| 3. EngineHat1      | Cooling water engine sender                                  | A1        | yes     | propulsion.1.temperature                     | 127489        |
| 3. EngineHat1      | Cooling water engine alarm                                  | D1        | no    | notifications.propulsion.1.overTemperature                                             |               |
| 3. EngineHat1      | Oil pressure engine sender                                   | A2        | yes     | propulsion.1.oilPressure                     | 127489        |
| 3. EngineHat1      | Oil pressure engine alarm                                   | D3        | yes    | notifications.propulsion.1.lowOilPressure                                             |               |
| 2. EngineHat2      | Fuel tank sender                               | A3        | nee    | tanks.fuel.1.currentLevel en currentVolume   |               |
| 2. EngineHat2      | RPM (W-terminal alternator)                                | D2        | no     | propulsion.1.revolutions                     | 127489,127488 |
| 2. EngineHat2      | NOT USED                                        | A4        | -      |                                              |               |
| 2. EngineHat2      | NOT USED                                        | D4        | -      |                                              |               |
| 1. 1-Wire #2       | Future: fridge                                  | 1-wire #2 | N/A    | environment.inside.refridgerator.temperature | 130312        |
| 1. 1-wire #2       | Future: starter battery                         | 1-wire #2 | N/A    |                                              | 127508        |
| Internal BME280 | Temperature                                     | I2C       | N/A    | environment.inside.enginehat.temperature                 |               |
| Internal BME280 | Atmospheric Pressure                            | I2C       | N/A    | environment.outside.pressure                 | 130314        |
| Internal BME280 | Relative Humidity                               | I2C       | N/A    | environment.outside.relativeHumidity         | 130313        |
| Top - micro USB |                                | USB       | USB-port    |          |         |

Explanation of this table:
* Connector: the connectors on the front from right to left
* Sensor: the connected sensor
* PIN: the pin (of other interface) where the sensor is connected to the SH-ESP32 or Engine Hat add-on board
* Jumper: if the jumper on the Engine Hat is enabled or not
* Signalk-path: the path where the sensor will be reporting into
* NMEA2000 PGN: the NMEA2000 PGN that is supported on Raymarine devices (should be mapped by signak-to-nmea2000 plugin)
 
Credits and thanks to [Hat Labs](https://github.com/hatlabs) and [SignalK](https://signalk.org/) community for providing such high quality hardware, software and support.


