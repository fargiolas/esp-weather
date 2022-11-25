# esp-weather

A super simple ESP8266 weather station firmware.

Reads Temperature, Humidity and Pressure data from BME280 and streams them with MQTT over wifi.
Nothing too fancy, just needed a few of these to monitor humidity around the house.

## hardware

Just take a ESP8266 dev board and connect bme280 3V3, GND, SCL and SDA to their pins in the board.
Default i2c configuration assumes GPIO0 for SDA and GPIO2 for SCL but you can configure them in `make menuconfig`. 
BME280 default address is also configurable (defaults to `0x76`).

## building

1. download ESP8266_RTOS_SDK, set the proper env variables
2. clone this repo
3. `git submodule init` and `git submodule update` to get bosch driver
4. `make menuconfig`
5. `make flash`
6. `make monitor`

serial port, wifi ssid, password and mqtt broker can be configured in `make menuconfig`

## credits

BME280 driver from Bosch https://github.com/BoschSensortec/BME280_driver

ESP8266 RTOS SDK (probably works with ESP-IDF with no changes) https://github.com/espressif/ESP8266_RTOS_SDK
