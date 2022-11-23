# esp-weather

A super simple ESP8266 weather station firmware.

Reads Temperature, Humidity and Pressure data from BME280 and streams them with MQTT over wifi.
Nothing too fancy, just needed a few of these to monitor humidity around the house.

## hardware

just take an ESP8266 dev board and connect bme280 SCL to GPIO2 and SDA to GPIO0

## building

1. download ESP8266_RTOS_SDK, set the proper env variables
2. `make menuconfig`
3. `make flash`
4. `make monitor`

serial port, wifi ssid, password and mqtt broker can be configured in `make menuconfig`

## credits

BME280 driver from Bosch https://github.com/BoschSensortec/BME280_driver

ESP8266 RTOS SDK (probably works with ESP-IDF with no changes) https://github.com/espressif/ESP8266_RTOS_SDK
