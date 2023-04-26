# big-sensor
Instantly read data from over 15 popular Adafruit I2C environmental sensors without any configuration.

This is based off the lighter-weight balena [Sensor block](https://github.com/balenablocks/sensor) which has similar functionality but supports fewer sensors.

## Use

Connect one or more of the supported sensors below to your device via I2C. Add this container to your project and it should auto-detect your sensor(s) and start displaying readings every 10 seconds on the console output. The sensor data is also available via http on port 7575 of the IP address of the device or via MQTT.

If there is another service on the device named `mqtt` the sensor readings will be published to that broker. Otherwise, you can specify an MQTT broker address with the environment variable `MQTT_ADDRESS`. If MQTT is active, the http server will be disabled. To enable the http server on port 7575 anyway, set `ALWAYS_USE_HTTPSERVER` to `1`. 

You can specify the MQTT topic to be published with `MQTT_PUB_TOPIC` otherwise the default topic is `sensors`. The `MQTT_PUB_INTERVAL` sets the number of seconds between packet publication. The default value is 8 seconds. This interval also controls how often the sensor data is printed to the standard output. To disable this printing, set `VERBOSE` to `0`. (This also eliminates other repetitive non-error printing.)

To add the block to your docker-compose file, see this example:

```
version: '2.1'

services:
  connector:
    image: bh.cr/al_s_apps/big-sensor-armv7hf
    restart: always
    labels:
      io.balena.features.supervisor-api: 1  # necessary to discover services
    privileged: true # necessary to read sensors

```


## Currently supported sensors:

 - PMSA003I Air Quality
 - SCD-30 - NDIR CO2 Temperature and Humidity
 - SCD-40 True CO2, Temperature and Humidity
 - SGP30 TVOC/eCO2 Gas Sensor
 - VEML6070 UV Sensor
 - VEML7700 Ambient Light
 - BME688 - Temperature, Humidity, Pressure and Gas
 - BME680 - Temperature, Humidity, Pressure and Gas
 - BME280 Humidity + Barometric Pressure + Temperature
 - BMP280 Barometric Pressure + Temperature
 - MS8607 Pressure, Temperature, and Humidity
 - HTU21D-F Temperature & Humidity
 - AHT20 - Temperature & Humidity 
 - LTR390 UV Light Sensor
 - MPRLS Ported Pressure
 - TSL2591 High Dynamic Range Digital Light Sensor
 - SGP40 VOC Index
 - Sensirion SHT45 Precision Temperature & Humidity Sensor
 - Sensirion SHT40 Temperature & Humidity Sensor
 - ENS160 MOX Gas Sensor
 
## Options 

The sensors are read by default from bus number 1 (`/dev/i2c-1`). To change the bus number, use the variable `BUS_NUMBER`.
 
## How it works

A Python dictionary holds all of the information about each sensor, including a reference to the sensor class and a function for reading each sensor's measurement. A loop tries to load each sensor and if successful adds a reference to the sensor in a Python list. Finally, we loop through the found sensor list and pass each sensor to its measurement reading function and print the result.

