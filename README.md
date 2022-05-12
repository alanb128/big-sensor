# big-sensor
Instantly read data from 20 popular Adafruit I2C sensors without any configuration.

This is a work in progress, based off the lighter-weight balena [Sensor block](https://github.com/balenablocks/sensor) which has similar functionality but supports fewer sensors.

## Use

Connect one or more of the supported sensors below to your device via I2C. Add this container to your project and it should auto-detect your sensor(s) and start displaying readings every 10 seconds on the console output. The sensor data is also available via http on port 7575 of the IP address of the device.

## How it works

A Python dictionary holds all of the information about each sensor, including a reference to the sensor class and a function for reading each sensor's measurement. A loop tries to load each sensor and if successful adds a reference to the sensor in a Python list. Finally, we loop through the found sensor list and pass each sensor to its measurement reading function and print the result.

## Currently supported sensors:

 - LIS3DH Triple-Axis Accelerometer
 - APDS9960 Proximity, Light, RGB, and Gesture
 - PMSA003I Air Quality
 - SCD-30 - NDIR CO2 Temperature and Humidity
 - SCD-40 True CO2, Temperature and Humidity
 - SGP30 TVOC/eCO2 Gas Sensor
 - VEML6070 UV
 - VEML7700 Ambient Light
 - BME680 - Temperature, Humidity, Pressure and Gas
 - BME280 Humidity + Barometric Pressure + Temperature
 - BMP280 Barometric Pressure + Temperature
 - MS8607 Pressure, Temperature, and Humidity
 - HTU21D-F Temperature & Humidity
 - ADS1015 4-Channel ADC
 - ADS1115 4-Channel ADC
 - LSM303 Accelerometer + Compass
