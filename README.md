# big-sensor
Instantly read data from 20 popular Adafruit sensors without any configuration.

This is a work in progress. Currently the project does one pass of detecting sensors and printing their measurements. In the future, HTTP/MQTT access will be added.

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
 - AHT20 Temperature & Humidity
 - MPRLS Ported Pressure
 - TLV493D Triple-Axis Magnetometer
 - TSL2591 High Dynamic Range Digital Light Sensor
 - AS7341 10-Channel Light / Color Sensor
 - VCNL4040 Proximity Sensor
 - INA219 Current Sensor
 - ADXL343 Digital MEMS Accelerometer
