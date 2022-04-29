# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

# This version uses disct + list and works well

import time
import board
import busio
import adafruit_lis3dh
from adafruit_apds9960.apds9960 import APDS9960
from adafruit_pm25.i2c import PM25_I2C
import adafruit_scd30
import adafruit_veml7700
import adafruit_veml6070
import adafruit_bme680
from adafruit_bme280 import basic as adafruit_bme280
import adafruit_bmp280
from adafruit_ms8607 import MS8607
from adafruit_htu21d import HTU21D
import adafruit_ltr390
import adafruit_ads1x15.ads1015 as ADS_1015
import adafruit_ads1x15.ads1115 as ADS_1115
from adafruit_ads1x15.analog_in import AnalogIn
import adafruit_lsm303_accel
import adafruit_lis2mdl
import adafruit_lsm303dlh_mag
import adafruit_ahtx0
import adafruit_mprls
import adafruit_scd4x
import adafruit_sgp30

#
# This section has all of the functions specific to each supported sensor to read its data
#

def sensor_lis3dh(sensor):
    # Set range of accelerometer (can be RANGE_2_G, RANGE_4_G, RANGE_8_G or RANGE_16_G).
    sensor.range = adafruit_lis3dh.RANGE_2_G
    # Read accelerometer values (in m / s ^ 2).  Returns a 3-tuple of x, y,
    # z axis values.  Divide them by 9.806 to convert to Gs.
    return { "x": sensor.acceleration[0], "y": sensor.acceleration[1], "z": sensor.acceleration[2] }

def sensor_apds(sensory):
    # eventually make these selectable with device vars
    sensory.enable_proximity = True
    sensory.enable_color = True
    sensory.enable_gesture = True

    return { "proximity": sensor.proximity, "r": sensory.color_data[0],  "g": sensory.color_data[1], "b": sensory.color_data[2], "c": sensory.color_data[3], "gesture": sensory.gesture() }

def sensor_pm25(sensor):
    try:
        aqdata = sensor.read()
    except RuntimeError:
        logger.warning("Unable to read from PM25 sensor, retrying...")
        time.sleep(1)
        try:
            aqdata = sensor.read()
        except RuntimeError:
            logger.warning("Unable to read from PM25 sensor, retrying (2)...")
            time.sleep(2)
            try:
                aqdata = sensor.read()
            except RuntimeError:
                logger.warning("Unable to read from PM25 sensor (3), skipping...")
                return {}
    return aqdata

def sensor_scd30(sensor):
    i = 0
    while not sensor.data_available:
        time.sleep(0.5)
        print("Waiting for new SCD30 data...")
        i = i + 1
        if i > 3:
            break
            return {}

    return {"CO2": sensor.CO2, "temperature": sensor.temperature, "humidity": sensor.relative_humidity}

def sensor_veml7700(sensor):

    return {"ambient_light": sensor.light,  "lux": sensor.lux}

def sensor_veml6070(sensor):

    uv_raw = sensor.read
    return {"uv_raw": uv_raw, "risk_level": sensor.get_index(uv_raw)}

def sensor_bme680(sensor):

    # eventually set sensor.seaLevelhPa = 1014.5 as a var
    return {"Temperature": sensor.temperature, "Gas": sensor.gas, "Humidity": sensor.humidity, "Pressure": sensor.pressure, "Altitude": sensor.altitude}

def sensor_bme280(sensor):

    # eventually set sensor.seaLevelhPa = 1014.5 as a var
    return {"Temperature": sensor.temperature, "Humidity": sensor.humidity, "Pressure": sensor.pressure, "Altitude": sensor.altitude}

def sensor_bmp280(sensor):

    # eventually set sensor.seaLevelhPa = 1014.5 as a var
    return {"Temperature": sensor.temperature, "Pressure": sensor.pressure, "Altitude": sensor.altitude}

def sensor_ms8607(sensor):

    return {"Temperature": sensor.temperature, "Humidity": sensor.relative_humidity, "Pressure": sensor.pressure}

def sensor_htu21d(sensor):

    return {"Temperature": sensor.temperature, "Humidity": sensor.relative_humidity}

def sensor_ltr390(sensor):

    return {"UV": sensor.uvs, "Ambient_Light": sensor.light, "UVI": sensor.uvi, "Lux": sensor.lux}

def sensor_ads1015(sensor):

    # Single mode (default) (not to be confused with single-ended mode) always waits until the analog to digital conversion is completed by the ADC 
    # Block only supports single-ended mode for now. Maybe differential in the future
    # eventually add gain var ie ads.gain = 16

    chan0 = AnalogIn(sensor, ADS.P0)
    chan1 = AnalogIn(sensor, ADS.P1)
    chan2 = AnalogIn(sensor, ADS.P2)
    chan1 = AnalogIn(sensor, ADS.P3)

    return {"chan0_value": chan0.value, "chan0_voltage": chan0.voltage, "chan1_value": chan1.value, "chan1_voltage": chan1.voltage,
            "chan2_value": chan2.value, "chan2_voltage": chan2.voltage, "chan3_value": chan3.value, "chan3_voltage": chan3.voltage}

def sensor_lsm303(sensor):

    # Need to choose LSM303AGR or LSM303DLH
    # No tap detection
    return {"Acceleration": sensor.acceleration, "Magnetometer": sensor.magnetic}

def sensor_aht20(sensor):

    return {"Temperature": sensor.temperature, "Humidity": sensor.relative_humidity}

def sensor_mprls(sensor):

    return {"pressure": sensor.pressure}

def sensor_scd40(sensor):

    if sensor.data_ready:
        return {"serial_number": [hex(i) for i in sensor.serial_number], "CO2": sensor.CO2, 
                "Temperature": sensor.temperature, "Humidity": sensor.relative_humidity}
    else:
        return {"serial_number": [hex(i) for i in sensor.serial_number]}

def sensor_sgp30(sensor):

    # Need set/get baseline, set humidity
    return {"serial_number": [hex(i) for i in sensor.serial], "eCO2": sensor.eCO2, "TVOC": sensor.TVOC}

def sensor_ads1015(sensor):

    return {"x": 0 }

def sensor_ads1115(sensor):

    return {"x": 0 }

#
# Program starts here
#

# Dictionary of all information about a supported sensor
sensor_dict = {
    0: {'name': 'LIS3DH Triple-Axis Accelerometer', 'short': 'LIS3DH', 'func': sensor_lis3dh, 'class_ref': adafruit_lis3dh.LIS3DH_I2C, 'i2c_addr': 0x18, 'chip_id': 0x0f, 'chip_value': 0x33},
    1: {'name': 'APDS9960 Proximity, Light, RGB, and Gesture', 'short': 'APDS9960', 'func': sensor_apds, 'class_ref': APDS9960, 'i2c_addr': 0x39, 'chip_id': 0x92, 'chip_value': 0xab},
    2: {'name': 'PMSA003I Air Quality', 'short': 'PM25', 'func': sensor_pm25, 'class_ref': PM25_I2C, 'i2c_addr': 0x12, 'chip_id': 0x1c, 'chip_value': 0x42},
    3: {'name': 'SCD-30 - NDIR CO2 Temperature and Humidity', 'short': 'SCD30', 'func': sensor_scd30, 'class_ref': adafruit_scd30.SCD30, 'i2c_addr': 0x61, 'chip_id': 0x00, 'chip_value': 0x00},
    4: {'name': 'SCD-40 True CO2, Temperature and Humidity', 'short': 'SCD40', 'func': sensor_scd40, 'class_ref': adafruit_scd4x.SCD4X, 'i2c_addr': 0x62, 'chip_id': 0x00, 'chip_value': 0x00},
    5: {'name': 'SGP30 TVOC/eCO2 Gas Sensor', 'short': 'SGP30', 'func': sensor_sgp30, 'class_ref': adafruit_sgp30.Adafruit_SGP30, 'i2c_addr': 0x58, 'chip_id': 0x00, 'chip_value': 0x00},
    6: {'name': 'VEML6070 UV', 'short': 'VEML6070', 'func': sensor_veml6070, 'class_ref': adafruit_veml6070.VEML6070, 'i2c_addr': 0x71, 'chip_id': 0x00, 'chip_value': 0x00},
    7: {'name': 'VEML7700 Ambient Light', 'short': 'VEML7700', 'func': sensor_veml7700, 'class_ref': adafruit_veml7700.VEML7700, 'i2c_addr': 0x10, 'chip_id': 0x00, 'chip_value': 0x00},
    8: {'name': 'BME680 - Temperature, Humidity, Pressure and Gas', 'short': 'BME680', 'func': sensor_bme680, 'class_ref': adafruit_bme680.Adafruit_BME680_I2C, 'i2c_addr': 0x77, 'chip_id': 0xd0, 'chip_value': 0x61},
    9: {'name': 'BME280 Humidity + Barometric Pressure + Temperature', 'short': 'BME280', 'func': sensor_bme280, 'class_ref': adafruit_bme280.Adafruit_BME280_I2C, 'i2c_addr': 0x77, 'chip_id': 0xd0, 'chip_value': 0x60},
    10: {'name': 'BMP280 Barometric Pressure + Temperature', 'short': 'BMP280', 'func': sensor_bmp280, 'class_ref': adafruit_bmp280.Adafruit_BMP280_I2C, 'i2c_addr': 0x77, 'chip_id': 0xd0, 'chip_value': 0x58},
    11: {'name': 'MS8607 Pressure, Temperature, and Humidity', 'short': 'MS8607', 'func': sensor_ms8607, 'class_ref': MS8607, 'i2c_addr': 0x76, 'chip_id': 0x00, 'chip_value': 0x08}, # also 0x40
    12: {'name': 'HTU21D-F Temperature & Humidity', 'short': 'HTU21D', 'func': sensor_htu21d, 'class_ref': HTU21D, 'i2c_addr': 0x40, 'chip_id': 0x00, 'chip_value': 0x00},
    13: {'name': 'LTR390 UV', 'short': 'LTR390', 'func': sensor_ltr390, 'class_ref': adafruit_ltr390.LTR390, 'i2c_addr': 0x53, 'chip_id': 0x06, 'chip_value': 0xb2}, # need to confirm ID
    14: {'name': 'ADS1015 4-Channel ADC', 'short': 'ADS1015', 'func': sensor_ads1015, 'class_ref': ADS_1015.ADS1015, 'i2c_addr': 0x48, 'chip_id': 0x00, 'chip_value': 0x00},
    15: {'name': 'ADS1115 4-Channel ADC', 'short': 'ADS1115', 'func': sensor_ads1115, 'class_ref': ADS_1115.ADS1115, 'i2c_addr': 0x48, 'chip_id': 0x00, 'chip_value': 0x00},
    16: {'name': 'LSM303 Accelerometer + Compass', 'short': 'LSM303', 'func': sensor_lsm303, 'class_ref': adafruit_lsm303_accel.LSM303_Accel, 'i2c_addr': 0x48, 'chip_id': 0x0f, 'chip_value': 0x33}, # and 2 different magnetometers
    17: {'name': 'AHT20 Temperature & Humidity', 'short': 'AHT20', 'func': sensor_aht20, 'class_ref': adafruit_ahtx0.AHTx0, 'i2c_addr': 0x38, 'chip_id': 0x00, 'chip_value': 0x00},
    18: {'name': 'MPRLS Ported Pressure', 'short': 'MPRLS', 'func': sensor_mprls, 'class_ref': adafruit_mprls.MPRLS, 'i2c_addr': 0x18, 'chip_id': 0x00, 'chip_value': 0x00}
}
# List of all sensors in same order as dict, with None for sensors not found
sensor_list = []

i2c = board.I2C()  # uses board.SCL and board.SDA
i = 0

# See which sensors we have attached and update 'found' value in dict
for sensor_id, sensor_info in sensor_dict.items():

    try:
        test_sensor = sensor_info['class_ref'](i2c)
    except Exception as e:
        print("{} not found.".format(sensor_info['short']))
        sensor_list.append(None)
    else:
        print("Found a {} sensor!".format(sensor_info['name']))

        try:
            test_sensor
        except NameError:
            test_sensor = None
        #else:
        #    sensor_info['found'] = True
        sensor_list.append(test_sensor)
    i = i + 1

# Loop through sensors and print values of attached ones using their specific function
i = 0
#print(sensor_list)
#print(sensor_dict)
print(" ")
for sensor in sensor_list:
    #print(sensor)
    if sensor is not None:
        print("{0}: {1}".format(sensor_dict[i]['short'], sensor_dict[i]['func'](sensor)))
    i = i + 1
