# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

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
import adafruit_ahtx0
import adafruit_mprls
import adafruit_scd4x
import adafruit_sgp30
import adafruit_tsl2591
import adafruit_sht4x
import adafruit_ens160
import adafruit_sgp40

import time
import board
import busio
from http.server import HTTPServer, BaseHTTPRequestHandler
import paho.mqtt.client as mqtt
import socket
import threading
import requests
from smbus2 import SMBus
import os
import datetime
import json

# Used by certain sensors as offsets
temperature = None
humidity = None

#
# This section has all of the functions specific to each supported sensor to read its data
#


def sensor_pm25(sensor):
    try:
        aqdata = sensor.read()
    except RuntimeError:
        print("Unable to read from PM25 sensor, retrying...")
        time.sleep(1)
        try:
            aqdata = sensor.read()
        except RuntimeError:
            print("Unable to read from PM25 sensor, retrying (2)...")
            time.sleep(2)
            try:
                aqdata = sensor.read()
            except RuntimeError:
                print("Unable to read from PM25 sensor (3), skipping...")
                return {'pm10 standard': None, 'pm25 standard': None, 'pm100 standard': None, 'pm10 env': None, 'pm25 env': None, 'pm100 env': None, 'particles 03um': None, 'particles 05um': None, 'particles 10um': None, 'particles 25um': None, 'particles 50um': None, 'particles 100um': None}
    return aqdata

def sensor_scd30(sensor):

    global temperature, humidity
	
    i = 0
    while not sensor.data_available:
        time.sleep(0.5)
        i = i + 1
        print("Waiting for new SCD30 data... (try {})".format(i))
        if i > 3:
            break
            print("No new SCD30 data...")
            return {"CO2": None, "Temperature": None, "Humidity": None}
    temperature = sensor.temperature
    humidity = sensor.relative_humidity
	
    return {"CO2": sensor.CO2, "temperature": sensor.temperature, "humidity": sensor.relative_humidity}

def sensor_veml7700(sensor):

    return {"ambient_light": sensor.light,  "lux": sensor.lux}

def sensor_veml6070(sensor):

    uv_raw = sensor.read
    return {"uv_raw": uv_raw, "risk_level": sensor.get_index(uv_raw)}

def sensor_bme680(sensor):

    global temperature, humidity
	
    temperature = sensor.temperature
    humidity = sensor.humidity
	
    # eventually set sensor.seaLevelhPa = 1014.5 as a var
    return {"temperature": sensor.temperature, "gas": sensor.gas, "humidity": sensor.humidity, "pressure": sensor.pressure, "altitude": sensor.altitude}

def sensor_bme280(sensor):

    global temperature, humidity
	
    temperature = sensor.temperature
    humidity = sensor.humidity
	
    # eventually set sensor.seaLevelhPa = 1014.5 as a var
    return {"temperature": sensor.temperature, "humidity": sensor.humidity, "pressure": sensor.pressure, "altitude": sensor.altitude}

def sensor_bmp280(sensor):

    global temperature

    temperature = sensor.temperature

    # eventually set sensor.seaLevelhPa = 1014.5 as a var
    return {"temperature": sensor.temperature, "pressure": sensor.pressure, "altitude": sensor.altitude}

def sensor_ms8607(sensor):

    global temperature, humidity

    temperature = sensor.temperature
    humidity = sensor.relative_humidity
	
    return {"temperature": sensor.temperature, "humidity": sensor.relative_humidity, "pressure": sensor.pressure}

def sensor_htu21d(sensor):

    global temperature, humidity

    temperature = sensor.temperature
    humidity = sensor.relative_humidity
	
    return {"temperature": sensor.temperature, "humidity": sensor.relative_humidity}

def sensor_ltr390(sensor):

    return {"UV": sensor.uvs, "ambient_light": sensor.light, "UVI": sensor.uvi, "lux": sensor.lux}

def sensor_aht20(sensor):

    global temperature, humidity

    temperature = sensor.temperature
    humidity = sensor.relative_humidity
	
    return {"temperature": sensor.temperature, "humidity": sensor.relative_humidity}

def sensor_mprls(sensor):

    return {"pressure": sensor.pressure}

def sensor_scd40(sensor):
    
    global scd40_started, temperature, humidity
    
    # See https://forums.balena.io/t/rpi3-clock-stretching-using-fleet-variables/1964 to fix CRC errors
    # And https://learn.adafruit.com/circuitpython-on-raspberrypi-linux/i2c-clock-stretching

    if not scd40_started:
        sensor.start_periodic_measurement()
        scd40_started = True

    if sensor.data_ready:
        temperature = sensor.temperature
        humidity = sensor.relative_humidity
        return {"CO2": sensor.CO2, "temperature": sensor.temperature, "humidity": sensor.relative_humidity}
    else:
       return {"CO2": None, "temperature": None, "humidity": None}

def sensor_sgp30(sensor):

    # Need set/get baseline, set humidity
    return {"serial_number": [hex(i) for i in sensor.serial], "eCO2": sensor.eCO2, "TVOC": sensor.TVOC}

def sensor_tsl2591(sensor):

    # TODO
    # Optionally change gain and integration time
    # Use device variables
    return {"total_light": sensor.lux, "infrared": sensor.infrared, "visible": sensor.visible}

def sensor_ens160(sensor):

    # Set the temperature compensation variable to the ambient temp
    # for best sensor calibration
    if temperature is not None:
        sensor.temperature_compensation = temperature
        print("Using ENS160 temperature compensation.")
    # Same for ambient relative humidity
    if pressure is not None:
        sensor.humidity_compensation = humidity
        print("Using ENS160 humidity compensation.")

    return {"AQI": sensor.AQI, "TVOC": sensor.TVOC, "eCO2": sensor.eCO2 }

def sensor_sgp40(sensor):

    if (temperature is not None) and (humidity is not None):
        print("Using SGP40 temperature and humidity compensation.")
        return {"raw_gas": sensor.measure_raw(temperature = temperature, relative_humidity = humidity)}
    else:
        return {"raw_gas": sensor.raw }

def sensor_sht4x(sensor):

    global temperature, humidity
	
    #TODO:
    sensor.mode = adafruit_sht4x.Mode.NOHEAT_HIGHPRECISION
    # Can also set the mode to enable heater - make a device variable
    # sht.mode = adafruit_sht4x.Mode.LOWHEAT_100MS
    temperature = sensor.temperature
    humidity = sensor.relative_humidity
    return {"temperature": sensor.temperature, "humidity": sensor.relative_humidity, "serial_number": hex(sensor.serial_number), "mode": adafruit_sht4x.Mode.string[sensor.mode] }


#
# End of sensor classes
#

def mqtt_detect():
    
    # Use the supervisor api to get services
    # See https://www.balena.io/docs/reference/supervisor/supervisor-api/
    
    address = os.getenv('BALENA_SUPERVISOR_ADDRESS', '')
    api_key = os.getenv('BALENA_SUPERVISOR_API_KEY', '')
    app_name = os.getenv('BALENA_APP_NAME', '')

    url = "{0}/v2/applications/state?apikey={1}".format(address, api_key)

    try:
        r = requests.get(url).json()
    except Exception as e:
        print("Error looking for MQTT service: {0}".format(str(e)))
        return False
    else:
        services = r[app_name]['services'].keys()

        if "mqtt" in services:
            return True
        else:
            return False
 
 
# Simple webserver:

def background_web(server_socket):
    while True:
        # Wait for client connections
        client_connection, client_address = server_socket.accept()

        # Get the client request
        request = client_connection.recv(1024).decode()
        print(request)

        # Send HTTP response
        response = 'HTTP/1.0 200 OK\n\n' + json.dumps(get_reading())
        client_connection.sendall(response.encode())
        client_connection.close()


def read_chip_id(bus, device, loc):
    #print("{0} - {1} - {2}".format(bus, device, loc))
    chip_id = -1
    time.sleep(0.2)
    try:
        chip_id = bus.read_byte_data(device, loc)
    except Exception as e:
        # Likely because device isn't present, a remote i/o error, which in this case we can ignore
        pass

    return chip_id

def get_reading():
    i = 0
    return_dict = {}
    for sensor in sensor_list:
    #print(sensor)
        if sensor is not None:
            return_dict[sensor_dict[i]['short']] = sensor_dict[i]['func'](sensor)
        #print("{0}: {1}".format(sensor_dict[i]['short'], sensor_dict[i]['func'](sensor)))
        i = i + 1

    return return_dict

###############################
# Program starts here
###############################


scd40_started = False # Flag for this sensor only because it needs a startup call

print_readings = os.getenv('PRINT_READINGS', True)
mqtt_address = os.getenv('MQTT_ADDRESS', 'none')
use_httpserver = os.getenv('ALWAYS_USE_HTTPSERVER', 0)
try:
    interval = os.getenv('MQTT_PUB_INTERVAL', '8')
except Exception as e:
    print("Error converting MQTT_PUB_INTERVAL: Must be integer or float! Using default.")
    interval = 8
publish_topic = os.getenv('MQTT_PUB_TOPIC', 'sensors')
try:
    bus_number = int(os.getenv('BUS_NUMBER', '1'))  # default 1 indicates /dev/i2c-1
except Exception as e:
    print("Error in BUS_NUMBER: Must be integer! Using default /dev/i2c-1.")
    bus_number = 1
bus = SMBus(bus_number)
time.sleep(1)  # wait here to avoid 121 IO Error 

if use_httpserver == "1":
    enable_httpserver = "True"
else:
    enable_httpserver = "False"

if mqtt_detect() and mqtt_address == "none":
        mqtt_address = "mqtt"

if mqtt_address != "none":
    print("Starting mqtt client, publishing to {0}:1883".format(mqtt_address))
    print("Using MQTT publish interval: {0} sec(s)".format(interval))
    client = mqtt.Client()
    try:
        client.connect(mqtt_address, 1883, 60)
    except Exception as e:
        print("Error connecting to mqtt. ({0})".format(str(e)))
        mqtt_address = "none"
        enable_httpserver = "True"
    else:
        client.loop_start()
else:
    enable_httpserver = "True"

# See https://www.codementor.io/@joaojonesventura/building-a-basic-http-server-from-scratch-in-python-1cedkg0842
if enable_httpserver == "True":
    SERVER_HOST = '0.0.0.0'
    SERVER_PORT = 7575

    # Create socket
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((SERVER_HOST, SERVER_PORT))
    server_socket.listen(1)
    print("HTTP server listening on port {0}...".format(SERVER_PORT))

    t = threading.Thread(target=background_web, args=(server_socket,))
    t.start()

            
# Dictionary of all information about a supported sensor, add a row for each sensor's supported I2C address
sensor_dict = {
    0: {'name': 'PMSA003I Air Quality', 'short': 'PM25', 'func': sensor_pm25, 'class_ref': PM25_I2C, 'arrgh': {'reset_pin': None}, 'i2c_addr': 0x12, 'chip_id': 0x1c, 'chip_value': 0x42},
    1: {'name': 'SCD-30 - NDIR CO2 Temperature and Humidity', 'short': 'SCD30', 'func': sensor_scd30, 'class_ref': adafruit_scd30.SCD30, 'arrgh': {}, 'i2c_addr': 0x61, 'chip_id': 0x00, 'chip_value': 0x00},
    2: {'name': 'SCD-40 True CO2, Temperature and Humidity', 'short': 'SCD40', 'func': sensor_scd40, 'class_ref': adafruit_scd4x.SCD4X,'arrgh': {}, 'i2c_addr': 0x62, 'chip_id': 0x00, 'chip_value': 0x00},
    3: {'name': 'SGP30 TVOC/eCO2 Gas Sensor', 'short': 'SGP30', 'func': sensor_sgp30, 'class_ref': adafruit_sgp30.Adafruit_SGP30,'arrgh': {}, 'i2c_addr': 0x58, 'chip_id': 0x00, 'chip_value': 0x00},
    4: {'name': 'VEML6070 UV', 'short': 'VEML6070', 'func': sensor_veml6070, 'class_ref': adafruit_veml6070.VEML6070,'arrgh': {},  'i2c_addr': 0x38, 'chip_id': 0x00, 'chip_value': 0x00}, # and 0x39
    5: {'name': 'VEML7700 Ambient Light', 'short': 'VEML7700', 'func': sensor_veml7700, 'class_ref': adafruit_veml7700.VEML7700,'arrgh': {},  'i2c_addr': 0x10, 'chip_id': 0x00, 'chip_value': 0x00},
    6: {'name': 'BME680 - Temperature, Humidity, Pressure and Gas', 'short': 'BME680-0x76', 'func': sensor_bme680, 'class_ref': adafruit_bme680.Adafruit_BME680_I2C, 'arrgh': {'address': 0x76}, 'i2c_addr': 0x76, 'chip_id': 0xd0, 'chip_value': 0x61},
    7: {'name': 'BME680 - Temperature, Humidity, Pressure and Gas', 'short': 'BME680-0x77', 'func': sensor_bme680, 'class_ref': adafruit_bme680.Adafruit_BME680_I2C, 'arrgh': {'address': 0x77}, 'i2c_addr': 0x77, 'chip_id': 0xd0, 'chip_value': 0x61},
    8: {'name': 'BME280 Humidity + Barometric Pressure + Temperature', 'short': 'BME280-0x76', 'func': sensor_bme280, 'class_ref': adafruit_bme280.Adafruit_BME280_I2C,'arrgh': {'address': 0x76},  'i2c_addr': 0x76, 'chip_id': 0xd0, 'chip_value': 0x60},
    8: {'name': 'BME280 Humidity + Barometric Pressure + Temperature', 'short': 'BME280-0x77', 'func': sensor_bme280, 'class_ref': adafruit_bme280.Adafruit_BME280_I2C,'arrgh': {'address': 0x77},  'i2c_addr': 0x77, 'chip_id': 0xd0, 'chip_value': 0x60},
    9: {'name': 'BMP280 Barometric Pressure + Temperature', 'short': 'BMP280-0x76', 'func': sensor_bmp280, 'class_ref': adafruit_bmp280.Adafruit_BMP280_I2C, 'arrgh': {'address': 0x76}, 'i2c_addr': 0x76, 'chip_id': 0xd0, 'chip_value': 0x58},
    9: {'name': 'BMP280 Barometric Pressure + Temperature', 'short': 'BMP280-0x77', 'func': sensor_bmp280, 'class_ref': adafruit_bmp280.Adafruit_BMP280_I2C, 'arrgh': {'address': 0x77}, 'i2c_addr': 0x77, 'chip_id': 0xd0, 'chip_value': 0x58},
    10: {'name': 'MS8607 Pressure, Temperature, and Humidity', 'short': 'MS8607', 'func': sensor_ms8607, 'class_ref': MS8607, 'arrgh': {}, 'i2c_addr': 0x76, 'chip_id': 0x00, 'chip_value': 0x08}, # also uses 0x40
    11: {'name': 'HTU21D-F Temperature & Humidity', 'short': 'HTU21D', 'func': sensor_htu21d, 'class_ref': HTU21D, 'arrgh': {}, 'i2c_addr': 0x40, 'chip_id': 0x00, 'chip_value': 0x00},
    12: {'name': 'LTR390 UV', 'short': 'LTR390', 'func': sensor_ltr390, 'class_ref': adafruit_ltr390.LTR390, 'arrgh': {}, 'i2c_addr': 0x53, 'chip_id': 0x06, 'chip_value': 0xb2}, # need to confirm ID
    13: {'name': 'AHT20 Temperature & Humidity', 'short': 'AHT20', 'func': sensor_aht20, 'class_ref': adafruit_ahtx0.AHTx0, 'arrgh': {}, 'i2c_addr': 0x38, 'chip_id': 0x00, 'chip_value': 0x00},
    14: {'name': 'MPRLS Ported Pressure', 'short': 'MPRLS', 'func': sensor_mprls, 'class_ref': adafruit_mprls.MPRLS, 'arrgh': {'psi_min': 0, 'psi_max':25}, 'i2c_addr': 0x18, 'chip_id': 0x00, 'chip_value': 0x00},
    15: {'name': 'TSL2591 High Dynamic Range Digital Light Sensor', 'short': 'TSL2591', 'func': sensor_tsl2591, 'class_ref': adafruit_tsl2591.TSL2591, 'arrgh': {}, 'i2c_addr': 0x29, 'chip_id': 0x00, 'chip_value': 0x00}, # Also uses 0x28
    16: {'name': 'SGP40 VOC Index', 'short': 'SGP40', 'func': sensor_sgp40, 'class_ref': adafruit_sgp40.SGP40, 'arrgh': {}, 'i2c_addr': 0x59, 'chip_id': 0x00, 'chip_value': 0x00},
    17: {'name': 'ENS160 MOX Gas Sensor', 'short': 'ENS160-0x52', 'func': sensor_ens160, 'class_ref': adafruit_ens160.ENS160, 'arrgh': {'address': 0x52}, 'i2c_addr': 0x52, 'chip_id': 0x00, 'chip_value': 0x00},
    17: {'name': 'ENS160 MOX Gas Sensor', 'short': 'ENS160-0x53', 'func': sensor_ens160, 'class_ref': adafruit_ens160.ENS160, 'arrgh': {'address': 0x53}, 'i2c_addr': 0x53, 'chip_id': 0x00, 'chip_value': 0x00}, #Has Part ID at 0x00
    18: {'name': 'SHT4X Temperature & Humidity Sensor', 'short': 'SHT4X', 'func': sensor_sht4x, 'class_ref': adafruit_sht4x.SHT4x, 'arrgh': {}, 'i2c_addr': 0x44, 'chip_id': 0x00, 'chip_value': 0x00}
}
# List of all sensors in same order as dict, with None for sensors not found
sensor_list = []

#i2c = board.I2C()  # uses board.SCL and board.SDA
i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
i = 0

# See which sensors we have attached and update sensor_list
for sensor_id, sensor_info in sensor_dict.items():

    skip_sensor = False
    
    # If we already found a sensor at this address, don't try finding another at the same address
    # This means that the sensors earlier in the dict have priority...
    # Also better to have ones with a chip ID earlier in dict
    j = 0
    for sensor in sensor_list:
        if sensor is not None:
            test_i2c =  sensor_dict[j]['i2c_addr']
            if test_i2c == sensor_info['i2c_addr']:
                skip_sensor = True
                print("Skipping sensor {} because the i2c address is already in use by another sensor.".format(sensor_info['short']))
        j = j + 1 
        
    # Next read the chip_id if the sensor supports it...
    if not skip_sensor:
        if sensor_info['chip_id'] != 0x00:
            read_chip = read_chip_id(bus, sensor_info['i2c_addr'], sensor_info['chip_id'])
            if read_chip != sensor_info['chip_value'] and read_chip != -1:
                skip_sensor = True
                print("Skipping sensor {0} because chip ID {1} does not match {2}.".format(sensor_info['short'], hex(sensor_info['chip_value']), hex(read_chip)))
            
                
    if not skip_sensor:           
        try:
            print("testing: {}".format(sensor_info['short']))
            # See if sensor is loaded by trying to instantiate it...
            # Using arg list (arrgh) from sensor_info if additional parameters are required
            # ** is for unpacking kwargs, see https://www.educative.io/answers/what-is-unpacking-keyword-arguments-with-dictionaries-in-python
            test_sensor = sensor_info['class_ref'](i2c, **(sensor_info['arrgh']))
        except Exception as e:
            print(e)
            print("{} not found.".format(sensor_info['short']))
            sensor_list.append(None)  # keep list in sync with dict
        else:
            print("Found a {} sensor!".format(sensor_info['name']))

            try:
                test_sensor
            except NameError:
                test_sensor = None
            #else:
            #    sensor_info['found'] = True
            sensor_list.append(test_sensor)
    else:
        sensor_list.append(None)  # keep list in sync with dict

    i = i + 1

while True:

    if print_readings:
	    print("{}:".format(datetime.datetime.now()))
        print(get_reading())
        print("------------------------------")
    if mqtt_address != "none":
        client.publish(publish_topic, json.dumps(get_reading()))
        print("Published MQTT.")

    time.sleep(interval)

