FROM balenalib/armv7hf-ubuntu-python:3.8-bionic-build

# Set our working directory
WORKDIR /usr/src/app

RUN pip3 install Adafruit-Blinka adafruit-circuitpython-pm25 \
    adafruit-circuitpython-scd30 \
    adafruit-circuitpython-scd4x adafruit-circuitpython-sgp30 \
    adafruit-circuitpython-bme680 adafruit-circuitpython-bme280 \
    adafruit-circuitpython-bmp280 adafruit-circuitpython-veml7700 \
    adafruit-circuitpython-veml6070 \
    adafruit-circuitpython-lis2mdl \
    adafruit-circuitpython-ms8607 adafruit-circuitpython-htu21d \
    adafruit-circuitpython-apds9960 adafruit-circuitpython-ltr390 \
    adafruit-circuitpython-ads1x15 adafruit-circuitpython-lsm303-accel \
    adafruit-circuitpython-lis2mdl adafruit-circuitpython-lsm303dlh-mag \
    adafruit-circuitpython-sgp40 adafruit-circuitpython-lis3dh \
    adafruit-circuitpython-ahtx0 adafruit-circuitpython-mprls \
    adafruit-circuitpython-lsm303-accel \
    adafruit-circuitpython-lis2mdl \
    adafruit-circuitpython-lsm303dlh-mag \
    adafruit-circuitpython-tlv493d \
    adafruit-circuitpython-tsl2591 \
    adafruit-circuitpython-as7341 \
    adafruit-circuitpython-vcnl4040 \
    adafruit-circuitpython-ina219 \
    adafruit-circuitpython-adxl34x \
    RPi.GPIO paho-mqtt requests smbus
    
COPY *.py ./

# Enable udevd so that plugged dynamic hardware devices show up in our container.
ENV UDEV=1

#
# basic command
# CMD ["python3", "sensor.py"]
#
# sleeping for dev work
CMD ["sleep", "infinity"] 
