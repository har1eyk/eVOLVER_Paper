#!/usr/bin/python3

import time
from w1thermsensor import W1ThermSensor
sensor = W1ThermSensor()

while True:
	for sensor in sensor.get_available_sensors():
	    print("Sensor %s has temperature %.2f" % (sensor.id, sensor.get_temperature()))
	time.sleep(2)

# while True:
#     temperature = sensor.get_temperature()
#     print("The temperature is %s celsius" % temperature)
#     time.sleep(2)