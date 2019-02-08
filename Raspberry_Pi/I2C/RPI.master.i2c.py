#!/usr/bin/python3

from smbus2 import SMBus
import time

# This is the address we setup in the Arduino Program
address = 0x04
# Open i2c bus 1 
bus = SMBus(1)

def writeNumber(value):
	# Write a byte to address, offset 0
	bus.write_byte_data(address, 0, value)
	return

def readNumber():
	#read one byte from address 80, offset 0
	number = bus.read_byte_data(address, 0)
	# number = bus.read_byte_data(address, 1)
	return number

while True:
	var = int(input("Enter 1 to 9: "))
	if not var:
		continue
	writeNumber(var)
	print ("RPI: Hi Arduino, I sent you ", var)
	# sleep one second
	time.sleep(1)
	number = readNumber()
	print ("Arduino: Hey RPI, I received a digit ", number)
