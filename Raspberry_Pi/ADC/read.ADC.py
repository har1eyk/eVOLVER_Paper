#!/usr/bin/python3

# This reads inputs from ADC 

# the numbers represent BCM convention e.g. "GPIO24" on RPi 3
# http://webofthings.org/wp-content/uploads/2016/10/pi-gpio.png
# RPi 3 GPIO 

# +---------------+-----------+-----------------+---------------------+-------------------------------+
# | RPI 3 model B | 74HC4067  | ADC (schematic) | Arduino (schematic) | Arduino Numbering (schematic) |
# +---------------+-----------+-----------------+---------------------+-------------------------------+
# | GPIO_5        | M0        | S0              | UC1_D7              | 19                            |
# | GPIO_6        | M1        | S1              | UC1_D8              | 21                            |
# | GPIO_13       | M2        | S2              | UC1_D9              | 23                            |
# | GPIO_26       | M3        | S3              | UC1_D10             | 24                            |
# | GPIO_12       | Z         | M_OUT           | UC1_A0              | 16                            |
# | GND           | E(enable) | E(enable)       | GND                 | GND                           |
# +---------------+-----------+-----------------+---------------------+-------------------------------+


# {0, 0, 0, 0}, //channel 0; Vial 1
# {1, 1, 0, 0}, //channel 3; Vial 2
# {1, 0, 0, 0}, //channel 1; Vial 3
# {0, 1, 0, 0}, //channel 2; Vial 4
# {0, 0, 1, 0}, //channel 4; Vial 5
# {1, 1, 1, 0}, //channel 7; Vial 6
# {1, 0, 1, 0}, //channel 5; Vial 7
# {0, 1, 1, 0}, //channel 6; Vial 8
# {0, 0, 0, 1}, //channel 8; Vial 9
# {1, 1, 0, 1}, //channel 11; Vial 10
# {1, 0, 0, 1}, //channel 9; Vial 11
# {0, 1, 0, 1}, //channel 10; Vial 12
# {0, 0, 1, 1}, //channel 12; Vial 13
# {1, 1, 1, 1}, //channel 15; Vial 14
# {1, 0, 1, 1}, //channel 13; Vial 15
# {0, 1, 1, 1}, //channel 14; Vial 16

import RPi.GPIO as GPIO
import time
# Import SPI library (for hardware SPI) and MCP3008 library.
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008

# Hardware SPI configuration:
SPI_PORT   = 0
SPI_DEVICE = 0
mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))

try:
  'Initialise the Raspberry Pi pins'
  GPIO.setwarnings(True)
  #GPIO.setup
  GPIO.setmode(GPIO.BCM)

  # GPIO.setup(12, GPIO.IN) #Z, need to measure input 16  
  GPIO.setup(5, GPIO.OUT) #M0 6
  GPIO.setup(6, GPIO.OUT) #M1 13
  GPIO.setup(13, GPIO.OUT) #M2 19
  GPIO.setup(26, GPIO.OUT) #M3 26
  
  GPIO.output(5, 0) #M0
  GPIO.output(6, 0) #M1
  GPIO.output(13, 0) #M2
  GPIO.output(26, 0) #M3

  temp = 0
  while 1:
    time.sleep(.5)
    
    GPIO.output(5, 1) #M0
    GPIO.output(6, 1) #M1
    GPIO.output(13, 1) #M2
    GPIO.output(26, 0) #M3
    
    time.sleep(.5)
    
    # temp = GPIO.input(12) #measure the temp at channel 5
    # potVal = mcp.read_adc(0)
    # print ("Potentiometer is: ", potVal)
    
    # potValTwo = mcp.read_adc(1)
    # print ("2nd Potentiometer is: ", potValTwo)
    
    myTemp = mcp.read_adc(0)
    print ("Temp is: ", myTemp)
    


except KeyboardInterrupt:
  print ("\nkeyboard interruptus")
  GPIO.cleanup()    




# leds = tlc5940(blankpin = 27,
#                latchpin = 17,
#                gsclkpin = 18,
#                serialpin = 23,
#                clkpin = 24)

# try:
#     leds.initialise()
#     intensities_val = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

#     while 1:
#         for led in range (0, 16):
#             leds.set_grey(led, intensities_val[led]) 

#         leds.write_grey_values()
#         leds.pulse_clk()

#         intensities_val=[0,0,0,0,4200,0,0,0,0,0,0,0,0,0,0,0]
                
# except KeyboardInterrupt:
#     print ("\nkeyboard interruptus")
#     leds.blank(1)
#     leds.cleanup() # may cause odd flickering due to default Rpi pin settings.
#     print ("\nGPIO set to 0")
