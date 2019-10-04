#!/usr/bin/python3

import RPi.GPIO as GPIO
import time
# Import SPI library (for hardware SPI) and MCP3008 library.
import Adafruit_GPIO.SPI as SPI
import Adafruit_MCP3008

# Hardware SPI configuration:
SPI_PORT   = 0
SPI_DEVICE = 0
mcp = Adafruit_MCP3008.MCP3008(spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE))


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

# myArr = [
# [0, 0, 0, 0], 
# [1, 1, 0, 0], 
# [1, 0, 0, 0], 
# [0, 1, 0, 0], 
# [0, 0, 1, 0], 
# [1, 1, 1, 0], 
# [1, 0, 1, 0], 
# [0, 1, 1, 0], 
# [0, 0, 0, 1], 
# [1, 1, 0, 1], 
# [1, 0, 0, 1], 
# [0, 1, 0, 1], 
# [0, 0, 1, 1], 
# [1, 1, 1, 1], 
# [1, 0, 1, 1], 
# [0, 1, 1, 1]]

myArr = [
[0, 0, 0, 0], 
[1, 1, 0, 0], 
[1, 0, 0, 0], 
[0, 1, 0, 0], 
[0, 0, 1, 0], 
[1, 1, 1, 0]]


def readAnalog (s0, s1, s2, s3):
  # print (s0, s1, s2, s3)
  GPIO.output(5, s0) #M0
  GPIO.output(6, s1) #M1
  GPIO.output(13, s2) #M2
  GPIO.output(26, s3) #M3
  # time.sleep(0.5)
  # read M_out from MCP3008 chip
  valueTemp = mcp.read_adc(0)
  # print ("value for", s0, s1, s2, s3, ": ", valueTemp)
  # print ("Temp reading for channel: ", i, "is: ", valueTemp)
  # time.sleep(1)
  return valueTemp

try:
  # Initialise the Raspberry Pi pins
  GPIO.setwarnings(True)
  # GPIO.setup
  GPIO.setmode(GPIO.BCM)

  GPIO.setup(5, GPIO.OUT) #M0 
  GPIO.setup(6, GPIO.OUT) #M1
  GPIO.setup(13, GPIO.OUT) #M2
  GPIO.setup(26, GPIO.OUT) #M3
  
  GPIO.output(5, 0) #M0
  GPIO.output(6, 0) #M1
  GPIO.output(13, 0) #M2
  GPIO.output(26, 0) #M3

  temp = 0
  # print('| {0:>4} | {1:>4} | {2:>4} | {3:>4} | {4:>4} | {5:>4} | {6:>4} | {7:>4} | {8:>4} | {9:>4} | {10:>4} | {11:>4} | {12:>4} | {13:>4} | {14:>4} | {15:>4} |'.format(*range(16)))
  print('| {0:>4} | {1:>4} | {2:>4} | {3:>4} | {4:>4} | {5:>4} |'.format(*range(6)))
  print('-' * 113)
  while True:
    # valArray = [0]*16
    valArray = [0]*6
    i = 0
    for setting in myArr:
      # loop through array, set each GPIO to settings in array
      valArray[i]=readAnalog(setting[0], setting[1], setting[2], setting[3])
      # valArray[i]=0
      # time.sleep(0.5)
      i+=1
      # print ("i is: ", i)
    # print('| {0:>4} | {1:>4} | {2:>4} | {3:>4} | {4:>4} | {5:>4} | {6:>4} | {7:>4} | {8:>4} | {9:>4} | {10:>4} | {11:>4} | {12:>4} | {13:>4} | {14:>4} | {15:>4} |'.format(*valArray))
    print('| {0:>4} | {1:>4} | {2:>4} | {3:>4} | {4:>4} | {5:>4} |'.format(*valArray))
    time.sleep(1)

    # print ("*"*40) 

except KeyboardInterrupt:
  print ("\nkeyboard interruptus")
  GPIO.cleanup()    




# # leds = tlc5940(blankpin = 27,
# #                latchpin = 17,
# #                gsclkpin = 18,
# #                serialpin = 23,
# #                clkpin = 24)

# # try:
# #     leds.initialise()
# #     intensities_val = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

# #     while 1:
# #         for led in range (0, 16):
# #             leds.set_grey(led, intensities_val[led]) 

# #         leds.write_grey_values()
# #         leds.pulse_clk()

# #         intensities_val=[0,0,0,0,4200,0,0,0,0,0,0,0,0,0,0,0]
                
# # except KeyboardInterrupt:
# #     print ("\nkeyboard interruptus")
# #     leds.blank(1)
# #     leds.cleanup() # may cause odd flickering due to default Rpi pin settings.
# #     print ("\nGPIO set to 0")
