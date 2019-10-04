#!/usr/bin/python3

# This tests the tlc5940 chip by activating 1 or all of the 16 channels


from tlc import tlc5940
import time
# the numbers represent BCM convention e.g. "GPIO24" on RPi 3
# http://webofthings.org/wp-content/uploads/2016/10/pi-gpio.png
# RPi 3 GPIO 

# +----------------+-----------+------------+-----------+
# |  TLC Pin Name  | GPIO_Pins | PWP Layout | eVOLVER   |
# +----------------+-----------+------------+-----------+
# | BLANK blankpin |        27 |          2 | 5"Uco_A1" |
# | VPRG progpin   |        22 |          6 | no use *  |
# | XLAT latchpin  |        17 |          3 | 11"Uco_D3"|
# | GSCLK gsclkpin |        18 |         25 | 9"Uco_D2" |
# | SIN serialpin  |        23 |          5 | 15"Uco_D5"|
# | SCLK clkpin    |        24 |          4 | 13"Uco_D4"|
# +----------------+-----------+------------+-----------+
# * this pin is grounded in eVOLVER. From TLC spec sheet: "When VPRG = GND, the device is in GS mode." GS = grayscale
# see tlc5940 programming spec sheet: http://bit.ly/2Irodyz

leds = tlc5940(blankpin = 27,
               latchpin = 17,
               gsclkpin = 18,
               serialpin = 23,
               clkpin = 24)

try:
    leds.initialise()
    intensities_val = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]

    while 1:
        for led in range (0, 16):
            leds.set_grey(led, intensities_val[led]) 

        leds.write_grey_values()
        leds.pulse_clk()

        # intensities_val=[3500,3500,3500,0,0,0,0,0,0,0,0,0,0,0,0,0]
        intensities_val=[1200,1200,1200,1200,1200,1200,1200,1200,1200,1200,1200,1200,1200,1200,1200,1200]
        # intensities_val=[100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100]
                
except KeyboardInterrupt:
    print ("\nkeyboard interruptus")
    leds.blank(1)
    leds.cleanup() # may cause odd flickering due to default Rpi pin settings.
    print ("\nGPIO set to 0")
