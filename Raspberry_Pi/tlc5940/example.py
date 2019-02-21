#!/usr/bin/python3

# TLC5940 Driver Example for Python Copyright (C) 2017  Aidan Holmes
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# Animates leds connected to channels 0 - 15 on a TLC5940 chip
# Note the pin configuration to replicate on a Rasperry Pi

from tlc import tlc5940

# the numbers represent BCM convention e.g. "GPIO24" on RPi 3
# http://webofthings.org/wp-content/uploads/2016/10/pi-gpio.png
# RPi 3 GPIO 
leds = tlc5940(blankpin = 27,
               latchpin = 17,
               gsclkpin = 18,
               serialpin = 23,
               clkpin = 24)
# Mapping
# Format: param -> TCL5940 pin on 28-Pin PDIP layout
# blankpin -> 23 BLANK progpin -> 27 VPRG latchpin -> 24 XLAT gsclkpin -> 18 GSCLK serialpin -> 26 SIN clkpin -> 25 SCLK
# Format: param -> TCL5940 pin on 28-Pin PWP layout
# blankpin -> 2 BLANK progpin -> 6 VPRG latchpin -> 3 XLAT gsclkpin -> 25 GSCLK serialpin -> 5 SIN clkpin -> 4 SCLK


# +----------------+-----------+------------+-----------+
# |  TLC Pin Name  | GPIO_Pins | PWP Layout | eVOLVER   |
# +----------------+-----------+------------+-----------+
# | BLANK blankpin |        27 |          2 | 5"Uco_A0" |
# | VPRG progpin   |        22 |          6 | no use *  |
# | XLAT latchpin  |        17 |          3 | 11"Uco_D3"|
# | GSCLK gsclkpin |        18 |         25 | 9"Uco_D2" |
# | SIN serialpin  |        23 |          5 | 15"Uco_D5"|
# | SCLK clkpin    |        24 |          4 | 13"Uco_D4"|
# +----------------+-----------+------------+-----------+
# * this pin is grounded in eVOLVER. From TLC spec sheet: "When VPRG = GND, the device is in GS mode." GS = grayscale

try:
    leds.initialise()

    step = 100

    intensities_val = [0] * 16
    intensities_dir = [0] * 16
    led_index = 0
    print ("successful initilization")
    while 1:
        # Do the setting and displaying
        for led in range (0, 16):
            # leds.set_grey(led, intensities_val[led]) 
            print ("Setting: ", led, "to range: ", )

        leds.write_grey_values()
        leds.pulse_clk()

        # Update LEDs
        animating = 0
        for ledval in intensities_val:
            if ledval > 0: animating = 1
        if animating == 0:
            led_index = 0
                        
        if led_index < 16 and intensities_val[led_index] == 0:
            intensities_dir[led_index] = step
            led_index += 1

        for led in range (0, 16):
            intensities_val[led] += intensities_dir[led]
            if intensities_val[led] > 4095: intensities_dir[led] = -step
            elif intensities_val[led] < 0:
                intensities_dir[led] = 0
                intensities_val[led] = 0

        
except KeyboardInterrupt:
    print ("keyboard interruptus")
#     pass
    leds.blank(1)
    leds.cleanup() # may cause odd flickering due to default Rpi pin settings.
