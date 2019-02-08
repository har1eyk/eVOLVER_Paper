/*  Copyright (c) 2009 by Alex Leone <acleone ~AT~ gmail.com>

    This file is part of the Arduino TLC5940 Library.

    The Arduino TLC5940 Library is free software: you can redistribute it
    and/or modify it under the terms of the GNU General Public License as
    published by the Free Software Foundation, either version 3 of the
    License, or (at your option) any later version.

    The Arduino TLC5940 Library is distributed in the hope that it will be
    useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with The Arduino TLC5940 Library.  If not, see
    <http://www.gnu.org/licenses/>. */
#include "Arduino.h"
#ifndef AT_SAM_D21_H
#define AT_SAM_D21_H

/** \file
    SPI and timer pins for the ATmega168/48/88.  Don't edit these.  All
    changeable pins are defined in tlc_config.h */



/** SIN (Arduino digital pin 5) -> SIN (TLC pin 26) */
#define DEFAULT_BB_SIN_PIN      PIN_PA15
#define DEFAULT_BB_SIN_PORT     REG_PORT_OUT0 //high
#define DEFAULT_BB_SIN_DDR      REG_PORT_DIR0 //output
/** SCLK (Arduino digital pin 4) -> SCLK (TLC pin 25) */
#define DEFAULT_BB_SCLK_PIN     PIN_PA08
#define DEFAULT_BB_SCLK_PORT    REG_PORT_OUT0 //low
#define DEFAULT_BB_SCLK_DDR     REG_PORT_DIR0 //output

/** OC1A (Arduino digital pin 3) -> XLAT (TLC pin 24) */
#define XLAT_PIN     PIN_PA09
#define XLAT_PORT    REG_PORT_OUT0 //high
#define XLAT_DDR     REG_PORT_DIR0 //output

/** OC1B (Arduino analog pin 1) -> BLANK (TLC pin 23) */
#define BLANK_PIN    PIN_PA08
#define BLANK_PORT   REG_PORT_OUT1 //high
#define BLANK_DDR    REG_PORT_DIR1 //output

/** OC2B (Arduino digital pin 2) -> GSCLK (TLC pin 18) */
#define GSCLK_PIN    PIN_PA14
#define GSCLK_PORT   REG_PORT_OUT0 //high
#define GSCLK_DDR    REG_PORT_DIR0 //output

#endif
