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

/** \file
    Tlc5940 class functions. */
#if defined (__SAMD21G18A__)
#include <sam.h>
//#include <interrupt_sam_nvic.h>
#else
#include <avr/io.h>
#include <avr/interrupt.h>
#endif

#include "tlc_config.h"
#include "Tlc5940_SAMD.h"

/** Pulses a pin - high then low. */
#define pulse_pin(port, pin)   port |= (1 << pin); port &=~ (1 << pin)

/** This will be true (!= 0) if update was just called and the data has not
    been latched in yet. */
volatile uint8_t tlc_needXLAT;

/** Some of the extened library will need to be called after a successful
    update. */
volatile void (*tlc_onUpdateFinished)(void);

/** Packed grayscale data, 24 bytes (16 * 12 bits) per TLC.

    Format: Lets assume we have 2 TLCs, A and B, daisy-chained with the SOUT of
    A going into the SIN of B.
    - byte 0: upper 8 bits of B.15
    - byte 1: lower 4 bits of B.15 and upper 4 bits of B.14
    - byte 2: lower 8 bits of B.0
    - ...
    - byte 24: upper 8 bits of A.15
    - byte 25: lower 4 bits of A.15 and upper 4 bits of A.14
    - ...
    - byte 47: lower 8 bits of A.0

    \note Normally packing data like this is bad practice.  But in this
    situation, shifting the data out is really fast because the format of
    the array is the same as the format of the TLC's serial interface. */
uint8_t tlc_GSData[NUM_TLCS * 24];

/** Don't add an extra SCLK pulse after switching from dot-correction mode. */
static uint8_t firstGSInput;

/** Interrupt called after an XLAT pulse to prevent more XLAT pulses. */
void TCC1_Handler()
{
  if ((TCC1->INTFLAG.bit.MC0 && TCC1->INTENSET.bit.MC0 )){
    //pulse_pin(BLANK_PORT, BLANK_PIN);
    REG_TCC1_INTFLAG = TCC_INTFLAG_MC0;
    BLANK_PORT |= (1 << BLANK_PIN);
  }
  if (TCC1->INTFLAG.bit.OVF && TCC1->INTENSET.bit.OVF){
    BLANK_PORT &= ~(1 << BLANK_PIN);
    disable_XLAT_pulses();
    clear_XLAT_interrupt();
    tlc_needXLAT = 0;
    if (tlc_onUpdateFinished) {
      set_XLAT_interrupt();
      tlc_onUpdateFinished();
    }
  }
}

/** \defgroup ReqVPRG_ENABLED Functions that Require VPRG_ENABLED
    Functions that require VPRG_ENABLED == 1.
    You can enable VPRG by changing
    \code #define VPRG_ENABLED    0 \endcode to
    \code #define VPRG_ENABLED    1 \endcode in tlc_config.h

    You will also have to connect Arduino digital pin 6 to TLC pin 27. (The
    pin to be used can be changed in tlc_config.h).  If VPRG is not enabled,
    the TLC pin should grounded (remember to unconnect TLC pin 27 from GND
    if you do enable VPRG). */
/* @{ */ /* @} */

/** \defgroup CoreFunctions Core Libary Functions
    These function are all prefixed with "Tlc." */
/* @{ */

/** Pin i/o and Timer setup.  The grayscale register will be reset to all
    zeros, or whatever initialValue is set to and the Timers will start.
    \param initialValue = 0, optional parameter specifing the inital startup
    value */
void Tlc5940::init(uint16_t initialValue)
{
  REG_TCC1_INTENSET |=  TCC_INTENSET_MC0;
  REG_TCC1_INTENSET |=  TCC_INTENSET_MC2;

  REG_GCLK_GENDIV = GCLK_GENDIV_DIV(3) |  // Divide the 48MHz clock source by divisor 1: 48MHz/1=48MHz
                    GCLK_GENDIV_ID(4);    // Select Generic Clock (GCLK) 4
  while (GCLK->STATUS.bit.SYNCBUSY);      // Wait for synchronization

  REG_GCLK_GENCTRL = GCLK_GENCTRL_IDC |           // Set the duty cycle to 50/50 HIGH/LOW
		     GCLK_GENCTRL_GENEN |         // Enable GCLK4
                     GCLK_GENCTRL_SRC_DFLL48M |   // Set the 48MHz clock source
                     GCLK_GENCTRL_ID(4);          // Select GCLK4
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization

  // Feed GCLK4 to TCC0 and TCC1
  REG_GCLK_CLKCTRL = GCLK_CLKCTRL_CLKEN |         // Enable GCLK4 to TCC0 and TCC1
                     GCLK_CLKCTRL_GEN_GCLK4 |     // Select GCLK4
                     GCLK_CLKCTRL_ID_TCC0_TCC1 ;  // Feed GCLK4 to TCC0 and TCC1
  while (GCLK->STATUS.bit.SYNCBUSY);              // Wait for synchronization


  NVIC_SetPriority(TCC1_IRQn, 0);    // Set the Nested Vector Interrupt Controller (NVIC) priority for TC1 to 0 (highest)
  NVIC_EnableIRQ(TCC1_IRQn);         // Connect TCC1 to Nested Vector Interrupt Controller (NVIC)

  /* Pin Setup */
  XLAT_DDR |= (1 << XLAT_PIN);
  BLANK_DDR |= (1 << BLANK_PIN);
  GSCLK_DDR |= (1 << GSCLK_PIN);
#if XERR_ENABLED
  XERR_DDR &=~ (1 << XERR_PIN);   // XERR as input
  XERR_PORT |= (1 << XERR_PIN);   // enable pull-up resistor
#endif
  BLANK_PORT |= (1 << BLANK_PIN); // leave blank high (until the timers start)

  tlc_shift8_init();

  setAll(initialValue);
  update();
  disable_XLAT_pulses();
  clear_XLAT_interrupt();
  tlc_needXLAT = 0;
  pulse_pin(XLAT_PORT, XLAT_PIN);

  /* Timer Setup */

  /* Timer 1 -  XLAT */
   disable_XLAT_pulses();  // non inverting, output on TCC1:3, XLAT //arduino pin 9
   REG_TCC1_WAVE |=  /*TCC_WAVE_POL(0xF) |         // Reverse the output polarity on all TCC0 outputs
		      */TCC_WAVE_WAVEGEN_NPWM;    // Setup dual slope PWM on TCC0
   while (TCC1->SYNCBUSY.bit.WAVE);               // Wait for synchronization
   REG_TCC1_PER = TLC_PWM_PERIOD; // see tlc_config.h
   while (TCC1->SYNCBUSY.bit.PER);                // Wait for synchronization
   REG_TCC1_CC0 = 16360; //set TCC1 interrupt timing
   while (TCC1->SYNCBUSY.bit.CC0);                // Wait for synchronization
   /* Timer 2 - GSCLK */
 #ifdef __SAMD21G18A__
   enable_GSCLK();      // output on TCC0:4 //arduino pin 2
   REG_TCC0_WAVE |= /*TCC_WAVE_POL(0xF) |         // Reverse the output polarity on all TCC0 outputs
		     */TCC_WAVE_WAVEGEN_NPWM;    // Setup normal PWM on TCC1
   REG_TCC0_CC0 = 1;                // duty factor (as short a pulse as possible)
   while (TCC0->SYNCBUSY.bit.CC0);                // Wait for synchronization
   REG_TCC0_PER = TLC_GSCLK_PERIOD; // see tlc_config.h
   while (TCC0->SYNCBUSY.bit.PER);                // Wait for synchronization
   REG_TCC0_CTRLA |= TCC_CTRLA_ENABLE | TCC_CTRLA_PRESCALER_DIV1;// no prescale, (start pwm output)
  while (TCC0->SYNCBUSY.bit.ENABLE);              // Wait for synchronization
#endif
  REG_TCC1_CTRLA |= TCC_CTRLA_ENABLE | TCC_CTRLA_PRESCALER_DIV1;// no prescale, (start pwm output)
  while (TCC1->SYNCBUSY.bit.ENABLE);              // Wait for synchronization
  enable_XLAT_pulses();
  update();
}

/** Clears the grayscale data array, #tlc_GSData, but does not shift in any
    data.  This call should be followed by update() if you are turning off
    all the outputs. */
void Tlc5940::clear(void)
{
  setAll(0);
}

/** Shifts in the data from the grayscale data array, #tlc_GSData.
    If data has already been shifted in this grayscale cycle, another call to
    update() will immediately return 1 without shifting in the new data.  To
    ensure that a call to update() does shift in new data, use
    \code while(Tlc.update()); \endcode
    or
    \code while(tlc_needXLAT); \endcode
    \returns 1 if there is data waiting to be latched, 0 if data was
    successfully shifted in */
uint8_t Tlc5940::update(void)
{
  if (tlc_needXLAT) {
    return 1;
  }
  disable_XLAT_pulses();
  if (firstGSInput) {
    // adds an extra SCLK pulse unless we've just set dot-correction data
    firstGSInput = 0;
  } else {
    pulse_pin(SCLK_PORT, SCLK_PIN);
  }
  uint8_t *p = tlc_GSData;
  while (p < tlc_GSData + NUM_TLCS * 24) {
    tlc_shift8(*p++);
    tlc_shift8(*p++);
    tlc_shift8(*p++);
  }
  tlc_needXLAT = 1;
  set_XLAT_interrupt();
  enable_XLAT_pulses();
  return 0;
}

/** Sets channel to value in the grayscale data array, #tlc_GSData.
    \param channel (0 to #NUM_TLCS * 16 - 1).  OUT0 of the first TLC is
    channel 0, OUT0 of the next TLC is channel 16, etc.
    \param value (0-4095).  The grayscale value, 4095 is maximum.
    \see get */
void Tlc5940::set(TLC_CHANNEL_TYPE channel, uint16_t value)
{
  TLC_CHANNEL_TYPE index8 = (NUM_TLCS * 16 - 1) - channel;
  uint8_t *index12p = tlc_GSData + ((((uint16_t)index8) * 3) >> 1);
  if (index8 & 1) { // starts in the middle
    // first 4 bits intact | 4 top bits of value
    *index12p = (*index12p & 0xF0) | (value >> 8);
    // 8 lower bits of value
    *(++index12p) = value & 0xFF;
  } else { // starts clean
    // 8 upper bits of value
    *(index12p++) = value >> 4;
    // 4 lower bits of value | last 4 bits intact
    *index12p = ((uint8_t)(value << 4)) | (*index12p & 0xF);
  }
}

/** Gets the current grayscale value for a channel
    \param channel (0 to #NUM_TLCS * 16 - 1).  OUT0 of the first TLC is
    channel 0, OUT0 of the next TLC is channel 16, etc.
    \returns current grayscale value (0 - 4095) for channel
    \see set */
uint16_t Tlc5940::get(TLC_CHANNEL_TYPE channel)
{
  TLC_CHANNEL_TYPE index8 = (NUM_TLCS * 16 - 1) - channel;
  uint8_t *index12p = tlc_GSData + ((((uint16_t)index8) * 3) >> 1);
  return (index8 & 1)? // starts in the middle
    (((uint16_t)(*index12p & 15)) << 8) | // upper 4 bits
    *(index12p + 1)                       // lower 8 bits
    : // starts clean
    (((uint16_t)(*index12p)) << 4) | // upper 8 bits
    ((*(index12p + 1) & 0xF0) >> 4); // lower 4 bits
  // that's probably the ugliest ternary operator I've ever created.
}

/** Sets all channels to value.
    \param value grayscale value (0 - 4095) */
void Tlc5940::setAll(uint16_t value)
{
  uint8_t firstByte = value >> 4;
  uint8_t secondByte = (value << 4) | (value >> 8);
  uint8_t *p = tlc_GSData;
  while (p < tlc_GSData + NUM_TLCS * 24) {
    *p++ = firstByte;
    *p++ = secondByte;
    *p++ = (uint8_t)value;
  }
}

/* @} */

#if DATA_TRANSFER_MODE == TLC_BITBANG

/** Sets all the bit-bang pins to output */
void tlc_shift8_init(void)
{
  SIN_DDR |= (1 << SIN_PIN);   // SIN as output
  SCLK_DDR |= (1 << SCLK_PIN); // SCLK as output
  SCLK_PORT &=~ (1 << SCLK_PIN);
}

/** Shifts a byte out, MSB first */
void tlc_shift8(uint8_t byte)
{
  for (uint8_t bit = 0x80; bit; bit >>= 1) {
    if (bit & byte) {
      SIN_PORT |= (1 << SIN_PIN);
    } else {
      SIN_PORT &=~ (1 << SIN_PIN); //set SIN_PORT low
    }
    pulse_pin(SCLK_PORT, SCLK_PIN);
  }
}
/*
#elif DATA_TRANSFER_MODE == TLC_SPI

// Initializes the SPI module to double speed (f_osc / 2)
void tlc_shift8_init(void)
{
  SIN_DDR    |= (1 << SIN_PIN);    // SPI MOSI as output
  SCLK_DDR   |= (1 << SCLK_PIN);   // SPI SCK as output
  TLC_SS_DDR |= (1 << TLC_SS_PIN); // SPI SS as output

  SCLK_PORT &= ~(1 <<SCLK_PIN);

  SPSR = (1 <<SPI2X); // double speed (f_osc / 2)
  SPCR = (1 <<SPE)    // enable SPI
    | (1 << MSTR);  // master mode
}

// Shifts out a byte, MSB first
void tlc_shift8(uint8_t byte)
{
    SPDR = byte; // starts transmission
    while (!(SPSR & _BV(SPIF)))
        ; // wait for transmission complete
}
*/
#endif


/** Preinstantiated Tlc variable. */
Tlc5940 Tlc;

/** \defgroup ExtendedFunctions Extended Library Functions
    These functions require an include statement at the top of the sketch. */
/* @{ */ /* @} */

/** \mainpage
    The <a href="http://www.ti.com/lit/gpn/TLC5940">Texas Instruments TLC5940
    </a> is a 16-channel, constant-current sink LED driver.  Each channel has
    an individually adjustable 4096-step grayscale PWM brightness control and
    a 64-step, constant-current sink (no LED resistors needed!).  This chip
    is a current sink, so be sure to use common anode RGB LEDs.

    Check the <a href="http://code.google.com/p/tlc5940arduino/">tlc5940arduino
    project</a> on Google Code for updates.  To install, unzip the "Tlc5940"
    folder to &lt;Arduino Folder&gt;/hardware/libraries/

    &nbsp;

    \section hardwaresetup Hardware Setup
    The basic hardware setup is explained at the top of the Examples.  A good
    place to start would be the BasicUse Example.  (The examples are in
    File->Sketchbook->Examples->Library-Tlc5940).

    All the options for the library are located in tlc_config.h, including
    #NUM_TLCS, what pins to use, and the PWM period.  After changing
    tlc_config.h, be sure to delete the Tlc5940.o file in the library folder
    to save the changes.

    &nbsp;

    \section libref Library Reference
    \ref CoreFunctions "Core Functions" (see the BasicUse Example and Tlc5940):
    - \link Tlc5940::init Tlc.init(int initialValue (0-4095))\endlink - Call this is
    to setup the timers before using any other Tlc functions.
    initialValue defaults to zero (all channels off).
    - \link Tlc5940::clear Tlc.clear()\endlink - Turns off all channels
    (Needs Tlc.update())
    - \link Tlc5940::set Tlc.set(uint8_t channel (0-(NUM_TLCS * 16 - 1)),
    int value (0-4095))\endlink - sets the grayscale data for channel.
    (Needs Tlc.update())
    - \link Tlc5940::setAll Tlc.setAll(int value(0-4095))\endlink - sets all
    channels to value. (Needs Tlc.update())
    - \link Tlc5940::get uint16_t Tlc.get(uint8_t channel)\endlink - returns
    the grayscale data for channel (see set).
    - \link Tlc5940::update Tlc.update()\endlink - Sends the changes from any
    Tlc.clear's, Tlc.set's, or Tlc.setAll's.

    \ref ExtendedFunctions "Extended Functions".  These require an include
    statement at the top of the sketch to use.

    \ref ReqVPRG_ENABLED "Functions that require VPRG_ENABLED".  These
    require VPRG_ENABLED == 1 in tlc_config.h

    &nbsp;

    \section expansion Expanding the Library
    Lets say we wanted to add a function like "tlc_goCrazy(...)".  The first
    thing to do is to create a source file in the library folder,
    "tlc_my_crazy_functions.h".
    A template for this .h file is
    \code
    // Explination for my crazy function extension

    #ifndef TLC_MY_CRAZY_FUNCTIONS_H
    #define TLC_MY_CRAZY_FUNCTIONS_H

    #include "tlc_config.h"
    #include "Tlc5940.h"

    void tlc_goCrazy(void);

    void tlc_goCrazy(void)
    {
    uint16_t crazyFactor = 4000;
    Tlc.clear();
    for (uint8_t channel = 4; channel < 9; channel++) {
    Tlc.set(channel, crazyFactor);
    }
    Tlc.update();
    }

    #endif
    * \endcode
    * Now to use your library in a sketch, just add
    * \code
    #include "tlc_my_crazy_functions.h"

    // ...

    tlc_goCrazy();
    \endcode
    If you would like to share your extended functions for others to use,
    email me (acleone ~AT~ gmail.com) with the file and an example and I'll
    include them in the library.

    &nbsp;

    \section bugs Contact
    If you found a bug in the library, email me so I can fix it!
    My email is acleone ~AT~ gmail.com

    &nbsp;

    \section license License - GPLv3
    Copyright (c) 2009 by Alex Leone <acleone ~AT~ gmail.com>

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
