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

#ifndef TLC5940_H
#define TLC5940_H

/** \file
    Tlc5940 library header file. */
//#include "sam.h"
#include "Arduino.h"
#include <stdint.h>
#include "tlc_config.h"

#ifdef __SAMD21G18A__

/** Enables the Timer1 Overflow interrupt, which will fire after an XLAT
    pulse */
#define set_XLAT_interrupt()    REG_TCC1_INTENSET |= TCC_INTENSET_OVF
/** Disables any Timer1 interrupts */
#define clear_XLAT_interrupt()  REG_TCC1_INTFLAG |= TCC_INTFLAG_OVF // Clear the OVF interrupt flag

/** Enables the output of XLAT pulses */
#define enable_XLAT_pulses()      PORT->Group[g_APinDescription[3].ulPort].PINCFG[g_APinDescription[3].ulPin].bit.PMUXEN = 1; PORT->Group[g_APinDescription[4].ulPort].PMUX[g_APinDescription[4].ulPin >> 1].reg = PORT_PMUX_PMUXO_F; REG_TCC1_CC1 = 1; while (TCC1->SYNCBUSY.bit.CC1)
/** Disables the output of XLAT pulses */
#define disable_XLAT_pulses()   PORT->Group[g_APinDescription[3].ulPort].PINCFG[g_APinDescription[3].ulPin].bit.PMUXEN = 0; REG_TCC1_CC1 = 0; while (TCC1->SYNCBUSY.bit.CC1)


#define enable_GSCLK() PORT->Group[g_APinDescription[2].ulPort].PINCFG[g_APinDescription[2].ulPin].bit.PMUXEN = 1; PORT->Group[g_APinDescription[2].ulPort].PMUX[g_APinDescription[2].ulPin >> 1].reg = PORT_PMUX_PMUXE_F

#define disable_GSCLK() PORT->Group[g_APinDescription[2].ulPort].PINCFG[g_APinDescription[2].ulPin].bit.PMUXEN = 0;

#endif



extern volatile uint8_t tlc_needXLAT;
extern volatile void (*tlc_onUpdateFinished)(void);
extern uint8_t tlc_GSData[NUM_TLCS * 24];

/** The main Tlc5940 class for the entire library.  An instance of this class
    will be preinstantiated as Tlc. */
class Tlc5940
{
  public:
    void init(uint16_t initialValue = 0);
    void clear(void);
    uint8_t update(void);
    void set(TLC_CHANNEL_TYPE channel, uint16_t value);
    uint16_t get(TLC_CHANNEL_TYPE channel);
    void setAll(uint16_t value);
#if VPRG_ENABLED
    void setAllDC(uint8_t value);
#endif
#if XERR_ENABLED
    uint8_t readXERR(void);
#endif

};

void tlc_shift8_init(void);
void tlc_shift8(uint8_t byte);

#if VPRG_ENABLED
void tlc_dcModeStart(void);
void tlc_dcModeStop(void);
#endif

// for the preinstantiated Tlc variable.
extern Tlc5940 Tlc;

#endif
