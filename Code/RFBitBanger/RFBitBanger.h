/* RFBitBanger.h

*/

/*
   Copyright (c) 2021 Daniel Marks

  This software is provided 'as-is', without any express or implied
  warranty. In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
   claim that you wrote the original software. If you use this software
   in a product, an acknowledgment in the product documentation would be
   appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
   misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.
*/

#ifndef _RFBITBANGER_H
#define _RFBITBANGER_H

#define EEPROM_CONFIGURATION_ADDRESS 0

#include "si5351simple.h"

#define LCDB_RS A3
#define LCDB_E A2
#define LCDB_DB4 5
#define LCDB_DB5 6
#define LCDB_DB6 7
#define LCDB_DB7 8

#define AUDIOFILT_PIN A0
#define TRANSMIT_PIN 9
#define BACKLIGHT_PIN 11
#define MUTEAUDIO_PIN 4
#define BEEPOUT_PIN 3
#define PTT_PIN 13
#define MIC_PIN A1

extern PS2Keyboard PSkey;
extern LiquidCrystalButtons lcd;
extern si5351simple si5351;

#endif /* _RFBITBANGER_H */
