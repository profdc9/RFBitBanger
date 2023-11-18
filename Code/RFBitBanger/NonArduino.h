/*
 * Copyright (c) 2023 Daniel Marks

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

/*
This header file provides some definitions needed when building
the code outside of the Arduino environment.  IT IS NOT USED IN NORMAL
RFBITBANGER OPERATION.
 */
#ifndef _NONARDUINO_H
#define _NONARDUINO_H

#include <stdint.h>
#include <stddef.h>

// This is an AVR-specific function that allows you to store data in flash 
// (program) memory instead of SRAM.
// Ignore this directive completely.
#define PROGMEM

// Used to read from the program memory
#define pgm_read_byte_near(addr) (pgm_read_byte_near_2((const uint8_t*)addr))
#define pgm_read_word_near(addr) (pgm_read_word_near_2((const uint16_t*)addr))

uint8_t pgm_read_byte_near_2(const uint8_t* addr);
uint16_t pgm_read_word_near_2(const uint16_t* addr);

// Arduino platform
unsigned long millis();
void cli();
void sei();

#endif
