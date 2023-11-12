/*
 * Copyright (c) 2021 Daniel Marks

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
This header file is used to provide some definitions needed when building
the code outside of the Arduino environment.  
 */
#ifndef _NonArduino_h
#define _NonArduino_h

#include <stdint.h>
#include <stddef.h>

// Ignore this directive
#define PROGMEM

#define pgm_read_byte_near(addr) (pgm_read_byte_near_2((const uint8_t*)addr))
#define pgm_read_word_near(addr) (pgm_read_word_near_2((const uint16_t*)addr))

uint8_t pgm_read_byte_near_2(const uint8_t* addr);
uint16_t pgm_read_word_near_2(const uint16_t* addr);

unsigned long millis();
void cli();
void sei();
void* memset(void* dest, int c, size_t count);

// This is defined in RFBitBanger.io
void received_character(uint8_t ch);

#endif
