/*  enc4b5b.c */

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
 
#define CODE_4B5B_HALT    0b00100
#define CODE_4B5B_IDLE    0b11111
#define CODE_4B5B_START1  0b11000
#define CODE_4B5B_START2  0b10001
#define CODE_4B5B_START3  0b00110
#define CODE_4B5B_QUIET   0b00000
#define CODE_4B5B_RESET   0b00111
#define CODE_4B5B_SET     0b11001
#define CODE_4B5B_END     0b01101
 
const uint8_t table4b5b[16] =
 { 
   0b11110, 0b01001, 0b10100, 0b10101, 0b01010, 0b01011, 0b01110, 0b01111,
   0b10010, 0b10011, 0b10110, 0b10111, 0b11010, 0b11011, 0b11100, 0b11101
 };

#define IMP 0xFF 
 
const uint8_t table5b4b[32] = 
{
   IMP,    IMP,    IMP,    IMP,    IMP,    IMP,    IMP,    IMP,
   IMP, 0b0001, 0b0100, 0b0101,    IMP,    IMP, 0b0110, 0b0111,
   IMP,    IMP, 0b1000, 0b1001, 0b0010, 0b0011, 0b1010, 0b1011,
   IMP,    IMP, 0b1100, 0b1101, 0b1110, 0b1111, 0b0000,    IMP 
};
