/*
  Copyright (C) 2020 by Daniel Marks

  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
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
  
  Daniel L. Marks profdc9@gmail.com

*/

#include <Arduino.h>
#include <avr/pgmspace.h>
#include "PS2Keyboard.h"

#define FIFOSIZE 16

static uint8_t state, paritybit;
static uint8_t curbyte;
static uint8_t shiftkey;
static uint8_t ctrlkey;
static uint8_t lastkeyup;

struct scancodetable 
{
  uint8_t scancode;
  uint8_t nonshift;
  uint8_t shifted;
  uint8_t ctrled;
};

static const struct scancodetable scancodes[] PROGMEM =
{
   { 0x1C, 'a', 'A', 'A'-64 },
   { 0x32, 'b', 'B', 'B'-64 },
   { 0x21, 'c', 'C', 'C'-64 },
   { 0x23, 'd', 'D', 'D'-64 },
   { 0x24, 'e', 'E', 'E'-64 },
   { 0x2B, 'f', 'F', 'F'-64 },
   { 0x34, 'g', 'G', 'G'-64 },
   { 0x33, 'h', 'H', 'H'-64 },
   { 0x43, 'i', 'I', 'I'-64 },
   { 0x3B, 'j', 'J', 'J'-64 },
   { 0x42, 'k', 'K', 'K'-64 },
   { 0x4B, 'l', 'L', 'L'-64 },
   { 0x3A, 'm', 'M', 'M'-64 },
   { 0x31, 'n', 'N', 'N'-64 },
   { 0x44, 'o', 'O', 'O'-64 },
   { 0x4D, 'p', 'P', 'P'-64 },
   { 0x15, 'q', 'Q', 'Q'-64 },
   { 0x2D, 'r', 'R', 'R'-64 },
   { 0x1B, 's', 'S', 'S'-64 },
   { 0x2C, 't', 'T', 'T'-64 },
   { 0x3C, 'u', 'U', 'U'-64 },
   { 0x2A, 'v', 'V', 'V'-64 },
   { 0x1D, 'w', 'W', 'W'-64 },
   { 0x22, 'x', 'X', 'X'-64 },
   { 0x35, 'y', 'Y', 'Y'-64 },
   { 0x1A, 'z', 'Z', 'Z'-64 },
   { 0x45, '0', ')', '0' },
   { 0x16, '1', '!', '1' },
   { 0x1E, '2', '@', '2' },
   { 0x26, '3', '#', '3' },
   { 0x25, '4', '$', '4' },
   { 0x2E, '5', '%', '5' },
   { 0x36, '6', '^', '6' },
   { 0x3D, '7', '&', '7' },
   { 0x3E, '8', '*', '8' },
   { 0x46, '9', '(', '9' },
   { 0x0E, '`', '~', '`'-64 },
   { 0x4E, '-', '_', '-' },
   { 0x55, '=', '+', '=' },
   { 0x5D, '\\', '|', '\\'-64 },
   { 0x66, 0x08, 0x08, 0x08 },
   { 0x29, ' ', ' ', ' ' },
   { 0x0D, 0x09, 0x09, 0x09 },
   { 0x5A, 0x0D, 0x0D, 0x0D },
   { 0x76, 27,   27,   27 },
   { 0x54, '[', '{', '['-64 },
   { 0x5B, ']', '}', ']'-64 },
   { 0x4C, ';', ':', ';' },
   { 0x52, '\'', '\"', '\'' },
   { 0x41, ',', '<', ',' },
   { 0x49, '.', '>', '.' },
   { 0x4A, '/', '?', '/' },
   { 0x75, PS2KEY_UP, 0, 0 },
   { 0x6B, PS2KEY_LEFT, 0, 0 },
   { 0x72, PS2KEY_DOWN, 0, 0 },
   { 0x74, PS2KEY_RIGHT, 0, 0 },
   { 0x76, PS2KEY_ESC, 0, 0 } };

#define KB_LEFTSHIFT 0x12
#define KB_RIGHTSHIFT 0x59
#define KB_CTRL 0x14
#define KB_ALT 0x11
#define KB_KEY_UP 0xF0
   
struct kbdfifo
{
  volatile uint8_t buf[FIFOSIZE];
  volatile uint8_t fifohead;
  volatile uint8_t fifotail;
};

struct kbdfifo kbdf;

static void initkbdfifo(struct kbdfifo *fifo)
{
	fifo->fifohead = fifo->fifotail = 0;
}

static int getkbdfifo(struct kbdfifo *fifo)
{	
	int ch;
	int newpos;
	if (fifo->fifotail == fifo->fifohead)
		return PS2KEY_NONE;
	ch = fifo->buf[fifo->fifotail];
	newpos = fifo->fifotail+1;
	if (newpos >= FIFOSIZE) newpos = 0;
	fifo->fifotail = newpos;
	return ch;
}

static void putkbdfifo(struct kbdfifo *fifo, int ch)
{
	int newpos = fifo->fifohead + 1;
	if (newpos >= FIFOSIZE) newpos = 0;	
	if (newpos == fifo->fifotail)
		return;
	fifo->buf[fifo->fifohead] = ch;
	fifo->fifohead = newpos;
}

int PS2Keyboard::getkey()
{
	return getkbdfifo(&kbdf);
}

int PS2Keyboard::waitkey()
{
    int ch;
    while ((ch=getkey()) == PS2KEY_NONE);
	return ch;
}

static void irqHandler (void) 
{
	uint8_t databit;

	databit = digitalRead(PS2_DATALINE);
		
	if (state == 0) {
		if (!databit) {
			paritybit = 0;
			curbyte = 0;
			state++;
		}
	} else if (state <= 8) {
		curbyte >>= 1;
		if (databit) {
			curbyte |= 0x80;
			paritybit ^= 0x01;
		}
		state++;
	} else if (state == 9) {
		state = (paritybit != (databit != 0)) ? 10 : 0;
	} else if (state == 10) {
		if (databit) {
			if (curbyte == KB_KEY_UP) {
				lastkeyup = 1;
			} else {
				if ((curbyte == KB_LEFTSHIFT) || (curbyte == KB_RIGHTSHIFT)) {
					shiftkey = !lastkeyup;
				} else if (curbyte == KB_CTRL) {
					ctrlkey = !lastkeyup;
				} else {
					if (!lastkeyup) {
            struct scancodetable *s = &scancodes[0];
            while (s < &scancodes[sizeof(scancodes)/sizeof(struct scancodetable)])
						{
							if (pgm_read_byte_near(&s->scancode) == curbyte) {
							  if (ctrlkey) 
									putkbdfifo(&kbdf,pgm_read_byte_near(&s->ctrled));
								  	else
									putkbdfifo(&kbdf,shiftkey ? pgm_read_byte_near(&s->shifted) : pgm_read_byte_near(&s->nonshift));
								break;
							}
             s++;
						}
					}
				}
				lastkeyup = 0;
			}
		}
		state=0;
	}
}

void PS2Keyboard::begin()
{
	pinMode(PS2_CLOCKLINE,INPUT);
	pinMode(PS2_DATALINE,INPUT);
	
	attachInterrupt(digitalPinToInterrupt(PS2_CLOCKLINE),irqHandler,FALLING);
	
	state = 0;
	curbyte = 0;
	paritybit = 0;
	shiftkey = 0;
	ctrlkey = 0;
	lastkeyup = 0;
	initkbdfifo(&kbdf);
}

PS2Keyboard::PS2Keyboard()
{
}
