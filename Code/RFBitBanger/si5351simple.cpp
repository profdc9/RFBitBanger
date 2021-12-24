/* si5351simple.cpp

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

// This is cobbled together from many sources

#include "si5351simple.h"

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "Arduino.h"

#define TWI_HARDWARE

#ifdef TWI_HARDWARE
void twi_init(void)
{
    TWSR = 0x00;
    TWBR = 0x0C;
    TWCR = (1<<TWEN);
}

void twi_start(void)
{
    TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
    while ((TWCR & (1<<TWINT)) == 0);
}

void twi_stop(void)
{
    TWCR = (1<<TWINT)|(1<<TWSTO)|(1<<TWEN);
}

void twi_write(uint8_t u8data)
{
	  TWDR = u8data;
    TWCR = (1<<TWINT)|(1<<TWEN);
    while ((TWCR & (1<<TWINT)) == 0);
}

void si5351_write(uint8_t reg_addr, uint8_t reg_value)
{
   twi_start();
   twi_write(SI5351_ADDRESS << 1);
   twi_write(reg_addr);
   twi_write(reg_value);
   twi_stop();
} 
#else
#include <Wire.h>
void twi_init(void)
{
  Wire.begin();                    
}

void si5351_write(uint8_t regist, uint8_t value)
{
  Wire.beginTransmission(SI5351_ADDRESS);                      
  Wire.write(regist);                        
  Wire.write(value);                                
  Wire.endTransmission();                           
}
#endif

si5351simple::si5351simple(uint8_t p_cap, uint32_t p_xo_freq)
{
  twi_init();
  cap = p_cap;
  xo_freq = p_xo_freq;
}

void si5351simple::set_registers(uint8_t synth_no, si5351_synth_regs *s_regs, 
								 uint8_t multisynth_no, si5351_multisynth_regs *m_regs)
{
   uint8_t synth_base = 0xFF, multisynth_base = 0xFF, i;
   switch (multisynth_no)
   {
	   case 0: multisynth_base = SI5351_MULTISYNTH_0; break;
	   case 1: multisynth_base = SI5351_MULTISYNTH_1; break;
	   case 2: multisynth_base = SI5351_MULTISYNTH_2; break;
   }
   if (multisynth_base == 0xFF) return;
   switch (synth_no)
   {
	   case 0: synth_base = SI5351_SYNTH_PLL_A; break;
	   case 1: synth_base = SI5351_SYNTH_PLL_B; break;
   }

   setSourceAndPower(multisynth_no, 0, 0, 0, 0);
   if (synth_base != 0xFF)
   {
	   for (i=0;i<8;i++) si5351_write(synth_base+i, s_regs->regs[i]);
   }
   for (i=0;i<8;i++) si5351_write(multisynth_base+i, m_regs->regs[i]);
   si5351_write(multisynth_no+165, m_regs->offset);
	   
   si5351_write(177, synth_no ? 0x20 : 0x80);
   setSourceAndPower(multisynth_no, m_regs->offset != 0, 1, synth_no, 3);
}									 

void si5351simple::calc_registers(uint32_t frequency, si5351_synth_regs *s_regs, si5351_multisynth_regs *m_regs)
{
  #define c 1048574               // "c" part of Feedback-Multiplier from XTAL to PLL
  uint32_t fvco;                  // VCO frequency (600-900 MHz) of PLL
  uint32_t outdivider;            // Output divider in range [4,6,8-900], even numbers preferred
  uint8_t R = 1;                  // Additional Output Divider in range [1,2,4,...128]
  uint8_t a;                      // "a" part of Feedback-Multiplier from XTAL to PLL in range [15,90]
  uint32_t b;                     // "b" part of Feedback-Multiplier from XTAL to PLL
  uint32_t f;                        // floating variable, needed in calculation
  uint32_t MS0_P1;                // Si5351a Output Divider register MS0_P1, P2 and P3 are hardcoded below
  uint32_t MSNA_P1;               // Si5351a Feedback Multisynth register MSNA_P1
  uint32_t MSNA_P2;               // Si5351a Feedback Multisynth register MSNA_P2
  uint32_t MSNA_P3;               // Si5351a Feedback Multisynth register MSNA_P3

  outdivider = 900000000 / frequency;  // With 900 MHz beeing the maximum internal PLL-Frequency
  
  while (outdivider > 900)
  {                               // If output divider out of range (>900) use additional Output divider
    R <<= 1;
    outdivider >>= 1;
  }
  if (outdivider & 0x01) outdivider--;  // finds the even divider which delivers the intended Frequency

  fvco = outdivider * R * frequency;   // Calculate the PLL-Frequency (given the even divider)

  switch (R)
  {                                    // Convert the Output Divider to the bit-setting required in register 44
    case 1: R = 0; break;              // Bits [6:4] = 000
    case 2: R = 16; break;             // Bits [6:4] = 001
    case 4: R = 32; break;             // Bits [6:4] = 010
    case 8: R = 48; break;             // Bits [6:4] = 011
    case 16: R = 64; break;            // Bits [6:4] = 100
    case 32: R = 80; break;            // Bits [6:4] = 101
    case 64: R = 96; break;            // Bits [6:4] = 110
    case 128: R = 112; break;          // Bits [6:4] = 111
  }

  a = fvco / xo_freq;                   // Multiplier to get from Quartz-Oscillator Freq. to PLL-Freq.
  f = fvco - a * xo_freq;               // Multiplier = a+b/c
  f = (((uint64_t)f) * c) / xo_freq;    // 64-bit precision is needed here
  b = f;

  MS0_P1 = (outdivider << 7) - 512;     // Calculation of Output Divider registers MS0_P1 to MS0_P3
                                        // MS0_P2 = 0 and MS0_P3 = 1; these values are hardcoded, see below

  f = (b << 7) / c;                     // Calculation of Feedback Multisynth registers MSNA_P1 to MSNA_P3
  MSNA_P1 = (a << 7) + f - 512;
  MSNA_P2 = f;
  MSNA_P2 = (b << 7) - MSNA_P2 * c; 
  MSNA_P3 = c;

  s_regs->regs[0] = (MSNA_P3 & 65280) >> 8;
  s_regs->regs[1] = MSNA_P3 & 255; 
  s_regs->regs[2] = (MSNA_P1 & 196608) >> 16; 
  s_regs->regs[3] = (MSNA_P1 & 65280) >> 8;
  s_regs->regs[4] = MSNA_P1 & 255;
  s_regs->regs[5] = ((MSNA_P3 & 983040) >> 12) | ((MSNA_P2 & 983040) >> 16);
  s_regs->regs[6] = (MSNA_P2 & 65280) >> 8;
  s_regs->regs[7] = MSNA_P2 & 255;

  m_regs->offset = 0;
  m_regs->regs[0] = 0;
  m_regs->regs[1] = 1;
  m_regs->regs[5] = 0;
  m_regs->regs[6] = 0;
  m_regs->regs[7] = 0;

  if (outdivider == 4)
  {
    m_regs->regs[2] = 12 | R;
    m_regs->regs[3] = 0;
  	m_regs->regs[4] = 0;
  } else
  {	  
	  m_regs->regs[2] = ((MS0_P1 & 196608) >> 16) | R;
	  m_regs->regs[3] = (MS0_P1 & 65280) >> 8;
	  m_regs->regs[4] = MS0_P1 & 255;
  }
}

// off_on = 0 is off, off_on = 1 is on
// pll_source = 1 is PLLB, pll_source = 0 is PLLA
// power=0 2mA, power=1 4 mA, power=2 6 mA, power=3 8 mA
void si5351simple::setSourceAndPower(uint8_t clock_no, uint8_t frac, uint8_t off_on, uint8_t pll_source, uint8_t power)
{                        
  if (power > 3) return;
  si5351_write(clock_no+16, (off_on ? 0 : 0x80) + (frac ? 0 : 0x40) + (pll_source ? 0x20 : 0x00) + power + 0x0C);
}

void si5351simple::setOutputOnOff(uint8_t clock_no, uint8_t off_on)
{
  if (off_on) on_mask &= ~(1 << clock_no);
    else on_mask |= (1 << clock_no);
  si5351_write(3, on_mask);
}

void si5351simple::start(void)
{
  si5351_write(24, 0xAA);  // make all outputs high impedance when disabled.
  si5351_write(149, 0);    // disable spread spectrum
  si5351_write(9, 0xFF);   // OEB does not control on
  si5351_write(3, 0xFF);   // all outputs off initially
  on_mask = 0xFF;
    
  switch (cap)
  {
     case 6: si5351_write(183, 0x40); break;
     case 8: si5351_write(183, 0x80); break;
     case 10: si5351_write(183, 0xC0); break;
  }
  
  si5351_synth_regs s_regs;
  si5351_multisynth_regs m_regs;
  
  calc_registers(5000000, &s_regs, &m_regs);
  set_registers(0, &s_regs, 0, &m_regs);
  setOutputOnOff(0,1);
}
