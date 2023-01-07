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

void si5351simple::set_xo_freq(uint32_t p_xo_freq)
{
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

   setSourceAndPower(multisynth_no, 0, 0, 0, 0, 0);
   if (synth_base != 0xFF)
   {
	   for (i=0;i<8;i++) si5351_write(synth_base+i, s_regs->regs[i]);
   }
   for (i=0;i<8;i++) si5351_write(multisynth_base+i, m_regs->regs[i]);
   si5351_write(multisynth_no+165, m_regs->offset);
	   
   setSourceAndPower(multisynth_no, m_regs->offset != 0, 1, synth_no, 3, m_regs->inv);
   si5351_write(177, synth_no ? 0x80 : 0x20);
}									 

static void calc_multisynth_registers(uint8_t regs[], uint8_t R, uint32_t a, uint32_t b, uint32_t c)
{
  uint32_t P1, P2, P3;

  P3 = (b << 7) / c;
  P1 = (a << 7) + P3 - 512;
  P2 = (b << 7) - c * P3;
  P3 = c;
  
  regs[0] = (P3 >> 8) & 0xFF;
  regs[1] = P3 & 0xFF;
  regs[2] = (((uint8_t)(P1 >> 16)) & 0x03) | (R << 4);
  regs[3] = (P1 >> 8) & 0xFF;
  regs[4] = P1 & 0xFF;  
  regs[5] = ((uint8_t)(P3 >> 12) & 0xF0) | ((uint8_t)(P2 >> 16) & 0x0F);
  regs[6] = (P2 >> 8) & 0xFF;
  regs[7] = P2 & 0xFF;
}

void si5351simple::calc_registers(uint32_t frequency, uint8_t phase, si5351_synth_regs *s_regs, si5351_multisynth_regs *m_regs)
{
  #define c 1048574               // "c" part of Feedback-Multiplier from XTAL to PLL
  uint32_t fvco;                  // VCO frequency (600-900 MHz) of PLL
  uint8_t R = 0;                  // Additional Output Divider in range [1,2,4,...128]
  uint8_t mult_ratio;
  uint16_t a;                      // "a" part of Feedback-Multiplier from XTAL to PLL in range [15,90]
  uint32_t b;                     // "b" part of Feedback-Multiplier from XTAL to PLL

  if (phase >= 0x80)
  {
    phase -= 0x80;
    m_regs->inv = 1;
  } else m_regs->inv = 0;

   if (frequency < 5000000)
      mult_ratio = 15;
   else if (frequency < 7000000)
      mult_ratio = 22;
   else mult_ratio = 33;  

  if (s_regs != NULL)
      calc_multisynth_registers(s_regs->regs, 0, mult_ratio, 0, 1);
  
  fvco = xo_freq * mult_ratio;

  for (;;)
  {
      a = (fvco / frequency);
      if (a <= 900) break;
      frequency <<= 1;
      R += 1;
  }

  b = fvco - a * frequency;
  b = (((uint64_t)b) * c) / frequency;

  if (phase)
     m_regs->offset = (uint8_t)((((uint32_t)a)*phase) >> 6) & 0x7F;
  else
     m_regs->offset = 0;
  calc_multisynth_registers(m_regs->regs, R, a, b, c);
}

// off_on = 0 is off, off_on = 1 is on
// pll_source = 1 is PLLB, pll_source = 0 is PLLA
// power=0 2mA, power=1 4 mA, power=2 6 mA, power=3 8 mA
void si5351simple::setSourceAndPower(uint8_t clock_no, uint8_t frac, uint8_t off_on, uint8_t pll_source, uint8_t power, uint8_t inv)
{                        
  if (power > 3) return;
  si5351_write(clock_no+16, (off_on ? 0 : 0x80) + (frac ? 0 : 0x40) + (pll_source ? 0x20 : 0x00) + (inv ? 0x10 : 0x00) + power + 0x0C);
}

void si5351simple::setOutputOnOffMask(uint8_t new_on_mask)
{
  on_mask = new_on_mask ^ 0xFF;
  si5351_write(3, on_mask);
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
  
  // si5351_synth_regs s_regs;
  // si5351_multisynth_regs m_regs;
  
  //calc_registers(7000000, 0, &s_regs, &m_regs);
  //set_registers(0, &s_regs, 0, &m_regs);

  //calc_registers(7000000, 64, &s_regs, &m_regs);
  //set_registers(0, &s_regs, 1, &m_regs);

  //setOutputOnOff(0,1);
  //setOutputOnOff(1,0);
}
