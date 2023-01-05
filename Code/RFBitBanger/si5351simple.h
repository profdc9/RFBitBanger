/* si5351simple.h

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

#ifndef _si5351simple_h
#define _si5351simple_h

#include <inttypes.h>

#define SI5351_ADDRESS 0x60

#define SI5351_SYNTH_PLL_A 26
#define SI5351_SYNTH_PLL_B 34

typedef struct _si5351_synth_regs
{ 
  uint8_t regs[8];
} si5351_synth_regs;

#define SI5351_MULTISYNTH_0 42
#define SI5351_MULTISYNTH_1 50
#define SI5351_MULTISYNTH_2 58

typedef struct _si5351_multisynth_regs
{
  uint8_t regs[8];
  uint8_t offset;
  uint8_t inv;
} si5351_multisynth_regs;

class si5351simple  {
public:
  si5351simple(uint8_t cap, uint32_t xo_freq); 
  void start(void);
  void calc_registers(uint32_t frequency, uint8_t phase, si5351_synth_regs *s_regs, si5351_multisynth_regs *m_regs);
  void setSourceAndPower(uint8_t clock_no, uint8_t frac, uint8_t off_on, uint8_t pll_source, uint8_t power, uint8_t inv);
  void set_registers(uint8_t synth_no, si5351_synth_regs *s_regs, uint8_t multisynth_no, si5351_multisynth_regs *m_regs);
  void setOutputOnOff(uint8_t clock_no, uint8_t off_on);
  void setOutputOnOffMask(uint8_t new_on_mask);

private:
  uint8_t cap;
  uint8_t on_mask;
  uint32_t xo_freq;
};

#endif
