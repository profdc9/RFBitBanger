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

#include "common.h"

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

#define SI5351_FREQ_OFFSET 2048
#define SI5351_FREQ_OFFSET_SHIFT 11

#define FEEDBACK_MULTIPLIER_C 524288               // "c" part of Feedback-Multiplier from XTAL to PLL
#define FEEDBACK_MULTIPLIER_SHIFT 19

typedef struct _si5351_multisynth_regs
{
  uint8_t regs[8];
  uint8_t offset;
  uint8_t inv;
} si5351_multisynth_regs;

typedef struct _si5351_cached_regs
{
  uint8_t R;             // Additional Output Divider in range [1,2,4,...128]
  uint8_t mult_ratio;
  uint32_t fvco;         // VCO frequency (600-900 MHz) of PLL
  uint16_t a;            // "a" part of Feedback-Multiplier from XTAL to PLL in range [15,90]
  int32_t  b;            // "b" part of Feedback-Multiplier from XTAL to PLL
  int32_t  b_offset_pos; // offset b
  int32_t  b_offset_neg; // offset b
} si5351_cached_regs;

class si5351simple  {
  si5351_cached_regs c_regs;
public:
  si5351simple(uint8_t cap, uint32_t xo_freq); 
  void set_xo_freq(uint32_t p_xo_freq);
  uint32_t get_xo_freq();
  void start(void);
  void calc_registers(uint32_t frequency, uint8_t phase, uint8_t calc_offset, si5351_synth_regs *s_regs, si5351_multisynth_regs *m_regs);
  void setSourceAndPower(uint8_t clock_no, uint8_t frac, uint8_t off_on, uint8_t pll_source, uint8_t power, uint8_t inv);
  void set_registers(uint8_t synth_no, si5351_synth_regs *s_regs, uint8_t multisynth_no, si5351_multisynth_regs *m_regs);
  void setOutputOnOff(uint8_t clock_no, uint8_t off_on);
  void setOutputOnOffMask(uint8_t new_on_mask);
  int32_t calculate_b(uint32_t frequency);
  void print_c_regs(void);
  void calc_mult_registers(uint32_t frequency, int16_t offset);
  void set_offset_fast(int16_t offset);

private:
  uint8_t cap;
  uint8_t on_mask;
  uint32_t xo_freq;
};

#endif
