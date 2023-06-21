
/*  ssb.h */

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

#ifndef __SSB_H
#define __SSB_H

#ifdef __cplusplus
extern "C" {
#endif

#define SSB_FIR_LENGTH 16

#undef SSB_DEBUG_REGISTERS

typedef struct _ssb_state
{
  uint8_t   protocol;
  uint8_t   sampling_clock;
  uint8_t   gain;
  int16_t   cumulative_sample;
  int16_t   dc_level;
  int16_t   lpf_z1;
  int16_t   ssb_fir_buffer[SSB_FIR_LENGTH];
  int16_t   previous_phase;
  uint16_t  magnitude;
#ifdef SSB_DEBUG_REGISTERS
  int16_t   last_sample, last_sample2;
  int16_t   phase_difference;
  int16_t   frequency_shift;
  uint32_t  no_interrupts;
  uint32_t  phase_inversions;
#endif
} ssb_state;

#ifdef __cplusplus
}
#endif

void ssb_interrupt(int16_t sample);
void ssb_state_change(uint8_t state);
uint16_t ssb_get_magnitude(void);
int16_t ssb_frequency_offset(void);

#endif  /* __SSB_H */
