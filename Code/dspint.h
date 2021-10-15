
/*  dspint.h */

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

#ifndef __DSPINT_H
#define __DSPINT_H

#define DSPINT_MAX_SAMPLEBUFFER 80

typedef struct _dsp_state_fixed
{
  uint8_t   buffer_size;

  uint8_t   dly_8;
  uint8_t   dly_12;
  uint8_t   dly_16;
  uint8_t   dly_20;
  uint8_t   dly_24;
} dsp_state_fixed;

typedef struct _dsp_state
{
  uint8_t   sample_no;

  uint8_t   count_8;
  uint8_t   count_12;
  uint8_t   count_16;
  uint8_t   count_20;
  uint8_t   count_24;

  int16_t  state_i_8;
  int16_t  state_i_12;
  int16_t  state_i_16;
  int16_t  state_i_20;
  int16_t  state_i_24;

  int16_t  state_q_8;
  int16_t  state_q_12;
  int16_t  state_q_16;
  int16_t  state_q_20;
  int16_t  state_q_24;

  uint32_t state_m_8;
  uint32_t state_m_12;
  uint32_t state_m_16;
  uint32_t state_m_20;
  uint32_t state_m_24;

  uint16_t  sample_buffer[DSPINT_MAX_SAMPLEBUFFER];
} dsp_state;

extern dsp_state       ds;
extern dsp_state_fixed df;

void dsp_new_sample(uint16_t sample);
void dsp_initialize(uint8_t sample_buffer_size);
void dsp_reset_state(void);

#endif  /* __DSPINT_H */
