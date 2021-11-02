
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

#define DSPINT_MAX_SAMPLEBUFFER 64
#define DSPINT_MAX_DEMODBUFFER 16
#define DSPINT_PWR_THR_DEF 32

#define DSPINT_AVG_CT_PWR2 8

#define DSPINT_OOK 0
#define DSPINT_OOK2 1
#define DSPINT_FSK 2
#define DSPINT_FSK2 3

#define DSPINT_SYNC_CODEWORD 0b111110110100011001110100011110ul
                              /* 0x3ED19D1E */

#define DSPINT_BLANK_CODEWORD 0xAAAAAAAA

#define DSPINT_FRAME_FIFO_LENGTH 8

/* structure for FRAME FIFO */
typedef struct _dspint_frame_fifo
{
    uint8_t   head;
    uint8_t   tail;
    uint32_t  frames[DSPINT_FRAME_FIFO_LENGTH];
} dspint_frame_fifo;

/* this is stuff that is initialized when the modulation mode
   is changed and doesn't change otherwise */
typedef struct _dsp_state_fixed
{
  uint8_t   mod_type;
  uint8_t   buffer_size;
  uint8_t   fsk;

  uint8_t   dly_8;
  uint8_t   dly_12;
  uint8_t   dly_16;
  uint8_t   dly_20;
  uint8_t   dly_24;

  uint8_t   demod_samples_per_bit;
  uint8_t   demod_edge_window;
  uint16_t  power_thr_min;
} dsp_state_fixed;

/* this is the current state of the demodulator and is designed
   so that is might be reset quickly without disturbing the
   dsp_state_fixed state */
typedef struct _dsp_state
{
  uint8_t   slow_samp;
  uint8_t   slow_samp_num;
  uint16_t  total_num;

  uint8_t   sample_no;
  uint8_t   mag_new_sample;
  uint8_t   demod_sample_no;
  uint8_t   edge_ctr;
  uint8_t   next_edge_ctr;
  uint8_t   polarity;
  uint8_t   resync;
  uint8_t   demod_edge_window;

  uint8_t   bitflips_in_phase;
  uint8_t   bitflips_lag;
  uint8_t   bitflips_lead;
  uint8_t   bitflips_ctr;

  uint16_t  edge_thr;
  uint16_t  power_thr;

  uint16_t  ct_average;
  uint32_t  ct_sum;

  int8_t    current_bit_no;
  uint32_t  current_word;

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

  uint16_t  mag_value_8;
  uint16_t  mag_value_12;
  uint16_t  mag_value_16;
  uint16_t  mag_value_20;
  uint16_t  mag_value_24;
  uint16_t  sample_buffer[DSPINT_MAX_SAMPLEBUFFER];
  int16_t   demod_buffer[DSPINT_MAX_DEMODBUFFER];

  uint16_t  bit_edge_val;
  uint16_t  max_bit_edge_val;
  int16_t   cur_bit;
} dsp_state;

extern dsp_state       ds;
extern dsp_state_fixed df;
extern volatile dspint_frame_fifo dsp_input_fifo;
extern volatile dspint_frame_fifo dsp_output_fifo;

void dsp_new_sample(uint16_t sample);
void dsp_initialize(uint8_t sample_buffer_size);
void dsp_reset_state(void);

void dsp_initialize_frame_fifo(volatile dspint_frame_fifo *dff);
uint8_t dsp_insert_into_frame_fifo(volatile dspint_frame_fifo *dff, uint32_t frame);
uint32_t dsp_remove_from_frame_fifo(volatile dspint_frame_fifo *dff);

#endif  /* __DSPINT_H */
