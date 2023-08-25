
/*  rtty.h */

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

#ifndef __RTTY_H
#define __RTTY_H

#ifdef __cplusplus
extern "C" {
#endif

#define RTTY_PWR_THR_DEF 8
#define RTTY_AVG_CT_PWR2 9
#define RTTY_FRAME_FIFO_LENGTH 8
#define RTTY_MAX_DEMODBUFFER 12

typedef struct _rtty_frame_fifo
{
    uint8_t   head;
    uint8_t   tail;
    uint8_t   frames[RTTY_FRAME_FIFO_LENGTH];
} rtty_frame_fifo;

typedef struct _rtty_state
{
  uint8_t   protocol;

  uint8_t   last_sample_ct;

  int8_t    current_bit_no;
  uint8_t   current_word;
  
  uint16_t  ct_average;
  uint32_t  ct_sum;

  uint8_t   demod_samples_per_bit;
  uint8_t   demod_edge_window;
  uint16_t  power_thr_min;

  uint8_t   demod_sample_no;
  uint8_t   edge_ctr;
  uint8_t   next_edge_ctr;
  uint8_t   resync;
  uint8_t   cur_demod_edge_window;

  uint16_t  edge_thr;
  uint16_t  power_thr;

  uint16_t  bit_edge_val;
  uint16_t  max_bit_edge_val;
  int16_t   cur_bit;

  int16_t   demod_buffer[RTTY_MAX_DEMODBUFFER];

  volatile rtty_frame_fifo rtty_output_fifo;

  uint8_t   figures;
  
} rtty_state;

void rtty_new_sample(void);
void rtty_decode_process(void);
void rtty_reset_codeword(void);
uint8_t rtty_txmit(dsp_txmit_message_state *dtms, dsp_dispatch_callback ddc);
int16_t rtty_frequency_offset(void);

#ifdef __cplusplus
}
#endif

#endif  /* __RTTY_H */
