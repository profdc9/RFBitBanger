
/*  scamp.h */

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

#ifndef __SCAMP_H
#define __SCAMP_H

#include "dspint.h"

#define SCAMP_VERSION_NO "0.91"

#ifdef __cplusplus
extern "C" {
#endif

#define SCAMP_RES_CODE_END_TRANSMISSION 0b000000111100
#define SCAMP_RES_CODE_END_TRANSMISSION_FRAME 0x1B75426C

#define SCAMP_VERY_SLOW_MODES

#define SCAMP_SOLID_CODEWORD  0b111111111111111111111111111111ul
#define SCAMP_DOTTED_CODEWORD 0b101010101010101010101010101010ul
#define SCAMP_INIT_CODEWORD   0b111111111111111111111111010101ul
#define SCAMP_SYNC_CODEWORD   0b111110110100011001110100011110ul
                              /* 0x3ED19D1E */

#define SCAMP_THRESHOLD_COUNTER_MAX 2000

#define SCAMP_BLANK_CODEWORD 0xAAAAAAAA

#define SCAMP_PWR_THR_DEF_FSK 0
#define SCAMP_PWR_THR_DEF_OOK 3

#define SCAMP_AVG_CT_PWR2_FSK 9
#define SCAMP_AVG_CT_PWR2_OOK 12

#define SCAMP_FRAME_FIFO_LENGTH 8

#ifdef SCAMP_VERY_SLOW_MODES
#define SCAMP_MAX_DEMODBUFFER 36
#else
#define SCAMP_MAX_DEMODBUFFER 16
#endif

/* structure for FRAME FIFO */
typedef struct _scamp_frame_fifo
{
    uint8_t   head;
    uint8_t   tail;
    uint32_t  frames[SCAMP_FRAME_FIFO_LENGTH];
} scamp_frame_fifo;

typedef struct _scamp_state
{
  uint8_t   protocol;

  uint8_t   last_sample_ct;

  int8_t    current_bit_no;
  uint32_t  current_word;

  uint16_t  ct_average;
  uint32_t  ct_sum;

  uint8_t   fsk;
  uint8_t   reset_protocol;

  uint8_t   demod_samples_per_bit;
  uint8_t   demod_edge_window;
  uint16_t  power_thr_min;

  uint8_t   demod_sample_no;
  uint8_t   edge_ctr;
  uint8_t   next_edge_ctr;
  uint8_t   polarity;
  uint8_t   resync;
  uint8_t   cur_demod_edge_window;

  uint8_t   bitflips_in_phase;
  uint8_t   bitflips_lag;
  uint8_t   bitflips_lead;
  uint8_t   bitflips_ctr;

  uint16_t  edge_thr;
  uint16_t  power_thr;
  uint16_t  squelch_thr;

  uint16_t  bit_edge_val;
  uint16_t  max_bit_edge_val;
  int16_t   cur_bit;

  int16_t   demod_buffer[SCAMP_MAX_DEMODBUFFER];

  volatile scamp_frame_fifo scamp_output_fifo;

  uint16_t  last_code;
  uint16_t  threshold_counter;

  uint16_t  clock_bit;
} scamp_state;

uint8_t scamp_insert_into_frame_fifo(volatile scamp_frame_fifo *dff, uint32_t frame);
uint32_t scamp_remove_from_frame_fifo(volatile scamp_frame_fifo *dff);
void scamp_initialize_frame_fifo(volatile scamp_frame_fifo *dff);

typedef uint8_t (*scamp_code_word_put)(uint16_t, void *, uint8_t, uint8_t);
typedef uint16_t (*scamp_code_word_get)(void *);

void scamp_bytes_to_code_words(uint8_t *bytes, uint8_t num_bytes, scamp_code_word_put ecwp, void *st);
void scamp_raw_bytes_to_code_words(uint8_t *bytes, uint8_t num_bytes, scamp_code_word_put ecwp, void *st);
uint8_t scamp_code_words_to_bytes(scamp_code_word_get ecwg, void *st, uint8_t *bytes, uint8_t max_bytes);
uint32_t scamp_add_reversal_bits(uint32_t codeword);
uint32_t scamp_remove_reversal_bits(uint32_t outword);
int16_t scamp_frequency_offset();

uint8_t scamp_txmit(dsp_txmit_message_state *dtms, dsp_dispatch_callback ddc);
void scamp_new_sample(void);
void scamp_decode_process(void);
uint8_t hamming_weight_16(uint16_t n);
void scamp_reset_codeword(void);

#ifdef __cplusplus
}
#endif

#endif   /* __SCAMP_H */
