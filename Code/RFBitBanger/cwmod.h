
/*  cwmod.h */

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

#ifndef __CWMOD_H
#define __CWMOD_H

#ifdef __cplusplus
extern "C" {
#endif

#define CWMOD_AVG_CT_PWR2 10
#define CWMOD_THRESHOLD_MIN 1000
#define CWMOD_TIMING_LENGTHS 32
#define CWMOD_TIMING_BINS 16
#define CWMOD_PROCESS_DELAY 2000
#define CWMOD_FIFO_DECODE_THRESHOLD 4

#define CWMOD_SMOOTH_SHIFT_LENGTH 4
#define CWMOD_SMOOTH_MAX_LENGTH (1 << CWMOD_SMOOTH_SHIFT_LENGTH)

typedef struct _cwmod_symbol
{
    uint8_t   cwbits;     /* 0=dit, 1=dah, LSB is the last bit to be sent/received */
    uint8_t   num;        /* number of dits/dahs (num LSBs are observed) */
    uint8_t   symbol;     /* ASCII symbol */
} cwmod_symbol;

typedef struct _cwmod_state
{
  uint8_t   protocol;

  uint8_t   last_sample_ct;
  uint8_t   sticky_interval_length;

  uint8_t   ct_smooth;
  uint8_t   ct_smooth_ind;
  uint8_t   ct_smooth_ind_max;
  uint32_t  ct_smooth_sum;
  uint16_t  ct_smooth_mag[CWMOD_SMOOTH_MAX_LENGTH];
  uint16_t  ct_min_val;

  uint8_t   spaces_from_mark_timing;
  uint8_t   key_state;
  uint16_t  state_ctr;
  uint8_t   sticky_interval;
  uint16_t  keydown_threshold;
  uint16_t  keyup_threshold;
  uint16_t  ct_average;
  uint32_t  ct_sum;
  uint32_t  ct_sum_sq;
  uint8_t   timing_head;
  uint8_t   timing_tail;
  uint8_t   timing_peek_tail;
  uint16_t  timing_lengths[CWMOD_TIMING_LENGTHS];

  uint16_t  total_ticks;
  uint16_t  last_tick;

  uint8_t   histogram_marks[CWMOD_TIMING_BINS];
  uint8_t   histogram_spaces[CWMOD_TIMING_BINS];

  uint16_t  dit_dah_threshold;
  uint16_t  intrainterspace_threshold;
  uint16_t  interspaceword_threshold;

  uint8_t   num_ditdahs;
  uint8_t   cur_ditdahs;
} cwmod_state;

#ifdef __cplusplus
}
#endif

#endif  /* __CWMOD_H */
