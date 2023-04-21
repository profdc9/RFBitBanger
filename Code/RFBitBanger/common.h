/*  common.h */

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

#ifndef __COMMON_H
#define __COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

#define RC_MAGIC_NUMBER 0xFABE

#define TONEFREQ_BASEFREQ (16000000ul / 256)
#define TONEFREQ(x) (((uint16_t)TONEFREQ_BASEFREQ) / (x))
#define TONEFREQVOL(x,vol) (((TONEFREQ_BASEFREQ / (x)) * (vol)) >> 8)
#define TONE_ON(freq,vol) tone_on(TONEFREQ(freq),TONEFREQVOL(freq,vol))
#define TONE_OFF() tone_off()

typedef struct _radio_configuration
{ 
  uint16_t  magic_number;
  uint32_t  frequency_calibration;
  uint8_t   cw_send_speed;
  uint8_t   scamp_resync_frames;
  uint8_t   scamp_resend_frames;
  uint8_t   rtty_figs_resend;
  uint16_t  sidetone_frequency;
  uint8_t   sidetone_on;
  uint8_t   cw_practice;
  uint8_t   wide;
  uint8_t   ext_fast_mode;
  uint8_t   ext_lsb;
  uint8_t   band_warning_off;
  uint8_t   cw_spaces_from_mark_timing;
  uint8_t   cw_smooth;
  uint8_t   cw_sticky_interval_length;
  uint16_t  rit_shift_freq;
  uint8_t   rit_shift_dir;
  uint8_t   cw_iambic;
  uint8_t   cw_iambic_type;
  uint8_t   cw_iambic_switch;
  uint8_t   erase_on_send;
} radio_configuration;

extern radio_configuration rc;

void set_frequency(uint32_t freq, uint8_t clockno);
void set_clock_onoff(uint8_t onoff, uint8_t clockno);
void set_clock_onoff_mask(uint8_t on_mask);
void transmit_set(uint8_t set);
void muteaudio_set(uint8_t set);
void idle_task(void);
void delayidle(uint32_t ms);
void tone_on(uint8_t freq, uint8_t vol);
void tone_off(void);

#ifdef __cplusplus
}
#endif
#endif  /* __COMMON_H */
