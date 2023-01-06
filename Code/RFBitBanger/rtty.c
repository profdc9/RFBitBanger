
/* rtty.c */

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


#undef RTTY_DEBUG

#ifdef ARDUINO
#include <Arduino.h>
#endif

#ifdef RTTY_DEBUG
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#endif

#include "common.h"
#include "dspint.h"
#include "rtty.h"

/* SPACE=0, MARK=1 */
/* CODE IS SPACE, BIT 4, BIT 3, BIT 2, BIT 1, BIT 0, MARK, MARK */
/* FOR 45/170, MARK/SPACE length is 1000.0 / 45 = 22.2 ms */
/* MARK FREQUENCY IS 170 HZ ABOVE SPACE FREQUENCY (at actual RF frequencies) */

const uint8_t PROGMEM rtty_letters[32] =
{ '\000',  'T',  '\r', 'O',    ' ', 'H', 'N', 'M',
    '\n',  'L',   'R', 'G',    'I', 'P', 'C', 'V',
     'E',  'Z',   'D', 'B',    'S', 'Y', 'F', 'X',
     'A',  'W',   'J', 'f',    'U', 'Q', 'K', 'l' };
const uint8_t PROGMEM rtty_figures[32] =
{ '\000',  '5',  '\r', '9',    ' ', '#', ',', '.',
    '\n',  ')',   '4', '&',    '8', '0', ':', ';',
     '3', '\"',   '$', '?',   '\'', '6', '!', '/',
     '-',  '2',  '\'', 'f',    '7', '1', '(', 'l' };

/* initialize frame fifo */
void rtty_initialize_frame_fifo(volatile rtty_frame_fifo *dff)
{
    dff->head = dff->tail = 0;
}

/* insert a frame into the fifo.  this is intended to be interrupt safe */
uint8_t rtty_insert_into_frame_fifo(volatile rtty_frame_fifo *dff, uint8_t frame)
{
    uint8_t next = dff->head >= (RTTY_FRAME_FIFO_LENGTH - 1) ? 0 : (dff->head+1);
    if (next == dff->tail) return 0;
    dff->frames[next] = frame;
    dff->head = next;
    return 1;
}

/* remove a frame from the fifo.  this is intended to be interrupt safe */
uint16_t rtty_remove_from_frame_fifo(volatile rtty_frame_fifo *dff)
{
    uint8_t frame;
    uint8_t next;
    if (dff->tail == dff->head) return 0xFFFF;
    next = dff->tail >= (RTTY_FRAME_FIFO_LENGTH - 1) ? 0 : (dff->tail+1);
    frame = dff->frames[next];
    dff->tail = next;
    return frame;
}

/* convert character to 6-bit code if it exists */
/* should probably replace this with an inverse look up table later */
uint8_t rtty_find_code_in_table(uint8_t c, uint8_t figs)
{
  uint8_t i;
  uint8_t *table = figs ? rtty_figures : rtty_letters;
  if ((c >= 'a') && (c <= 'z')) c -= 32;
  for (i=1;i<(sizeof(rtty_letters)/sizeof(uint8_t));i++)
        if (pgm_read_byte_near(&table[i]) == c) return i;
  return 0xFF;
}

#define RTTY_OFFSET_1 583
#define RTTY_OFFSET_2 750

void rtty_send_code(uint8_t code)
{
  uint8_t b;
  uint16_t delay_millis = 22;
  uint16_t current_time = millis(), read_time;
  code = (code << 2) | 0x03;
  for (b=0;b<8;b++)
  {
    set_clock_onoff_mask(code & 0x80 ? 0x02 : 0x01) ;
    code <<= 1;
    idle_task();
    do
    {
      read_time = millis();
    } while ((read_time - current_time) < delay_millis);
    current_time += delay_millis;
  }
}

uint8_t rtty_txmit(dsp_txmit_message_state *dtms, dsp_dispatch_callback ddc)
{
  uint8_t figs = 0;
  
  set_frequency(dtms->frequency + RTTY_OFFSET_1, 0);
  set_frequency(dtms->frequency + RTTY_OFFSET_2, 1);
  muteaudio_set(1);
  transmit_set(1);
  rtty_send_code(0x1F);  /* send letters diddle three times to allow receiver to wake up and sync */
  rtty_send_code(0x1F);
  rtty_send_code(0x1F);
  for (dtms->current_symbol=0;dtms->current_symbol<dtms->length;dtms->current_symbol++)
  {
    uint8_t ch = dtms->message[dtms->current_symbol], num=0, cwbits;
    uint8_t code;

    code = rtty_find_code_in_table(ch, figs);
    if (code == 0xFF)
    {    
      code = rtty_find_code_in_table(ch, !figs);
      if (code == 0xFF) continue;
      figs = !figs;
      for (uint8_t j=0;j<rc.rtty_figs_resend;j++)
         rtty_send_code(figs ? 0x1B : 0x1F);
    }
    rtty_send_code(code);
    ddc(dtms);
    if (dtms->aborted) break;
  }
  set_clock_onoff_mask(0) ;
  transmit_set(0);
  muteaudio_set(0);
}

void rtty_reset_codeword(void)
{
   ps.rs.current_bit_no = 0;
   ps.rs.current_word = SCAMP_BLANK_CODEWORD;
}

void rtty_new_sample(void)
{
    int16_t demod_sample, temp;
    uint8_t received_bit, b, v;

    uint8_t ct = ds.sample_ct;
    /* only proceed if we have new magnitude samples */
    if (ct == ps.rs.last_sample_ct)
        return;
    ps.rs.last_sample_ct = ct;

    demod_sample = ds.mag_value_8 - ds.mag_value_24;
    ps.rs.ct_sum += (ds.mag_value_8 + ds.mag_value_24);
    
    /* This is the automatic "gain" control (threshold level control).
       find the average of a certain number of samples (power of two for calculation
       speed) so that we can determine what to set the thresholds at for the bit on/off
       threshold for ook and the edge threshold for ook/fsk */

    if ((++ps.rs.ct_average) >= (((uint16_t)1) << RTTY_AVG_CT_PWR2))
    {
       uint16_t temp;
       temp = (ps.rs.ct_sum) >> (RTTY_AVG_CT_PWR2);
       /* don't allow threshold to get too low, or we'll be having bit edges constantly */
       ps.rs.power_thr = temp > ps.rs.power_thr_min ? temp : ps.rs.power_thr_min;
       ps.rs.edge_thr = ps.rs.power_thr;
       ps.rs.power_thr >>= 1;
       ps.rs.ct_average = 0;
       ps.rs.ct_sum = 0;
    }

    /* calculate the difference between the modulated signal between now and one bit period ago to see
       if there is a bit edge */
    temp = ps.rs.demod_buffer[ps.rs.demod_sample_no];
    ps.rs.bit_edge_val = demod_sample > temp ? (demod_sample - temp) : (temp - demod_sample);
    /* store the demodulated sample into a circular buffer to calculate the edge */
    ps.rs.demod_buffer[ps.rs.demod_sample_no] = demod_sample;
    if ((++ps.rs.demod_sample_no) >= ps.rs.demod_samples_per_bit)
        ps.rs.demod_sample_no = 0;

    if (ps.rs.edge_ctr > 0)  /* count down the edge counter */
        --ps.rs.edge_ctr;

    received_bit = 0;

    if (ps.rs.edge_ctr < ps.rs.cur_demod_edge_window)  /* if it below the edge window, start looking for the bit edge */
    {
        if (ps.rs.bit_edge_val > ps.rs.edge_thr)  /* the edge should come around now, does it exceed threshold */
        {
            if (ps.rs.bit_edge_val > ps.rs.max_bit_edge_val)  /* if so, have we reached the peak of the edge */
            {
                ps.rs.max_bit_edge_val = ps.rs.bit_edge_val;  /* if so, reset the edge center counter */
                ps.rs.next_edge_ctr = 1;
                ps.rs.cur_bit = demod_sample;              /* save the bit occurring at the edge */
            } else
                ps.rs.next_edge_ctr++;                     /* otherwise count that we have passed the edge peak */
        } else
        {
            if (ps.rs.max_bit_edge_val != 0)   /* if we have detected an edge */
            {
                ps.rs.edge_ctr = ps.rs.demod_samples_per_bit > ps.rs.next_edge_ctr ? ps.rs.demod_samples_per_bit - ps.rs.next_edge_ctr : 0;
                                /* reset edge ctr to look for next edge */
                ps.rs.max_bit_edge_val = 0;    /* reset max_bit_edge_val for next edge peak */
                received_bit = 1;
                ps.rs.cur_demod_edge_window = ps.rs.demod_edge_window;
            } else /* we haven't detected an edge */
            {
                if (ps.rs.edge_ctr == 0)
                {
                    ps.rs.cur_bit = demod_sample;              /* save the bit */
                    ps.rs.edge_ctr = ps.rs.demod_samples_per_bit;  /* reset and wait for the next bit edge to come along */
                    received_bit = 1;                       /* an edge hasn't been detected but a bit interval happened */
                }
            }
        }
    }
    if (!received_bit)   /* no bit available yet, wait */
        return;
        
    b = ps.rs.cur_bit > 0;
    v = (ds.mag_value_8 >= ps.rs.power_thr) || (ds.mag_value_24 >= ps.rs.power_thr);

    if ((ps.rs.current_bit_no == 0) && v)
    {
      if (b) return;   /* can't have first bit as mark */
      ps.rs.current_bit_no = 1;
      return;
    } else if (ps.rs.current_bit_no < 6)  /* if it's one of the five payload bits */
    {
       ps.rs.current_bit_no++;
       ps.rs.current_word = (ps.rs.current_word << 1) | b;
       return;
    }
    if (b && v)   /* if we have received a mark bit for the 7th bit, this is a valid code */
       rtty_insert_into_frame_fifo(&ps.rs.rtty_output_fifo, ps.rs.current_word);
    ps.rs.current_bit_no = 0;   /* clear and wait for the next space */
    ps.rs.current_word = 0;
}

void rtty_decode_process(void)
{
  uint8_t code;
  uint16_t fr = rtty_remove_from_frame_fifo(&ps.rs.rtty_output_fifo);
  
  if (fr == 0xFFFF) return;
  code = fr;
  if (code == 0x1B) 
  {
    ps.rs.figures = 1;
  } else if (code == 0x1F)
  {
    ps.rs.figures = 0;
  } else if (code > 0)
  {
    if (ps.rs.figures)
      decode_insert_into_fifo(pgm_read_byte_near(&rtty_figures[code]));
    else
      decode_insert_into_fifo(pgm_read_byte_near(&rtty_letters[code]));
  }
}
