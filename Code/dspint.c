
/*  dspint.c */

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


#define DSPINT_DEBUG

#ifdef DSPINT_DEBUG
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#endif

#include "dspint.h"

dsp_state       ds;
dsp_state_fixed df;


uint8_t dsp_hamming_weight_16(uint16_t n)
{
  uint8_t s = 0, v;
  v = (n >> 8) & 0xFF;
  while (v)
  {
      v &= (v - 1);
      s++;
  }
  v = n & 0xFF;
  while (v)
  {
      v &= (v - 1);
      s++;
  }
  return s;
}

uint8_t dsp_hamming_weight_30(uint32_t n)
{
  uint8_t s = 0, v;
  v = (n >> 24) & 0x3F;
  while (v)
  {
      v &= (v - 1);
      s++;
  }
  v = (n >> 16) & 0xFF;
  while (v)
  {
      v &= (v - 1);
      s++;
  }
  v = (n >> 8) & 0xFF;
  while (v)
  {
      v &= (v - 1);
      s++;
  }
  v = n & 0xFF;
  while (v)
  {
      v &= (v - 1);
      s++;
  }
  return s;
}

void dsp_reset_codeword(void)
{
   ds.current_bit_no = 0;
   ds.current_word = DSPINT_BLANK_CODEWORD;
}

/* reset state of buffer in case we get some kind
   of a dc offset mismatch */

void dsp_reset_state(void)
{
   memset(&ds,'\000',sizeof(dsp_state));
   dsp_reset_codeword();
   ds.demod_edge_window =  df.demod_edge_window;
}

/* initialize the buffer including the signs to be subtracted
   from the end of the buffer */

void dsp_initialize(uint8_t mod_type)
{
    uint8_t i;
    df.mod_type = mod_type;
    switch (df.mod_type)
    {
        case DSPINT_OOK:  df.buffer_size = 32;
                          break;
        case DSPINT_OOK2: df.buffer_size = 64;
                          break;
        case DSPINT_FSK:  df.buffer_size = 60;
                          break;
        case DSPINT_FSK2: df.buffer_size = 24;
                          break;
    }
    df.dly_8 = (df.buffer_size / 8) * 8;
    df.dly_12 = (df.buffer_size / 12) * 12;
    df.dly_16 = (df.buffer_size / 16) * 16;
    df.dly_20 = (df.buffer_size / 20) * 20;
    df.dly_24 = (df.buffer_size / 24) * 24;
    df.demod_samples_per_bit = df.buffer_size / 4;
    df.edge_thr = df.buffer_size * DSPINT_EDGE_THR_DEF;
    df.power_thr = df.edge_thr / 2;
    df.demod_edge_window = 4;
    dsp_reset_state();
}

#define SET_DSP_SQRT_APPROX(s,x,y) do { \
   uint16_t ux = (x < 0 ? -x : x); \
   uint16_t uy = (y < 0 ? -y : y); \
   s = ((ux + uy) >> 1)+(uy > ux ? uy : ux); \
} while(0)

/* called by the interrupt routine to update the spectral channels */
void dsp_new_sample(uint16_t sample)
{
   uint8_t b;
   int16_t fir;
   ds.mag_new_sample = !(ds.count_8 & 0x03);

   /* update 8 count I & Q */
   b = (ds.sample_no < df.dly_8) ? (ds.sample_no + df.buffer_size - df.dly_8) : (ds.sample_no - df.dly_8);
   fir = sample - ds.sample_buffer[b];
   if (ds.count_8 >= 4)
       ds.state_i_8 += fir;
   else
       ds.state_i_8 -= fir;
   if ((ds.count_8 >= 2) && (ds.count_8 < 6))
       ds.state_q_8 += fir;
   else
       ds.state_q_8 -= fir;
   ds.count_8 = (ds.count_8 >= 7) ? 0 : (ds.count_8 + 1);
   if (ds.mag_new_sample == 0)
       SET_DSP_SQRT_APPROX(ds.mag_value_8, ds.state_q_8, ds.state_i_8);

   /* update 12 count I & Q */
   b = (ds.sample_no < df.dly_12) ? (ds.sample_no + df.buffer_size - df.dly_12) : (ds.sample_no - df.dly_12);
   fir = sample - ds.sample_buffer[b];
   if (ds.count_12 >= 6)
       ds.state_i_12 += fir;
   else
       ds.state_i_12 -= fir;
   if ((ds.count_12 >= 3) && (ds.count_12 < 9))
       ds.state_q_12 += fir;
   else
       ds.state_q_12 -= fir;
   ds.count_12 = (ds.count_12 >= 11) ? 0 : (ds.count_12 + 1);
   if (ds.mag_new_sample)
       SET_DSP_SQRT_APPROX(ds.mag_value_12, ds.state_q_12, ds.state_i_12);

   /* update 16 count I & Q */
   b = (ds.sample_no < df.dly_16) ? (ds.sample_no + df.buffer_size - df.dly_16) : (ds.sample_no - df.dly_16);
   fir = sample - ds.sample_buffer[b];
   if (ds.count_16 >= 8)
       ds.state_i_16 += fir;
   else
       ds.state_i_16 -= fir;
   if ((ds.count_16 >= 4) && (ds.count_16 < 12))
       ds.state_q_16 += fir;
   else
       ds.state_q_16 -= fir;
   ds.count_16 = (ds.count_16 >= 15) ? 0 : (ds.count_16 + 1);
   if (ds.mag_new_sample)
        SET_DSP_SQRT_APPROX(ds.mag_value_16, ds.state_q_16, ds.state_i_16);

   /* update 20 count I & Q */
   b = (ds.sample_no < df.dly_20) ? (ds.sample_no + df.buffer_size - df.dly_20) : (ds.sample_no - df.dly_20);
   fir = sample - ds.sample_buffer[b];
   if (ds.count_20 >= 10)
       ds.state_i_20 += fir;
   else
       ds.state_i_20 -= fir;
   if ((ds.count_20 >= 5) && (ds.count_20 < 15))
       ds.state_q_20 += fir;
   else
       ds.state_q_20 -= fir;
   ds.count_20 = (ds.count_20 >= 19) ? 0 : (ds.count_20 + 1);
   if (ds.mag_new_sample)
        SET_DSP_SQRT_APPROX(ds.mag_value_20, ds.state_q_20, ds.state_i_20);

   /* update 24 count I & Q */
   b = (ds.sample_no < df.dly_24) ? (ds.sample_no + df.buffer_size - df.dly_24) : (ds.sample_no - df.dly_24);
   fir = sample - ds.sample_buffer[b];
   if (ds.count_24 >= 12)
       ds.state_i_24 += fir;
   else
       ds.state_i_24 -= fir;
   if ((ds.count_24 >= 6) && (ds.count_24 < 18))
       ds.state_q_24 += fir;
   else
       ds.state_q_24 -= fir;
   ds.count_24 = (ds.count_24 >= 23) ? 0 : (ds.count_24 + 1);
   if (ds.mag_new_sample)
        SET_DSP_SQRT_APPROX(ds.mag_value_24, ds.state_q_24, ds.state_i_24);

   /* store in circular buffer so that is can be subtracted from the end
      to make a moving average filter */
   ds.sample_buffer[ds.sample_no++] = sample;
   if (ds.sample_no >= df.buffer_size)
      ds.sample_no = 0;
}

void dsp_interrupt_sample(uint16_t sample)
{
    int16_t demod_sample;
    uint8_t received_bit;
    uint8_t hamming_weight;

    /* update the filter channel I & Q */
    dsp_new_sample(sample);

    /* only proceed if we have new magnitude samples */
    if (!ds.mag_new_sample)
        return;

    /* demodulate a sample either based on the amplitude in one frequency
       channel for OOK, or the difference in amplitude between two frequency
       channels for FSK */

    switch (df.mod_type)
    {
        case DSPINT_OOK:
        case DSPINT_OOK2: demod_sample = ds.mag_value_16 - df.power_thr;
                          break;
        case DSPINT_FSK:  demod_sample = ds.mag_value_20 - ds.mag_value_12;
                          break;
        case DSPINT_FSK2: demod_sample = ds.mag_value_12 - ds.mag_value_8;
                          break;
    }
    /* calculate the difference between the modulated signal between now and one bit period ago to see
       if there is a bit edge */
    ds.bit_edge_val = demod_sample > ds.demod_buffer[ds.demod_sample_no] ?
       (demod_sample - ds.demod_buffer[ds.demod_sample_no]) : (ds.demod_buffer[ds.demod_sample_no] - demod_sample);
    /* store the demodulated sample into a circular buffer to calculate the edge */
    ds.demod_buffer[ds.demod_sample_no] = demod_sample;
    if ((++ds.demod_sample_no) >= df.demod_samples_per_bit)
        ds.demod_sample_no = 0;

    if (ds.edge_ctr > 0)  /* count down the edge counter */
        --ds.edge_ctr;

    received_bit = 0;

    if (ds.edge_ctr < ds.demod_edge_window)  /* if it below the edge window, start looking for the bit edge */
    {
        if (ds.bit_edge_val > df.edge_thr)  /* the edge should come around now, does it exceed threshold */
        {
            if (ds.bit_edge_val > ds.max_bit_edge_val)  /* if so, have we reached the peak of the edge */
            {
                ds.max_bit_edge_val = ds.bit_edge_val;  /* if so, reset the edge center counter */
                ds.next_edge_ctr = 1;
                ds.cur_bit = demod_sample;              /* save the bit occurring at the edge */
            } else
                ds.next_edge_ctr++;                     /* otherwise count that we have passed the edge peak */
        } else
        {
            if (ds.max_bit_edge_val != 0)   /* if we have detected an edge */
            {
                ds.edge_ctr = df.demod_samples_per_bit > ds.next_edge_ctr ? df.demod_samples_per_bit - ds.next_edge_ctr : 0;
                                /* reset edge ctr to look for next edge */
                ds.max_bit_edge_val = 0;    /* reset max_bit_edge_val for next edge peak */
                received_bit = 1;
                ds.demod_edge_window = df.demod_edge_window;
            } else /* we haven't detected an edge */
            {
                if (ds.edge_ctr == 0)
                {
                    ds.cur_bit = demod_sample;              /* save the bit */
                    ds.edge_ctr = df.demod_samples_per_bit;  /* reset and wait for the next bit edge to come along */
                    received_bit = 1;                       /* an edge hasn't occurred but a bit interval happened */
                }
            }
        }
    }
    if (!received_bit)   /* no bit available yet, wait */
        return;

    ds.current_word = (ds.current_word << 1) | (ds.polarity ^ (ds.cur_bit > 0));
    ds.current_bit_no++;
    hamming_weight = dsp_hamming_weight_30(ds.current_word ^ DSPINT_SYNC_CODEWORD);
    printf("received: %08X %05d %02d %02d %02d\n", ds.current_word, ds.cur_bit, ds.current_bit_no, hamming_weight, ds.polarity);
    if (hamming_weight < 3)  /* resync has occurred! */
    {
        printf("resync!\n");
        dsp_reset_codeword();
        ds.resync = 1;
        return;
    }
    hamming_weight = dsp_hamming_weight_16(ds.current_word);
    if ((hamming_weight < 2) && ((df.mod_type == DSPINT_FSK) || (df.mod_type == DSPINT_FSK2)))
        /* 15 or 16 zeros in a row, must be reversed polarity, only applies to FSK */
    {
        ds.demod_edge_window = df.demod_samples_per_bit;
        ds.polarity = !ds.polarity;
        dsp_reset_codeword();
        ds.resync = 0;
        return;
    }
        /* 15 or 16 ones in a row, sync has been received */
    if (hamming_weight >= 15)
    {
        ds.demod_edge_window = df.demod_samples_per_bit;
        dsp_reset_codeword();
        ds.resync = 0;
        return;
    }
}

#ifdef DSPINT_DEBUG
void test_dsp_sample(void)
{
    srand(4001);
    dsp_initialize(DSPINT_OOK);
    uint16_t c, samp;
//    const uint8_t bits[36] = { 1,1,1,1,1,1,1,0,1,1,0,1,0,0,0,1,1,0,0,1,1,1,0,1,0,0,0,1,1,1,1,0,0,0};
    const uint8_t bits[64] = { 1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,1,1, 1,1,0,0, 0,0,1,1,
          1,1,1,1,1,0,1,1,0,1,0,0,0,1,1,0,0,1,1,1,0,1,0,0,0,1,1,1,1,0,0,0};
    for (c=0;c<(64*32);c++)
    {
        uint8_t cbit = bits[c/32];
        int freq = cbit ? 16.0 : 9999999920.0;
        float fs = ((sin(2.0*M_PI*c/freq+1.0*M_PI)*128.0)+512.0) + 0.5*(rand())*(128.0/16384.0);
        //float fs = (rand()-16384)/800.0 + 20.0;
        /*if (c < 50)
            samp = (uint16_t) (fs*c/50.0);
        else if (c > 950)
            samp = (uint16_t) (fs*(1000.0-c)/50.0);
        else */
            samp = (uint16_t) (fs);
        dsp_interrupt_sample(samp);

        if (ds.mag_new_sample)
        {
            printf("%d %05d %05d %05d %05d %05d %05d %05d %05d %02d %02d xx\n",cbit,c,samp,
                ds.mag_value_8,ds.mag_value_12,ds.mag_value_16,ds.mag_value_20,ds.mag_value_24,
                ds.bit_edge_val,ds.edge_ctr,ds.demod_edge_window);
        }
    }
}

void main(void)
{
  test_dsp_sample();
}
#endif /* DSPINT_DEBUG */
