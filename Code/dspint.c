
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

/* reset state of buffer in case we get some kind
   of a dc offset mismatch */

void dsp_reset_state(void)
{
  memset(&ds,'\000',sizeof(dsp_state));
}

/* initialize the buffer including the signs to be subtracted
   from the end of the buffer */

void dsp_initialize(uint8_t sample_buffer_size)
{
    uint8_t i;
    if (sample_buffer_size > DSPINT_MAX_SAMPLEBUFFER)
        sample_buffer_size = DSPINT_MAX_SAMPLEBUFFER;
    dsp_reset_state();
    df.buffer_size = sample_buffer_size;
    df.dly_8 = (sample_buffer_size / 8) * 8;
    df.dly_12 = (sample_buffer_size / 12) * 12;
    df.dly_16 = (sample_buffer_size / 16) * 16;
    df.dly_20 = (sample_buffer_size / 20) * 20;
    df.dly_24 = (sample_buffer_size / 24) * 24;
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
   ds.mag_new_sample = (ds.count_8 & 0x03);

   if (ds.mag_new_sample == 0)
        ds.mag_count = (ds.mag_count >= (DSPINT_MAX_MAGCOUNT-1)) ? 0 : (ds.mag_count + 1);

   /* update 8 count */
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
       SET_DSP_SQRT_APPROX(ds.mag_values_8[ds.mag_count], ds.state_q_8, ds.state_i_8);

   /* update 12 count */
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
   if (ds.mag_new_sample == 0)
       SET_DSP_SQRT_APPROX(ds.mag_values_12[ds.mag_count], ds.state_q_12, ds.state_i_12);

   /* update 16 count */
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
   if (ds.mag_new_sample == 0)
        SET_DSP_SQRT_APPROX(ds.mag_values_16[ds.mag_count], ds.state_q_16, ds.state_i_16);

   /* update 20 count */
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
   if (ds.mag_new_sample == 0)
        SET_DSP_SQRT_APPROX(ds.mag_values_20[ds.mag_count], ds.state_q_20, ds.state_i_20);

   /* update 24 count */
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
   if (ds.mag_new_sample == 0)
        SET_DSP_SQRT_APPROX(ds.mag_values_24[ds.mag_count], ds.state_q_24, ds.state_i_24);

   ds.sample_buffer[ds.sample_no++] = sample;
   if (ds.sample_no >= df.buffer_size)
      ds.sample_no = 0;

}

#ifdef DSPINT_DEBUG
void test_dsp_sample(void)
{
    dsp_initialize(60);
    uint16_t c, samp;
    for (c=0;c<1000;c++)
    {
        int freq = (c/60) & 0x01 ? 12.0 : 20.0;
        float fs = ((cos(2.0*M_PI*c/freq+1.0*M_PI)*10.0)+20.0);
        if (c < 50)
            samp = (uint16_t) (fs*c/50.0);
        else if (c > 950)
            samp = (uint16_t) (fs*(1000.0-c)/50.0);
        else
            samp = (uint16_t) (fs);
        dsp_new_sample(samp);

#if 0
        printf("%05d %05d %05d,%05d %05d,%05d %05d,%05d\n",c,samp,ds.state_i_8,ds.state_q_8,ds.state_i_12,ds.state_q_12,ds.state_i_16,ds.state_q_16);
#else
        if (ds.mag_new_sample == 0)
        {
            printf("%05d %05d %05d %05d %05d %05d %05d \n",c,samp,
                ds.mag_values_8[ds.mag_count],ds.mag_values_12[ds.mag_count],
                ds.mag_values_16[ds.mag_count],ds.mag_values_20[ds.mag_count],
                ds.mag_values_24[ds.mag_count]);
        }
#endif
    }
}

void main(void)
{
  test_dsp_sample();
}
#endif /* DSPINT_DEBUG */
