
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
#endif

#include "dspint.h"

#define DSPINT_MAX_SAMPLEBUFFER 80

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

/* called the interrupt routine to update the spectral channels */

void dsp_new_sample(uint16_t sample)
{
   uint8_t b;

   /* update 8 count */
   b = (ds.sample_no < df.dly_8) ? (ds.sample_no + df.buffer_size - df.dly_8) : (ds.sample_no - df.dly_8);
   if (ds.count_8 >= 4)
   {
       ds.state_i_8 += sample;
       ds.state_i_8 -= ds.sample_buffer[b];
   } else
   {
       ds.state_i_8 -= sample;
       ds.state_i_8 += ds.sample_buffer[b];
   }
   if ((ds.count_8 >= 2) && (ds.count_8 < 6))
   {
       ds.state_q_8 += sample;
       ds.state_q_8 -= ds.sample_buffer[b];
   } else
   {
       ds.state_q_8 -= sample;
       ds.state_q_8 += ds.sample_buffer[b];
   }
   ds.count_8 = (ds.count_8 >= 7) ? 0 : (ds.count_8 + 1);

   /* update 12 count */
   b = (ds.sample_no < df.dly_12) ? (ds.sample_no + df.buffer_size - df.dly_12) : (ds.sample_no - df.dly_12);
   if (ds.count_12 >= 6)
   {
       ds.state_i_12 += sample;
       ds.state_i_12 -= ds.sample_buffer[b];
   } else
   {
       ds.state_i_12 -= sample;
       ds.state_i_12 += ds.sample_buffer[b];
   }
   if ((ds.count_12 >= 3) && (ds.count_12 < 9))
   {
       ds.state_q_12 += sample;
       ds.state_q_12 -= ds.sample_buffer[b];
   } else
   {
       ds.state_q_12 -= sample;
       ds.state_q_12 += ds.sample_buffer[b];
   }
   ds.count_12 = (ds.count_12 >= 11) ? 0 : (ds.count_12 + 1);

   /* update 16 count */
   b = (ds.sample_no < df.dly_16) ? (ds.sample_no + df.buffer_size - df.dly_16) : (ds.sample_no - df.dly_16);
   if (ds.count_16 >= 8)
   {
       ds.state_i_16 += sample;
       ds.state_i_16 -= ds.sample_buffer[b];
   } else
   {
       ds.state_i_16 -= sample;
       ds.state_i_16 += ds.sample_buffer[b];
   }
   if ((ds.count_16 >= 4) && (ds.count_16 < 12))
   {
       ds.state_q_16 += sample;
       ds.state_q_16 -= ds.sample_buffer[b];
   } else
   {
       ds.state_q_16 -= sample;
       ds.state_q_16 += ds.sample_buffer[b];
   }
   ds.count_16 = (ds.count_16 >= 15) ? 0 : (ds.count_16 + 1);

   /* update 20 count */
   b = (ds.sample_no < df.dly_20) ? (ds.sample_no + df.buffer_size - df.dly_20) : (ds.sample_no - df.dly_20);
   if (ds.count_20 >= 10)
   {
       ds.state_i_20 += sample;
       ds.state_i_20 -= ds.sample_buffer[b];
   } else
   {
       ds.state_i_20 -= sample;
       ds.state_i_20 += ds.sample_buffer[b];
   }
   if ((ds.count_20 >= 5) && (ds.count_20 < 15))
   {
       ds.state_q_20 += sample;
       ds.state_q_20 -= ds.sample_buffer[b];
   } else
   {
       ds.state_q_20 -= sample;
       ds.state_q_20 += ds.sample_buffer[b];
   }
   ds.count_20 = (ds.count_20 >= 19) ? 0 : (ds.count_20 + 1);

   /* update 24 count */
   b = (ds.sample_no < df.dly_24) ? (ds.sample_no + df.buffer_size - df.dly_24) : (ds.sample_no - df.dly_24);
   if (ds.count_24 >= 12)
   {
       ds.state_i_24 += sample;
       ds.state_i_24 -= ds.sample_buffer[b];
   } else
   {
       ds.state_i_24 -= sample;
       ds.state_i_24 += ds.sample_buffer[b];
   }
   if ((ds.count_24 >= 6) && (ds.count_24 < 18))
   {
       ds.state_q_24 += sample;
       ds.state_q_24 -= ds.sample_buffer[b];
   } else
   {
       ds.state_q_24 -= sample;
       ds.state_q_24 += ds.sample_buffer[b];
   }
   ds.count_24 = (ds.count_24 >= 23) ? 0 : (ds.count_24 + 1);

   ds.sample_buffer[ds.sample_no++] = sample;
   if (ds.sample_no >= df.buffer_size)
      ds.sample_no = 0;

}

#ifdef DSPINT_DEBUG
void test_dsp_sample(void)
{
    dsp_initialize(DSPINT_MAX_SAMPLEBUFFER);
    uint16_t c, samp;
    for (c=0;c<300;c++)
    {
        if (c < 80) samp = 1;
        if ((c >= 80) && (c < 160))
            //samp = (rand() % 1023);
            samp = (((c-80)+14) % 16) >= 8 ? 3 : 1;
        else
            samp = 1;
        dsp_new_sample(samp);

        printf("%05d %05d %05d,%05d %05d,%05d %05d,%05d\n",c,samp,ds.state_i_8,ds.state_q_8,ds.state_i_12,ds.state_q_12,ds.state_i_16,ds.state_q_16);
    }
}

void main(void)
{
  test_dsp_sample();
}
#endif /* DSPINT_DEBUG */
