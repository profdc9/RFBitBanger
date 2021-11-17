
/*  cwmod.c */

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


#define CWMOD_DEBUG

#ifdef CWMOD_DEBUG
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#endif

#include "dspint.h"
#include "cwmod.h"

cwmod_state       cs;
cwmod_state_fixed cf;

const uint8_t cwmod_bit_mask[8] = {0x00, 0x01, 0x03, 0x07, 0x0F, 0x1F, 0x3F, 0x7F };

/*  This is formula round(10 * 3^(n/4))  n=1 to 16 */
const uint16_t cwmod_timing_histogram_bins[CWMOD_TIMING_BINS] =
{  13, 17, 23, 30, 39, 52, 68, 90, 118, 156, 205, 270, 355, 468, 615, 810 };


/* FROM ITU-R M.1677-1 */
/* zero dit, 1 dah */
const cwmod_symbol morse_pattern[] =
{
//    { 0b010101,   6, '.' },
//    { 0b110011,   6, ',' },
//    { 0b111000,   6, ':' },
//    { 0b001100,   6, '?' },
//    { 0b011110,   6, '\'' },
//    { 0b100001,   6, '-' },
//    { 0b101101,   6, ']' },
//    { 0b010010,   6, '\"' },
    { 0b01111,    5, '1' },
    { 0b00111,    5, '2' },
    { 0b00011,    5, '3' },
    { 0b00001,    5, '4' },
    { 0b00000,    5, '5' },
    { 0b10000,    5, '6' },
    { 0b11000,    5, '7' },
    { 0b11100,    5, '8' },
    { 0b11110,    5, '9' },
    { 0b11111,    5, '0' },
//    { 0b10010,    5, '/' },
//    { 0b10110,    5, '[' },
//    { 0b10001,    5, '=' },
//    { 0b00010,    5, '!' },
//    { 0b01010,    5, '+' },
    { 0b1000,     4, 'B' },
    { 0b1010,     4, 'C' },
    { 0b0010,     4, 'F' },
    { 0b0000,     4, 'H' },
    { 0b0111,     4, 'J' },
    { 0b0100,     4, 'L' },
    { 0b0110,     4, 'P' },
    { 0b1101,     4, 'Q' },
    { 0b0001,     4, 'V' },
    { 0b1001,     4, 'X' },
    { 0b1011,     4, 'Y' },
    { 0b1100,     4, 'Z' },
    { 0b100,      3, 'D' },
    { 0b110,      3, 'G' },
    { 0b101,      3, 'K' },
    { 0b111,      3, 'O' },
    { 0b010,      3, 'R' },
    { 0b000,      3, 'S' },
    { 0b001,      3, 'U' },
    { 0b011,      3, 'W' },
    { 0b01,       2, 'A' },
    { 0b00,       2, 'I' },
    { 0b11,       2, 'M' },
    { 0b10,       2, 'N' },
    { 0b0,        1, 'E' },
    { 0b1,        1, 'T' }
};

/* insert a timing the fifo.  this is intended to be interrupt safe */
uint8_t cw_insert_into_timing_fifo(uint16_t tim)
{
    uint8_t next = cs.timing_head >= (CWMOD_TIMING_LENGTHS - 1) ? 0 : (cs.timing_head+1);
    if (next == cs.timing_tail) return 0;
    cs.timing_lengths[next] = tim;
    cs.timing_head = next;
    return 1;
}

uint8_t cw_fifo_available(void)
{
    uint8_t head = cs.timing_head;
    return cs.timing_tail > head ? cs.timing_tail - head : cs.timing_tail - head + CWMOD_TIMING_LENGTHS;
}

/* remove a timing from the fifo.  this is intended to be interrupt safe */
uint16_t cw_remove_from_timing_fifo(void)
{
    uint16_t tim;
    uint8_t next;
    if (cs.timing_tail == cs.timing_head) return 0;
    next = cs.timing_tail >= (CWMOD_TIMING_LENGTHS - 1) ? 0 : (cs.timing_tail+1);
    tim = cs.timing_lengths[next];
    cs.timing_tail = next;
    return tim;
}

/* peek from the timing fifo.  this is intended to be interrupt safe */
uint16_t cw_peek_from_timing_fifo(void)
{
    uint16_t tim;
    uint8_t next;
    if (cs.timing_peek_tail == cs.timing_head) return 0;
    next = cs.timing_peek_tail >= (CWMOD_TIMING_LENGTHS - 1) ? 0 : (cs.timing_peek_tail+1);
    tim = cs.timing_lengths[next];
    cs.timing_peek_tail = next;
    return tim;
}

void cw_initialize(void)
{
   dsp_initialize_cw();
   ds.slow_samp_num = 4;
   memset(&cs,'\000',sizeof(cs));
   memset(&cf,'\000',sizeof(cf));

   cs.keydown_threshold = CWMOD_THRESHOLD_MIN;
   cs.keyup_threshold = CWMOD_THRESHOLD_MIN >> 1;
}

void cw_reset_timing(uint16_t tim)
{
   tim = (tim < CWMOD_THRESHOLD_MIN) ? CWMOD_THRESHOLD_MIN : tim;
   cs.keydown_threshold = tim;
   cs.keyup_threshold = tim >> 1;
   cs.ct_average = 0;
   cs.ct_sum = 0;
}

void cw_interrupt_sample(uint16_t sample)
{
    uint16_t mag_sample, c1, ctr;
    int16_t edge_val;
    /* update the filter channels I & Q */
    dsp_new_sample(sample);

    /* only proceed if we have new magnitude samples */
    if (!ds.mag_new_sample)
        return;

    cs.total_ticks++;  /* increment the total ticks counter */

    mag_sample = ds.mag_value_12;

    cs.ct_sum += mag_sample;
    if (cs.keydown_threshold <= CWMOD_THRESHOLD_MIN)
    {
       if ((++cs.ct_average) >= (1 << (CWMOD_AVG_CT_PWR2-2)))
          cw_reset_timing( (cs.ct_sum) >> (CWMOD_AVG_CT_PWR2-1) );
    } else
    {
       if ((++cs.ct_average) >= (1 << (CWMOD_AVG_CT_PWR2)))
          cw_reset_timing( (cs.ct_sum) >> (CWMOD_AVG_CT_PWR2+1) );
    }
    cs.state_ctr++;
    if (cs.key_state)
    {
       if (mag_sample < cs.keyup_threshold)
       {
          if ((++cs.sticky_interval) >= CWMOD_STICKY_INTERVAL)
          {
            cs.key_state = 0;
            //printf("key up %d %d\n",cs.state_ctr, mag_sample);
            cw_insert_into_timing_fifo(cs.state_ctr);
            cs.state_ctr = 0;
            cs.sticky_interval = 0;
          }
       } else cs.sticky_interval = 0;
    } else
    {
       if (mag_sample > cs.keydown_threshold)
       {
          if ((++cs.sticky_interval) >= CWMOD_STICKY_INTERVAL)
          {
            cs.key_state = 1;
            //printf("key down %d %d\n",cs.state_ctr, mag_sample);
            cw_insert_into_timing_fifo(((uint16_t)cs.state_ctr) | 0x8000);
            cs.state_ctr = 0;
            cs.sticky_interval = 0;
            cs.keyup_threshold = mag_sample >> 1;
          }
       } else cs.sticky_interval = 0;
    }
}

void cw_find_two_greatest(uint8_t array[], uint8_t length, uint8_t sep,
                          uint8_t *e1, uint8_t *e2)
{
    uint8_t gr_val, i, gr, gr2;

    gr_val = 0;
    for (i=1;i<length;i++)
    {
        uint8_t val = array[i-1] + array[i];
        if (val > gr_val)
        {
            gr_val = val;
            gr = i;
        }
    }
    *e1 = gr;

    gr_val = 0;
    for (i=1;i<length;i++)
    {
        if ((i > (gr+sep)) || ((sep+i) < gr))
        {
            uint8_t val = array[i-1] + array[i];
            if (val > gr_val)
            {
                gr_val = val;
                gr2 = i;
            }
        }
    }
    *e2 = gr2;
}

void cw_decode_process(void)
{
    uint8_t is_mark, bin, large_bin, decode_now;
    uint16_t dly, tim = cw_peek_from_timing_fifo();
    is_mark = (tim & 0x8000) == 0;
    tim = tim & 0x7FFF;

    dly = cs.total_ticks - cs.last_tick;
    /* assume a really short timing is a glitch */
    decode_now = (dly >= CWMOD_PROCESS_DELAY) || (cw_fifo_available() < CWMOD_FIFO_DECODE_THRESHOLD);
    if ((!decode_now) && (tim < 10)) return;
    cs.last_tick = cs.total_ticks;

    if (tim > 0)
    {
        for (bin=0; bin < CWMOD_TIMING_BINS; bin++)   /* search for histogram bin */
            if (tim < cwmod_timing_histogram_bins[bin]) break;
        if (bin < CWMOD_TIMING_BINS)
        {
            if (is_mark) cs.histogram_marks[bin]++;
                else cs.histogram_spaces[bin]++;
        }
    }

    if (!decode_now) return;

#if 0
    printf("mark bin:  ");
    for (bin=0; bin<(CWMOD_TIMING_BINS); bin++)
        printf("%02d,",cs.histogram_marks[bin]);
    printf("\n");


    printf("space bin: ");
    for (bin=0; bin<(CWMOD_TIMING_BINS); bin++)
        printf("%02d,",cs.histogram_spaces[bin]);
    printf("\n");
#endif


    uint8_t g1, g2;

    cw_find_two_greatest(cs.histogram_marks, sizeof(cs.histogram_marks)/sizeof(cs.histogram_marks[0]),
                        2, &g1, &g2);

    // printf(" lm: %d/%d %d/%d ",g1,cs.histogram_marks[g1],g2,cs.histogram_marks[g2]);

    cs.dit_dah_threshold = cwmod_timing_histogram_bins[(g1+g2)/2];

    cw_find_two_greatest(cs.histogram_spaces, sizeof(cs.histogram_spaces)/sizeof(cs.histogram_spaces[0]),
                         2, &g1, &g2);

    cs.intrainterspace_threshold = cwmod_timing_histogram_bins[(g1+g2)/2];
    cs.interspaceword_threshold = 3*cs.intrainterspace_threshold;


    // printf("ls: %d/%d %d/%d\n",g1,cs.histogram_spaces[g1],g2,cs.histogram_spaces[g2]);

    //  printf("x: %d %d %d\n",cs.dit_dah_threshold,cs.intrainterspace_threshold,cs.interspaceword_threshold);

    for (;;)
    {
        tim = cw_remove_from_timing_fifo();
        if (tim == 0) break;
        is_mark = (tim & 0x8000) == 0;
        tim = tim & 0x7FFF;
        if (is_mark)
        {
            cs.num_ditdahs++;
            cs.cur_ditdahs <<= 1;
            if (tim > cs.dit_dah_threshold)
                cs.cur_ditdahs |= 1;
      //      printf("%c",tim > cs.dit_dah_threshold ? '-' : '.');
        } else
        {
            if (tim > cs.intrainterspace_threshold)
            {
               const cwmod_symbol *cws = &morse_pattern[0];
               while (cws < (&morse_pattern[(sizeof(morse_pattern)/sizeof(cwmod_symbol))]))
               {
                   int8_t diff = cs.num_ditdahs - cws->num;
                   if (diff >= 0)
                   {
                       uint8_t test = cs.cur_ditdahs >> diff;
                       if (cws->cwbits == test)
                       {
                            printf("%c",cws->symbol);
                            break;
                       }
                   }
                   cws++;
               }
               cs.num_ditdahs = 0;
               cs.cur_ditdahs = 0;
               if (tim > cs.interspaceword_threshold)
                   printf(" ");
            }
        }
    }
    // printf("!!!\n");
    for (g1=0;g1<(sizeof(cs.histogram_marks)/sizeof(cs.histogram_marks[0]));g1++)
        cs.histogram_marks[g1] >>= 1;
    for (g1=0;g1<(sizeof(cs.histogram_spaces)/sizeof(cs.histogram_spaces[0]));g1++)
        cs.histogram_spaces[g1] >>= 1;
}

#ifdef CWMOD_DEBUG

//const char filename[]="d:\\projects\\RFBitBanger\\Ignore\\processed-cw\\kw4ti-msg.wav";
//const char filename[]="d:\\projects\\RFBitBanger\\Ignore\\processed-cw\\wnu-processed.wav";
//const char filename[]="d:\\projects\\RFBitBanger\\Ignore\\processed-cw\\n1ea-processed.wav";
//const char filename[]="d:\\projects\\RFBitBanger\\Ignore\\processed-cw\\vix-processed.wav";
//const char filename[]="d:\\projects\\RFBitBanger\\Ignore\\processed-cw\\px-processed.wav";
//const char filename[]="d:\\projects\\RFBitBanger\\Ignore\\processed-cw\\kfs-processed.wav";
//const char filename[]="d:\\projects\\RFBitBanger\\Ignore\\processed-cw\\cootie-processed.wav";
const char filename[]="d:\\projects\\RFBitBanger\\Ignore\\processed-cw\\wxdewcc-processed.wav";

void test_cwmod_decode()
{
   FILE *fp = fopen(filename,"rb");
   cw_initialize();
   int samplecount = 0;

   if (fp == NULL)
   {
       printf("Could not open file %s\n",filename);
       return;
   }

   fseek(fp,44*sizeof(uint8_t),SEEK_SET);
   while (!feof(fp))
   {
       int16_t sample;
       fread((void *)&sample,1,sizeof(int16_t),fp);
       sample = sample / 64 + 512;
//       if ((sample<200) || (sample > 800)) printf("sample=%d\n",sample);
       /* if ((samplecount/1024) & 0x1) sample = 512;
        else
       sample = 512 - 64 * cos(2*M_PI*samplecount/64.0); */
       cw_interrupt_sample(sample);
       cw_decode_process();
       samplecount++;
   }
   for (samplecount=0;samplecount<360000;samplecount++)
   {
       cw_interrupt_sample(512);
       cw_decode_process();
   }
}
#endif /* CWMOD_DEBUG */
