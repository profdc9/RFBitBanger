
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
protocol_state  ps;

volatile scamp_frame_fifo scamp_input_fifo;
volatile scamp_frame_fifo scamp_output_fifo;

/* initialize frame fifo */
void scamp_initialize_frame_fifo(volatile scamp_frame_fifo *dff)
{
    dff->head = dff->tail = 0;
}

/* insert a frame into the fifo.  this is intended to be interrupt safe */
uint8_t scamp_insert_into_frame_fifo(volatile scamp_frame_fifo *dff, uint32_t frame)
{
    uint8_t next = dff->head >= (SCAMP_FRAME_FIFO_LENGTH - 1) ? 0 : (dff->head+1);
    if (next == dff->tail) return 0;
    dff->frames[next] = frame;
    dff->head = next;
    return 1;
}

/* remove a frame from the fifo.  this is intended to be interrupt safe */
uint32_t scamp_remove_from_frame_fifo(volatile scamp_frame_fifo *dff)
{
    uint32_t frame;
    uint8_t next;
    if (dff->tail == dff->head) return 0xFFFF;
    next = dff->tail >= (SCAMP_FRAME_FIFO_LENGTH - 1) ? 0 : (dff->tail+1);
    frame = dff->frames[next];
    dff->tail = next;
    return frame;
}

/* calculate the hamming weight of a 16 bit integer
   GCC could use __builtin__popcount() */
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

/* calculate the hamming weight a 30 bit number for
   to find hamming distance with sync word */
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

void scamp_reset_codeword(void)
{
   ds.current_bit_no = 0;
   ds.current_word = SCAMP_BLANK_CODEWORD;
   ps.ss.bitflips_in_phase = 0;
   ps.ss.bitflips_lag = 0;
   ps.ss.bitflips_lead = 0;
   ps.ss.bitflips_ctr = 0;
}

/* reset state of buffer in case we get some kind
   of a dc offset mismatch */

void dsp_reset_state(void)
{
   memset(&ds,'\000',sizeof(dsp_state));
}

/* initialize the buffer including the signs to be subtracted
   from the end of the buffer */
void dsp_initialize_scamp(uint8_t mod_type)
{
    memset(&ps.ss,'\000',sizeof(scamp_state));
    ps.ss.protocol = PROTOCOL_SCAMP;
    ps.ss.mod_type = mod_type;
    switch (ps.ss.mod_type)
    {
        case SCAMP_OOK_FAST:  df.buffer_size = 32;
                               ps.ss.demod_edge_window = 3;
                               ps.ss.fsk = 0;
                               break;
        case SCAMP_OOK:       df.buffer_size = 64;
                               ps.ss.fsk = 0;
                               ps.ss.demod_edge_window = 5;
                               break;
        case SCAMP_FSK:       df.buffer_size = 60;
                               ps.ss.fsk = 1;
                               ps.ss.demod_edge_window = 5;
                               break;
        case SCAMP_FSK_FAST:  df.buffer_size = 24;
                               ps.ss.fsk = 1;
                               ps.ss.demod_edge_window = 2;
                               break;
#ifdef SCAMP_VERY_SLOW_MODES
        case SCAMP_OOK_SLOW:  df.buffer_size = 128;
                               ps.ss.demod_edge_window = 5;
                               ps.ss.fsk = 0;
                               break;
        case SCAMP_FSK_SLOW:  df.buffer_size = 120;
                               ps.ss.fsk = 1;
                               ps.ss.demod_edge_window = 5;
                               break;
#endif // SCAMP_VERY_SLOW_MODES
    }
    df.dly_8 = (df.buffer_size / 8) * 8;
    df.dly_12 = (df.buffer_size / 12) * 12;
    df.dly_16 = (df.buffer_size / 16) * 16;
    df.dly_20 = (df.buffer_size / 20) * 20;
    df.dly_24 = (df.buffer_size / 24) * 24;
    ps.ss.demod_samples_per_bit = df.buffer_size / 4;
    ps.ss.power_thr_min = ((uint16_t)df.buffer_size) * SCAMP_PWR_THR_DEF * (ps.ss.fsk ? 2 : 1);
    dsp_reset_state();

    scamp_reset_codeword();
    ps.ss.cur_demod_edge_window =  ps.ss.demod_edge_window;
    ps.ss.power_thr = ps.ss.power_thr_min << 1;
    if (ps.ss.fsk)
       ps.ss.edge_thr = ps.ss.power_thr;
    else
       ps.ss.edge_thr = ps.ss.power_thr << 1;
}

/* initialize DSP for CW mode */
void dsp_initialize_cw(uint8_t wide)
{
    df.buffer_size = 24;
    df.dly_8 = 16;
    df.dly_12 = wide ? 12 : 24;
    df.dly_16 = 16;
    df.dly_20 = 20;
    df.dly_24 = 24;
    dsp_reset_state();
}

/* this is an approximation to the sqrt(x^2+y^2) function that approximates the
   circular set as an octagon.  seems to work quite well */
#define SET_DSP_SQRT_APPROX(s,x,y) do { \
   uint16_t ux = (x < 0 ? -x : x); \
   uint16_t uy = (y < 0 ? -y : y); \
   s = ((ux + uy) >> 1)+(uy > ux ? uy : ux); \
} while(0)

/* called by the interrupt routine to update the spectral channels */
/* this is used to update the I & Q of each spectral channel and update
   the magnitude of the signal on each channel.  more channels are used (5)
   than needed (1 or 2) because this will help the receiver to track the
   frequency of the channels */
void dsp_interrupt_sample(uint16_t sample)
{
   uint8_t b, prep_sample;
   int16_t fir;

   /* super slow mode.  used for experimentation with very low baud rates */
   if (ds.slow_samp_num > 1)
   {
       ds.total_num += sample;
       if ((++ds.slow_samp) >= ds.slow_samp_num)
       {
           sample = ds.total_num;
           ds.slow_samp = 0;
           ds.total_num = 0;
       } else
           return;
   }

   /* we do this so that the square roots are computed in the interrupt cycle
      before they are used to reduce the worst-case time for the interrupt
      execution */

   prep_sample = (ds.count_8 & 0x03) == 0;

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
   if (prep_sample)
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
   if (prep_sample)
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
   if (prep_sample)
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
   if (prep_sample)
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
   if (prep_sample)
        SET_DSP_SQRT_APPROX(ds.mag_value_24, ds.state_q_24, ds.state_i_24);

   /* store in circular buffer so that is can be subtracted from the end
      to make a moving average filter */
   ds.sample_buffer[ds.sample_no++] = sample;
   if (ds.sample_no >= df.buffer_size)
      ds.sample_no = 0;

   if (prep_sample) ds.sample_ct++;
}

/* this is called by the interrupt handle with a new sample from the DAC.
   first it updates the spectral channels.  then it figures out the magnitude of
   the signal given the current modulation type (OOK/FSK).  it tries to detect
   an edge to determine when the current bit has finished and when it should
   expect the next bit.  it resets the bit counter when the sync signal has
   been received, and when 30 bits of a frame have been received, stores the
   frame in the frame FIFO */
void scamp_decode_process(void)
{
    int16_t demod_sample, temp;
    uint8_t received_bit;
    uint8_t hamming_weight;

    uint8_t ct = ds.sample_ct;
    /* only proceed if we have new magnitude samples */
    if (ct == ps.ss.last_sample_ct)
        return;
    ps.ss.last_sample_ct = ct;

    /* demodulate a sample either based on the amplitude in one frequency
       channel for OOK, or the difference in amplitude between two frequency
       channels for FSK */

    switch (ps.ss.mod_type)
    {
        case SCAMP_OOK_SLOW:
        case SCAMP_OOK_FAST:
        case SCAMP_OOK:       demod_sample = ds.mag_value_16 - ps.ss.power_thr;
                               ds.ct_sum += ds.mag_value_16;
                               break;
        case SCAMP_FSK_SLOW:
        case SCAMP_FSK:       demod_sample = ds.mag_value_20 - ds.mag_value_12;
                               ds.ct_sum += (ds.mag_value_20 + ds.mag_value_12);
                               break;
        case SCAMP_FSK_FAST:  demod_sample = ds.mag_value_12 - ds.mag_value_8;
                               ds.ct_sum += (ds.mag_value_12 + ds.mag_value_8);
                               break;
    }

    /* This is the automatic "gain" control (threshold level control).
       find the average of a certain number of samples (power of two for calculation
       speed) so that we can determine what to set the thresholds at for the bit on/off
       threshold for ook and the edge threshold for ook/fsk */
    if ((++ds.ct_average) >= (1 << SCAMP_AVG_CT_PWR2))
    {
       uint16_t temp;
       temp = (ds.ct_sum) >> (SCAMP_AVG_CT_PWR2);
       /* don't allow threshold to get too low, or we'll be having bit edges constantly */
       ps.ss.power_thr = temp > ps.ss.power_thr_min ? temp : ps.ss.power_thr_min;
       if (ps.ss.fsk)
            ps.ss.edge_thr = ps.ss.power_thr;
       else
            ps.ss.edge_thr = ps.ss.power_thr << 1;
       printf("power %d edge %d min %d ---------------------------------\n",ps.ss.power_thr,ps.ss.edge_thr,ps.ss.power_thr_min);
       ds.ct_average = 0;
       ds.ct_sum = 0;
    }

    /* calculate the difference between the modulated signal between now and one bit period ago to see
       if there is a bit edge */
    temp = ps.ss.demod_buffer[ps.ss.demod_sample_no];
    ps.ss.bit_edge_val = demod_sample > temp ? (demod_sample - temp) : (temp - demod_sample);
    /* store the demodulated sample into a circular buffer to calculate the edge */
    ps.ss.demod_buffer[ps.ss.demod_sample_no] = demod_sample;
    if ((++ps.ss.demod_sample_no) >= ps.ss.demod_samples_per_bit)
        ps.ss.demod_sample_no = 0;

    if (ps.ss.edge_ctr > 0)  /* count down the edge counter */
        --ps.ss.edge_ctr;

    received_bit = 0;

    if (ps.ss.edge_ctr < ps.ss.cur_demod_edge_window)  /* if it below the edge window, start looking for the bit edge */
    {
        if (ps.ss.bit_edge_val > ps.ss.edge_thr)  /* the edge should come around now, does it exceed threshold */
        {
            if (ps.ss.bit_edge_val > ps.ss.max_bit_edge_val)  /* if so, have we reached the peak of the edge */
            {
                ps.ss.max_bit_edge_val = ps.ss.bit_edge_val;  /* if so, reset the edge center counter */
                ps.ss.next_edge_ctr = 1;
                ps.ss.cur_bit = demod_sample;              /* save the bit occurring at the edge */
            } else
                ps.ss.next_edge_ctr++;                     /* otherwise count that we have passed the edge peak */
        } else
        {
            if (ps.ss.max_bit_edge_val != 0)   /* if we have detected an edge */
            {
                ps.ss.edge_ctr = ps.ss.demod_samples_per_bit > ps.ss.next_edge_ctr ? ps.ss.demod_samples_per_bit - ps.ss.next_edge_ctr : 0;
                                /* reset edge ctr to look for next edge */
                ps.ss.max_bit_edge_val = 0;    /* reset max_bit_edge_val for next edge peak */
                received_bit = 1;
                ps.ss.cur_demod_edge_window = ps.ss.demod_edge_window;
            } else /* we haven't detected an edge */
            {
                if (ps.ss.edge_ctr == 0)
                {
                    ps.ss.cur_bit = demod_sample;              /* save the bit */
                    ps.ss.edge_ctr = ps.ss.demod_samples_per_bit;  /* reset and wait for the next bit edge to come along */
                    received_bit = 1;                       /* an edge hasn't been detected but a bit interval happened */
                }
            }
        }
    }
    if (!received_bit)   /* no bit available yet, wait */
        return;

    /* add the bit to the current word */
    ds.current_word = (ds.current_word << 1) | (ps.ss.polarity ^ (ps.ss.cur_bit > 0));
    ps.ss.bitflips_ctr++;
    /* this is done on the bit before it is needed to reduce worst case latency */
    if (ps.ss.bitflips_ctr == 4)
    {
        /* keep track of the number of complement bits in the word. should be 6 */
        uint8_t maskbits, lowerbyte = ((uint8_t)ds.current_word);
        maskbits = lowerbyte & 0x0C;
        ps.ss.bitflips_in_phase += (maskbits == 0x08) || (maskbits == 0x04);
        maskbits = lowerbyte & 0x18;
        ps.ss.bitflips_lag += (maskbits == 0x10) || (maskbits == 0x08);
        maskbits = lowerbyte & 0x06;
        ps.ss.bitflips_lead += (maskbits == 0x04) || (maskbits == 0x02);
    } else if (ps.ss.bitflips_ctr >= 5)
        ps.ss.bitflips_ctr = 0;
    hamming_weight = dsp_hamming_weight_30(ds.current_word ^ SCAMP_SYNC_CODEWORD);
    printf("received: %08X %05d %02d %02d %02d %02d %02d %02d\n", ds.current_word, ps.ss.cur_bit, ds.current_bit_no, hamming_weight, ps.ss.polarity, ps.ss.bitflips_in_phase, ps.ss.bitflips_lag, ps.ss.bitflips_lead);
    if (hamming_weight < (ps.ss.resync ? 4 : 8))  /* 30-bit resync word has occurred! */
    {
        printf("resync!\n");
        scamp_reset_codeword();
        ps.ss.resync = 1;
        return;
    }
    /* if we 15 of the last 16 bits zeros with fsk, that means we have a reversed polarity start */
    hamming_weight = dsp_hamming_weight_16(ds.current_word);
    if ((hamming_weight < 2) && (ps.ss.fsk))
    {
        ps.ss.cur_demod_edge_window = ps.ss.demod_samples_per_bit;
        ps.ss.polarity = !ps.ss.polarity; /* reverse the polarity of FSK frequencies and 0/1 */
        scamp_reset_codeword();
        ps.ss.resync = 0;
        return;
    }
    /* if we have 15 of the last 16 bits ones, that is a ook key down start, or a fsk start */
    if (hamming_weight >= 15)
    {
        ps.ss.cur_demod_edge_window = ps.ss.demod_samples_per_bit;
        scamp_reset_codeword();
        ps.ss.resync = 0;
        return;
    }
    /* if we have synced, and we have 30 bits, we have a frame */
    ds.current_bit_no++;
    if ((ds.current_bit_no >= 30) && (ps.ss.resync))  /* we have a complete frame */
    {
       if ((ps.ss.bitflips_lead > ps.ss.bitflips_in_phase) && (ps.ss.bitflips_lead >= 5))
        /* we are at least one bit flip ahead, we probably registered a spurious bit flip */
       {
         /* back up and get one more bit.*/
         ds.current_bit_no--;
         ps.ss.bitflips_in_phase = ps.ss.bitflips_lead;  /*lead now becomes in phase */
         ps.ss.bitflips_lag = 0;  /* lag is now two behind, so we can't use it */
         ps.ss.bitflips_lead = 0; /* clear bit_lead so we don't try a second time */
       } else
       {
          if ((ps.ss.bitflips_lag > ps.ss.bitflips_in_phase) && (ps.ss.bitflips_lag >= 5))
          /* we are at least one bit flip short, we probably fell a bit behind */
          {
            scamp_insert_into_frame_fifo(&scamp_output_fifo, ds.current_word >> 1);
            printf("inserted- word into fifo: %08X %02d %02d %02d -----------------------------\n",
                   ds.current_word >> 1, ps.ss.bitflips_in_phase, ps.ss.bitflips_lag, ps.ss.bitflips_lead);
            /* start with the next word with one flip */
            ds.current_bit_no = 1;
            ps.ss.bitflips_ctr = 1;
         } else
         {
             if (ps.ss.bitflips_in_phase >= 4)
             {
               /* otherwise we just place in buffer and the code word is probably aligned */
               scamp_insert_into_frame_fifo(&scamp_output_fifo, ds.current_word);
               printf("inserted0 word into fifo: %08X %02d %02d %02d -----------------------------\n",
                   ds.current_word, ps.ss.bitflips_in_phase, ps.ss.bitflips_lag, ps.ss.bitflips_lead);
             }
             ds.current_bit_no = 0;
             ps.ss.bitflips_ctr = 0;
          }
          ps.ss.bitflips_in_phase = 0;
          ps.ss.bitflips_lag = 0;
          ps.ss.bitflips_lead = 0;
       }
    }
}

#include "cwmod.c"

#ifdef DSPINT_DEBUG

typedef struct _wavefile_header
{
    unsigned char header[4];
    uint32_t file_size;
    unsigned char ftype[4];
    unsigned char fmt[4];
    uint32_t length;
    uint16_t type;
    uint16_t channels;
    uint32_t samplerate;
    uint32_t ratetotal;
    uint16_t bitspersamplechannels8;
    uint16_t bitspersample;
    unsigned char dataheader[4];
    uint32_t datasize;
} wavefile_header;

FILE *write_wav_file(const char *filename, uint32_t samples, uint32_t repeats)
{
    wavefile_header w;
    w.header[0]='R';w.header[1]='I';w.header[2]='F';w.header[3]='F';
    w.ftype[0]='W';w.ftype[1]='A';w.ftype[2]='V';w.ftype[3]='E';
    w.fmt[0]='f';w.fmt[1]='m';w.fmt[2]='t';w.fmt[3]=' ';
    w.length = 16;
    w.type = 1;
    w.channels = 1;
    w.samplerate = 8000;
    w.bitspersample = 16;
    w.bitspersamplechannels8 = w.bitspersample * w.channels / 8;
    w.ratetotal = w.samplerate * w.channels * w.bitspersample / 8;
    w.dataheader[0]='d';w.dataheader[1]='a';w.dataheader[2]='t';w.dataheader[3]='a';
    w.datasize = samples * repeats * w.bitspersamplechannels8;
    w.file_size = w.datasize + sizeof(wavefile_header);
    printf("headersize=%d file_size=%d datasize=%d\n",sizeof(wavefile_header),w.file_size,w.datasize);
    FILE *fp = fopen(filename,"wb");
    if (fp != NULL)
        fwrite(&w,sizeof(w),1,fp);
    return fp;
}

void write_sample(FILE *fp, uint16_t sample, uint16_t repeats)
{
  int i;
  for (i=0;i<repeats;i++)
    fwrite(&sample,sizeof(sample),1,fp);
}

float gaussian_deviate(float stddev)
{
    float x = (rand()+1) / 32768.0f;
    float y = (rand()+1) / 32768.0f;
    return stddev*sqrt(-2*log(x))*cos(2.0*M_PI*y);
}

#define MOD_TEST 1

#if MOD_TEST==2
#define MOD_TYPE SCAMP_OOK_FAST
#define MOD_REP 32
#define MOD_CHAN1 16.0
#define MOD_CHAN2 99999999999.0
#elif MOD_TEST==1
#define MOD_TYPE SCAMP_OOK
#define MOD_REP 64
#define MOD_CHAN1 16.0
#define MOD_CHAN2 99999999999.0
#elif MOD_TEST==2
#define MOD_TYPE SCAMP_FSK
#define MOD_REP 60
#define MOD_CHAN1 12.0
#define MOD_CHAN2 20.0
#elif MOD_TEST==3
#define MOD_TYPE SCAMP_FSK_FAST
#define MOD_REP 24
#define MOD_CHAN1 8.0
#define MOD_CHAN2 12.0
#elif MOD_TEST==4
#define MOD_TYPE SCAMP_OOK_SLOW
#define MOD_REP 128
#define MOD_CHAN1 16.0
#define MOD_CHAN2 9999999999.0
#elif MOD_TEST==5
#define MOD_TYPE SCAMP_FSK_SLOW
#define MOD_REP 120
#define MOD_CHAN1 12.0
#define MOD_CHAN2 20.0
#endif

void test_dsp_sample(void)
{
    uint8_t cbit;
    uint32_t c;
    float freq, samp;
    srand(2001);
    dsp_initialize_scamp(MOD_TYPE);
    uint32_t samples;
    uint32_t repeats = 4;
    const uint8_t bits[] = {    1,1,1,1,1, 1,1,1,1,1, 1,1,1,1,1, 1,1,1,1,1, 1,1,1,1,0, 0,0,0,1,1,  /* 30 bits */
                                1,1,1,1,1, 0,1,1,0,1, 0,0,0,1,1, 0,0,1,1,1, 0,1,0,0,0, 1,1,1,1,0,  /* 30 bits */
                                1,0,0,0,0, 0,1,0,0,0, 0,1,0,0,1, 1,0,1,1,0, 1,0,0,0,0, 0,1,0,1,0,  /* 30 bits */
                                1,0,1,1,0, 0,1,1,1,0, 0,1,1,1,0, 0,1,1,1,0, 1,0,1,1,0, 1,0,1,1,0,  /* 30 bits */
                                0,1,1,0,0, 1,0,0,0,0, 1,0,1,0,0, 0,1,0,0,0, 1,0,0,1,0, 1,0,0,0,0,  /* 30 bits */
//                              1,1,1,1,0, 1,1,1,1,0, 1,1,1,1,0, 1,1,1,1,0, 1,1,1,1,0, 1,1,1,1,0,  /* 30 bits */
                                0,1,0,0,0, 0,1,0,0,0, 1,0,0,1,0, 0,1,1,0,0, 0,1,1,0,0, 1,0,0,1,1,  /* 30 bits */
//                              1,1,1,1,1, 0,1,1,0,1, 0,0,0,1,1, 0,0,1,1,1, 0,1,0,0,0, 1,1,1,1,0,  /* 30 bits */
                                1,0,0,0,0, 1,0,0,0,0, 1,0,0,0,0, 1,0,0,0,0, 1,0,0,0,0, 1,0,0,0,0,  /* 30 bits */
                                0,1,1,0,0, 1,0,0,0,0, 1,0,1,0,0, 1,0,0,0,0, 1,0,0,1,0, 1,0,0,0,0,  /* 30 bits */
                                1,0,0,0,0, 0,1,0,0,0, 1,0,0,0,0, 0,1,1,0,0, 1,0,0,0,0, 0,1,0,0,1,  /* 30 bits */
                                1,0,1,1,0, 0,1,1,1,0, 0,1,1,1,0, 0,1,1,1,0, 1,0,1,1,0, 1,0,1,1,0,  /* 30 bits */
                                1,0,0,0,0, 0,1,0,0,0, 1,0,0,0,0, 0,1,1,0,0, 1,0,0,0,0, 0,1,0,0,1,  /* 30 bits */
                                0,1,1,0,0, 1,0,0,0,0, 1,0,1,0,0, 1,0,0,0,0, 1,0,0,1,0, 1,0,0,0,0,  /* 30 bits */
                                1,0,1,1,1, 1,0,1,1,1 };
    samples = sizeof(bits)*MOD_REP;
    FILE *fp = write_wav_file("synth.wav",samples,repeats);
    for (c=0;c<samples;c++)
    {
        uint8_t last_sample_ct = 0;
        cbit = bits[(c+7)/MOD_REP];
        freq = cbit ? MOD_CHAN1 : MOD_CHAN2;
        samp = ((sin(2.0*M_PI*c/freq+1.0*M_PI)*64.0)+512.0) + gaussian_deviate(64.0);
      //  if ((c>4000) && (c<6000)) samp = gaussian_deviate(48.0)+512;
        dsp_interrupt_sample(samp);
        scamp_decode_process();
        write_sample(fp,samp*16,repeats);

        if ((ds.sample_ct != last_sample_ct) && 0)
        {
            last_sample_ct = ds.sample_ct;
            printf("%d %05d %05d %05d %05d %05d %05d %05d %05d %02d %02d xx\n",cbit,c,(int)samp,
                ds.mag_value_8,ds.mag_value_12,ds.mag_value_16,ds.mag_value_20,ds.mag_value_24,
                ps.ss.bit_edge_val,ps.ss.edge_ctr,ps.ss.cur_demod_edge_window);
        }
    }
    if (fp != NULL) fclose(fp);
}

void main(void)
{
  //test_dsp_sample();
  test_cwmod_decode();
  //printf("size=%d %d %d\n",sizeof(ds)+sizeof(df)+sizeof(scamp_input_fifo)+sizeof(scamp_output_fifo),ps.ss.fsk,df.buffer_size);
}
#endif /* DSPINT_DEBUG */
