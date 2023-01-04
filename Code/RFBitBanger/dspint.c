
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


#undef DSPINT_DEBUG

#ifdef ARDUINO
#include <Arduino.h>
#endif

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

uint8_t decode_insert_into_fifo(uint8_t c)
{
  received_character(c);
}

#if 0
decode_fifo decfifo;

/* initialize decode fifo */
void decode_initialize_fifo(void)
{
    decfifo.head = decfifo.tail = 0;
}

/* insert a character into the decode fifo.  this is intended to be interrupt safe */
uint8_t decode_insert_into_fifo(uint8_t c)
{
    uint8_t next = decfifo.head >= (DECODE_FIFO_LENGTH - 1) ? 0 : (decfifo.head+1);
    if (next == decfifo.tail) return 0;
    decfifo.decodes[next] = c;
    decfifo.head = next;;
    return 1;
}

/* remove a character from the decode fifo.  this is intended to be interrupt safe */
uint16_t decode_remove_from_fifo(void)
{
    uint8_t c, next;
    if (decfifo.tail == decfifo.head) return 0xFFFF;
    next = decfifo.tail >= (DECODE_FIFO_LENGTH - 1) ? 0 : (decfifo.tail+1);
    c = decfifo.decodes[next];
    decfifo.tail = next;
    return c;
}
#endif


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

void dsp_reset_fixed_state(void)
{
  cli();
  memset(&df,'\000',sizeof(dsp_state_fixed));
  memset(&ps,'\000',sizeof(protocol_state));
  sei();  
}

/* reset state of buffer in case we get some kind
   of a dc offset mismatch */

void dsp_reset_state(void)
{
  cli();
  memset(&ds,'\000',sizeof(dsp_state));
  sei();
}

/* initialize the buffer including the signs to be subtracted
   from the end of the buffer */
void dsp_initialize_fastscan(void)
{
    dsp_reset_fixed_state();
    ps.ss.protocol = PROTOCOL_FASTSCAN;
    df.buffer_size = 24;
    df.dly_8 = 8;
    df.dly_12 = 12;
    df.dly_16 = 16;
    df.dly_20 = 20;
    df.dly_24 = 24; 
    dsp_reset_state();
}

/* initialize the buffer including the signs to be subtracted
   from the end of the buffer */
void dsp_initialize_scamp(uint8_t mod_type)
{
    dsp_reset_fixed_state();
    ps.ss.protocol = PROTOCOL_SCAMP;
    ps.ss.mod_type = mod_type;
    switch (ps.ss.mod_type)
    {
        case SCAMP_OOK_FAST:   df.buffer_size = 32;
                               ps.ss.demod_edge_window = 3;
                               ps.ss.fsk = 0;
                               df.dly_16 = 32;
                               break;
        case SCAMP_OOK:        df.buffer_size = 64;
                               ps.ss.fsk = 0;
                               ps.ss.demod_edge_window = 4;
                               df.dly_16 = 64;
                               break;
        case SCAMP_FSK:        df.buffer_size = 60;
                               ps.ss.fsk = 1;
                               ps.ss.demod_edge_window = 4;
                               df.dly_12 = 60;
                               df.dly_20 = 60;
                               break;
        case SCAMP_FSK_FAST:   df.buffer_size = 24;
                               ps.ss.fsk = 1;
                               ps.ss.demod_edge_window = 2;
                               df.dly_8 = 24;
                               df.dly_24 = 24;
                               break;
#ifdef SCAMP_VERY_SLOW_MODES
        case SCAMP_OOK_SLOW:  df.buffer_size = 128;
                               ps.ss.demod_edge_window = 4;
                               ps.ss.fsk = 0;
                               df.dly_16 = 64;
                               break;
        case SCAMP_FSK_SLOW:  df.buffer_size = 120;
                               ps.ss.fsk = 1;
                               ps.ss.demod_edge_window = 4;
                               df.dly_12 = 60;
                               df.dly_20 = 60;
                               break;
#endif // SCAMP_VERY_SLOW_MODES
    }
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
    dsp_reset_fixed_state();
    ps.cs.protocol = PROTOCOL_CW;
    df.buffer_size = 48;
    df.dly_12 = wide ? 12 : 24;
    dsp_reset_state();
}

void dsp_initialize_rtty(void)
{
    dsp_reset_fixed_state();
    ps.rs.protocol = PROTOCOL_RTTY;
    df.buffer_size = 48;
    df.dly_8 = 48;
    df.dly_24 = 48;
    dsp_reset_state();
}

void dsp_initialize_protocol(uint8_t protocol)
{
  switch (protocol)
  {
     case PROTOCOL_FASTSCAN:    dsp_initialize_fastscan();
                                break;
     case PROTOCOL_CW:          dsp_initialize_cw(0);
                                break;
     case PROTOCOL_RTTY:        dsp_initialize_rtty();
                                break;
     case PROTOCOL_SCAMP:       dsp_initialize_scamp(SCAMP_FSK);
                                break;
  }
}

void dsp_dispatch_interrupt(uint8_t protocol)
{
  switch (protocol)
  {
     case PROTOCOL_CW:          cw_new_sample();
                                break;
     case PROTOCOL_RTTY:        rtty_new_sample();
                                break;
     case PROTOCOL_SCAMP:       scamp_new_sample();
                                break;    
  }
}

void dsp_dispatch_receive(uint8_t protocol)
{
  switch (protocol)
  {
     case PROTOCOL_CW:          cw_decode_process();
                                break;
     case PROTOCOL_RTTY:        rtty_decode_process();
                                break;
     case PROTOCOL_SCAMP:       scamp_decode_process();
                                break;    
  }
}

uint8_t dsp_dispatch_txmit(uint8_t protocol, uint32_t frequency, uint8_t *message, uint8_t length, void *user_state, dsp_dispatch_callback ddc)
{
  dsp_txmit_message_state dtms;

  dtms.message = message;
  dtms.length = length;
  dtms.frequency = frequency;
  dtms.user_state = user_state;
  dtms.current_symbol = 0;
  dtms.aborted = 0;

  switch (protocol)
  {
     case PROTOCOL_CW:          return cwmod_txmit(&dtms,ddc);
     case PROTOCOL_RTTY:        return rtty_txmit(&dtms,ddc);
     case PROTOCOL_SCAMP:       return scamp_txmit(&dtms,ddc);
  }
}

uint16_t dsp_get_signal_magnitude(void)
{
  return (ds.mag_value_8 + ds.mag_value_12 + ds.mag_value_16 + ds.mag_value_20 + ds.mag_value_24) >> 1;
}

/* this is an approximation to the sqrt(x^2+y^2) function that approximates the
   circular set as an octagon.  seems to work quite well */
#define SET_DSP_SQRT_APPROX(s,x,y) do { \
   uint16_t ux,uy; \
   ux = ((x)>>16); \
   ux = (ux & 0x8000) ? ~ux : ux; \
   uy = ((y)>>16); \
   uy = (uy & 0x8000) ? ~uy : uy; \
   s = ((ux + uy) >> 1)+(uy > ux ? uy : ux); \
} while(0)

#define SET_DSP_SQRT_APPROX_16(s,x,y) do { \
   uint16_t ux = (x < 0 ? -x : x); \
   uint16_t uy = (y < 0 ? -y : y); \
   s = ((ux + uy) >> 1)+(uy > ux ? uy : ux); \
} while(0)

#define SHIFTBACK(x,bits) ((x) < 0 ? -((-x) >> (bits)) : ((x) >> (bits)))

// For 2000 Hz sample rate
const int16_t cos8[] PROGMEM = {  16384,-11585,0,11585,-16384,11585,0,-11585 };  // 750 Hz
const int16_t sin8[] PROGMEM = {  0,11585,-16384,11585,0,-11585,16384,-11585 };
const int16_t cos12[] PROGMEM = { 16384,-8192,-8192,16384,-8192,-8192,16384,-8192,-8192,16384,-8192,-8192 }; // 667 Hz
const int16_t sin12[] PROGMEM = { 0,14189,-14189,0,14189,-14189,0,14189,-14189,0,14189,-14189 };
const int16_t cos16[] PROGMEM = { 16384,-6270,-11585,15137,0,-15137,11585,6270,-16384,6270,11585,-15137,0,15137,-11585,-6270 };  // 625 Hz
const int16_t sin16[] PROGMEM = { 0,15137,-11585,-6270,16384,-6270,-11585,15137,0,-15137,11585,6270,-16384,6270,11585,-15137 };
const int16_t cos20[] PROGMEM = { 16384,-5063,-13255,13255,5063,-16384,5063,13255,-13255,-5063,16384,-5063,-13255,13255,5063,-16384,5063,13255,-13255,-5063 };  // 600 Hz
const int16_t sin20[] PROGMEM = { 0,15582,-9630,-9630,15582,0,-15582,9630,9630,-15582,0,15582,-9630,-9630,15582,0,-15582,9630,9630,-15582 };
const int16_t cos24[] PROGMEM = { 16384,-4240,-14189,11585,8192,-15826,0,15826,-8192,-11585,14189,4240,-16384,4240,14189,-11585,-8192,15826,0,-15826,8192,11585,-14189,-4240 }; // 583 Hz
const int16_t sin24[] PROGMEM = { 0,15826,-8192,-11585,14189,4240,-16384,4240,14189,-11585,-8192,15826,0,-15826,8192,11585,-14189,-4240,16384,-4240,-14189,11585,8192,-15826 };

#ifdef EXTRA_CHANNELS
const int16_t cos4[] PROGMEM = {16384,0,-16384,0};  // 500 Hz
const int16_t sin4[] PROGMEM = {0,16384,0,-16384};
const int16_t cos8n[] PROGMEM = { 16384,11585,0,-11585,-16384,-11585,0,11585 }; // 250 Hz
const int16_t sin8n[] PROGMEM = { 0,11585,16384,11585,0,-11585,-16384,-11585 };
const int16_t cos12n[] PROGMEM = { 16384,8192,-8192,-16384,-8192,8192,16384,8192,-8192,-16384,-8192,8192 }; // 333 Hz
const int16_t sin12n[] PROGMEM = { 0,14189,14189,0,-14189,-14189,0,14189,14189,0,-14189,-14189 };
const int16_t cos16n[] PROGMEM = { 16384,6270,-11585,-15137,0,15137,11585,-6270,-16384,-6270,11585,15137,0,-15137,-11585,6270 };  // 375 Hz
const int16_t sin16n[] PROGMEM = { 0,15137,11585,-6270,-16384,-6270,11585,15137,0,-15137,-11585,6270,16384,6270,-11585,-15137 };
const int16_t cos20n[] PROGMEM = { 16384,5063,-13255,-13255,5063,16384,5063,-13255,-13255,5063,16384,5063,-13255,-13255,5063,16384,5063,-13255,-13255,5063 }; // 400 Hz
const int16_t sin20n[] PROGMEM = { 0,15582,9630,-9630,-15582,0,15582,9630,-9630,-15582,0,15582,9630,-9630,-15582,0,15582,9630,-9630,-15582 };
const int16_t cos24n[] PROGMEM = { 16384,4240,-14189,-11585,8192,15826,0,-15826,-8192,11585,14189,-4240,-16384,-4240,14189,11585,-8192,-15826,0,15826,8192,-11585,-14189,4240 };  // 417 Hz
const int16_t sin24n[] PROGMEM = {  0,15826,8192,-11585,-14189,4240,16384,4240,-14189,-11585,8192,15826,0,-15826,-8192,11585,14189,-4240,-16384,-4240,14189,11585,-8192,-15826 };
#endif

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

   prep_sample = (ds.count++ & 0x03) == 0;

   if (df.dly_8 != 0)
   {
     /* update 8 count I & Q */
     b = (ds.sample_no < df.dly_8) ? (ds.sample_no + df.buffer_size - df.dly_8) : (ds.sample_no - df.dly_8);
     fir = sample - ds.sample_buffer[b];
     ds.state_i_8 += ((int32_t)fir)*((int32_t)((int16_t)pgm_read_word_near(&cos8[ds.count_8])));
     ds.state_q_8 += ((int32_t)fir)*((int32_t)((int16_t)pgm_read_word_near(&sin8[ds.count_8])));
     if (prep_sample)
        SET_DSP_SQRT_APPROX(ds.mag_value_8, ds.state_q_8, ds.state_i_8);
     //ds.count_8 = (ds.count_8 >= 7) ? 0 : (ds.count_8 + 1);
     ds.count_8 = (ds.count_8+1) & 0x07;
   }

   if (df.dly_12 != 0)
   {
     /* update 12 count I & Q */
     b = (ds.sample_no < df.dly_12) ? (ds.sample_no + df.buffer_size - df.dly_12) : (ds.sample_no - df.dly_12);
     fir = sample - ds.sample_buffer[b];
     ds.state_i_12 += ((int32_t)fir)*((int32_t)((int16_t)pgm_read_word_near(&cos12[ds.count_12])));
     ds.state_q_12 += ((int32_t)fir)*((int32_t)((int16_t)pgm_read_word_near(&sin12[ds.count_12])));
     if (prep_sample)
       SET_DSP_SQRT_APPROX(ds.mag_value_12, ds.state_q_12, ds.state_i_12);
     ds.count_12 = (ds.count_12 >= 11) ? 0 : (ds.count_12 + 1);
   }

   if (df.dly_16 != 0)
   {
     /* update 16 count I & Q */
     b = (ds.sample_no < df.dly_16) ? (ds.sample_no + df.buffer_size - df.dly_16) : (ds.sample_no - df.dly_16);
     fir = sample - ds.sample_buffer[b];
     ds.state_i_16 += ((int32_t)fir)*((int32_t)((int16_t)pgm_read_word_near(&cos16[ds.count_16])));
     ds.state_q_16 += ((int32_t)fir)*((int32_t)((int16_t)pgm_read_word_near(&sin16[ds.count_16])));
     if (prep_sample)
       SET_DSP_SQRT_APPROX(ds.mag_value_16, ds.state_q_16, ds.state_i_16);
     //ds.count_16 = (ds.count_16 >= 15) ? 0 : (ds.count_16 + 1);
     ds.count_16 = (ds.count_16+1) & 0x0F;
   }

   if (df.dly_20 != 0)
   {
     /* update 20 count I & Q */
     b = (ds.sample_no < df.dly_20) ? (ds.sample_no + df.buffer_size - df.dly_20) : (ds.sample_no - df.dly_20);
     fir = sample - ds.sample_buffer[b];
     ds.state_i_20 += ((int32_t)fir)*((int32_t)((int16_t)pgm_read_word_near(&cos20[ds.count_20])));
     ds.state_q_20 += ((int32_t)fir)*((int32_t)((int16_t)pgm_read_word_near(&sin20[ds.count_20])));
     if (prep_sample)
        SET_DSP_SQRT_APPROX(ds.mag_value_20, ds.state_q_20, ds.state_i_20);
     ds.count_20 = (ds.count_20 >= 19) ? 0 : (ds.count_20 + 1);
   }

   if (df.dly_24 != 0)
   {
     /* update 24 count I & Q */
     b = (ds.sample_no < df.dly_24) ? (ds.sample_no + df.buffer_size - df.dly_24) : (ds.sample_no - df.dly_24);
     fir = sample - ds.sample_buffer[b];
     ds.state_i_24 += ((int32_t)fir)*((int32_t)((int16_t)pgm_read_word_near(&cos24[ds.count_24])));
     ds.state_q_24 += ((int32_t)fir)*((int32_t)((int16_t)pgm_read_word_near(&sin24[ds.count_24])));
     if (prep_sample)
        SET_DSP_SQRT_APPROX(ds.mag_value_24, ds.state_q_24, ds.state_i_24);
     ds.count_24 = (ds.count_24 >= 23) ? 0 : (ds.count_24 + 1);
   }

   /* store in circular buffer so that is can be subtracted from the end
      to make a moving average filter */
   ds.sample_buffer[ds.sample_no] = sample;
   if ((++ds.sample_no) >= df.buffer_size)
      ds.sample_no = 0;

   if (prep_sample) ds.sample_ct++;
}

#ifdef DSPINT_DEBUG

#include "cwmod.c"

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

#define MOD_TEST 2

#if MOD_TEST==0
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

#define MODULATION_OFFSET 0.25

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
                                0,1,1,0,0, 1,0,0,0,0, 1,0,1,0,0, 1,0,0,0,0, 1,0,0,1,0, 1,0,0,0,0,  /* 30 bits */
                                1,0,1,1,0, 0,1,1,1,0, 0,1,1,1,0, 0,1,1,1,0, 1,0,1,1,0, 1,0,1,1,0,  /* 30 bits */
                                1,0,1,1,1, 1,0,1,1,1 };
    samples = sizeof(bits)*MOD_REP;
    FILE *fp = write_wav_file("synth.wav",samples,repeats);
    for (c=0;c<samples;c++)
    {
        uint8_t last_sample_ct = 0;
        int8_t spacing = 0*(sin(c*2.0*M_PI/1000.0));
        cbit = bits[(c+7+spacing)/MOD_REP];
        freq = cbit ? MOD_CHAN1 : MOD_CHAN2;
        samp = ((sin(2.0*M_PI*(((float)c)/freq+((float)c)*MODULATION_OFFSET)+0.0*M_PI)*64.0)+512.0) + gaussian_deviate(64.0);
        //if ((c>4000) && (c<10000)) samp = gaussian_deviate(48.0)+512;
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
  test_dsp_sample();
  //test_cwmod_decode();
  printf("size=%d\n",sizeof(ds)+sizeof(df)+sizeof(ps));
}
#endif /* DSPINT_DEBUG */
