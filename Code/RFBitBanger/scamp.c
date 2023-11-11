
/*  scamp.c */

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

/*  The 12 bit code words are of two types: */

/*  XXXXXX  YYYYYY -- two 6-bit symbol codes (for text exchange) */
/*  1111 XXXX XXXX -- 8 bit raw data (for binary protocols) */
/*  text communication is the priority for efficiency and speed
    (e.g. contesting and rag chewing - the lost art of conversation) */

#undef SCAMP_DEBUG

#ifdef ARDUINO
#include <Arduino.h>
#endif

#ifdef SCAMP_DEBUG
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#endif

#include "common.h"
#include "dspint.h"
#include "scamp.h"
#include "golay.h"

const uint8_t PROGMEM scamp_6bit_codesymbols[60] = 
{'\0',   '\b',   '\r',    ' ',    '!',   0x22,   0x27,    '(',
  ')',    '*',    '+',    ',',    '-',    '.',    '/',    '0',
  '1',    '2',    '3',    '4',    '5',    '6',    '7',    '8',
  '9',    ':',    ';',    '=',    '?',    '@',    'A',    'B',
  'C',    'D',    'E',    'F',    'G',    'H',    'I',    'J',
  'K',    'L',    'M',    'N',    'O',    'P',    'Q',    'R',
  'S',    'T',    'U',    'V',    'W',    'X',    'Y',    'Z',
 0x5C,   '^',    '`',    '~' };
   /* Last 4 symbols can be interpreted as diacritical marks, 0x5C is diaeresis/umlaut */
   /* 0x27 can be interpreted as acute diacritical mark */


/* calculate the hamming weight of a 16 bit integer
   GCC could use __builtin__popcount() */
uint8_t hamming_weight_16(uint16_t n)
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
uint8_t hamming_weight_30(uint32_t n)
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
    if (dff->tail == dff->head) return 0xFFFFFFFF;
    next = dff->tail >= (SCAMP_FRAME_FIFO_LENGTH - 1) ? 0 : (dff->tail+1);
    frame = dff->frames[next];
    dff->tail = next;
    return frame;
}

/* in memory buffer encoder / decoder callbacks */
typedef struct _scamp_code_word_put_mem_buf_struct
{
  uint16_t *code_word_array;
  uint8_t cur_word;
  uint8_t max_words;
} scamp_code_word_put_mem_buf_struct;

void scamp_code_word_put_mem_buf(uint16_t code, void *st, uint8_t pos, uint8_t frame)
{
  scamp_code_word_put_mem_buf_struct *s = (scamp_code_word_put_mem_buf_struct *) st;
  if (s->cur_word < s->max_words)
     s->code_word_array[s->cur_word++] = code;
}

typedef struct _scamp_code_word_get_mem_buf_struct
{
  uint16_t *code_word_array;
  uint8_t cur_word;
  uint8_t max_words;
} scamp_code_word_get_mem_buf_struct;

uint16_t scamp_code_word_get_mem_buf(void *st)
{
  scamp_code_word_get_mem_buf_struct *s = (scamp_code_word_get_mem_buf_struct *) st;
  if (s->cur_word < s->max_words)
     return s->code_word_array[s->cur_word++];
  return 0xFFFF;
}

/*  The input word has 24 bits.  The output word has 30 bits, with bits
    1, 5, 9, 13, 17, 21 preceded by its complement bit inserted into
    the word */

uint32_t scamp_add_reversal_bits(uint32_t codeword)
{
  uint32_t outword = 0;
  uint8_t i;

  for (i=0;i<6;i++)
  {
      if (i>0)
          outword = (outword << 5);
      codeword = (codeword << 4);
      uint8_t temp = (codeword >> 24) & 0x0F;
      outword |= (temp | (((temp & 0x08) ^ 0x08) << 1));
  }
  return outword;
}

/* remove complement bits from inside 30 bit word to yield 24 bit Golay
   code word */

uint32_t scamp_remove_reversal_bits(uint32_t outword)
{
  uint32_t codeword = 0;
  uint8_t i;

  for (i=0;i<6;i++)
  {
      if (i>0)
      {
          outword = (outword << 5);
          codeword = (codeword << 4);
      }
      uint8_t temp = (outword >> 25) & 0x0F;
      codeword |= temp;
  }
  return codeword;
}

#define SCAMP_IS_DATA_CODE(x) (((uint8_t)((x) >> 8)) == 0x0F)
#define SCAMP_RES_CODE_BITS_SET(x) ((((uint8_t)(x)) & 0x3C) == 0x3C)
#define SCAMP_IS_RES_CODE(x) (SCAMP_RES_CODE_BITS_SET(x) && (!SCAMP_IS_DATA_CODE(x)))

#if 0
/* decode 12 bit code words to 8 bit bytes */
uint8_t scamp_code_words_to_bytes(scamp_code_word_get ecwg, void *st, uint8_t *bytes, uint8_t max_bytes)
{
    uint8_t cur_byte = 0;
    uint16_t last_code = 0xFFFF;
    while (cur_byte < max_bytes)
    {
        uint16_t code = ecwg(st);
        if (code == 0xFFFF) break;
        last_code = code;
        if (SCAMP_IS_DATA_CODE(code))
        {
            bytes[cur_byte++] = code & 0xFF;
            continue;
        }
        if (code == last_code) continue;
        uint16_t code1 = (code & 0x3F);
        uint16_t code2 = ((code >> 6) & 0x3F);
        if ((code1 != 0) && (code1 < (sizeof(scamp_6bit_codesymbols)/sizeof(uint8_t))))
            bytes[cur_byte++] = pgm_read_byte_near(&scamp_6bit_codesymbols[code1]);
        if ((code2 != 0) && (code2 < (sizeof(scamp_6bit_codesymbols)/sizeof(uint8_t))) && (cur_byte < max_bytes))
            bytes[cur_byte++] = pgm_read_byte_near(&scamp_6bit_codesymbols[code2]);
    }
    return cur_byte;
}
#endif

/* decode a 12 bit code word to 8 bit bytes */
uint8_t scamp_code_word_to_bytes(uint16_t code, uint8_t bytes[])
{
  uint8_t c;
  if (SCAMP_IS_DATA_CODE(code))
  {
    bytes[0] = code & 0xFF;
    return 1;
  }
  c = (code & 0x3F);
  if ((c == 0) || (c >= (sizeof(scamp_6bit_codesymbols)/sizeof(uint8_t))))
    return 0;
  bytes[0] = pgm_read_byte_near(&scamp_6bit_codesymbols[c]);
  c = ((code >> 6) & 0x3F);
  if ((c == 0) || (c >= (sizeof(scamp_6bit_codesymbols)/sizeof(uint8_t))))
     return 1;
  bytes[1] = pgm_read_byte_near(&scamp_6bit_codesymbols[c]);
  return 2;
}

/* encode 8 bit bytes as 12 bit code words, always encoding as 8-bit raw */
void scamp_raw_bytes_to_code_words(uint8_t *bytes, uint8_t num_bytes, scamp_code_word_put ecwp, void *st)
{
  uint8_t cur_byte = 0;
  uint8_t frame = 0;
  while (cur_byte < num_bytes)
  {
     uint8_t b = bytes[cur_byte++];
     ecwp( ((uint16_t)(0xF00)) | b, st, cur_byte, frame++ );
  }
}

/* convert character to 6-bit code if it exists */
/* should probably replace this with an inverse look up table later */
uint8_t scamp_find_code_in_table(uint8_t c)
{
    uint8_t i;
    if ((c >= 'a') && (c <= 'z')) c -= 32;
	if (c =='\n') c = '\r';
	if (c == 127) c = '\b';
    for (i=1;i<(sizeof(scamp_6bit_codesymbols)/sizeof(uint8_t));i++)
        if (pgm_read_byte_near(&scamp_6bit_codesymbols[i]) == c) return i;
    return 0xFF;
}

/* encode 8 bit bytes to 12 bit code words.
   code words that correspond to a 6-bit symbols are encoded as 6 bits.
   otherwise they are encoded as an 8-bit binary raw data word.
   If a byte that can be encoded as a 6 bit symbol precedes one that can
   not be encoded as a 6 bit symbol, and there is an extra symbol slot
   in the current word, fill it with a zero. */
void scamp_bytes_to_code_words(uint8_t *bytes, uint8_t num_bytes, scamp_code_word_put ecwp, void *st)
{
  uint8_t cur_byte = 0;
  uint8_t frame = 0;
  uint16_t last_code_word = 0;
  uint16_t code_word;
  while (cur_byte < num_bytes)
  {
     uint8_t b = bytes[cur_byte++];
     uint8_t code1 = scamp_find_code_in_table(b);
     if (code1 == 0xFF)
     {
         code_word = ((uint16_t)(0xF00)) | b;
     } else
     {
         code_word = (uint16_t)code1;
         if (cur_byte < num_bytes)
         {
            b = bytes[cur_byte];
            uint8_t code2 = scamp_find_code_in_table(b);
            if (code2 != 0xFF)
            {
               code_word |= (((uint16_t)code2) << 6);
               cur_byte++;
            }
         }
         if (code_word == last_code_word)
         {
           if (ecwp(0, st, cur_byte, frame++)) break;
         }
     }
     if (ecwp(code_word, st, cur_byte, frame++)) break;
     last_code_word = code_word;
  }
}

void scamp_set_mod_frequencies(dsp_txmit_message_state *dtms)
{
  int16_t offset1, offset2;
  switch (ps.ss.protocol)
  {
#ifdef SCAMP_VERY_SLOW_MODES
      case PROTOCOL_SCAMP_OOK_SLOW:
#endif
      case PROTOCOL_SCAMP_OOK:        offset1 = 0;
                                      offset2 = 0;
                                      break;
#ifdef SCAMP_VERY_SLOW_MODES
      case PROTOCOL_SCAMP_FSK_SLOW:   offset1 = (625-667)/2;
                                      offset2 = (667-625)/2;
                                      break;
      case PROTOCOL_SCAMP_FSK_VSLW:   offset1 = (625-667)/4;
                                      offset2 = (667-625)/4;
                                      break;
#endif
      case PROTOCOL_SCAMP_FSK:        offset1 = (600-667)/2;
                                      offset2 = (667-600)/2;
                                      break;
      case PROTOCOL_SCAMP_FSK_FAST:   offset1 = (583-750)/2;
                                      offset2 = (750-583)/2;
                                      break;
  }
  set_frequency(dtms->frequency + offset1, 0);
  set_frequency(dtms->frequency + offset2, 1);
}

void scamp_wait_bit(void)
{
  uint16_t read_bit;
  uint8_t bit_ms = df.buffer_size / (df.slow_samp_num > 1 ? 1 : 2); // (divided by 2 for 2000 Hz -> ms)
  ps.ss.clock_bit += bit_ms;
  do
  {
    read_bit = millis();
  } while (((int16_t)(ps.ss.clock_bit - read_bit)) > 0);
}

void scamp_send_frame(uint32_t bits)
{
  for (uint8_t num=0;num<30;num++)
  {
    scamp_wait_bit();
    uint8_t bitv = (bits & 0x20000000) != 0;
    if (ps.ss.fsk)
      set_clock_onoff_mask(bitv ? 0x02 : 0x01) ;
    else
      transmit_set(bitv);
    idle_task();
    bits <<= 1;
  }
}

void scamp_send_frame_rep(uint32_t bits, uint8_t rep)
{
  while (rep > 0)
  {
    scamp_send_frame(bits);
    --rep;
  }
}

typedef struct _scamp_code_word_transmit_data
{
  dsp_dispatch_callback ddc;
  dsp_txmit_message_state *dtms;
} scamp_code_word_transmit_data;

uint8_t scamp_code_word_transmit(uint16_t code, void *st, uint8_t pos, uint8_t frame)
{
  uint8_t data_code = SCAMP_IS_DATA_CODE(code);
  uint32_t code_30;
  scamp_code_word_transmit_data *scwtd = (scamp_code_word_transmit_data *) st;
  
  code_30 = golay_encode(code);
  code_30 = scamp_add_reversal_bits(code_30);
  scamp_send_frame_rep(code_30, data_code ? 1 : rc.scamp_resend_frames);
  if ((rc.scamp_resync_frames != 0) && (((frame+1) % rc.scamp_resync_frames) == 0))
  {
    scamp_send_frame(SCAMP_INIT_CODEWORD);
    scamp_send_frame(SCAMP_SYNC_CODEWORD);
  }
  while (scwtd->dtms->current_symbol <= pos)
  {
    scwtd->ddc(scwtd->dtms);
    scwtd->dtms->current_symbol++;
  }
  return scwtd->dtms->aborted;
}

uint8_t scamp_txmit(dsp_txmit_message_state *dtms, dsp_dispatch_callback ddc)
{
  uint8_t solids;
  scamp_code_word_transmit_data scwtd;
  scwtd.ddc = ddc;
  scwtd.dtms = dtms;
  
  scamp_set_mod_frequencies(dtms);
  muteaudio_set(1);
  ps.ss.clock_bit = millis();
  if (ps.ss.fsk) 
  {
    transmit_set(1);
    scamp_send_frame(SCAMP_SOLID_CODEWORD);
  } else
  {
    for (uint8_t i=0;i<4;i++)
      scamp_send_frame(SCAMP_DOTTED_CODEWORD);
  }
  scamp_send_frame_rep(SCAMP_INIT_CODEWORD, rc.scamp_resend_frames);
  scamp_send_frame_rep(SCAMP_SYNC_CODEWORD, rc.scamp_resend_frames);
  scamp_bytes_to_code_words(dtms->message, dtms->length, scamp_code_word_transmit, (void *) &scwtd);
  if (!dtms->aborted)
     scamp_send_frame_rep(SCAMP_RES_CODE_END_TRANSMISSION, rc.scamp_resend_frames);
  scamp_wait_bit();
  set_clock_onoff(0,0);
  set_clock_onoff(1,0);
  transmit_set(0);
  muteaudio_set(0);
  ps.ss.reset_protocol = 1;
}

void scamp_reset_codeword(void)
{
   ps.ss.current_bit_no = 0;
   ps.ss.current_word = SCAMP_BLANK_CODEWORD;
   ps.ss.bitflips_in_phase = 0;
   ps.ss.bitflips_lag = 0;
   ps.ss.bitflips_lead = 0;
   ps.ss.bitflips_ctr = 0;
}

void scamp_retrain(void)
{
  ps.ss.cur_demod_edge_window = ps.ss.demod_samples_per_bit;
  scamp_reset_codeword();
  ps.ss.last_code = 0;
  ps.ss.resync = 0;  
}

/* this is called by the interrupt handle to decode the SCAMP frames from
   the spectral channels.  it figures out the magnitude of
   the signal given the current modulation type (OOK/FSK).  it tries to detect
   an edge to determine when the current bit has finished and when it should
   expect the next bit.  it resets the bit counter when the sync signal has
   been received, and when 30 bits of a frame have been received, stores the
   frame in the frame FIFO */
void scamp_new_sample(void)
{
    int16_t demod_sample, temp;
    uint8_t received_bit;
    uint8_t hamming_weight;
    uint16_t max_val;

    uint8_t ct = ds.sample_ct;
    /* only proceed if we have new magnitude samples */
    if (ct == ps.ss.last_sample_ct)
        return;
    ps.ss.last_sample_ct = ct;

    /* demodulate a sample either based on the amplitude in one frequency
       channel for OOK, or the difference in amplitude between two frequency
       channels for FSK */

    switch (ps.ss.protocol)
    {
#ifdef SCAMP_VERY_SLOW_MODES
        case PROTOCOL_SCAMP_OOK_SLOW:
#endif
        case PROTOCOL_SCAMP_OOK:        demod_sample = ds.mag_value_16 - ps.ss.power_thr;
                                        max_val = ds.mag_value_16;
                                        break;
#ifdef SCAMP_VERY_SLOW_MODES
        case PROTOCOL_SCAMP_FSK_VSLW:
        case PROTOCOL_SCAMP_FSK_SLOW:   demod_sample = ds.mag_value_16 - ds.mag_value_12;
                                        max_val = ds.mag_value_12 > ds.mag_value_16 ? ds.mag_value_12 : ds.mag_value_16;
                                        break;
#endif
        case PROTOCOL_SCAMP_FSK:        demod_sample = ds.mag_value_20 - ds.mag_value_12;
                                        max_val = ds.mag_value_12 > ds.mag_value_20 ? ds.mag_value_12 : ds.mag_value_20;
                                        break;
        case PROTOCOL_SCAMP_FSK_FAST:   demod_sample = ds.mag_value_24 - ds.mag_value_8;
                                        max_val = ds.mag_value_8 > ds.mag_value_24 ? ds.mag_value_8 : ds.mag_value_24;
                                        break;
    }
    ps.ss.ct_sum += max_val;
    /* This is the automatic "gain" control (threshold level control).
       find the average of a certain number of samples (power of two for calculation
       speed) so that we can determine what to set the thresholds at for the bit on/off
       threshold for ook and the edge threshold for ook/fsk */
    if (ps.ss.fsk)
    {
      if ((++ps.ss.ct_average) >= (((uint16_t)1) << SCAMP_AVG_CT_PWR2_FSK))
      {
       temp = (ps.ss.ct_sum) >> (SCAMP_AVG_CT_PWR2_FSK);
       /* don't allow threshold to get too low, or we'll be having bit edges constantly */
       ps.ss.edge_thr = temp > ps.ss.power_thr_min ? temp : ps.ss.power_thr_min;
       ps.ss.power_thr = ps.ss.edge_thr >> 1;
       if (ps.ss.protocol != PROTOCOL_SCAMP_FSK_FAST) ps.ss.edge_thr = (ps.ss.edge_thr * 3) / 2;
       ps.ss.squelch_thr = ps.ss.power_thr; 
       ps.ss.ct_average = 0;
       ps.ss.ct_sum = 0;
      }
    } else
    {
      if ((++ps.ss.ct_average) >= (((uint16_t)1) << SCAMP_AVG_CT_PWR2_OOK))
      {
        temp = (ps.ss.ct_sum) >> (SCAMP_AVG_CT_PWR2_OOK);
        /* don't allow threshold to get too low, or we'll be having bit edges constantly */
        ps.ss.edge_thr = temp > ps.ss.power_thr_min ? temp : ps.ss.power_thr_min;
        ps.ss.edge_thr >>= 1;
        ps.ss.power_thr = ps.ss.edge_thr;
        ps.ss.squelch_thr = ps.ss.power_thr >> 1; 
        ps.ss.ct_average = 0;
        ps.ss.ct_sum = 0;
      }
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
    ps.ss.current_word = (ps.ss.current_word << 1) | (ps.ss.polarity ^ (ps.ss.cur_bit > 0));
    ps.ss.bitflips_ctr++;
    /* this is done on the bit before it is needed to reduce worst case latency */
    if (ps.ss.bitflips_ctr == 4)
    {
        /* keep track of the number of complement bits in the word. should be 6 */
        uint8_t maskbits, lowerbyte = ((uint8_t)ps.ss.current_word);
        maskbits = lowerbyte & 0x0C;
        ps.ss.bitflips_in_phase += (maskbits == 0x08) || (maskbits == 0x04);
        maskbits = lowerbyte & 0x18;
        ps.ss.bitflips_lag += (maskbits == 0x10) || (maskbits == 0x08);
        maskbits = lowerbyte & 0x06;
        ps.ss.bitflips_lead += (maskbits == 0x04) || (maskbits == 0x02);
    } else if (ps.ss.bitflips_ctr >= 5)
        ps.ss.bitflips_ctr = 0;

    uint8_t thr = max_val >= ps.ss.squelch_thr;
    
    if (thr)
      ps.ss.threshold_counter = SCAMP_THRESHOLD_COUNTER_MAX;
    else
    {
      if (ps.ss.threshold_counter > 0)
          ps.ss.threshold_counter--;
    }
    if (ps.ss.reset_protocol != 0)
    {
      scamp_retrain();
      ps.ss.reset_protocol = 0;
    }
    if ((!ps.ss.fsk) || thr)  /* if there is a bit to sync to */
    {
      hamming_weight = hamming_weight_30(ps.ss.current_word ^ SCAMP_SYNC_CODEWORD);
      if (hamming_weight < 4)  /* 30-bit resync word has occurred! */
      {
        scamp_reset_codeword();
        ps.ss.resync = 1;
        return;
      }
      /* if we 15 of the last 16 bits zeros with fsk, that means we have a reversed polarity start */
      hamming_weight = hamming_weight_16(ps.ss.current_word);
      if ((hamming_weight < 2) && (ps.ss.fsk))
      {
        ps.ss.polarity = !ps.ss.polarity; /* reverse the polarity of FSK frequencies and 0/1 */
        scamp_retrain();
        return;
      }
      /* if we have 15 of the last 16 bits ones, that is a ook key down start, or a fsk start */
      if (hamming_weight >= 15)
      {
        scamp_retrain();
        return;
      }
    } 
    /* if we have synced, and we have 30 bits, we have a frame */
    ps.ss.current_bit_no++;
    if ((ps.ss.current_bit_no >= 30) && (ps.ss.resync))  /* we have a complete frame */
    {
       if ((ps.ss.bitflips_lead > ps.ss.bitflips_in_phase) && (ps.ss.bitflips_lead >= 5))
        /* we are at least one bit flip ahead, we probably registered a spurious bit flip */
       {
         /* back up and get one more bit.*/
         ps.ss.current_bit_no--;
         ps.ss.bitflips_in_phase = ps.ss.bitflips_lead;  /*lead now becomes in phase */
         ps.ss.bitflips_lag = 0;  /* lag is now two behind, so we can't use it */
         ps.ss.bitflips_lead = 0; /* clear bit_lead so we don't try a second time */
       } else
       {
          if ((ps.ss.bitflips_lag > ps.ss.bitflips_in_phase) && (ps.ss.bitflips_lag >= 5))
          /* we are at least one bit flip short, we probably fell a bit behind */
          {
            if (ps.ss.threshold_counter > 0)
              scamp_insert_into_frame_fifo(&ps.ss.scamp_output_fifo, ps.ss.current_word >> 1);
            /* start with the next word with one flip */
            ps.ss.current_bit_no = 1;
            ps.ss.bitflips_ctr = 1;
         } else
         {
             if (ps.ss.bitflips_in_phase >= 4)
             {
               /* otherwise we just place in buffer and the code word is probably aligned */
               if (ps.ss.threshold_counter > 0)
                 scamp_insert_into_frame_fifo(&ps.ss.scamp_output_fifo, ps.ss.current_word);
             }
             ps.ss.current_bit_no = 0;
             ps.ss.bitflips_ctr = 0;
          }
          ps.ss.bitflips_in_phase = 0;
          ps.ss.bitflips_lag = 0;
          ps.ss.bitflips_lead = 0;
       }
    }
}

void scamp_decode_process(void)
{
  uint8_t biterrs, bytes[2], nb;
  uint16_t gf;
  uint32_t fr = scamp_remove_from_frame_fifo(&ps.ss.scamp_output_fifo);
  if (fr == 0xFFFFFFFF) return;
  fr = scamp_remove_reversal_bits(fr);
  gf = golay_decode(fr,&biterrs);
  if (gf == 0xFFFF)
  {
    decode_insert_into_fifo('#');
    return;
  }
  if (!SCAMP_IS_DATA_CODE(gf))
  {
    if (SCAMP_RES_CODE_BITS_SET(gf))
    {
      switch (gf)
      {
        case SCAMP_RES_CODE_END_TRANSMISSION:
            ps.ss.reset_protocol = 1;
            break;
      }
      return;
    }
    if (gf == ps.ss.last_code) return;
  }
  ps.ss.last_code = gf;
  nb = scamp_code_word_to_bytes(gf, bytes);
  if (nb > 0) decode_insert_into_fifo(bytes[0]);
  if (nb > 1) decode_insert_into_fifo(bytes[1]);
}

int16_t scamp_frequency_offset()
{
  switch (ps.ss.protocol)
  {
#ifdef SCAMP_VERY_SLOW_MODES
      case PROTOCOL_SCAMP_OOK_SLOW:
#endif
      case PROTOCOL_SCAMP_OOK:        return -625;
#ifdef SCAMP_VERY_SLOW_MODES
      case PROTOCOL_SCAMP_FSK_SLOW:   return -((625+667)/2);
      case PROTOCOL_SCAMP_FSK_VSLW:   return -((625+667)/4);
#endif
      case PROTOCOL_SCAMP_FSK:        return -((600+667)/2);
      case PROTOCOL_SCAMP_FSK_FAST:   return -((583+750)/2);
  }
  return -646;
}
