/*  ssb.c */

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


#undef SSB_DEBUG

#ifdef ARDUINO
#include <Arduino.h>
#endif

#ifdef CWMOD_DEBUG
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#endif

#include "common.h"
#include "dspint.h"
#include "ssb.h"

void ssb_state_change(uint8_t state)
{
  if (state == 0)
  {
    if (ds.ssb_active != 0)
    {
      ds.ssb_active = 0;
      transmit_pwm_mode(0);
      delay(3); // wait for any interrupt to finish
      set_frequency_receive();
    }
  } else
  {
    if (ds.ssb_active == 0)
    {
      if (IS_SSB_PROTOCOL(ps.ssbs.protocol))
      {
        muteaudio_set(1);
        set_clock_onoff_mask(0x01); 
        transmit_pwm_mode(1);
        set_transmit_pwm(0);
        ds.ssb_active = 1;
      }
    }
  }
  return;
}

#define SAMPLING_CLOCK (OVERSAMPLING_CLOCKS / 2)  
  // note the base rate is 8000 Hz.  Protocols sample at 2000 Hz, hence the OVERSAMPLING_CLOCKS
  // is (8000 / 2000) Hz = 4.  However, ssb routine works at 4000 Hz, so we cut it in half.

#define SAMPLING_FREQUENCY (TIMER1_INTERRUPT_FREQ / SAMPLING_CLOCK)

// This code is adapted from PE1NNZ ssb() function.   

#define _UA (22*32)                                         

#define FRACTIONAL_MULT_FREQUENCY (uint32_t)(((SAMPLING_FREQUENCY)*256ul)/_UA)

#define ATAN_APPROX(z1,z2) (((((_UA/8) + (_UA/22)) - ((_UA/22) * (z1)) / (z2)) * (z1)) / (z2))  
   // approx Pi/4 + 0.285*z*(1-abs(z)) from 
   // [1] http://www-labs.iro.umontreal.ca/~mignotte/IFT2425/Documents/EfficientApproximationArctgFunction.pdf
   // (8/22)*Pi/4 approx 0.285
   // formula scaled by (1/8+1/22)/(Pi/4+0.285) or 0.159

#define OCTAGON_MAGNITUDE_PHASE(mag,phs,x,y) do { \
   uint16_t ux = ((x) < 0 ? -(x) : (x)); \
   uint16_t uy = ((y) < 0 ? -(y) : (y)); \
   mag = ((uy > ux) ? (uy+ux/4) : (ux+uy/4)); \
   phs = ((uy > ux) ? ((_UA/4) - ATAN_APPROX(ux,uy)) : ((ux == 0) ? 0 : ATAN_APPROX(uy,ux))); \ 
   phs = (((x) < 0) ? ((_UA/2) - phs) : phs); \ 
   phs = (((y) < 0) ? -phs : phs); \  
} while(0)

#undef PWR_075_AMPLITUDE_TABLE
#undef SQUARE_ROOT_AMPLITUDE_TABLE

#ifdef SQUARE_ROOT_AMPLITUDE_TABLE
#define AMPLITUDE_TABLE_PRESENT
const uint8_t PROGMEM amplitude_table[256] =
{
  0,16,22,27,32,35,39,42,45,48,50,53,55,57,59,61,64,65,67,69,71,73,75,76,78,80,81,83,84,86,87,89,90,91,93,94,96,97,98,99,101,102,103,104,106,107,108,109,110,112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,128,128,129,130,131,132,133,134,135,136,137,138,139,140,141,142,143,144,144,145,146,147,148,149,150,150,151,152,153,154,155,155,156,157,158,159,160,160,161,162,163,163,164,165,166,167,167,168,169,170,170,171,172,173,173,174,175,176,176,177,178,178,179,180,181,181,182,183,183,184,185,185,186,187,187,188,189,189,190,191,192,192,193,193,194,195,195,196,197,197,198,199,199,200,201,201,202,203,203,204,204,205,206,206,207,208,208,209,209,210,211,211,212,212,213,214,214,215,215,216,217,217,218,218,219,219,220,221,221,222,222,223,224,224,225,225,226,226,227,227,228,229,229,230,230,231,231,232,232,233,234,234,235,235,236,236,237,237,238,238,239,240,240,241,241,242,242,243,243,244,244,245,245,246,246,247,247,248,248,249,249,250,250,251,251,252,252,253,253,254,254,255 
};
#endif

#ifdef PWR_075_AMPLITUDE_TABLE
#define AMPLITUDE_TABLE_PRESENT
const uint8_t PROGMEM amplitude_table[256] =
{
 0,4,6,9,11,13,15,17,19,20,22,24,25,27,28,30,32,33,34,36,37,39,40,42,43,44,46,47,48,49,51,52,53,55,56,57,58,60,61,62,63,64,65,67,68,69,70,71,72,74,75,76,77,78,79,80,81,82,84,85,86,87,88,89,90,91,92,93,94,95,96,97,98,99,100,101,102,103,104,105,106,108,108,109,110,111,112,113,114,115,116,117,118,119,120,121,122,123,124,125,126,127,128,129,130,131,132,133,134,134,135,136,137,138,139,140,141,142,143,144,145,145,146,147,148,149,150,151,152,153,153,154,155,156,157,158,159,160,161,161,162,163,164,165,166,167,168,168,169,170,171,172,173,174,174,175,176,177,178,179,179,180,181,182,183,184,184,185,186,187,188,189,189,190,191,192,193,194,194,195,196,197,198,199,199,200,201,202,203,203,204,205,206,207,207,208,209,210,211,211,212,213,214,215,215,216,217,218,219,219,220,221,222,223,223,224,225,226,226,227,228,229,230,230,231,232,233,233,234,235,236,237,237,238,239,240,240,241,242,243,243,244,245,246,246,247,248,249,249,250,251,252,252,253,254,255
};
#endif

void ssb_interrupt(int16_t sample)
{
  uint16_t magnitude;
  int16_t phase, phase_difference, frequency;
  int16_t ac, in_val, quad_val;
  int32_t in_prod, quad_prod;

  if (!ds.ssb_active) return;

  ps.ssbs.cumulative_sample += sample; 
  if ((++ps.ssbs.sampling_clock) < SAMPLING_CLOCK)
   return;

  ps.ssbs.sampling_clock = 0;

  ADMUX = (1 << REFS0) | (1 << MUX0);  // set to sample the ADC1 which is the MIC audio
  ADCSRA = (1 << ADEN) | (1 << ADPS1) | (1 << ADPS0) | (1 << ADSC);  
    
  for (uint8_t i=0;i<SSB_FIR_LENGTH-1;i++)
    ps.ssbs.ssb_fir_buffer[i] = ps.ssbs.ssb_fir_buffer[i+1];

  ac = ps.ssbs.cumulative_sample * 2;
  ac += ps.ssbs.lpf_z1;              // LPF
  ps.ssbs.lpf_z1 = (ps.ssbs.cumulative_sample - (ps.ssbs.lpf_z1 * 4) + ps.ssbs.lpf_z1) / (3 + 1);  // guess that (3+1) is to make this an affine sum
  ps.ssbs.dc_level = (ac + (ps.ssbs.dc_level * 4) - ps.ssbs.dc_level) / (3 + 1); 
  ps.ssbs.ssb_fir_buffer[SSB_FIR_LENGTH-1] = (ac - ps.ssbs.dc_level) / 2;

  in_val = ps.ssbs.ssb_fir_buffer[SSB_FIR_LENGTH/2-1];
  quad_val = ( (ps.ssbs.ssb_fir_buffer[0] - ps.ssbs.ssb_fir_buffer[14]) * 2 + 
               (ps.ssbs.ssb_fir_buffer[2] - ps.ssbs.ssb_fir_buffer[12]) * 8 +
               (ps.ssbs.ssb_fir_buffer[4] - ps.ssbs.ssb_fir_buffer[10]) * 21 + 
               (ps.ssbs.ssb_fir_buffer[6] - ps.ssbs.ssb_fir_buffer[8]) * 15) / 128
           + (ps.ssbs.ssb_fir_buffer[6] - ps.ssbs.ssb_fir_buffer[8]) / 2; 
     // PE1NNZ Hilbert transform, 40dB side-band rejection in 400..1900Hz (@4kSPS) when used in image-rejection scenario; (Hilbert transform require 5 additional bits)
     
#ifdef SSB_DEBUG_REGISTERS
  ps.ssbs.last_sample = in_val;
  ps.ssbs.last_sample2 = quad_val;
#endif

  OCTAGON_MAGNITUDE_PHASE(magnitude, phase, in_val, quad_val); 

  magnitude = (magnitude * ps.ssbs.gain) >> 3;
  if (magnitude > 255) magnitude = 255;
#ifdef AMPLITUDE_TABLE_PRESENT
  magnitude = pgm_read_byte_near(&amplitude_table[magnitude]);
#endif
  
  set_transmit_pwm(((int32_t)magnitude*(int32_t)(TIMER1_COUNT_MAX/2)) >> 8); 
  ps.ssbs.magnitude = magnitude;
  
  phase_difference = phase -  ps.ssbs.previous_phase;
  ps.ssbs.previous_phase = phase;   // calculate phase difference to get instantaneous frequency of signal

  if (phase_difference < 0)       // negative phase shifts should not occur if the Hilbert filter is calculating a slowly varying signal
  {
    phase_difference += _UA;        // do not allow negative shifts to prevent spurs
#ifdef SSB_DEBUG_REGISTERS
    ps.ssbs.phase_inversions++;
#endif
  }

#ifdef MAX_PHASE
  if (phase_difference > MAX_PHASE)
  {
    ps.ssbs.previous_phase = phase + MAX_PHASE - phase_difference;
    phase_difference = MAX_PHASE;
  }
#endif

  frequency = (((uint32_t)phase_difference) * FRACTIONAL_MULT_FREQUENCY) / 256;
  if (ps.ssbs.protocol == PROTOCOL_LSB)
    frequency = -frequency;
  set_frequency_offset(frequency);

#ifdef SSB_DEBUG_REGISTERS
  ps.ssbs.frequency_shift = frequency; 
  ps.ssbs.phase_difference = phase_difference;
  ps.ssbs.no_interrupts++;
#endif
  ps.ssbs.cumulative_sample = 0;
}

int16_t ssb_frequency_offset(void)
{
  return 0;
}

uint16_t ssb_get_magnitude(void)
{
  return ps.ssbs.magnitude;
}
