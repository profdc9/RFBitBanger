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
      set_frequency_receive();
      ds.ssb_active = 0;
    }
  } else
  {
    if (ds.ssb_active == 0)
    {
      if ((ps.ssbs.protocol == PROTOCOL_USB) || (ps.ssbs.protocol == PROTOCOL_LSB))
      {
/*        transmit_set(1);
        muteaudio_set(1);
        set_clock_onoff_mask(0x01); */
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

#define FRACTIONAL_MULT (uint32_t)(((SAMPLING_FREQUENCY)*256ul)/_UA)

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


void ssb_interrupt(int16_t sample)
{
  uint16_t magnitude;
  int16_t phase, phase_difference;
  int16_t ac, in_val, quad_val;
  int32_t in_prod, quad_prod;

  if ((ps.ssbs.protocol != PROTOCOL_USB) && (ps.ssbs.protocol != PROTOCOL_LSB))
    return;

  ps.ssbs.cumulative_sample += sample; 
  if ((++ps.ssbs.sampling_clock) < SAMPLING_CLOCK)
   return;

  ps.ssbs.sampling_clock = 0;

  ADMUX = (1 << REFS0) | (1 << MUX0);  // set to sample the ADC1 which is the MIC audio
  ADCSRA = (1 << ADEN) | (1 << ADPS1) | (1 << ADPS0) | (1 << ADSC);  
    
  for (uint8_t i=0;i<SSB_FIR_LENGTH-1;i++)
    ps.ssbs.ssb_fir_buffer[i] = ps.ssbs.ssb_fir_buffer[i+1];

  ac = ps.ssbs.cumulative_sample * 2;
  //ac += ps.ssbs.lpf_z1;              // LPF
  ps.ssbs.lpf_z1 = (ps.ssbs.cumulative_sample - (ps.ssbs.lpf_z1 * 4) - ps.ssbs.lpf_z1) / (3 + 1);  // guess that (3+1) is to make this an affine sum
  ps.ssbs.dc_level = (ac + (ps.ssbs.dc_level * 4) - ps.ssbs.dc_level) / (3 + 1); 
  ps.ssbs.ssb_fir_buffer[SSB_FIR_LENGTH-1] = (ac - ps.ssbs.dc_level) / 2;

  in_val = ps.ssbs.ssb_fir_buffer[SSB_FIR_LENGTH/2-1];
  quad_val = ( (ps.ssbs.ssb_fir_buffer[0] - ps.ssbs.ssb_fir_buffer[14]) * 2 + 
               (ps.ssbs.ssb_fir_buffer[2] - ps.ssbs.ssb_fir_buffer[12]) * 8 +
               (ps.ssbs.ssb_fir_buffer[4] - ps.ssbs.ssb_fir_buffer[10]) * 21 + 
               (ps.ssbs.ssb_fir_buffer[6] - ps.ssbs.ssb_fir_buffer[8]) * 15) / 128
           + (ps.ssbs.ssb_fir_buffer[6] - ps.ssbs.ssb_fir_buffer[8]) / 2; 
     // PE1NNZ Hilbert transform, 40dB side-band rejection in 400..1900Hz (@4kSPS) when used in image-rejection scenario; (Hilbert transform require 5 additional bits)
     
  ps.ssbs.last_sample = in_val;
  ps.ssbs.last_sample2 = quad_val;

  OCTAGON_MAGNITUDE_PHASE(magnitude, phase, in_val, quad_val); 

//  magnitude <<= ps.ssbs.drive;  
//  if (magnitude > 255) magnitude = 255;

  phase_difference = phase -  ps.ssbs.previous_phase;
  ps.ssbs.previous_phase = phase;   // calculate phase difference to get instantaneous frequency of signal

  if (phase_difference < 0)       // negative phase shifts should not occur if the Hilbert filter is calculating a slowly varying signal
    phase_difference += _UA;        // do not allow negative shifts to prevent spurs

#ifdef MAX_PHASE
  if (phase_difference > MAX_PHASE)
  {
    ps.ssbs.previous_phase = phase + MAX_PHASE - phase_difference;
    phase_difference = MAX_PHASE;
  }
#endif
  ps.ssbs.frequency_shift = (phase_difference * FRACTIONAL_MULT) / 256;
  if (ps.ssbs.protocol == PROTOCOL_LSB) 
    ps.ssbs.frequency_shift = -ps.ssbs.frequency_shift; 
  ps.ssbs.magnitude = magnitude;
  ps.ssbs.phase_difference = phase_difference;
  ps.ssbs.cumulative_sample = 0;

}
