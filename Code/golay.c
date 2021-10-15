
/*  golay.c */

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

#include "golay.h"

#define GOLAY_DEBUG

#ifdef GOLAY_DEBUG
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#endif

const uint16_t golay_matrix[12] =
{
    0b110111000101,
    0b101110001011,
    0b011100010111,
    0b111000101101,
    0b110001011011,
    0b100010110111,
    0b000101101111,
    0b001011011101,
    0b010110111001,
    0b101101110001,
    0b011011100011,
    0b111111111110
};

uint16_t golay_mult(uint16_t wd_enc)
{
   uint16_t enc = 0;
   uint8_t i;
   for (i=12;i>0;)
   {
      i--;
      if (wd_enc & 1) enc ^= golay_matrix[i];
      wd_enc >>= 1;
   }
   return enc;
}

uint8_t golay_hamming_weight(uint16_t n)
{
  uint8_t s = 0;
  while (n != 0)
  {
     s += (n & 0x1);
     n >>= 1;
  }
  return s;
}

uint32_t golay_encode(uint16_t wd_enc)
{
  uint16_t enc = golay_mult(wd_enc);
  return (((uint32_t)enc) << 12) | wd_enc;
}

uint16_t golay_decode(uint32_t codeword)
{
  uint16_t enc = codeword & 0xFFF;
  uint16_t parity = codeword >> 12;
  uint8_t i;
  uint16_t syndrome, parity_syndrome;

  /* if there are three or fewer errors in the parity bits, then
     we hope that there are no errors in the data bits, otherwise
     the error is undetected */
  syndrome = golay_mult(enc) ^ parity;
  if (golay_hamming_weight(syndrome) <= 3)
   return enc;

  /* check to see if the parity bits have no errors */
  parity_syndrome = golay_mult(parity) ^ enc;
  if (golay_hamming_weight(parity_syndrome) <= 3)
     return enc ^ parity_syndrome;

  /* we flip each bit of the data to see if we have two or fewer errors */
  for (i=12;i>0;)
  {
      i--;
      if (golay_hamming_weight(syndrome ^ golay_matrix[i]) <= 2)
        return enc ^ (((uint16_t)0x800) >> i);
  }

  /* we flip each bit of the parity to see if we have two or fewer errors */
  for (i=12;i>0;)
  {
      i--;
      uint16_t par_bit_synd = parity_syndrome ^ golay_matrix[i];
      if (golay_hamming_weight(par_bit_synd) <= 2)
        return enc ^ par_bit_synd;
  }

  return 0xFFFF;   /* uncorrectable error */
}

#ifdef GOLAY_DEBUG
void print_binary(uint32_t n, uint8_t d)
{
  uint8_t i;
  for (i=d;i>0;)
  {
    printf( (n & (1 << (--i))) ? "1" : "0");
    if (i>0) printf(",");
  }
}

uint32_t generate_n_errors(uint8_t n)
{
  uint32_t d = 0;
  while (n>0)
  {
    uint32_t bitmask = 1u << ((rand() % 24)+0);
    if (!(d & bitmask))
    {
        d |= bitmask;
        n--;
    }
  }
  return d;
}

void test_decode(void)
{
    uint8_t i;
    char s[100];
    for (i=0;i<100;i++)
    {
    uint16_t input = rand() & 0xFFF;
    printf("input:   ");
    print_binary(input,12);
    uint32_t enc = golay_encode(input);
    printf("\nencoded: ");
    print_binary(enc,24);
    uint16_t wd = golay_mult(enc >> 12);
    printf("\nmult:    ");
    print_binary(wd,12);
    uint32_t errors = generate_n_errors(3);
    printf("\nerrors:  ");
    print_binary(errors,24);
    enc ^= errors;
    printf("\nencerr:  ");
    print_binary(enc,24);
    uint16_t dec = golay_decode(enc);
    printf("\ndecode:  ");
    print_binary(dec,12);
    printf("\n");
    if (dec != input)
        printf("\n!!!!!!!!!!!!!!!!!\n");
    gets(s);
    }
}

void main(void)
{
    srand(111);
    test_decode();
}
#endif

