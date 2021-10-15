
/*  eccfr.c */

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

#define ECCFR_DEBUG

#ifdef ECCFR_DEBUG
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#endif

#include "eccfr.h"

const uint8_t ecc_6bit_codesymbols[60] = {'\0',   '\b',   '\r',    ' ',    '!',   0x22,   0x27,    '(',
                                           ')',    '*',    '+',    ',',    '-',    '.',    '/',    '0',
                                           '1',    '2',    '3',    '4',    '5',    '6',    '7',    '8',
                                           '9',    ':',    ';',    '=',    '?',    '@',    'A',    'B',
                                           'C',    'D',    'E',    'F',    'G',    'H',    'I',    'J',
                                           'K',    'L',    'M',    'N',    'O',    'P',    'Q',    'R',
                                           'S',    'T',    'U',    'V',    'W',    'X',    'Y',    'Z',
                                           0x5C,   '^',    '`',    '~' };
                                           /* Last 4 symbols can be interpreted as diacritical marks, 0x5C is diaeresis/umlaut */
                                           /* 0x27 can be interpreted as acute diacritical mark */

/*  The input word has 24 bits.  The output word has 30 bits, with bits
    1, 5, 9, 13, 17, 21 preceded by its complement bit inserted into
    the word */

uint32_t eccfr_add_reversal_bits(uint32_t codeword)
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

uint32_t eccfr_remove_reversal_bits(uint32_t outword)
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

/* decode 12 bit code words to 8 bit bytes */
uint8_t eccfr_code_words_to_bytes(uint16_t *codewords, uint8_t num_words, uint8_t *bytes, uint8_t max_bytes)
{
    uint8_t cur_word = 0;
    uint8_t cur_byte = 0;
    while ((cur_word < num_words) && (cur_byte < max_bytes))
    {
        uint16_t code = codewords[cur_word++];
        if ((code & 0xF00) == 0xF00)
        {
            bytes[cur_byte++] = code & 0xFF;
            continue;
        }
        uint16_t code1 = (code & 0x3F);
        uint16_t code2 = ((code >> 6) & 0x3F);
        if ((code1 != 0) && (code1 < (sizeof(ecc_6bit_codesymbols)/sizeof(uint8_t))))
            bytes[cur_byte++] = ecc_6bit_codesymbols[code1];
        if ((code2 != 0) && (code1 < (sizeof(ecc_6bit_codesymbols)/sizeof(uint8_t))) && (cur_byte < max_bytes))
            bytes[cur_byte++] = ecc_6bit_codesymbols[code2];
    }
    return cur_byte;
}

/* encode 8 bit bytes as 12 bit code words, always encoding as 8-bit raw */
uint8_t eccfr_raw_bytes_to_code_words(uint8_t *bytes, uint8_t num_bytes, uint16_t *codewords, uint8_t max_words)
{
  uint8_t cur_byte = 0;
  uint8_t cur_word = 0;
  while ((cur_word < max_words) && (cur_byte < num_bytes))
  {
     uint8_t b = bytes[cur_byte++];
     codewords[cur_word++] = ((uint16_t)(0xF00)) | b;
  }
  return cur_word;
}

/* convert character to 6-bit code if it exists */
/* should probably replace this with an inverse look up table later */
uint8_t eccfr_find_code_in_table(uint8_t c)
{
    uint8_t i;
    if ((c >= 'a') && (c <= 'z')) c -= 32;
	if (c =='\n') c = '\r';
    for (i=1;i<(sizeof(ecc_6bit_codesymbols)/sizeof(uint8_t));i++)
        if (ecc_6bit_codesymbols[i] == c) return i;
    return 0xFF;
}

/* encode 8 bit bytes to 12 bit code words.
   code words that correspond to a 6-bit symbols are encoded as 6 bits.
   otherwise they are encoded as an 8-bit binary raw data word.
   If a byte that can be encoded as a 6 bit symbol precedes one that can
   not be encoded as a 6 bit symbol, and there is an extra symbol slot
   in the current word, fill it with a zero. */
uint8_t eccfr_bytes_to_code_words(uint8_t *bytes, uint8_t num_bytes, uint16_t *codewords, uint8_t max_words)
{
  uint8_t cur_byte = 0;
  uint8_t cur_word = 0;
  while ((cur_word < max_words) && (cur_byte < num_bytes))
  {
     uint8_t b = bytes[cur_byte++];
     uint8_t code1 = eccfr_find_code_in_table(b);
     if (code1 == 0xFF)
     {
         codewords[cur_word++] = ((uint16_t)(0xF00)) | b;
         continue;
     }
     if (cur_byte < num_bytes)
     {
         b = bytes[cur_byte];
         uint8_t code2 = eccfr_find_code_in_table(b);
         if (code2 != 0xFF)
         {
             codewords[cur_word++] = (((uint16_t)code2) << 6) | code1;
             cur_byte++;
             continue;
         }
     }
     codewords[cur_word++] = (uint16_t)code1;
  }
  return cur_word;
}

#ifdef ECCFR_DEBUG
void print_binary(uint32_t n, uint8_t d)
{
  uint8_t i;
  for (i=d;i>0;)
  {
    printf( (n & (1 << (--i))) ? "1" : "0");
    if (i>0) printf(",");
  }
}


void test_reversal_bits(void)
{
  uint32_t word1, ins, after;
  char s[100];
  uint8_t i;

  for (i=0;i<100;i++)
  {
      word1 = (((uint32_t)rand()) << 8) ^ rand();
      ins = eccfr_add_reversal_bits(word1);
      after = eccfr_remove_reversal_bits(ins);
      printf("before: ");
      print_binary(word1,24);
      printf("\nins: ");
      print_binary(ins,30);
      printf("\nafter: ");
      print_binary(after,24);
      printf("\n");
      if (after != word1)
        printf("!!!!!!!!!!!!!!!\n");
      gets(s);
  }
}

void print_hex_string(uint16_t *codes, uint8_t numcodes)
{
    uint8_t i;
    for (i=0;i<numcodes;i++)
    {
        if (i != 0) printf(",");
        printf("%03X",codes[i]);
    }
}

void test_words_to_bytes(void)
{
    uint8_t s[255];
    uint16_t codes[255];
    uint8_t decode[255];

    uint8_t i;
    for (i=0;i<10;i++)
    {
       uint8_t l, n, m;
       printf("enter string: ");
       gets(s);
       l = strlen(s);
       printf("entered string: %s, length: %d\n",s,l);
       n = eccfr_bytes_to_code_words(s, l, codes, (sizeof(codes)/sizeof(uint16_t)));
       printf("codes: ");
       print_hex_string(codes, n);
       m = eccfr_code_words_to_bytes(codes, n, decode, sizeof(decode)-1);
       decode[m] = '\000';
       printf("\ndecoded string: %s\n",decode);
    }
}

void main(void)
{
    test_words_to_bytes();
}

#endif

