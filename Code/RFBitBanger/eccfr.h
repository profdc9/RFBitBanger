
/*  eccfr.h */

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

#ifndef __ECCFR_H
#define __ECCFR_H

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*eccfr_code_word_put)(uint16_t, void *);
typedef uint16_t (*eccfr_code_word_get)(void *);

void eccfr_bytes_to_code_words(uint8_t *bytes, uint8_t num_bytes, eccfr_code_word_put ecwp, void *st);
void eccfr_raw_bytes_to_code_words(uint8_t *bytes, uint8_t num_bytes, eccfr_code_word_put ecwp, void *st);
uint8_t eccfr_code_words_to_bytes(eccfr_code_word_get ecwg, void *st, uint8_t *bytes, uint8_t max_bytes);
uint32_t eccfr_add_reversal_bits(uint32_t codeword);
uint32_t eccfr_remove_reversal_bits(uint32_t outword);

#ifdef __cplusplus
}
#endif

#endif   /* __ECCFR_H */
