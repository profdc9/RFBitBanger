/* ui.h

*/

/*
   Copyright (c) 2021 Daniel Marks

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

#ifndef _UI_H
#define _UI_H

uint8_t do_menu(const char *items[], const char *prompt, uint8_t item);
bool show_messages(const char *message1, const char *message2);
bool go_or_abort(const char *message);
char *number_str(char *s, uint32_t n, uint8_t digits, uint8_t decs);
void lcdPrint(const char *str);
void lcdPrintFlash(const char *str);
void lcdPrintNum(uint32_t n, uint8_t digits, uint8_t decs);
void scroll_number(uint32_t *num, uint8_t *init_position,
                   uint32_t minimum_number, uint32_t maximum_number,
                   uint8_t digits, uint8_t decs, const char *prompt);



#endif  /* _UI_H */
