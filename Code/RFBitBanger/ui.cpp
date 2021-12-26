/* ui.cpp

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

#include <Arduino.h>
#include <avr/pgmspace.h>
#include "LiquidCrystalButtons.h"
#include "ui.h"
#include "mini-printf.h"
#include "RFBitBanger.h"

extern LiquidCrystalButtons lcd;

void lcdPrint(const char *str) 
{
  while (*str) lcd.write(*str++);
}

void lcdPrintNum(uint32_t n, uint8_t digits, uint8_t decs)
{
  char s[20];
  char *p = number_str(s, n, digits, decs);
  lcdPrint(p);
}

void lcdPrintFlash(const char *str)
{
  for (;;)
  {
    char c = pgm_read_byte(str++);
    if (c==0) break;
    lcd.write(c);
  }
}

uint8_t do_menu(const char *items[], const char *prompt, uint8_t item)
{
  for (;;)
  {
    lcd.clear();
    lcdPrintFlash(prompt);
    lcd.setCursor(0, 1);
    lcdPrintFlash(pgm_read_word_near(&items[item]));
    for (;;)
    {
      idle_task();
      if ((lcd.getButtonPressed(0)) && (item > 0)) 
      {
        item--;
        break;
      }
      if ((lcd.getButtonPressed(1)) && (pgm_read_word_near(&items[item+1]) != NULL))
      {
        item++;
        break;
      }
      if (lcd.waitButtonPressed(2))
      {
        lcd.clearButtons();
        return item;
      }
    }
  }
}

char *number_str(char *s, uint32_t n, uint8_t digits, uint8_t decs)
{
   char *d = s+digits+2;
   *d = '\000';
   if (!decs) --decs;   
   while (digits>0)
   {
      uint32_t n10 = n / 10;
      uint8_t dig = n - n10 * 10;
      n = n10;
      if ((decs--) == 0) *(--d) = '.';
      *(--d) = dig+'0';
      digits--;
   }
   return d;
}

uint32_t pow10(uint8_t n)
{
  uint32_t v = 1;
  while ((n--)>0) v *= 10;
  return v;
}

void scroll_number(uint32_t *num, uint8_t *init_position,
                   uint32_t minimum_number, uint32_t maximum_number,
                   uint8_t digits, uint8_t decs, const char *prompt)
{
  uint8_t redraw = 1;
  uint32_t n = *num;
  uint8_t position = *init_position;

  if (prompt != NULL) lcdPrintFlash(prompt);
  lcd.cursor();
  lcd.blink();
  for (;;)
  {
    idle_task();
    if (redraw) 
    {
      lcd.setCursor(0,1);
      lcdPrintNum(n, digits, decs);
      lcd.setCursor(position + ((decs != 0) && ((position + decs) >= digits)), 1);
      redraw = 0;
    }
    if ((lcd.getButtonPressed(0)) && (position > 0))
    {
      position--;
      redraw = 1;
    }
    if ((lcd.getButtonPressed(1)) && (position < digits))
    {
      position++;
      redraw = 1;
    }
    if (lcd.getButtonPressed(2))
    {
      if (position >= digits) break;
      uint32_t p10 = pow10(digits - position - 1);
      if ((n + p10) <= maximum_number)
      {
        n += p10;
        if (prompt == NULL) break;
      }
      redraw = 1;
    }
    if (lcd.getButtonPressed(3))
    {
      if (position >= digits) break;
      uint32_t p10 = pow10(digits - position - 1);
      if (n >= (minimum_number + p10))
      {
        n -= p10;
        if (prompt == NULL) break;
      }
      redraw = 1;
    }
  }
  *num = n;
  *init_position = position;
  lcd.noBlink();
  lcd.noCursor();
  return;
}

bool show_messages(const char *message1, const char *message2)
{
  lcd.clearButtons();
  lcd.clear();
  lcd.print(message1);
  lcd.setCursor(0,1);
  lcd.print(message2);
  for (;;) {
     idle_task();
     if (lcd.waitButtonPressed(0)) 
       return false;
     if (lcd.waitButtonPressed(1))
       return true;
  }
}

bool go_or_abort(const char *message)
{
  return show_messages(message, "Rt cont,lft abrt");
}
