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
#include "PS2Keyboard.h"
#include "RFBitBanger.h"

#undef BAR_GRAPH_DEBUG

void lcdBarGraph(bargraph_dat *bgd)
{
  for (uint8_t n=0; n<bgd->num_bars; n++)
  {
    uint8_t ct = bgd->width_bars;
    int8_t width = bgd->bars[n];
    lcd.setCursor(bgd->col_bars,bgd->row_bars+n);
#ifdef BAR_GRAPH_DEBUG
    lcd.print(width);
    lcd.print("    ");
#else
    while ((width > 0) && (ct > 0))
    {
      if (width > 4) 
        lcd.write(0xFF);
      else
        lcd.write(width-1);
      width -= 5;
      ct--;
    }
    while (ct>0)
    {
      lcd.write(' ');
      ct--;
    }
#endif
  }
}

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
    char c = pgm_read_byte_near(str++);
    if (c==0) break;
    lcd.write(c);
  }
}

void lcdPrintFlashSpaces(const char *str, uint8_t len)
{
  while (len>0)
  {
    char c = pgm_read_byte_near(str++);
    if (c==0) break;
    lcd.write(c);
    len--;
  }
  while (len>0)
  {
    lcd.write(' ');
    len--;
  }
}

static uint8_t horiz_keys = 0;

void set_horiz_menu_keys(uint8_t horiz)
{
  horiz_keys = horiz;
}

void do_show_menu_item(menu_str *menu)
{
  lcd.setCursor(menu->col, menu->row);
  lcdPrintFlashSpaces(pgm_read_word_near(&menu->items[menu->item]),menu->width);
}

uint8_t button_left_actual(uint8_t key)
{
  return (lcd.waitButtonPressed(2) || (key == PS2KEY_LEFT));
}

uint8_t button_right_actual(uint8_t key)
{
  return (lcd.waitButtonPressed(3) || (key == PS2KEY_RIGHT));
}

uint8_t button_up_actual(uint8_t key)
{
  return (lcd.getButtonPressed(0) || (key == PS2KEY_UP));
}

uint8_t button_down_actual(uint8_t key)
{
  return (lcd.getButtonPressed(1) || (key == PS2KEY_DOWN));
}

uint8_t button_left(uint8_t key)
{
  return horiz_keys ? button_up_actual(key) : button_left_actual(key);
}

uint8_t button_right(uint8_t key)
{
  return horiz_keys ? button_down_actual(key) : button_right_actual(key);
}

uint8_t button_up(uint8_t key)
{
    return horiz_keys ? button_left_actual(key) : button_up_actual(key);
}

uint8_t button_down(uint8_t key)
{
  return horiz_keys ? button_right_actual(key) : button_down_actual(key);
}

uint8_t do_menu(menu_str *menu)
{
  uint8_t key = PSkey.getkey();
  idle_task();
  if (button_left(key))
  {
     lcd.clearButtons();
     return 1;
  } else if (button_right(key))
  {
     lcd.clearButtons();
     return 2;  
  } else if ((button_up(key)) && (menu->item > 0)) 
  {
     menu->item--;
     do_show_menu_item(menu);
  } else if ((button_down(key)) && (pgm_read_word_near(&menu->items[menu->item+1]) != NULL))
  {
     menu->item++;
     do_show_menu_item(menu);
  } 
  return 0;
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

void scroll_redraw(scroll_number_dat *snd)
{
  lcd.setCursor(snd->col,snd->row);
  lcdPrintNum(snd->n, snd->digits, snd->decs);
}

uint8_t abort_button_left(void)
{
  uint8_t key = PSkey.getkey();
  return ((key == PS2KEY_LEFT) || lcd.readUnBounced(2));
}

uint8_t abort_button_right(void)
{
  uint8_t key = PSkey.getkey();
  return ((key == PS2KEY_RIGHT) || lcd.readUnBounced(3));
}

void scroll_key(scroll_number_dat *snd)
{
  uint8_t redraw = 1;
  uint8_t key = PSkey.getkey();

  lcd.setCursor(snd->col + snd->position + ((snd->decs != 0) && ((snd->position + snd->decs) >= snd->digits)), snd->row);
  idle_task();
  if (key == PS2KEY_ENTER)
  { 
    snd->entered = 1;
    return;
  } else if (((key >= '0') && (key <= '9')) && (snd->position < snd->digits))
  {
    uint32_t p10 = pow10(snd->digits - snd->position - 1);
    uint8_t dig = (snd->n / p10) % 10;
    int8_t dif =  key - '0' - dig;
    if (dif != 0)
    {
        snd->n += ((int32_t)p10) * dif;
        snd->changed = 1;
    }
    snd->position++;
    key = 0;
  } else if (button_left(key))
  {
    if (snd->position > 0)
        snd->position--;
    else
        snd->entered = 1;
  } else if (button_right(key))
  {
    if (snd->position < (snd->digits-1))
        snd->position++;
    else
        snd->entered = 1;
  } else if (button_up(key))
  {
    uint32_t p10 = pow10(snd->digits - snd->position - 1);
    if ((snd->n + p10) <= snd->maximum_number)
    {
      snd->n += p10;
      snd->changed = 1;
    }
  } else if (button_down(key))
  {
    if (snd->position >= snd->digits)
    {
      snd->entered = 1;
      return;
    }
    uint32_t p10 = pow10(snd->digits - snd->position - 1);
    if (snd->n >= (snd->minimum_number + p10))
    {
      snd->n -= p10;
      snd->changed = 1;
    }
  } else redraw = 0;
  if (redraw)
    scroll_redraw(snd);
}

void scroll_number_start(scroll_number_dat *snd)
{
  snd->changed = 0;
  snd->entered = 0;
  scroll_redraw(snd);
  lcd.cursor();
}

void scroll_number_stop(scroll_number_dat *snd)
{
  lcd.noCursor();
}

void scroll_number(scroll_number_dat *snd)
{
  uint8_t redraw = 1;

  scroll_number_start(snd);
  for (;;)
  {
    scroll_key(snd);
    if (snd->entered) break;
  }
  scroll_number_stop(snd);
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
     uint8_t key = PSkey.getkey();
     idle_task();
     if (button_left(key)) 
       return false;
     if (button_right(key))
       return true;
  }
}

bool go_or_abort(const char *message)
{
  return show_messages(message, "Rt cont,lft abrt");
}
