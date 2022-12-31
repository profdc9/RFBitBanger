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

void lcdBarGraph(bargraph_dat *bgd)
{
    uint8_t ct = bgd->width_bars;
    int8_t width = bgd->bars;
    lcd.setCursor(bgd->col_bars,bgd->row_bars);
    while ((width > 0) && (ct > 0))
    {
      if (width > 4)
        lcd.write(0xFF);
      else
        lcd.write(width - 1);
      width -= 5;
      ct--;
    }
    while (ct > 0)
    {
      lcd.write(' ');
      ct--;
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
    if (c == 0) break;
    lcd.write(c);
  }
}

void lcdPrintFlashSpaces(const char *str, uint8_t len)
{
  while (len > 0)
  {
    char c = pgm_read_byte_near(str++);
    if (c == 0) break;
    lcd.write(c);
    len--;
  }
  while (len > 0)
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
  lcdPrintFlashSpaces(pgm_read_word_near(&menu->items[menu->item]), menu->width);
}

uint8_t button_left_actual(uint8_t key)
{
  return (lcd.getButtonPressed(2) || (key == PS2KEY_LEFT));
}

uint8_t button_right_actual(uint8_t key)
{
  return (lcd.getButtonPressed(3) || (key == PS2KEY_RIGHT));
}

uint8_t button_up_actual(uint8_t key)
{
  return (lcd.getButtonPressed(0) || (key == PS2KEY_UP));
}

uint8_t button_down_actual(uint8_t key)
{
  return (lcd.getButtonPressed(1) || (key == PS2KEY_DOWN));
}

uint8_t button_enter(uint8_t key)
{
  return (lcd.getButtonPressed(4) || (key == 0x0D));
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
  } else if ((button_down(key)) && (menu->item > 0))
  {
    menu->item--;
    do_show_menu_item(menu);
  } else if ((button_up(key)) && (pgm_read_word_near(&menu->items[menu->item + 1]) != NULL))
  {
    menu->item++;
    do_show_menu_item(menu);
  }
  return 0;
}

char *number_str(char *s, uint32_t n, uint8_t digits, uint8_t decs)
{
  char *d = s + digits + 2;
  *d = '\000';
  if (!decs) --decs;
  while (digits > 0)
  {
    uint32_t n10 = n / 10;
    uint8_t dig = n - n10 * 10;
    n = n10;
    if ((decs--) == 0) *(--d) = '.';
    *(--d) = dig + '0';
    digits--;
  }
  return d;
}

uint32_t pow10(uint8_t n)
{
  uint32_t v = 1;
  while ((n--) > 0) v *= 10;
  return v;
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

void scroll_number_redraw(scroll_number_dat *snd)
{
  lcd.setCursor(snd->col, snd->row);
  lcdPrintNum(snd->n, snd->digits, snd->decs);
}

void scroll_number_key(scroll_number_dat *snd)
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
    if (snd->position < snd->digits)
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
    if (snd->position < (snd->digits - 1))
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
    scroll_number_redraw(snd);
}

void scroll_number_start(scroll_number_dat *snd)
{
  snd->changed = 0;
  snd->entered = 0;
  scroll_number_redraw(snd);
  lcd.cursor();
}

void scroll_number_stop(scroll_number_dat *snd)
{
  lcd.noCursor();
}

void scroll_alpha_redraw(scroll_alpha_dat *sad)
{
  uint8_t startpos, dp;

  dp = sad->displen >> 1;
  if (sad->position < dp)
    startpos = 0;
  else if (sad->position > (sad->numchars - dp))
    startpos = sad->numchars - sad->displen;
  else
    startpos = sad->position - dp;
  sad->cursorpos = sad->position - startpos;
  lcd.setCursor(sad->col, sad->row);
  for (dp = 0; dp < sad->displen; dp++)
    lcd.write(sad->buffer[dp + startpos]);
}

void scroll_alpha_clear(scroll_alpha_dat *sad)
{
  lcd.setCursor(sad->col, sad->row);
  for (uint8_t dp = 0; dp < sad->displen; dp++)
    lcd.write(' ');
}

uint8_t scroll_alpha_find_key(scroll_alpha_dat *sad, uint8_t key)
{
  uint8_t cnt;

  for (cnt = 0; cnt < sad->num_validchars; cnt++)
  {
    uint8_t cmp_key = (uint8_t)pgm_read_byte_near(&sad->validchars[cnt]);
    if (key == cmp_key) return cnt;
  }
  return 0xFF;
}

void scroll_alpha_key(scroll_alpha_dat *sad)
{
  uint8_t redraw = 1;
  uint8_t key = PSkey.getkey();

  lcd.setCursor(sad->col + sad->cursorpos, sad->row);
  idle_task();
  if (button_enter(key))
  { 
    sad->entered = 1;
    return;
  } else if (button_left(key))
  {
    if (sad->position > 0)
      sad->position--;
    else
      sad->entered = 1;
  } else if (button_right(key))
  {
    if (sad->position < (sad->numchars - 1))
      sad->position++;
    else
      sad->entered = 1;
  } else if (button_up(key))
  {
    uint8_t curkey = sad->buffer[sad->position];
    uint8_t val = scroll_alpha_find_key(sad, curkey);
    if (val != 0xFF)
    {
      if ((++val) >= sad->num_validchars) val = 0;
      sad->buffer[sad->position] = (uint8_t)pgm_read_byte_near(&sad->validchars[val]);
      sad->changed = 1;
    }
  } else if (button_down(key))
  {
    uint8_t curkey = sad->buffer[sad->position];
    uint8_t val = scroll_alpha_find_key(sad, curkey);
    if (val != 0xFF)
    {
      if (val == 0) val = sad->num_validchars - 1;
      else val--;
      sad->buffer[sad->position] = (uint8_t)pgm_read_byte_near(&sad->validchars[val]);
      sad->changed = 1;
    }
  } else
  {
    if (key != PS2KEY_NONE)
    {
      key = (key >= 'a') && (key <= 'z') ? key - 32 : key;
      uint8_t val = scroll_alpha_find_key(sad, key);
      if (val != 0xFF)
      {
        sad->buffer[sad->position] = key;
        sad->changed = 1;
        if (sad->position < (sad->numchars - 1))
          sad->position++;
      }
    } else redraw = 0;
  }
  if (redraw)
    scroll_alpha_redraw(sad);
}

void scroll_alpha_stop(scroll_alpha_dat *sad)
{
  lcd.noCursor();
}

void scroll_alpha_start(scroll_alpha_dat *sad)
{
  sad->changed = 0;
  sad->entered = 0;
  if (sad->buffer[0] == 0)
    memset(sad->buffer, ' ', sad->numchars);
  scroll_alpha_redraw(sad);
  lcd.cursor();
}

bool show_lr(uint8_t row, const char *message)
{
  lcd.clearButtons();
  lcd.setCursor(0,row);
  lcdPrintFlashSpaces(message,16);
  for (;;) {
    uint8_t key = PSkey.getkey();
    idle_task();
    if (button_left(key))
      return false;
    if (button_right(key))
      return true;
  }
}

void scroll_readout_initialize(scroll_readout_dat *srd)
{
  for (uint8_t i = 0; i < srd->numchars; i++)
    //srd->buffer[i] = ' ';
    srd->buffer[i] = '!' + i;
  srd->position = srd->numchars - srd->displen;
  srd->exited = 0;
  srd->notchanged = 0;
}

void scroll_readout_add_character(scroll_readout_dat *srd, uint8_t ch)
{
  for (uint8_t i = 0; i < srd->numchars; i++)
    srd->buffer[i] = srd->buffer[i + 1];
  srd->buffer[srd->numchars - 1] = ch;
  srd->notchanged = 0;
}

void scroll_readout_back_character(scroll_readout_dat *srd, uint8_t ch)
{
  for (uint8_t i = srd->numchars; i > 0;)
  {
    --i;
    srd->buffer[i] = srd->buffer[i - 1];
  }
  srd->buffer[0] = ch;
  srd->notchanged = 0;
}

void scroll_readout_display(scroll_readout_dat *srd)
{
  if (!srd->notchanged)
  {
    lcd.setCursor(srd->col, srd->row);
    for (uint8_t dp = 0; dp < srd->displen; dp++)
      lcd.write(srd->buffer[dp + srd->position]);
    srd->notchanged = 1;
  }
}

void scroll_readout_key(scroll_readout_dat *srd)
{
  uint8_t key = PSkey.getkey();

  lcd.setCursor(srd->col, srd->row);
  idle_task();
  if (button_left(key))
  {
    if (srd->position > 0)
    {
      srd->position--;
      srd->notchanged = 0;
      scroll_readout_display(srd);
    }
  } else if (button_right(key))
  {
    if (srd->position < (srd->numchars - srd->displen))
    {
      srd->position++;
      srd->notchanged = 0;
      scroll_readout_display(srd);
    }
  } else if (button_up(key) || button_down(key))
  {
    srd->position = srd->numchars - srd->displen;
    srd->notchanged = 0;
    scroll_readout_display(srd);
    srd->exited = 1;
  }
}
