/* RFBitBanger.ino

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
#include <Wire.h>
#include <avr/pgmspace.h>
#include "RFBitBanger.h"
#include "LiquidCrystalButtons.h"
#include "si5351simple.h"
#include "ui.h"

#define LCDB_RS A2
#define LCDB_E A3
#define LCDB_DB4 7
#define LCDB_DB5 8
#define LCDB_DB6 9
#define LCDB_DB7 10

#define TRANSMIT_PIN 5

LiquidCrystalButtons lcd(LCDB_RS, LCDB_E, LCDB_DB4, LCDB_DB5, LCDB_DB6, LCDB_DB7);
si5351simple si5351(8,27000000u);

const char mainmenutitle[] PROGMEM = "Menu";
const char mainmenu1[] PROGMEM = "Optionmenu1";
const char mainmenu2[] PROGMEM = "Optionmenu2";
const char mainmenu3[] PROGMEM = "Optionmenu3";
const char mainmenu4[] PROGMEM = "Optionmenu4";
const char mainmenu5[] PROGMEM = "Optionmenu5";
const char mainmenu6[] PROGMEM = "Optionmenu6";
const char mainmenu7[] PROGMEM = "Optionmenu7";

const char *const mainmenu[] PROGMEM = {mainmenu1,mainmenu2,mainmenu3,mainmenu4,mainmenu5,mainmenu6,mainmenu7,NULL };

const char freqmenutitle[] PROGMEM = "Frequency";

void idle_task(void)
{
  lcd.pollButtons();
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  pinMode(TRANSMIT_PIN,OUTPUT);
  si5351.start();
  lcd.begin(20,4);
  // Print a message to the LCD.
  //lcd.print("hello, world!");
  lcd.setCursor(0,0);
  lcdPrintFlash(freqmenutitle);
  digitalWrite(TRANSMIT_PIN,LOW);
}

void loop() {
  static uint32_t n = 0;
  static uint8_t pos = 0;
  scroll_number(&n, &pos, 0, 99999999, 8, 0, NULL);
  lcd.setCursor(0,2);
  lcd.print(n);

  si5351_synth_regs s_regs;
  si5351_multisynth_regs m_regs;
  
  si5351.calc_registers(n, &s_regs, &m_regs);
  si5351.set_registers(0, &s_regs, 0, &m_regs);
  si5351.setOutputOnOff(0,1);
  
  //do_menu(mainmenu,mainmenutitle,0);
}

void loop2() {
  static uint16_t b = 0;
  uint16_t c;

  c = millis() / 1000;
  {
    b = c;
    lcd.setCursor(0, 1);
    // print the number of seconds since reset:
    lcd.print(millis() / 1000);
    lcd.print(" ");
    lcd.print(lcd.readButton(0));
    lcd.print(lcd.readButton(1));
    lcd.print(lcd.readButton(2));
    lcd.print(lcd.readButton(3));
    Serial.println(millis() / 1000);
  }
  lcd.pollButtons();
}
