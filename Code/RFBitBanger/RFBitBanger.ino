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

#define LCDB_RS A3
#define LCDB_E A2
#define LCDB_DB4 5
#define LCDB_DB5 6
#define LCDB_DB6 7
#define LCDB_DB7 8

#define TRANSMIT_PIN 9
#define BACKLIGHT_PIN 11
#define MUTEAUDIO_PIN 4
#define BEEPOUT_BIN 3

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

#define TIMER1_COUNT_MAX 4000

void toggle_backlight(void)
{
  static uint16_t count = 0;
  static uint8_t b = 0;
  if ((++count) > 1000)
  {
    count = 0;
    b=!b;
    //digitalWrite(BACKLIGHT_PIN,b);
  }
}

ISR(TIMER1_OVF_vect)
{
  toggle_backlight();
}

void write_transmit_pwm(uint16_t pwm)
{
  OCR1AH = pwm >> 8;
  OCR1AL = pwm & 0xFF;
}

void write_tuning_pwm(uint16_t pwm)
{
  OCR1BH = pwm >> 8;
  OCR1BL = pwm & 0xFF;
}

void setup_timers(void)
{
  cli();
  // WGM13 = 1, WGM12 = 1, WGM11 = 1, WGM10 = 0
  TCCR1A = /* (1<<COM1A1) | */ (1<<COM1B1) | (1<<WGM11);  
  TCCR1B = (1<<WGM13) | (1<<WGM12) | (1<<CS10);
  ICR1H = (TIMER1_COUNT_MAX >> 8);
  ICR1L = (TIMER1_COUNT_MAX & 0xFF);
  write_transmit_pwm(1);
  write_tuning_pwm(0);
  TIMSK1 = (1<<TOIE1);

  TCCR2A = (1<<COM2B1) | (1 << WGM21) | (1 << WGM20);
  TCCR2B = (1 << WGM22) | (1<<CS22) | (1<<CS21);   // 256 divider
  OCR2A = 0xFF;
  OCR2B = 0x80;
  TIMSK2 = 0;
  sei();
}

void tone_off(void)
{
  pinMode(BEEPOUT_BIN,INPUT);
}

#define TONEFREQ_BASEFREQ (16000000ul / 256)
#define TONEFREQ(x) (TONEFREQ_BASEFREQ / (x))
#define TONEFREQVOL(x,vol) (((TONEFREQ_BASEFREQ / (x)) * (vol)) >> 8)
#define TONE_ON(freq,vol) tone_on(TONEFREQ(freq),TONEFREQVOL(freq,vol))
#define TONE_OFF() tone_off()

void tone_on(uint8_t freq, uint8_t vol)
{
  OCR2A = freq;
  OCR2B = vol;
  pinMode(BEEPOUT_BIN,OUTPUT);
}

void setup() {
  setup_timers();
  // put your setup code here, to run once:
  Serial.begin(57600);
  pinMode(TRANSMIT_PIN,OUTPUT);
  pinMode(BACKLIGHT_PIN,OUTPUT);
  pinMode(MUTEAUDIO_PIN,OUTPUT);
  TONE_OFF();
  si5351.start();
  lcd.begin(20,4);
  // Print a message to the LCD.
  //lcd.print("hello, world!");
  lcd.setCursor(0,0);
  lcdPrintFlash(freqmenutitle);
  digitalWrite(TRANSMIT_PIN,LOW);
  digitalWrite(MUTEAUDIO_PIN,LOW);
  digitalWrite(BACKLIGHT_PIN,HIGH);
}

void loop() {
  static uint32_t n = 0;
  static uint8_t pos = 0;
  scroll_number(&n, &pos, 0, 99999999, 8, 0, NULL);
//  digitalWrite(MUTEAUDIO_PIN,n & 0x01);
  //write_transmit_pwm(n & 0x01 ? TIMER1_COUNT_MAX-1 : 0);
  digitalWrite(TRANSMIT_PIN, n & 0x01);
  //lcd.setCursor(0,2);
  //lcd.print(n);

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
