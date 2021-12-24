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
#include "LiquidCrystalButtons.h"
#include "si5351simple.h"

#define LCDB_RS A2
#define LCDB_E A3
#define LCDB_DB4 7
#define LCDB_DB5 8
#define LCDB_DB6 9
#define LCDB_DB7 10

LiquidCrystalButtons lcd(LCDB_RS, LCDB_E, LCDB_DB4, LCDB_DB5, LCDB_DB6, LCDB_DB7);
si5351simple si5351(8,27000000u);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(57600);
  si5351.start();
#if 0
  pinMode(LCDB_DB6,OUTPUT);
  for (int i=0;i<100;i++)
  {
    digitalWrite(LCDB_DB6,1);
    delay(1000);
    digitalWrite(LCDB_DB6,0);
    delay(1000);
  }
#endif
  lcd.begin(20,4);
  // Print a message to the LCD.
  lcd.print("hello, world!");

}

void loop() {
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
