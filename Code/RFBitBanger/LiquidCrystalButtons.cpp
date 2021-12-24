#include "LiquidCrystalButtons.h"

#include <stdio.h>
#include <string.h>
#include <inttypes.h>
#include "Arduino.h"

// When the display powers up, it is configured as follows:
//
// 1. Display clear
// 2. Function set: 
//    DL = 1; 8-bit interface data 
//    N = 0; 1-line display 
//    F = 0; 5x8 dot character font 
// 3. Display on/off control: 
//    D = 0; Display off 
//    C = 0; Cursor off 
//    B = 0; Blinking off 
// 4. Entry mode set: 
//    I/D = 1; Increment by 1 
//    S = 0; No shift 
//
// Note, however, that resetting the Arduino doesn't reset the LCD, so we
// can't assume that it's in that state when a sketch starts (and the
// LiquidCrystalButtons constructor is called).

LiquidCrystalButtons::LiquidCrystalButtons(uint8_t rs,  uint8_t enable,
			     uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7)
{
  _rs_pin = rs;
  _enable_pin = enable;
  
  _data_pins[0] = d4;
  _data_pins[1] = d5;
  _data_pins[2] = d6;
  _data_pins[3] = d7; 

  _displayfunction = LCDB_1LINE;
}

#define SETDATALINEFAST

#ifdef SETDATALINEFAST
#define SETDATALINEOUTPUTS() { DDRD |= 0x80; DDRB |= 0x07; }
#define SETDATALINEINPUTS() { DDRD &= ~0x80; DDRB &= ~0x07; }
#else
#define SETDATALINEOUTPUTS() setDataLineOutput(OUTPUT)
#define SETDATALINEINPUTS() setDataLineOutput(INPUT)
#endif

void LiquidCrystalButtons::pollButtons()
{
  SETDATALINEINPUTS();
  delay(2);
  uint16_t m = millis();
  if (m == _last_millis) return;
  _last_millis = m; 
  
  for (uint8_t i=0; i<4; i++)
  {
     uint8_t state = digitalRead(_data_pins[i]);
     if (state == _last_state[i])
        _state_count[i] = 0;
     else 
     {  if (_state_count[i] > 50)
        {
           _state_count[i] = 0;
           _last_state[i] = state;
           if (!_button_pressed[i])
              _button_pressed[i] = true;
        } else _state_count[i]++;
     }
  }
}

bool LiquidCrystalButtons::getButtonPressed(uint8_t b)
{
  bool state = _button_pressed[b];
  if (state) _button_pressed[b] = false;
  return state;
}

uint8_t LiquidCrystalButtons::readButton(uint8_t b)
{
  return !(_last_state[b]);
}

void LiquidCrystalButtons::setDataLineOutput(uint8_t val)
{
  for (uint8_t i=0; i<4; ++i)
  {
    pinMode(_data_pins[i], val);
  } 
}

void LiquidCrystalButtons::begin(uint8_t cols, uint8_t lines, uint8_t dotsize) {
  if (lines > 1) {
    _displayfunction |= LCDB_2LINE;
  }
  _numlines = lines;

  setRowOffsets(0x00, 0x40, 0x00 + cols, 0x40 + cols);  

  pinMode(_rs_pin, OUTPUT);
  pinMode(_enable_pin, OUTPUT);
  
  SETDATALINEOUTPUTS();

  // SEE PAGE 45/46 FOR INITIALIZATION SPECIFICATION!
  // according to datasheet, we need at least 40 ms after power rises above 2.7 V
  // before sending commands. Arduino can turn on way before 4.5 V so we'll wait 50
  delayMicroseconds(50000); 
  // Now we pull both RS and R/W low to begin commands
  digitalWrite(_rs_pin, LOW);
  digitalWrite(_enable_pin, LOW);
  
    // this is according to the Hitachi HD44780 datasheet
    // figure 24, pg 46

    // we start in 8bit mode, try to set 4 bit mode
    write4bits(0x03);
    delayMicroseconds(4500); // wait min 4.1ms

    // second try
    write4bits(0x03);
    delayMicroseconds(4500); // wait min 4.1ms
    
    // third go!
    write4bits(0x03); 
    delayMicroseconds(150);

    // finally, set to 4-bit interface
    write4bits(0x02); 

  // finally, set # lines, font size, etc.
  command(LCDB_FUNCTIONSET | _displayfunction);  

  // turn the display on with no cursor or blinking default
  _displaycontrol = LCDB_DISPLAYON | LCDB_CURSOROFF | LCDB_BLINKOFF;  
  display();

  // clear it off
  clear();

  // Initialize to default text direction (for romance languages)
  _displaymode = LCDB_ENTRYLEFT | LCDB_ENTRYSHIFTDECREMENT;
  // set the entry mode
  command(LCDB_ENTRYMODESET | _displaymode);

}

void LiquidCrystalButtons::setRowOffsets(int row0, int row1, int row2, int row3)
{
  _row_offsets[0] = row0;
  _row_offsets[1] = row1;
  _row_offsets[2] = row2;
  _row_offsets[3] = row3;
}

/********** high level commands, for the user! */
void LiquidCrystalButtons::clear()
{
  command(LCDB_CLEARDISPLAY);  // clear display, set cursor position to zero
  delayMicroseconds(2000);  // this command takes a long time!
}

void LiquidCrystalButtons::home()
{
  command(LCDB_RETURNHOME);  // set cursor position to zero
  delayMicroseconds(2000);  // this command takes a long time!
}

void LiquidCrystalButtons::setCursor(uint8_t col, uint8_t row)
{
  const size_t max_lines = sizeof(_row_offsets) / sizeof(*_row_offsets);
  if ( row >= max_lines ) {
    row = max_lines - 1;    // we count rows starting w/ 0
  }
  if ( row >= _numlines ) {
    row = _numlines - 1;    // we count rows starting w/ 0
  }
  
  command(LCDB_SETDDRAMADDR | (col + _row_offsets[row]));
}

// Turn the display on/off (quickly)
void LiquidCrystalButtons::noDisplay() {
  _displaycontrol &= ~LCDB_DISPLAYON;
  command(LCDB_DISPLAYCONTROL | _displaycontrol);
}
void LiquidCrystalButtons::display() {
  _displaycontrol |= LCDB_DISPLAYON;
  command(LCDB_DISPLAYCONTROL | _displaycontrol);
}

// Turns the underline cursor on/off
void LiquidCrystalButtons::noCursor() {
  _displaycontrol &= ~LCDB_CURSORON;
  command(LCDB_DISPLAYCONTROL | _displaycontrol);
}
void LiquidCrystalButtons::cursor() {
  _displaycontrol |= LCDB_CURSORON;
  command(LCDB_DISPLAYCONTROL | _displaycontrol);
}

// Turn on and off the blinking cursor
void LiquidCrystalButtons::noBlink() {
  _displaycontrol &= ~LCDB_BLINKON;
  command(LCDB_DISPLAYCONTROL | _displaycontrol);
}
void LiquidCrystalButtons::blink() {
  _displaycontrol |= LCDB_BLINKON;
  command(LCDB_DISPLAYCONTROL | _displaycontrol);
}

// These commands scroll the display without changing the RAM
void LiquidCrystalButtons::scrollDisplayLeft(void) {
  command(LCDB_CURSORSHIFT | LCDB_DISPLAYMOVE | LCDB_MOVELEFT);
}
void LiquidCrystalButtons::scrollDisplayRight(void) {
  command(LCDB_CURSORSHIFT | LCDB_DISPLAYMOVE | LCDB_MOVERIGHT);
}

// This is for text that flows Left to Right
void LiquidCrystalButtons::leftToRight(void) {
  _displaymode |= LCDB_ENTRYLEFT;
  command(LCDB_ENTRYMODESET | _displaymode);
}

// This is for text that flows Right to Left
void LiquidCrystalButtons::rightToLeft(void) {
  _displaymode &= ~LCDB_ENTRYLEFT;
  command(LCDB_ENTRYMODESET | _displaymode);
}

// This will 'right justify' text from the cursor
void LiquidCrystalButtons::autoscroll(void) {
  _displaymode |= LCDB_ENTRYSHIFTINCREMENT;
  command(LCDB_ENTRYMODESET | _displaymode);
}

// This will 'left justify' text from the cursor
void LiquidCrystalButtons::noAutoscroll(void) {
  _displaymode &= ~LCDB_ENTRYSHIFTINCREMENT;
  command(LCDB_ENTRYMODESET | _displaymode);
}

// Allows us to fill the first 8 CGRAM locations
// with custom characters
void LiquidCrystalButtons::createChar(uint8_t location, uint8_t charmap[]) {
  location &= 0x7; // we only have 8 locations 0-7
  command(LCDB_SETCGRAMADDR | (location << 3));
  for (int i=0; i<8; i++) {
    write(charmap[i]);
  }
}

/*********** mid level commands, for sending data/cmds */

inline void LiquidCrystalButtons::command(uint8_t value) {
  SETDATALINEOUTPUTS();
  send(value, LOW);
  SETDATALINEINPUTS();
}

inline size_t LiquidCrystalButtons::write(uint8_t value) {
  SETDATALINEOUTPUTS();
  send(value, HIGH);
  SETDATALINEINPUTS();
  return 1; // assume success
}

/************ low level data pushing commands **********/

// write either command or data, with automatic 4/8-bit selection
void LiquidCrystalButtons::send(uint8_t value, uint8_t mode) {
  digitalWrite(_rs_pin, mode);

  write4bits(value>>4);
  write4bits(value);
}

void LiquidCrystalButtons::pulseEnable(void) {
  digitalWrite(_enable_pin, LOW);
  delayMicroseconds(1);    
  digitalWrite(_enable_pin, HIGH);
  delayMicroseconds(1);    // enable pulse must be >450 ns
  digitalWrite(_enable_pin, LOW);
  delayMicroseconds(100);   // commands need >37 us to settle
}

void LiquidCrystalButtons::write4bits(uint8_t value) {
  for (uint8_t i = 0; i < 4; i++) {
    digitalWrite(_data_pins[i], (value >> i) & 0x01);
  }

  pulseEnable();

  for (uint8_t i = 0; i < 4; i++) {
    digitalWrite(_data_pins[i], 0);
  }
}
