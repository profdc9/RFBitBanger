#ifndef LiquidCrystalButtons_h
#define LiquidCrystalButtons_h

#include <inttypes.h>
#include "Print.h"

// commands
#define LCDB_CLEARDISPLAY 0x01
#define LCDB_RETURNHOME 0x02
#define LCDB_ENTRYMODESET 0x04
#define LCDB_DISPLAYCONTROL 0x08
#define LCDB_CURSORSHIFT 0x10
#define LCDB_FUNCTIONSET 0x20
#define LCDB_SETCGRAMADDR 0x40
#define LCDB_SETDDRAMADDR 0x80

// flags for display entry mode
#define LCDB_ENTRYRIGHT 0x00
#define LCDB_ENTRYLEFT 0x02
#define LCDB_ENTRYSHIFTINCREMENT 0x01
#define LCDB_ENTRYSHIFTDECREMENT 0x00

// flags for display on/off control
#define LCDB_DISPLAYON 0x04
#define LCDB_DISPLAYOFF 0x00
#define LCDB_CURSORON 0x02
#define LCDB_CURSOROFF 0x00
#define LCDB_BLINKON 0x01
#define LCDB_BLINKOFF 0x00

// flags for display/cursor shift
#define LCDB_DISPLAYMOVE 0x08
#define LCDB_CURSORMOVE 0x00
#define LCDB_MOVERIGHT 0x04
#define LCDB_MOVELEFT 0x00

// flags for function set
#define LCDB_8BITMODE 0x10
#define LCDB_4BITMODE 0x00
#define LCDB_2LINE 0x08
#define LCDB_1LINE 0x00
#define LCDB_5x10DOTS 0x04
#define LCDB_5x8DOTS 0x00

class LiquidCrystalButtons : public Print {
public:
  LiquidCrystalButtons(uint8_t rs, uint8_t enable,
		uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3);

  void init(uint8_t fourbitmode, uint8_t rs, uint8_t rw, uint8_t enable,
	    uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3,
	    uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7);
    
  void begin(uint8_t cols, uint8_t rows, uint8_t charsize = LCDB_5x8DOTS);

  void clear();
  void home();
  void setDataLineOutput(uint8_t val);

  void noDisplay();
  void display();
  void noBlink();
  void blink();
  void noCursor();
  void cursor();
  void scrollDisplayLeft();
  void scrollDisplayRight();
  void leftToRight();
  void rightToLeft();
  void autoscroll();
  void noAutoscroll();

  void setRowOffsets(int row1, int row2, int row3, int row4);
  void createChar(uint8_t, uint8_t[]);
  void setCursor(uint8_t, uint8_t); 
  virtual size_t write(uint8_t);
  void command(uint8_t);

  void pollButtons();
  uint8_t readButton(uint8_t);
  bool getButtonPressed(uint8_t);
  
  using Print::write;
private:
  void send(uint8_t, uint8_t);
  void write4bits(uint8_t);
  void pulseEnable();

  uint8_t _rs_pin; // LOW: command.  HIGH: character.
  uint8_t _enable_pin; // activated by a HIGH pulse.
  uint8_t _data_pins[4];

  uint8_t _displayfunction;
  uint8_t _displaycontrol;
  uint8_t _displaymode;

  uint8_t _initialized;

  uint8_t _numlines;
  uint8_t _row_offsets[4];

  uint16_t _last_millis;
  uint8_t _last_state[4];
  uint8_t _state_count[4];
  bool    _button_pressed[4];
};

#endif
