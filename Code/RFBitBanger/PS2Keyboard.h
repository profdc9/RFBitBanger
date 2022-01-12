
#ifndef _PS2KEYBOARD_H
#define _PS2KEYBOARD_H

#define PS2_CLOCKLINE 2
#define PS2_DATALINE 12

#define PS2_EXTRACODES 128

#define PS2KEY_UP (PS2_EXTRACODES+1)
#define PS2KEY_DOWN (PS2_EXTRACODES+2)
#define PS2KEY_LEFT (PS2_EXTRACODES+3)
#define PS2KEY_RIGHT (PS2_EXTRACODES+4)
#define PS2KEY_ENTER 0x0D

#define PS2KEY_NONE 0xFF

class PS2Keyboard {    
  public:
   PS2Keyboard();
   void begin();
	 void end();                         
   int waitkey();
	 int getkey();
};

#endif /* _PS2KEYBOARD_H */
