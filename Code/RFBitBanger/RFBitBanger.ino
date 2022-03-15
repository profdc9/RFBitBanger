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
#include <wiring_private.h>
#include <avr/pgmspace.h>
#include "si5351simple.h"
#include "LiquidCrystalButtons.h"
#include "PS2Keyboard.h"
#include "RFBitBanger.h"
#include "ui.h"

#include "dspint.h"
#include "cwmod.h"
#include "scamp.h"

#define LCDB_RS A3
#define LCDB_E A2
#define LCDB_DB4 5
#define LCDB_DB5 6
#define LCDB_DB6 7
#define LCDB_DB7 8

#define TRANSMIT_PIN 9
#define BACKLIGHT_PIN 11
#define MUTEAUDIO_PIN 4
#define BEEPOUT_PIN 3

LiquidCrystalButtons lcd(LCDB_RS, LCDB_E, LCDB_DB4, LCDB_DB5, LCDB_DB6, LCDB_DB7);
si5351simple si5351(8,27000000u);
PS2Keyboard PSkey;

void setupADC() {
  ADCSRA = 0;
  PRR &= ~PRADC;
  ADCSRB = (1 << ADTS2) | (1 << ADTS1);
  ADCSRA = (1 << ADEN) | (1 << ADATE) | (1 << ADPS1) | (1 << ADPS0);
  ADMUX = (1 << REFS0);
}

void idle_task(void)
{
  lcd.pollButtons();
}

#define PROCESSOR_CLOCK_FREQ 16000000
#define TIMER1_INTERRUPT_FREQ 4000
#define TIMER1_COUNT_MAX (PROCESSOR_CLOCK_FREQ / TIMER1_INTERRUPT_FREQ)

volatile uint16_t adc_sample_0;
volatile uint16_t adc_sample_1;

ISR(TIMER1_OVF_vect)
{
  static uint8_t last_adc1 = 0;
  uint16_t adc_sample;
  do
  { uint8_t low = ADCL;
    uint8_t high = ADCH;
    adc_sample = (((uint16_t) high) << 8) | low;
  } while (0);
  sei();
  if (last_adc1)
  {
    last_adc1 = 0;
    adc_sample_1 = adc_sample;
    ADMUX = (1 << REFS0) | 0x01;
  } else
  {
    last_adc1 = 1;
    adc_sample_0 = adc_sample;
    ADMUX = (1 << REFS0);
    dsp_interrupt_sample(adc_sample);
  }
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
  TCCR1A = (1<<COM1B1) | (1<<WGM11);  
  TCCR1B = (1<<WGM13) | (1<<WGM12) | (1<<CS10);
  ICR1H = (TIMER1_COUNT_MAX >> 8);
  ICR1L = (TIMER1_COUNT_MAX & 0xFF);
  write_transmit_pwm(1);
  write_tuning_pwm(0);
  TIMSK1 = (1<<TOIE1);
  TIMSK2 = 0;
  sei();
}

uint8_t tone_state = 0;
uint8_t tone_last_freq = 0;

void tone_off(void)
{
  if (tone_state)
  {
    pinMode(BEEPOUT_PIN,INPUT);
    tone_state = 0;
  }
}

#define TONEFREQ_BASEFREQ (16000000ul / 256)
#define TONEFREQ(x) (TONEFREQ_BASEFREQ / (x))
#define TONEFREQVOL(x,vol) (((TONEFREQ_BASEFREQ / (x)) * (vol)) >> 8)
#define TONE_ON(freq,vol) tone_on(TONEFREQ(freq),TONEFREQVOL(freq,vol))
#define TONE_OFF() tone_off()

void tone_on(uint8_t freq, uint8_t vol)
{
  if ((!tone_state) || (freq != tone_last_freq))
  {
    if (!tone_state) pinMode(BEEPOUT_PIN,OUTPUT);  
    TCCR2B = 0;
    OCR2A = freq;
    OCR2B = vol;
    TCNT2 = 0;
    TCCR2A = (1<<COM2B1) | (1 << WGM21) | (1 << WGM20);
    TCCR2B = (1 << WGM22) | (1<<CS22) | (1<<CS21);   // 256 divider
    tone_last_freq = freq;
    tone_state = 1;
  } else
    OCR2B = vol;
}

void setup_tone_pcm(void)
{
   TCCR2B = (1 << CS20);  // 1 divider
   TCCR2A = (1<<COM2B1) | (1 << WGM21) | (1 << WGM20); // fast pwm
   TCNT2 = 0;
   OCR2B = 0;
   pinMode(BEEPOUT_PIN,OUTPUT);
}

uint8_t current_protocol;

void set_protocol(uint8_t protocol)
{
   dsp_initialize_protocol(protocol);
   current_protocol = protocol;  
}

void setup() {
  set_protocol(PROTOCOL_CW);
  setupADC();
  setup_timers();
//  setup_tone_pcm();
  PSkey.begin();
  // put your setup code here, to run once:
  Serial.begin(57600);
  pinMode(TRANSMIT_PIN,OUTPUT);
  pinMode(BACKLIGHT_PIN,OUTPUT);
  pinMode(MUTEAUDIO_PIN,OUTPUT);
  TONE_OFF();
  si5351.start();
  lcd.begin(20,4);
  digitalWrite(TRANSMIT_PIN,LOW);
  digitalWrite(MUTEAUDIO_PIN,LOW);
  digitalWrite(BACKLIGHT_PIN,HIGH);
}

#define UPDATE_MILLIS_BARS 100

uint8_t map_16_to_bar_40(uint16_t b)
{
  if (b < 64)
    return (b >> 3);                  // 0 to 64 is 0 to 7
  if (b < 192)
    return (b >> 4) + 4;              // 64 to 192 is 8 to 15
  if (b < 448)
    return (b >> 5) + 10;             // 192 to 448 is 16 to 23
  if (b < 960)
    return (b >> 6) + 17;             // 448 to 960 is 24 to 32
  if (b < 2048)
    return (b >> 7) + 24;             // 960 and up is 32 to 40
  return 40;
}

uint8_t map_16_to_bar_20(uint16_t b)
{
  return map_16_to_bar_40(b) >> 1;
}

scroll_number_dat snd_freq = { 0, 0, 8, 0, 500000, 29999999, 0, 7000000, 0, 0 };
#ifdef BIG_BAR_GRAPH
bargraph_dat bgd = { 4, 8, 12, 0 };
#endif
bargraph_dat bgs = { 1, 4, 12, 0 };

void set_frequency(uint32_t freq)
{
   si5351_synth_regs s_regs;
   si5351_multisynth_regs m_regs;
  
   si5351.calc_registers(freq, 0, &s_regs, &m_regs);
   si5351.set_registers(0, &s_regs, 0, &m_regs);
   si5351.setOutputOnOff(0,1);
}

void set_frequency_snd(void)
{
  set_frequency(snd_freq.n);
  delay(2);
}

void scroll_redraw_snd(void)
{
  scroll_number_redraw(&snd_freq);
}

#define PTSAMPLECT ((volatile uint8_t *)&ds.sample_ct)

uint16_t accumulate_all_channels(uint8_t ct)
{
  uint8_t last = *PTSAMPLECT;
  uint32_t total = 0;
  while (ct>0)
  {
    uint8_t sample_ct = *PTSAMPLECT;
    if (last != sample_ct)
    {
      last = sample_ct;
      total += dsp_get_signal_magnitude();
      ct--;
    }
  } 
  return total >> 8;
}

void update_bars()
{
  static uint16_t last_update_bars;
  uint16_t cur_update = millis();
  if ((cur_update - last_update_bars) >= UPDATE_MILLIS_BARS)
  {
    last_update_bars = cur_update;
#ifdef BIG_BAR_GRAPH
    bgd.bars[0] = map_16_to_bar_40(ps.rs.protocol == PROTOCOL_RTTY ? ds.mag_value_24 : ds.mag_value_20);
    bgd.bars[1] = map_16_to_bar_40(ds.mag_value_16);
    bgd.bars[2] = map_16_to_bar_40(ds.mag_value_12);
    bgd.bars[3] = map_16_to_bar_40(ds.mag_value_8);
    lcdBarGraph(&bgd);
#endif
    bgs.bars[0] = map_16_to_bar_20(dsp_get_signal_magnitude());
    //lcd.setCursor(9,1);
    //lcdPrintNum(bgs.bars[0],3,0);
    lcdBarGraph(&bgs);
  }
}

const char transmittitle[] PROGMEM = "Tx ";
const char freqmenutitle[] PROGMEM = "Frq";
const char scanfasttitle[] PROGMEM = "ScF";
const char scanslowtitle[] PROGMEM = "ScS";
const char tranmodetitle[] PROGMEM = "TrM";

const char *const mainmenu[] PROGMEM = {transmittitle,freqmenutitle,scanfasttitle,scanslowtitle,tranmodetitle,NULL };

const char cwtitle[] PROGMEM = "CW";
const char rttytitle[] PROGMEM = "RTTY";
const char scamptitle[] PROGMEM = "SCAMP";

const char *const protocolmenu[] PROGMEM = {cwtitle,rttytitle,scamptitle,NULL };

void increment_decrement_frequency(int16_t val)
{
  int32_t next_freq = snd_freq.n + val;
  if (next_freq < snd_freq.minimum_number) return;
  if (next_freq > snd_freq.maximum_number) return;
  snd_freq.n = next_freq;
  set_frequency_snd();
}

uint8_t scan_frequency(int8_t stepval, uint16_t maxsteps)
{
  uint16_t avg;
  uint8_t aborted = 0;

  avg = 0;
  for (uint8_t i=0;i<8;i++)
     avg += accumulate_all_channels(64);
  avg >>= 3; 
  for (;;)
  {
    uint16_t val;

    if (maxsteps == 0)
    {
      aborted = 2;
      break;
    } else --maxsteps;
    increment_decrement_frequency(stepval);
    scroll_redraw_snd();
    update_bars();
    if ( ((stepval < 0) && (abort_button_right())) ||
         ((stepval >= 0) && (abort_button_left())) )
    {
      aborted = 1;
      break;
    }
    val = accumulate_all_channels(64);
    if (val > (avg*2)) break;
    avg = (15*avg+val) >> 4;    
  }
  if (aborted == 1)
  {
    delay(250);
    lcd.clearButtons();
  }
  return aborted;
}

uint8_t scan_refine(uint8_t dir, uint8_t iter)
{
  uint8_t code;
  dsp_initialize_protocol(current_protocol);
  for (uint8_t i=0;i<iter;i++)
  {
     code = scan_frequency(dir ? -10 : 10, 50);
     if (code < 2) break;
     dir = !dir;
  }
  dsp_initialize_fastscan();
  return code;
}

uint8_t scan_frequency_mode(uint8_t dir, uint8_t stepval)
{
  uint8_t code;
  dsp_initialize_fastscan();
  for (;;)
  {
    code = scan_frequency(dir ? -stepval : stepval, 1000);
    if (code == 0)
    {
      increment_decrement_frequency(dir ? 250 : -250);
      code = scan_refine(dir, 5);
      if (code < 2) return code;
    } else return code;
  }
  dsp_initialize_protocol(current_protocol);
}

void set_frequency_mode(uint8_t selected)
{
  snd_freq.position = (selected == 1) ? 7 : 0;
  scroll_number_start(&snd_freq);
  while (!snd_freq.entered)
  {
    idle_task();
    scroll_number_key(&snd_freq);
    if (snd_freq.changed)
    {
       set_frequency_snd();
       snd_freq.changed = 0;
    }
    update_bars();
  }
}

void set_transmission_mode(void)
{  
  uint8_t selected;
  menu_str mn = { protocolmenu, 0, 0, 8, 0 };
  mn.item = ps.ss.protocol;
  do_show_menu_item(&mn);
  set_horiz_menu_keys(1);
  do
  {
    update_bars();
    selected = do_menu(&mn);
  } while (!selected);
  set_horiz_menu_keys(0);
  if (current_protocol != mn.item)
     set_protocol(mn.item+1);
}

uint8_t sad_buffer[80];
const uint8_t sad_validchars[] PROGMEM = { ' ',    '!',   0x22,   0x27,    '(',
                                           ')',    '*',    '+',    ',',    '-',    '.',    '/',    '0',
                                           '1',    '2',    '3',    '4',    '5',    '6',    '7',    '8',
                                           '9',    ':',    ';',    '=',    '?',    '@',    'A',    'B',
                                           'C',    'D',    'E',    'F',    'G',    'H',    'I',    'J',
                                           'K',    'L',    'M',    'N',    'O',    'P',    'Q',    'R',
                                           'S',    'T',    'U',    'V',    'W',    'X',    'Y',    'Z',
                                           0x5C,   '^',    '`',    '~' };
                                     
scroll_alpha_dat sad_buf = { 0, 1, 16, sizeof(sad_buffer), sad_buffer, sad_validchars, sizeof(sad_validchars), 0, 0, 0, 0 };

void transmit_mode(uint8_t selected)
{
  sad_buf.position = (selected == 1) ? (sad_buf.numchars-1) : 0;
  scroll_alpha_start(&sad_buf);
  while (!sad_buf.entered)
  {
    idle_task();
    scroll_alpha_key(&sad_buf);
    if (sad_buf.changed)
    {
/*       set_frequency_snd(); */
       snd_freq.changed = 0;
    }
    update_bars();
  }
}

uint8_t current_item = 1;

void select_command_mode()
{
  uint8_t selected;
  menu_str mn = { mainmenu, 9, 0, 8, 0 };
  mn.item = current_item;
  scroll_redraw_snd();
  do_show_menu_item(&mn);
  do
  {
    update_bars();
    selected = do_menu(&mn);
  } while (!selected);
  current_item = mn.item;
  switch (mn.item)
  {
    case 0: transmit_mode(selected);
            break;
    case 1: set_frequency_mode(selected);
            break;
    case 2: scan_frequency_mode(selected == 1, 100);
            break;
    case 3: scan_frequency_mode(selected == 1, 30);
            break;
    case 4: set_transmission_mode();
            break;
  }
}

void loop() {
  select_command_mode();
}
