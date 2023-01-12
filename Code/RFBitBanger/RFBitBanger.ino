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
#include <EEPROM.h>
#include "common.h"
#include "si5351simple.h"
#include "LiquidCrystalButtons.h"
#include "PS2Keyboard.h"
#include "RFBitBanger.h"
#include "ui.h"

#include "dspint.h"
#include "cwmod.h"
#include "scamp.h"

LiquidCrystalButtons lcd(LCDB_RS, LCDB_E, LCDB_DB4, LCDB_DB5, LCDB_DB6, LCDB_DB7);
si5351simple si5351(8,25000000u);
PS2Keyboard PSkey;
radio_configuration rc;
uint8_t current_protocol;

const radio_configuration PROGMEM default_rc =
{
  RC_MAGIC_NUMBER,  /* magic_number */
  25000000,  /* frequency_calibratin */
  20,  /* cw_send_speed */
  3,   /* scamp_resync_frames */
  1,   /* scamp_ressend_frames */
  2,   /* rtty_figs_resend */
  667, /* sidetone_frequency */
  1,   /* sidetone_on */
  0,   /* cw_practice */
  0,   /* wide */
  0,   /* ext_fast_mode */
  0,   /* ext_lsb */
  0,   /* band_warning_off */
};

bool check_band_warning(void);

void setupConfiguration(void)
{
  EEPROM.get(EEPROM_CONFIGURATION_ADDRESS, rc);
  if (rc.magic_number != RC_MAGIC_NUMBER)
    memcpy_P((void *)&rc,(void *)&default_rc,sizeof(rc));
}

void saveConfiguration(void)
{
  EEPROM.put(EEPROM_CONFIGURATION_ADDRESS, rc);
}

void setupADC() {
  ADCSRA = 0;
  PRR &= ~PRADC;
  ADCSRB = (1 << ADTS2) | (1 << ADTS1);
  ADCSRA = (1 << ADEN) | (1 << ADATE) | (1 << ADPS1) | (1 << ADPS0);
  ADMUX = (1 << REFS0);
}

void setupCompare()
{
  cli();
  ADCSRA = 0;
  ADMUX = (1 << REFS0) | (1 << MUX0);
  ADCSRB = (1<<ACME);               // select ADC1 as the source for negative comparator input
  ACSR = (1<<ACBG) | (1<<ACIC);     // select ACBG for positive comparator input, falling edge interrupt
  DIDR1 = 0;
  sei();
}

void stopCompare()
{
  ACSR = (1<<ACD);
  setupADC();
}

void setup_timers_external_control(void)
{
  cli();
  TCCR1A = 0;  
  TCCR1B = (1<<CS10);
  TCCR1B = (1<<ICNC1) | (1<<CS10);  
  ICR1 = 0;
  TCNT1 = 0;
  TIMSK1 = 0;
  TIMSK2 = 0;
  sei();
}

uint16_t external_control_time(void)
{
  uint16_t start_time, end_time;
  TCNT1 = 0;
  while ((ACSR & (1<<ACO)) != 0)
  {
    TCNT1L;
    if (TCNT1H >= 0xF0) return 0; 
  }
  while ((ACSR & (1<<ACO)) == 0)
  {
    TCNT1L;
    if (TCNT1H >= 0xF0) return 0;
  }
  TCNT1 = 0;
  while ((ACSR & (1<<ACO)) != 0)
  {
    TCNT1L;
    if (TCNT1H >= 0xF0) return 0; 
  }
  start_time = ICR1;
  while ((ACSR & (1<<ACO)) == 0)
  {
    TCNT1L;
    if (TCNT1H >= 0xF0) return 0; 
  }
  while ((ACSR & (1<<ACO)) != 0)
  {
    TCNT1L;
    if (TCNT1H >= 0xF0) return 0;
  }
  end_time = ICR1;
  return (end_time - start_time);
}

extern "C" {
void idle_task(void)
{
  dsp_dispatch_receive(current_protocol);
  lcd.pollButtons();
}
}

#define PROCESSOR_CLOCK_FREQ 16000000
#define TIMER1_INTERRUPT_FREQ 4000
#define TIMER1_COUNT_MAX (PROCESSOR_CLOCK_FREQ / TIMER1_INTERRUPT_FREQ)

volatile uint16_t adc_sample_0;
volatile uint16_t adc_sample_1;

uint8_t mute = 0;

uint8_t srd_buffer[80];
scroll_readout_dat srd_buf = { 0, 1, 16, sizeof(srd_buffer), srd_buffer, 0 };

uint8_t sad_buffer[80];
const uint8_t sad_validchars[] PROGMEM = { ' ',    '!',   0x22,   0x27,    '(',
                                           ')',    '*',    '+',    ',',    '-',    '.',    '/',    '0',
                                           '1',    '2',    '3',    '4',    '5',    '6',    '7',    '8',
                                           '9',    ':',    ';',    '=',    '?',    '@',    'A',    'B',
                                           'C',    'D',    'E',    'F',    'G',    'H',    'I',    'J',
                                           'K',    'L',    'M',    'N',    'O',    'P',    'Q',    'R',
                                           'S',    'T',    'U',    'V',    'W',    'X',    'Y',    'Z',
                                           0x5C,   '^',    '`',    '~' };
                                     
scroll_alpha_dat sad_buf = { 0, 0, 16, sizeof(sad_buffer), sad_buffer, sad_validchars, sizeof(sad_validchars), 0, 0, 0, 0 };

scroll_number_dat snd_freq = { 0, 0, 8, 0, 500000, 29999999, 0, 13560000, 0, 0 };
bargraph_dat bgs = { 4, 12, 0 };


ISR(TIMER1_OVF_vect)
{
  static uint8_t last_adc1 = 0;
  uint16_t adc_sample = ADC;
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
    dsp_dispatch_interrupt(current_protocol);
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
  TCCR1B = (1<<CS10);
  TCCR1B = (1<<WGM13) | (1<<WGM12) | (1<<CS10);
  ICR1 = TIMER1_COUNT_MAX;
  write_transmit_pwm(1);
  write_tuning_pwm(0);
  TIMSK1 = (1<<TOIE1);
  TIMSK2 = 0;
  sei();
}

uint8_t tone_state = 0;
uint8_t tone_last_freq = 0;

extern "C" {
void tone_off(void)
{
  if (tone_state)
  {
    pinMode(BEEPOUT_PIN,INPUT);
    tone_state = 0;
  }
}

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
}

void setup_tone_pcm(void)
{
   TCCR2B = (1 << CS20);  // 1 divider
   TCCR2A = (1<<COM2B1) | (1 << WGM21) | (1 << WGM20); // fast pwm
   TCNT2 = 0;
   OCR2B = 0;
   pinMode(BEEPOUT_PIN,OUTPUT);
}

extern "C" {

void delayidle(uint32_t ms)
{
        uint32_t start = micros();

        while (ms > 0) {
                idle_task();
                while ( ms > 0 && (micros() - start) >= 1000) {
                        ms--;
                        start += 1000;
                }
        }
}

void muteaudio_set(uint8_t seton)
{
  digitalWrite(MUTEAUDIO_PIN, seton != 0);
}

void transmit_set(uint8_t seton)
{
  digitalWrite(TRANSMIT_PIN, seton != 0);
}

void set_clock_onoff_mask(uint8_t on_mask)
{
  si5351.setOutputOnOffMask(on_mask);
}

void set_clock_onoff(uint8_t onoff, uint8_t clockno)
{
   si5351.setOutputOnOff(clockno,onoff != 0);  
}

void set_protocol(uint8_t protocol)
{
   dsp_initialize_protocol(protocol, rc.wide);
   current_protocol = protocol;  
}

void set_frequency(uint32_t freq, uint8_t clockno)
{
   si5351_synth_regs s_regs;
   si5351_multisynth_regs m_regs;
  
   si5351.calc_registers(freq, 0, &s_regs, &m_regs);
   si5351.set_registers(0, &s_regs, clockno, &m_regs);
   si5351.setOutputOnOff(clockno,1);
}

void received_character(uint8_t ch)
{
  if (ch == '\b')
     scroll_readout_back_character(&srd_buf,' ');
  else if ((ch >= ' ') && (ch <= '~'))
     scroll_readout_add_character(&srd_buf, ch);
}

}


void set_frequency_snd(void)
{
  set_frequency(snd_freq.n, 0);
  delay(2);
}

void setup() {
  setupConfiguration();
  si5351.set_xo_freq(rc.frequency_calibration);
  set_protocol(PROTOCOL_CW);
  setupADC();
  stopCompare();
  setup_timers();
  scroll_readout_initialize(&srd_buf);
//  setup_tone_pcm();
  PSkey.begin();
  // put your setup code here, to run once:
  Serial.begin(57600);
  pinMode(PTT_PIN,INPUT);
  pinMode(MIC_PIN,INPUT);
  pinMode(TRANSMIT_PIN,OUTPUT);
  pinMode(BACKLIGHT_PIN,OUTPUT);
  pinMode(MUTEAUDIO_PIN,OUTPUT);
  TONE_OFF();
  si5351.start();
  set_frequency_snd();
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
    bgs.bars = map_16_to_bar_20(dsp_get_signal_magnitude());
    lcdBarGraph(&bgs);
  }
}

void update_readout()
{
  scroll_readout_display(&srd_buf);  
}

void redraw_readout()
{
  srd_buf.notchanged = 0;
  update_readout();  
}


const char transmittitle[] PROGMEM = "Tx ";
const char receivetitle[] PROGMEM = "Rxv";
const char freqmenutitle[] PROGMEM = "Frq";
const char scanfasttitle[] PROGMEM = "ScF";
const char scanslowtitle[] PROGMEM = "ScS";
const char tranmodetitle[] PROGMEM = "TrM";
const char extctrlmode[] PROGMEM = "Ext";
const char confmodetitle[] PROGMEM = "Cfg";
const char keymodetitle[] PROGMEM = "Key";

const char *const mainmenu[] PROGMEM = {transmittitle,receivetitle,freqmenutitle,scanfasttitle,scanslowtitle,tranmodetitle,keymodetitle,extctrlmode,confmodetitle,NULL };

const char cw_title[] PROGMEM = "CW";
const char rtty_title[] PROGMEM = "RTTY";
const char rtty_rev_title[] PROGMEM = "RTTYREV";
const char scamp_fsk_title[] PROGMEM = "SCAMPFSK";
const char scamp_ook_title[] PROGMEM = "SCAMPOOK";
const char scamp_fsk_fast_title[] PROGMEM = "SCFSKFST";
#ifdef SCAMP_VERY_SLOW_MODES
const char scamp_fsk_slow_title[] PROGMEM = "SCFSKSLW";
const char scamp_ook_slow_title[] PROGMEM = "SCOOKSLW";
#endif

const char *const protocolmenu[] PROGMEM = {cw_title,rtty_title,rtty_rev_title,scamp_fsk_title,scamp_ook_title,scamp_fsk_fast_title,
#ifdef SCAMP_VERY_SLOW_MODES
    scamp_fsk_slow_title,scamp_ook_slow_title,
#endif
    NULL };

//const char righttx[] PROGMEM = "Rt Txmit Lf Abt";

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
#if 0
    if ( ((stepval < 0) && (abort_button_right())) ||
         ((stepval >= 0) && (abort_button_left())) )
#else
    if (abort_button_enter())
#endif
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
  dsp_initialize_protocol(current_protocol, rc.wide);
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
  dsp_initialize_protocol(current_protocol, rc.wide);
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
    update_readout();
  }
}

const char mode_set[] PROGMEM = "Mode Set";

void set_transmission_mode(void)
{  
  uint8_t selected;
  menu_str mn = { protocolmenu, 0, 0, 8, 0 };
  mn.item = current_protocol-1;
  do_show_menu_item(&mn);
  set_horiz_menu_keys(1);
  do
  {
    update_bars();
    selected = do_menu(&mn);
  } while (!selected);
  set_horiz_menu_keys(0);
  selected = mn.item+1;
  if (selected != current_protocol)
  {
    set_protocol(mn.item+1);
    temporary_message(mode_set);
  }
}

const char txtitle1[] PROGMEM = "Return";
const char txtitle2[] PROGMEM = "Txmit";
const char quittitle[] PROGMEM = "Quit";
const char txtitle4[] PROGMEM = "Clear";

const char *const txmenu[] PROGMEM = {txtitle1,txtitle2,quittitle,txtitle4,NULL };

static uint8_t transmit_message_length(void)
{
  uint8_t len = sad_buf.numchars;
  do
  {
    len--;
    if (sad_buf.buffer[len] != ' ') 
    {
      len++;
      break;
    }
  } while (len > 0);
  return len;
}

void transmit_mode_callback(dsp_txmit_message_state *dtms)
{

  if (dtms->current_symbol < sad_buf.numchars)
     sad_buf.position = dtms->current_symbol;
  scroll_alpha_redraw(&sad_buf);
  if (abort_button_enter())
    dtms->aborted = 1;
}


void transmit_mode(uint8_t selected)
{
  if (!check_band_warning()) return;

  sad_buf.position = (selected == 1) ? (sad_buf.numchars-1) : 0;
  for(;;)
  {
    scroll_alpha_start(&sad_buf);
    while ((!sad_buf.entered) && (!sad_buf.exited))
    {
      idle_task();
      scroll_alpha_key(&sad_buf);
      update_readout();
    }
    if (sad_buf.entered)
    {
      scroll_alpha_clear(&sad_buf);
      menu_str mn = { txmenu, 0, 0, 8, 0 };
      do_show_menu_item(&mn);
      uint8_t selected;
      do
      {
        update_readout();
        selected = do_menu(&mn);
      } while (!selected);
      if (mn.item == 1)
      {
        uint8_t msg_len = transmit_message_length();
        if (msg_len > 0)
        {
          dsp_dispatch_txmit(current_protocol, snd_freq.n, sad_buf.buffer, msg_len, NULL, transmit_mode_callback);
          set_frequency_snd();
          set_clock_onoff(0,1);
          break;
        }
      } else if (mn.item == 2)
      {
        /*quit*/
        break;
      } else if (mn.item == 3)
      {
        sad_buf.buffer[0] = 0;
        sad_buf.position = 0;
      }
    } else break; 
  }
  scroll_alpha_clear(&sad_buf);
}

void receive_scroll_mode(void)
{
  srd_buf.exited = 0;
  while (!srd_buf.exited)
  {
    idle_task();
    update_bars();
    update_readout();
    scroll_readout_key(&srd_buf);
  }  
}


uint8_t current_item = 2;

typedef struct _configuration_entry
{
  void     *entry;
  uint8_t   bytes;
  uint8_t   digits;
  uint32_t  min_value;
  uint32_t  max_value;
} configuration_entry;

const char calibfreq_title[] PROGMEM ="Calib Xtal Src";
const char conf_changed[] PROGMEM = "Config Changed";
const char conf_saved[] PROGMEM = "Config Saved";
const char save_title[] PROGMEM = "Save Conf";
const char fr_calib[] PROGMEM = "Xtal Freq";
const char cw_wpm[] PROGMEM = "CW WPM";
const char scamp_re[] PROGMEM = "Scamp Repeat";
const char scamp_rs[] PROGMEM = "Scamp Resync";
const char rtty_re[] PROGMEM = "RTTY Repeat";
const char sidetone_freq[] PROGMEM = "Sidetone Freq";
const char sidetone_on[] PROGMEM = "Sidetone On";
const char cw_practice[] PROGMEM = "CW Practice";
const char wide_mode[] PROGMEM = "Wide Filters";
const char ext_fast_mode[] PROGMEM = "Ext Fast Mode";
const char ext_lsb[] PROGMEM = "Ext LSB";
const char band_warning_off[] PROGMEM = "Band Warn Off";

const char *const confmenu[] PROGMEM = {quittitle, save_title, calibfreq_title, 
        wide_mode, cw_wpm, scamp_rs, scamp_re,
        rtty_re, sidetone_freq, sidetone_on,
        ext_fast_mode, ext_lsb,
        cw_practice, band_warning_off, fr_calib,
        NULL };

const configuration_entry PROGMEM configuration_entries[] = 
{
  { &rc.wide,                  1, 1, 0, 1 }, /* WIDE PROTOCOL */
  { &rc.cw_send_speed,         1, 2, 5, 40 }, /* CW WPM */
  { &rc.scamp_resync_frames,   1, 1, 0, 9  }, /* SCAMP RESYNC */
  { &rc.scamp_resend_frames,   1, 1, 1, 9 },  /* SCAMP RESEND */
  { &rc.rtty_figs_resend,      1, 1, 1, 5 },   /* RTTY REPEAT */
  { &rc.sidetone_frequency,    2, 4, 250, 2000 },   /* SIDETONE FREQ */
  { &rc.sidetone_on,           1, 1, 0, 1 },   /* SIDETONE ON */
  { &rc.ext_fast_mode,         1, 1, 0, 1 },   /* EXT_FAST_MODE */
  { &rc.ext_lsb,               1, 1, 0, 1 },   /* EXT_LSB */
  { &rc.cw_practice,           1, 1, 0, 1 },   /* CW_PRACTICE */
  { &rc.band_warning_off,      1, 1, 0, 1 },   /* BAND WARNING OFF */
  { &rc.frequency_calibration, 4, 8, 2500000, 29999999 }, /* FREQUENCY CALIBRATION */
 };

void display_clear_row_1(void)
{
   display_clear_row(0, 1, 16);
  
}

const char crystal_cal[] PROGMEM = "Xtal Calib";

void temporary_message(uint8_t *msg)
{
   lcdPrintFlashSpaces(0,1,msg,16);
   delayidle(750);
   display_clear_row_1();
} 

void calibFrequencyStandard(void)
{
  scroll_number_dat cal_freq = { 0, 1, 8, 0, 2500000, 29999999, 0, 500000, 0, 0 };
  cal_freq.n = snd_freq.n+CWMOD_SIDETONE_OFFSET;
  scroll_number_start(&cal_freq);
  while (!cal_freq.entered)
  {
     idle_task();
     scroll_number_key(&cal_freq);
  }
  uint32_t new_xo_freq = ((uint64_t)si5351.get_xo_freq())*((uint64_t)(cal_freq.n-CWMOD_SIDETONE_OFFSET)) / snd_freq.n;
  si5351.set_xo_freq(new_xo_freq);
  temporary_message(crystal_cal);
}

void configuration(void)
{
  lcd.clear();
  uint8_t selected;
  menu_str mn = { confmenu, 0, 0, 16, 0 };
  do_show_menu_item(&mn);
  for (;;)
  {
    void *v;
    uint32_t val;
    configuration_entry *c;
    scroll_number_dat snd = { 0, 1, 0, 0, 0, 0, 0, 0, 0, 0 };

    do
    {
      idle_task();
      update_readout();
      selected = do_menu(&mn);
    } while (!selected);
    if (mn.item == 0)  break;
    if (mn.item == 1)
    {
      saveConfiguration();
      temporary_message(conf_saved);
    } else
    if (mn.item == 2)
    {
      calibFrequencyStandard();
    } else
    {
      selected = mn.item - 3;    
      c = &configuration_entries[selected];
      v = pgm_read_word_near(&c->entry);
      snd.digits = pgm_read_word_near(&c->digits);
      snd.minimum_number = pgm_read_dword_near(&c->min_value);
      snd.maximum_number = pgm_read_dword_near(&c->max_value);
      switch (pgm_read_byte_near(&c->bytes))
      {
        case 1: snd.n = *((uint8_t *)v);
                break;
        case 2: snd.n = *((uint16_t *)v);
                break;
        case 4: snd.n = *((uint32_t *)v);
                break;
      } 
      display_clear_row_1();
      scroll_number_start(&snd);
      while (!snd.entered)
      {
        idle_task();
        scroll_number_key(&snd);
        }
      if (snd.changed)
      {
        switch (pgm_read_byte_near(&c->bytes))
        {
          case 1: *((uint8_t *)v) = snd.n;
                  break;
          case 2: *((uint16_t *)v) = snd.n;
                  break;
          case 4: *((uint32_t *)v) = snd.n;
                  break;
        }
        temporary_message(conf_changed);
      } 
    }
    display_clear_row_1();
  }
}

const char keying_mode[] PROGMEM = "Keying Mode";
const char keying_exit[] PROGMEM = "Keying Exit";

void key_mode(void)
{
  uint16_t last_tick;
  uint8_t current_state = 0, count_states = 0, last_check_time;

  uint8_t sidetone_freq = TONEFREQ(rc.sidetone_frequency);
  uint8_t sidetone_freq_2 = sidetone_freq >> 2;

  if (!check_band_warning()) return;

  temporary_message(keying_mode);
  redraw_readout();
  lcd.clearButtons();
  set_protocol(PROTOCOL_CW);  // set the mode to CW for receiving CW
  last_tick = ps.cs.total_ticks;
  last_check_time = millis();
  if (!rc.cw_practice)
  {
    set_frequency(snd_freq.n + CWMOD_SIDETONE_OFFSET, 1);
    set_clock_onoff_mask(0x01);
  }
  for (;;)
  {
    if (abort_button_enter()) break;          // if enter pressed, exit keying mode
    idle_task();
    update_bars();
    update_readout();
    uint8_t key_state = (lcd.readUnBounced(1)) || (!digitalRead(PTT_PIN));
    uint8_t current_check_time = millis();
    if (current_check_time != last_check_time)
    {
      last_check_time = current_check_time;
      uint16_t current_tick = ps.cs.total_ticks;
      if (current_state != key_state)
      {
        if ((++count_states) > 10)
        {
          uint16_t elapsed_ticks = current_tick - last_tick;
          last_tick = current_tick;
          current_state = key_state;
          count_states = 0;
          if (key_state)
          {
            if (!rc.cw_practice)
            {
              set_clock_onoff_mask(0x02);
              muteaudio_set(1);
              transmit_set(1);
            }
            cwmod_insert_into_timing_fifo_noint(elapsed_ticks|0x8000);
            if (rc.sidetone_on) tone_on(sidetone_freq, sidetone_freq_2);
          } else
          {
            if (!rc.cw_practice)
            {
              transmit_set(0);
              muteaudio_set(0);
              set_clock_onoff_mask(0x01);
            }
            cwmod_insert_into_timing_fifo_noint(elapsed_ticks);
            tone_off();
          }
        } 
      } else
        count_states = 0;
    }    
  }
  transmit_set(0);
  muteaudio_set(0);
  set_clock_onoff_mask(0x01);
  tone_off();
  temporary_message(keying_exit);
  redraw_readout();
  lcd.clearButtons();
}

const char ext_mode[] PROGMEM = "Ext Mode";
const char ext_exit[] PROGMEM = "Ext Exit";

#define TESTPIN (PINC & 0x02)
uint16_t __attribute__ ((noinline)) tickval_no_timer(void)
{
 register uint16_t val;

 cli();
 val = 0;
 do
 {
    val++;
    if (((uint8_t)((val >> 8) & 0xFF)) == 0xFF) goto abort_timer;
 } while (TESTPIN);
 do
 {
    val++;
    if (((uint8_t)((val >> 8) & 0xFF)) == 0xFF) goto abort_timer;
 } while (!TESTPIN);
 val = 0;
 do
 {
    val++;
    if (((uint8_t)((val >> 8) & 0xFF)) == 0xFF) goto abort_timer;
 } while (TESTPIN);
 do
 {
    val++;
    if (((uint8_t)((val >> 8) & 0xFF)) == 0xFF) goto abort_timer;
 } while (!TESTPIN);
 sei();
 return val;
abort_timer:
 sei();
 return 0;
}

void external_control_mode(void)
{
  uint8_t current_oscillator = 1;
  uint16_t current_frequency = 0;

  if (!check_band_warning()) return;

  temporary_message(ext_mode);
  redraw_readout();
  setupCompare();
  setup_timers_external_control();
  lcd.clearButtons();
  for (;;)
  { 
    uint8_t counts = 0;
    uint16_t count, count_total = 0;
    do
    {
      uint16_t new_count;
      count = external_control_time();
      if (count == 0) break;
      new_count = count_total + count;
      if (new_count < count_total)
        break;
      count_total = new_count;
      counts++;
    } while (count_total < (rc.ext_fast_mode ? 16000 : 32000));
    if (count != 0)
    {
      uint16_t next_frequency = (16000000ul * counts) / count_total;
      if (next_frequency > 3000)
         count = 0;
      else
      {
        uint8_t change_frequency = (current_frequency == 0);
        if (!change_frequency)
        {
          uint16_t difference_frequency = next_frequency > current_frequency ? next_frequency - current_frequency : current_frequency - next_frequency;
          if (difference_frequency > 2) change_frequency = 1;
        }
        if (change_frequency)
        {
          current_frequency = next_frequency;
          set_frequency(rc.ext_lsb ? snd_freq.n - current_frequency : snd_freq.n + current_frequency, current_oscillator == 0 ? 1 : 0);
          set_clock_onoff_mask(current_oscillator == 0 ? 0x02 : 0x01);
          current_oscillator == (current_oscillator == 0) ? 1 : 0;
          muteaudio_set(1);
          transmit_set(1);
        }
      }
    }
    if (count == 0)
    {
      current_frequency = 0;
      muteaudio_set(0);
      transmit_set(0);
    }
    if (abort_button()) break; 
  }
  setup_timers();
  stopCompare();
  set_frequency(snd_freq.n, 0);
  set_clock_onoff_mask(0x01);
  muteaudio_set(0);
  transmit_set(0);
  temporary_message(ext_exit);
  redraw_readout();
  lcd.clearButtons();
}

#if BAND_NAMES
const char band_160m[] PROGMEM = "160";
const char band_80m[] PROGMEM = "80m";
const char band_60m[] PROGMEM = "60m";
const char band_40m[] PROGMEM = "40m";
const char band_30m[] PROGMEM = "30m";
const char band_20m[] PROGMEM = "20m";
const char band_18m[] PROGMEM = "17m";
const char band_15m[] PROGMEM = "15m";
const char band_12m[] PROGMEM = "12m";
const char band_10m[] PROGMEM = "10m";

const char *const bandlist[] PROGMEM = { band_160m, band_80m, band_60m, band_40m, band_30m, band_20m, band_18m, band_15m, band_12m, band_10m };
#endif

uint8_t return_band(uint32_t freq)
{
  uint16_t freq_khz = freq / 1000;
  if ((freq_khz >= 1800) && (freq_khz < 2000)) return 1;
  if ((freq_khz >= 3500) && (freq_khz < 4000)) return 2;
  if ((freq_khz >= 5250) && (freq_khz < 5450)) return 3;
  if ((freq_khz >= 7000) && (freq_khz < 7400)) return 4;
  if ((freq_khz >= 10100) && (freq_khz < 10150)) return 5;
  if ((freq_khz >= 14000) && (freq_khz < 14350)) return 6;
  if ((freq_khz >= 18068) && (freq_khz < 18168)) return 7;
  if ((freq_khz >= 21000) && (freq_khz < 21450)) return 8;
  if ((freq_khz >= 24890) && (freq_khz < 24990)) return 9;
  if ((freq_khz >= 28000) && (freq_khz < 29700)) return 10;
  return 0;
}

uint8_t last_band = 0;

const char unknown_band[] PROGMEM = "Unknown band";
const char new_band[] PROGMEM = "Chg band module";
const char lr_prompt[] PROGMEM = "Lft abrt Rt cont";

bool check_band_warning(void)
{
  uint8_t band;
  bool res;
  
  if (rc.band_warning_off) return true;
  band = return_band(snd_freq.n);
  if (band == 0)
    return show_lr(1,unknown_band,lr_prompt);
  if (band == last_band) 
    return true;
  res = show_lr(1,new_band,lr_prompt);
  if (res) last_band = band;
  return res;
}

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
    update_readout();
    selected = do_menu(&mn);
  } while (!selected);
  current_item = mn.item;
  switch (mn.item)
  {
    case 0: transmit_mode(selected);
            break;
    case 1: receive_scroll_mode();
            break;
    case 2: set_frequency_mode(selected);
            break;
    case 3: if (selected < 3) scan_frequency_mode(selected == 1, 100);
            break;
    case 4: if (selected < 3) scan_frequency_mode(selected == 1, 30);
            break;
    case 5: set_transmission_mode();
            break;
    case 6: key_mode();
            break;
    case 7: external_control_mode();
            break;
    case 8: configuration();
            break;
  }
}

void loop() {
  select_command_mode();
}
