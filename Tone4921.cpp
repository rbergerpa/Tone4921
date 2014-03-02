#include "Tone4921.h"
#include "sin_table.h"
#include <Arduino.h>
#include <SPI.h>
#include <avr/io.h>

static const int CS_PIN = 9;
const int CS_PIN_MASK = PORTB1;

static const int DEFAULT_SAMPLE_RATE = 16000;

static const int SLAVE_SELECT = 20;

static const int QUADRANT_BITS = 6;
static const int MASK = QUADRANT_LENGTH-1;
static const int PHASE_SHIFT_BITS = 16;
static const float PHASE_SCALE = 1L << PHASE_SHIFT_BITS;

static const int CTL_LOW_GAIN = 0x30;
static const int CTL_HIGH_GAIN = 0x10;
static volatile int  ctl = CTL_LOW_GAIN;

static volatile int timerInitialized = 0;
static volatile int enabled = 0;

static volatile int sample_rate = DEFAULT_SAMPLE_RATE;
static volatile unsigned long phase = 0;
static volatile unsigned long increment = 2*PHASE_SCALE;
static void (*volatile callback)();

static volatile int8_t cs_pin_port;
static volatile int8_t cs_pin_mask;

static void startTimer1(int hertz);
static void stopTimer1();
static void initSPI();
static void dacWrite(int value);

Tone4921::Tone4921(int pin) {
  Tone4921::Tone4921(pin, DEFAULT_SAMPLE_RATE);
}

Tone4921::Tone4921(int pin, int _sample_rate) {
  cs_pin_port = digitalPinToPort(pin);
  cs_pin_mask =   digitalPinToBitMask(pin);

  sample_rate = _sample_rate;
}

void Tone4921::start() {
  initSPI();
  startTimer1(sample_rate);
}

void Tone4921::stop() {
  stopTimer1();
}


void Tone4921::setGain(int gain) {
  if (gain == 2) {
    ctl = CTL_HIGH_GAIN;
  } else {
    ctl = CTL_LOW_GAIN;
  }
}

void Tone4921::setFrequency(int hertz) {
  increment = (((float) hertz) * 4*QUADRANT_LENGTH * PHASE_SCALE) / sample_rate;
}

void Tone4921:: setEnabled(int _enabled) {
  enabled = _enabled;

  if (!enabled) {
    phase = 0;
    dacWrite(AMPLITUDE);
  }
}

// Interrupt handler
ISR(TIMER1_COMPA_vect)
{
  int p = phase >> PHASE_SHIFT_BITS;
  
  int quadrant = (p >> QUADRANT_BITS) & 3;
  int offset = p & MASK;
    
  unsigned int data = 0;

  switch (quadrant) {
    case 0:
      data = AMPLITUDE + SIN_TABLE[offset];
      break;
   case 1:
      data = AMPLITUDE + SIN_TABLE[MASK - offset];
      break;
   case 2:
      data = AMPLITUDE - SIN_TABLE[offset];
      break;
   case 3:
       data = AMPLITUDE - SIN_TABLE[MASK - offset];
      break;
  }

  dacWrite(data);

  if (enabled) {
    phase += increment;
  }

  if (callback) {
      (*callback)();
   }
}  // Interrupt handler

void Tone4921::setCallback(void (*_callback)()) {
  callback = _callback;
}

// Set up timer1 to trigger an interrupt with the given period in milliseconds
static void startTimer1(int hertz) {                
  cli();   // disable global interrupts

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  TCCR1B |= 1 << WGM12;   // CTC mode
  TCCR1B |= 1 << CS10;

  int divisor = F_CPU / hertz - 1;
  
  OCR1A = divisor;
  TIMSK1 |= (1 << OCIE1A); // enable Timer1 interrupt
 
  sei();   // enable global interrupts
}

static void stopTimer1() {
  cli();   // disable global interrupts

  TIMSK1 &= ~(1 << OCIE1A); // disable Timer1 interrupt
  TCCR1B = 0;

  sei();   // enable global interrupts
}

static void initSPI() {
  pinMode(CS_PIN, OUTPUT);
  PORTB |= _BV(CS_PIN_MASK);   //  Drive CS_PIN high

  pinMode(SLAVE_SELECT, OUTPUT);
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
}

static void dacWrite(int data) {
  volatile uint8_t *cs_register = portOutputRegister(cs_pin_port);

  uint8_t oldSREG = SREG;
  cli();   // Disble interrupts

  *cs_register &= ~cs_pin_mask;    // Drive CS_PIN low
  SPI.transfer(ctl | (data >> 8)); // Control bits and high 4 bits of data
  SPI.transfer(data & 0xFF);       // Low 8 bits of data
  *cs_register |= cs_pin_mask;    // Drive CS_PIN higth

  SREG = oldSREG;   // Restore interrupts to previous state
}
