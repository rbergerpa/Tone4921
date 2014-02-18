#include "Tone4921.h"
#include "sin_table.h"
#include <Arduino.h>
#include <SPI.h>
#include <avr/io.h>

static const int DEFAULT_SAMPLE_RATE = 16000;

static const int SLAVE_SELECT = 20;

static const int QUADRANT_BITS = 6;
static const int MASK = QUADRANT_LENGTH-1;
static const int PHASE_SHIFT_BITS = 16;
static const float PHASE_SCALE = 1L << PHASE_SHIFT_BITS;

static const int CTL_LOW_GAIN = 0x30;
static const int CTL_HIGH_GAIN = 0x10;
static int  ctl = CTL_LOW_GAIN;

static int timerInitialized = 0;
static int enabled = 0;

static int sample_rate = DEFAULT_SAMPLE_RATE;
static unsigned long phase = 0;
static unsigned long increment = 2*PHASE_SCALE;
static int pin;
static void (*callback)();

void initTimer1(int hertz);
void initSPI();

Tone4921::Tone4921(int _pin) {
  Tone4921::Tone4921(_pin, DEFAULT_SAMPLE_RATE);
}

Tone4921::Tone4921(int _pin, int _sample_rate) {
  pin = _pin;
  sample_rate = _sample_rate;
}

void Tone4921::start() {
  initSPI();
  initTimer1(sample_rate);
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
    if (!timerInitialized) {
    }
  } else {
    phase = 0;
  }
}

 // Interrupt handler
ISR(TIMER1_COMPA_vect)
{
  digitalWrite(pin, LOW);

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

  SPI.transfer(ctl | (data >> 8)); // Control bits and high 4 bits of data
  SPI.transfer(data & 0xFF);       // Low 8 bits of data

  if (enabled) {
    phase += increment;
  }
  digitalWrite(pin, HIGH);

  if (callback) {
      (*callback)();
   }
}  // Interrupt handler

void Tone4921::setCallback(void (*_callback)()) {
  callback = _callback;
}

// Set up timer1 to trigger an interrupt with the given period in milliseconds
void initTimer1(int hertz) {                
  cli();   // disable global interrupts

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1  = 0;

  TCCR1B |= 1 << WGM12;   // CTC mode
  TCCR1B |= 1 << CS10;

  int divisor = F_CPU / hertz - 1;
  
  OCR1A = divisor;
  TIMSK1 |= (1 << OCIE1A); // enable Timer1 intterupt
 
  sei();   // enable global interrupts
}

void initSPI() {
  pinMode(pin, OUTPUT);
  pinMode(SLAVE_SELECT, OUTPUT);
  SPI.begin();
  SPI.setDataMode(SPI_MODE0);
}
