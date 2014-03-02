#ifndef __TONE4921__
#define __TONE4921__

class Tone4921 {
 public:
  Tone4921(int _pin);
  Tone4921(int _pin, int _sample_rate);

  void start();
  void stop();
  void setFrequency(int hertz);
  void setGain(int gain);
  void setEnabled(int enabled);
  void setCallback(void (*callback)());
};

#endif
