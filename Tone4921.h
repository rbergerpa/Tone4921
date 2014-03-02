#ifndef __TONE4921__
#define __TONE4921__

class Tone4921 {
 public:
  Tone4921();
  Tone4921(int _sample_rate);

  void start();
  void setFrequency(int hertz);
  void setGain(int gain);
  void setEnabled(int enabled);
  void setCallback(void (*callback)());
};

#endif
