#include <stdbool.h>

#ifndef DCO_H
#define DCO_H

class DCO {
public:
  DCO();
  ~DCO();

  void ncoSetNote();
  void ncoStep();
  


private:
  uint32_t accumulator;
  uint32_t phase;
  uint8_t value;
  const uint8_t *wavetable;

};

#endif

