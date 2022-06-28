/*
  BFSK_TX.h - Open Modem Binary FSK transmission code
  Future Oceans Lab
*/
#ifndef BFSKtx_h
#define BFSKtx_h

#include "Arduino.h"

#define pinOut1 5
#define pinOut2 6
static volatile uint32_t toggle_count;
void mod_isr();

class BFSKtx {
public:
  BFSKtx(uint16_t freq_one, uint16_t freq_zero);
  void modBit(float bit_duration, int one_bit);
  void modCode(float bitPerSec, float wait_duration, int code[], int code_bit_size);
  void INFO();

private:
  uint16_t _freq0, _freq1;
  float _usec;
  volatile uint32_t _count;
};

#endif
