
#include "BFSKrx.h"

// TODO add calculation from frequency to coeff
// TODO add programmer pre amp

void setup() {
  Serial.begin(115200);
  // BFSKrxSetup takes in coeff of 0 bit freq, 1 bit freq, bit rate and code length
  BFSKrxSetup(float(0.13), float(-0.37), float (100), 100); // 24khz, 28khz 
  BFSKrxADCSetup();
  BFSKrxStart();
}

void loop() {
}
