#include "PSKrx.h"

// TODO add calculation from frequency to coeff
// TODO add programmer pre amp

void setup() {
  Serial.begin(115200);
  // BFSKrxSetup takes in coeff 
  // cos, sin, coeff, bitrate, code bit length
  BPSKrxSetup(float(0.0628), float(0.998), float(0.13), float(100), 101); // 24khz
  BPSKrxADCSetup();
  BPSKrxStart();
}

void loop() {
}
