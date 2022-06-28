/*   MIT Future Ocean Lab  
--------------------------------------------------------------------------------
 Project:       FOL OpenModem
 Version:       V1
 Design:        OpenModem Hardware V2
 Substrate:     Teensy 4.0
--------------------------------------------------------------------------------
 Module:        BFSK Modulation Library
 Filename:      BFSKtx.cpp
 Created:       June 10 2020
 Author:        Charlene Xia (cxia_1@mit.edu)
--------------------------------------------------------------------------------
 Description:   BFSKtx is a Teensy 4.0 executable that operates with the FOL 
                OpenModem V2 hardware. It is works with the processing board 
                connected to the transmission board. The main purpose is to ...

 1. Hardware connection and interface between Processing & Transmissions Board
 2. Setting BFSK initialization values: modulation frequency, transmission rate,
    and wait time
 3. Functions for transmission a single bit, and n-bit code

 In a little more detail.....
 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    A. How the Library Works

    First initialize a object under the class BFSKtx, to transmit call either modBit
    or modCode. The switching of the digital output is control by an interrupt on
    the Periodic Interrupt Time (PIT). Currently the interrupt priority is set at 
    255 (pretty low). If you are running other interrupts make sure to select the 
    appropriate priority setting so the transmission stay coherent. 

    B. Hardware connections:

    The Teensy 4.0 on the processing board outputs 2 square wave signals, inverse
    of each other, to the transmission board. The 2 square wave signals are input
    to the MOSFET gate drive ICs to control the H-Bridge signal. *The pin is hard 
    coded in the Library*, since the device's hardware is set. If changes are made
    in the hardware, make sure to change the pinOut1 & pinOut2 value on the header
    file. 

    C. Quick Summary of objects and functions, details above each function:
    
    BFSKtx (freq_one, freq_zero) - 
    BFSK:INFO() - 
    BFSKtx:modBit() - 
    BFSKtx:modCode() - 
    mod_isr() - 

 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 Notes: 
    
    Todo: Change from BFSK to N-FSK.
          Add more helper program, INFO, testing etc. 

*/

#include "Arduino.h" 
#include "BFSKtx.h"

/*
 object class: BFSKtx
 Description:   Setting up frequencies and hardware pinmode for transmission.
                

 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 Variable:

    bit_duration: the period of the one bit in milliseconds. Usually you want
    to think of transmission as x bit/bytes/baud per seconds.
    calculation is 1000 msec/sec / (N bit/sec)
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
NOTE: not sure why i convert bit_duration to milliseconds, then switch it back
to second in modBit.
*/
BFSKtx::BFSKtx(uint16_t freq_one, uint16_t freq_zero) {
  _freq0 = freq_zero;
  _freq1 = freq_one;
  pinMode(pinOut1, OUTPUT);
  pinMode(pinOut2, OUTPUT);

  // testing 
  // pinMode(4, OUTPUT);
}

void BFSKtx::INFO() {
  Serial.print("Frequency 0: ");
  Serial.println(_freq0);
  Serial.print("Frequency 1: ");
  Serial.println(_freq1);
  Serial.print("pinOut1: ");
  Serial.println(pinOut1);
  Serial.print("pinOut2: ");
  Serial.println(pinOut2);
  Serial.print("Toggle Count: ");
  Serial.println(toggle_count);
}

/*
 Function: modCode
 Description:   Transmit one modulated code > 1 bit. Takes in three arguments, 
                bit per second, wait time between bit tx, code list, and number
                of bit in the code. Uses modBit. 

 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 Variable:
    
    bit_duration: the period of the one bit in milliseconds. Usually you want 
    to think of transmission as x bit/bytes/baud per seconds. 
    calculation is 1000 msec/sec / (N bit/sec)
- - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
NOTE: not sure why i convert bit_duration to milliseconds, then switch it back 
to second in modBit. 
*/
void BFSKtx::modCode(float bitPerSec, float wait_duration, int code[], int code_bit_size){
  float bit_duration = float(1000/bitPerSec); // milliseconds
  for (int i = 0; i < code_bit_size; i ++){
    
    //digitalWriteFast(4, code[i]); //testing
    
    modBit(bit_duration, code[i]);
    delay (bit_duration+ wait_duration);
  }
}

/*
 Function: modBit
 Description:   Transmit one modulated bit. Takes in two arguments, tx bit 
                duration in milliseconds and data, either 0 or 1. 

 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 Variable: 

    _count: the count of the edge of the square wave, the number of edge up 
    and edge down.
    calculation is tx frequency x bit_duration x 2. 

    toggle_count: the public version of _count, used in interrupt service.
    Since interrupt service cannot use the private varible _count.

    _usec: the interrupt signal timing in microsecond, 1/2 of the period.
    calcuation is 1000000 usec/sec x (1/frequency x 2) 

 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 Setting up PIT:
    
    Detail in datasheet page:
    Also described in Pubpub page. 
*/
void BFSKtx::modBit(float bit_duration, int one_bit) {
  switch (one_bit){
  case 0:
      _count = (bit_duration / 1000 * _freq0) * 2;
      _usec = (float)500000.0 / (float)_freq0;
      break;
  case 1:
      _count = (bit_duration / 1000 * _freq1) * 2;
      _usec = (float)500000.0 / (float)_freq1;
      break;
  }
  toggle_count = _count;

  // setting the clock on for PIT
  // CCM_CCGR_1_PIT(CCM_CCGR_ON) = 0x3000
  CCM_CCGR1 |= CCM_CCGR1_PIT(CCM_CCGR_ON);

  // Turn on PIT module
  PIT_MCR = 0x00;

  // the cycles of interrupt setting depends on the microseconds 
  // (24000000/sec / 1000000 usec/sec) * usec - 1 = 
  uint32_t cycles = float(24000000 / 1000000) * _usec - 1;

  // set the Timer Load Value Register
  PIT_LDVAL0 = cycles;

  // enable timer and interrupt
  PIT_TCTRL0 = 0x03;

  // attach interrupt vector 
  attachInterruptVector(IRQ_PIT, &mod_isr);

  // set priority
  NVIC_SET_PRIORITY(IRQ_PIT, 255);
  
  // enable IRQ
  NVIC_ENABLE_IRQ(IRQ_PIT);  
}

/*
 Function: mod_isr
 Description:   Interrupt service routing. Triggered by PIT, when called
                it toggles the transmission pins high and low for the right  
                number of times, then disable the PIT timer and interrupts. 

 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 Setting up PIT:

    Detail in datasheet page:
    Also described in Pubpub page.
*/
void mod_isr() {

  // reset the TFLG 
  PIT_TFLG0 = 1;
  
  if (toggle_count) {
      digitalWriteFast(pinOut1, !digitalReadFast(pinOut1));
      digitalWriteFast(pinOut2, !digitalReadFast(pinOut1));
  }
  else{
      digitalWriteFast(pinOut1, 0);
      digitalWriteFast(pinOut2, 0);
      
      // disable timer and interrupt 
      PIT_TCTRL0 = 0;

      // disable IRQ_PIT
      NVIC_DISABLE_IRQ(IRQ_PIT);
  }
  
  toggle_count--;
  
  #if defined(__IMXRT1062__)  // Teensy 4.0
    asm("DSB");
  #endif
}
