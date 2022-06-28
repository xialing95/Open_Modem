/*   MIT Future Ocean Lab  
--------------------------------------------------------------------------------
 Project:       FOL OpenModem
 Version:       V1
 Design:        OpenModem Hardware V2
 Substrate:     Teensy 4.0
--------------------------------------------------------------------------------
 Module:        BPSK Modulation Library
 Filename:      BPSKtx.cpp
 Created:       June 10 2020
 Author:        Charlene Xia (cxia_1@mit.edu)
--------------------------------------------------------------------------------
 Description:   BPSKtx is a Teensy 4.0 executable that operates with the FOL 
                OpenModem V2 hardware. It is works with the processing board 
                connected to the transmission board. The main purpose is to ...

 1. Hardware connection and interface between Processing & Transmissions Board
 2. Setting BPSK initialization values: carrier frequency, transmission rate
 3. Functions for transmission BPSK signals. 

 In a little more detail.....
 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    A. How the Library Works

    The library is currently consisted of a group of functions. To 

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
   

 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 Notes: 
    
    Todo: reorganize to class library

*/

#include "Arduino.h"

// Hardware Pinout
#define pinOut1 5
#define pinOut2 6

// carrier frequency and bit duration in milliseconds
#define freq 24000
#define bit_duration 10 // 1/0.005

// Initization calcuation 
float usec;
int32_t count;
uint32_t cycles;

// code variables
// the start bits are [0 1], since DPSK needs two bits to differ
//int code[101] = {0,
//1,1,0,1,1,0,0,1,1,1,\
//0,1,1,0,1,0,0,1,1,1,\
//1,0,1,1,1,1,1,0,1,0,\
//1,0,0,0,0,1,1,0,1,0,\
//0,0,1,1,0,0,0,1,1,1,\
//0,1,1,0,0,0,1,0,1,0,\
//1,0,1,1,1,1,1,0,0,0,\
//1,0,1,0,1,0,0,0,1,0,\
//0,1,1,1,1,0,1,1,0,1,\
//0,0,1,1,1,0,1,0,0,0};


int code[101] = {0, 
1,1,1,0,1,1,1,1,1,1,\
1,1,0,0,1,1,0,0,1,0,\
0,0,1,0,1,0,0,0,0,0,\
1,0,0,0,1,1,0,1,1,1,\
1,1,0,0,1,0,0,1,0,1,\
0,1,0,1,1,0,1,0,0,1,\
1,0,0,1,1,1,0,0,0,0,\
1,1,0,0,1,0,1,1,1,0,\
0,1,1,1,0,0,1,0,1,1,\
1,0,1,0,0,1,1,1,1,0};

int code_length = 101;
int code_bit_size;

// Interrupt service variables
volatile int32_t toggle_count;
volatile int prevPinState = 0;
volatile int32_t delay_val;
volatile int code_count = 0;

// button trigger testing example
int preState;
const int buttonPin = 2;
int led = 13;
int buttonState = 0;

void setup() {
  usec = (float)500000.0 / (float)freq;
  count = ((float) bit_duration / 1000 * freq) * 2;
  cycles = float(24000000 / 1000000) * usec - 1;
  
  //pinMode(bit_tx_done, OUTPUT);
  pinMode(pinOut1, OUTPUT);
  pinMode(pinOut2, OUTPUT);

  //testing pin
  pinMode(4, OUTPUT);
  
  PIT_setup();
  Serial.begin(9600);
  Serial.println(count);
  
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(led, OUTPUT);
}


void loop() {
  buttonState = digitalRead(buttonPin);
  if (buttonState != preState){
    if (buttonState == HIGH){
      digitalWrite(led, LOW);
      delay(100);
      digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(100);               // wait for a second
      digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
      Serial.println("pressed");

      code_count = 0;
      psk_code(code_length);
  }
 }  
  preState = buttonState;
}

// triggered the bit tx is high, update wait period and toggle count
// wait 0 period, wait 1 trigger, wait 2 trigger, wait 3 trigger
// restart the mod_isr interrupts, and start with where the signal left off !pinlastvalue

void psk_code(int code_length){
  toggle_count = 0;
  code_bit_size = code_length;
  Serial.println(code_bit_size);
  
  // start mod interrupt & start with zero
  // enable timer and interrupt
  PIT_TCTRL0 = 0x03;
  
  // enable IRQ
  NVIC_ENABLE_IRQ(IRQ_PIT);   

  digitalWriteFast(pinOut1, LOW);
  digitalWriteFast(pinOut2, LOW);
}

// take the wait period and the toggle count
// update the last pin value and the bit tx done stop interrupts
void mod_isr(){
  // reset the TFLG 
  PIT_TFLG0 = 1;

  // wait for the right trigger for the number of delay value
  // example toggle_count < 2, 0 and 1
  if (toggle_count < delay_val){
    digitalWriteFast(pinOut1, LOW);
    digitalWriteFast(pinOut2, LOW);

    // increase toggle count
    toggle_count++;
  }

  // toggle for the right amount of count
  // example toggle_count >= 2 && toggle_count < 100, 2 to 99
  else if (toggle_count >= delay_val && toggle_count < count){
    // set pinOut1 and pinOut2 to low
    digitalWriteFast(pinOut1, !prevPinState);
    digitalWriteFast(pinOut2, prevPinState);
    prevPinState = digitalReadFast(pinOut1);

    // increase toggle count
    toggle_count++;
  }
  
  // when toggle_count is finish at the right count, switch to the next bit
  // example toggle_count >= 100, 100+
  else if (toggle_count >= count){
    
    // finish transmiting the one bit, reset toggle count
    toggle_count = 0;

    // move to the next bit
    code_count++;

    // if code count is same as input code then stop interrupt finish tx
    if (code_count >= code_bit_size){
            
      // disable timer and interrupt 
      PIT_TCTRL0 = 0;
  
      // disable IRQ_PIT
      NVIC_DISABLE_IRQ(IRQ_PIT);
  
      // set pinOut1 and pinOut2 to low
      digitalWriteFast(pinOut1, LOW);
      digitalWriteFast(pinOut2, LOW);
    } 

    // update delay value
    switch (code[code_count]){
    case 1:
      delay_val = 2;
      //testing output
      digitalWriteFast(4, HIGH);
      break;
    case 0:
      delay_val = 0;
      //testing output
      digitalWriteFast(4, LOW);
      break;
    }
  }
}

void PIT_setup(){
  // setting the clock on for PIT
  // CCM_CCGR_1_PIT(CCM_CCGR_ON) = 0x3000
  CCM_CCGR1 |= CCM_CCGR1_PIT(CCM_CCGR_ON);

  // Turn on PIT module
  PIT_MCR = 0x00;

  // set the Timer Load Value Register
  PIT_LDVAL0 = cycles;

  // attach interrupt vector 
  attachInterruptVector(IRQ_PIT, &mod_isr);

  // set priority
  NVIC_SET_PRIORITY(IRQ_PIT, 255);
}
