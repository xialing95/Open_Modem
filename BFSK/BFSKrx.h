/*   MIT Future Ocean Lab  
--------------------------------------------------------------------------------
 Project:       FOL OpenModem
 Version:       V1
 Design:        OpenModem Hardware V2
 Substrate:     Teensy 4.0
--------------------------------------------------------------------------------
 Module:        nFSK demodulation Library
 Filename:      BFSKrx.cpp
 Created:       June 10 2020
 Author:        Charlene Xia (cxia_1@mit.edu)
--------------------------------------------------------------------------------
 Description:   BFSKrx is a Teensy 4.0 executable that operates with the FOL 
                OpenModem V2 hardware. It is works with the processing board 
                connected to the receiver board. The main purpose is to ...

 1. Hardware connection and interface between Processing & Recevier board
 2. Decode received nFSK signal via Goertzel Algorithm and output received 
    data

 In a little more detail.....
 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    A. How the Library Works


    B. Hardware connections:


    C. Quick Summary of objects and functions, details above each function:

 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 Notes: 
    
    Todo: ADD ring buffer, threshold detection, windowing 
          ADD calculation for coefficients

*/

#ifndef BFSKRX_H
#define BFSKRX_H

//TODO: #include "CircularBuffer.h"
#include "arduino.h"
#include "ADC.h"
#include "ADC_util.h"

/*
 User Control Variables
 Details on how to change variables in Pubpub page.
 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
    Digital Pin Out
    analogInPin

*/

// hardware pin setup
#define digitalOutPin1 7
#define digitalOutPin2 8

// ADC hardwae setup 
#define analogInPin A0 // pin 14 
#define samplingFreq 100000
#define USE_ADC_0
ADC *adc = new ADC();

// FSK Receiver setting 
#define N_FSK 2
int package_length; 

// Goertzel Filter variable 
// TODO: CircularBuffer<uint16_t, BUFFER_SIZE> BUFFER;
#define BUFFER_SIZE 20
#define THRESHOLD_VALUE 1000000

// Goertzel Filter variable in ADC IRS
volatile uint16_t buffer_adc_0_count = 0;
float coeff[N_FSK];
float mag[N_FSK];
float q1[N_FSK], q2[N_FSK], q0[N_FSK];

// decision maker in PIT IRS
float usec;
uint32_t cycles;
volatile int start_check = 0;
volatile int currBit;
volatile int prevBit;
volatile int bitCount = 0;

// external function
void BFSKrxSetup(uint16_t, uint16_t, float, int);
void BFSKrxADCSetup();
void BFSKrxStart();
void BFSKrxStop();

// internal function
void adc_isr();
void PIT_setup();
void mod_isr();
 
#endif

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
void BFSKrxSetup(float freqCoeff0, float freqCoeff1, float bitRate, int packageBit){
    pinMode(analogInPin, INPUT);
    pinMode(digitalOutPin1, OUTPUT);
    pinMode(digitalOutPin2, OUTPUT);

    coeff[0] = freqCoeff0; 
    coeff[1] = freqCoeff1;
    
    package_length = packageBit;
    usec = (float)1000000.0 / (float)bitRate;
    cycles = float(24000000 / 1000000) * usec - 1;
}

/*
 Function: BFSKrxADCSetup()
 Description:   Setting up Teensy 4.0 ADC for sampling
                
 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 Setting up ADC:

    Detail in datasheet page:
    Also described in Pubpub page.
*/
void BFSKrxADCSetup(){
    // setup ADC0 configuration
    adc->adc0->setAveraging(1);
    adc->adc0->setResolution(8);
    adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED);
    adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED);
}

/*
 Function: BFSKrxStart()
 Description:   Starting ADC sampling and interrupts. 

 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 Setting up ADC:

    Detail in datasheet page:
    Also described in Pubpub page.
*/
void BFSKrxStart(){
    // Setup ADC0 interrupt start sampling
    adc->adc0->stopQuadTimer();
    adc->adc0->startSingleRead(analogInPin);
    adc->adc0->enableInterrupts(adc_isr);
    adc->adc0->startQuadTimer(samplingFreq);

    PIT_setup();
    // start PIT interrupt 
    // enable timer and interrupt
    PIT_TCTRL0 = 0x03;
    // enable IRQ
    NVIC_ENABLE_IRQ(IRQ_PIT);  
}

/*
 Function: BFSKrxStop()
 Description:   Stop ADC sampling and Interrupt.

 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 Setting up ADC:

    Detail in datasheet page:
    Also described in Pubpub page.
*/
void BFSKrxStop(){
  // Stop ADC0 stop interrupt sampling
  adc->adc0->stopTimer();
  adc->adc0->disableInterrupts();

  // disable timer and interrupt 
  PIT_TCTRL0 = 0;
  
  // disable IRQ_PIT
  NVIC_DISABLE_IRQ(IRQ_PIT);
}

/*
 Function: BFSKrxStop()
 Description:   Stop ADC sampling and Interrupt.

 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 Setting up ADC:

    Detail in datasheet page:
    Also described in Pubpub page.
*/
void BPSKrxStop(){
  // Stop ADC0 stop interrupt sampling
  adc->adc0->stopTimer();
  adc->adc0->disableInterrupts();

  // disable timer and interrupt 
  PIT_TCTRL0 = 0;
  
  // disable IRQ_PIT
  NVIC_DISABLE_IRQ(IRQ_PIT);
}

/*
 Function adc_isr()
 Description:   Interrupt service routing. Triggered by PIT, when called
                it toggles the transmission pins high and low for the right  
                number of times, then disable the PIT timer and interrupts. 

 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 Setting up Goertzel Algorithm:
    
    Detail in Pubpub Page. 
*/
void adc_isr(){
    // read a new value
    uint16_t adc_val = adc->adc0->readSingle();
    //Serial.println(adc_val);
    
    // saving the ADC value to the Buffer
    for (int i = 0; i < N_FSK; i++){
        // should just added the - 2 hmmmm. check the counting on this again
        if (buffer_adc_0_count < BUFFER_SIZE*N_FSK-2){
            q0[i] = float(adc_val) + coeff[i] * q1[i] - q2[i];
            q2[i] = q1[i];
            q1[i] = q0[i];
            buffer_adc_0_count++;
        } else {
            // reset buffer count
            buffer_adc_0_count = 0;

            // calculate goertzel magnitude, normalized by dividing via Buffer size
            mag[i] = q1[i] * q1[i] + q2[i] * q2[i] - q1[i]*q2[i]*coeff[i];
            
            // reset goertzel variable
            q1[i] = 0;
            q2[i] = 0;    
        }
    }

      // Threshold Value        
  if (mag[0] > THRESHOLD_VALUE && mag[1] < THRESHOLD_VALUE){ 
      digitalWriteFast(digitalOutPin1, 0);
      //Serial.println(1);
  } else if (mag[1] > THRESHOLD_VALUE && mag[0] < THRESHOLD_VALUE){
      digitalWriteFast(digitalOutPin1, 1);
      //Serial.println(0);
  } else {
      digitalWriteFast(digitalOutPin1, 0);
      //Serial.println(1);
  }
  
    // Serial.println(mag[0]);

    // Add low pass filter?
    // Add decision slicer?
    
    #if defined(__IMXRT1062__)  // Teensy 4.0
        asm("DSB");
    #endif
}

// setup PIT timer for time slicer
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

// add if start bit 
void mod_isr(){
  // reset the TFLG 
  PIT_TFLG0 = 1;

  // Test checking decision slicer rate
  //digitalWriteFast(digitalOutPin2, !digitalReadFast(digitalOutPin2));
 
  currBit = digitalReadFast(digitalOutPin1);

  if (currBit && start_check == 0){
    start_check = 1;
  }
  
  if (start_check){
    
    Serial.print(currBit);
    Serial.print(",");
    bitCount ++;
    
    if (bitCount == package_length){
      bitCount = 0;
      start_check = 0;
      Serial.println();
    }
  }
  prevBit = currBit;
}
