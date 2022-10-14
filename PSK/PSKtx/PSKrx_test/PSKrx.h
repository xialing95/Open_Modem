/*   MIT Future Ocean Lab  
--------------------------------------------------------------------------------
 Project:       FOL OpenModem
 Version:       V1
 Design:        OpenModem Hardware V2
 Substrate:     Teensy 4.0
--------------------------------------------------------------------------------
 Module:        PSK demodulation Library
 Filename:      BPSKrx.cpp
 Created:       June 10 2020
 Author:        Charlene Xia (cxia_1@mit.edu)
--------------------------------------------------------------------------------
*/

#ifndef BPSKRX_H
#define BPSKRX_H

//TODO: #include "CircularBuffer.h"
#include "arduino.h"
#include "ADC.h"
#include "ADC_util.h"
#include "math.h"

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

// PSK Receiver setting (match with Transmission)
int package_length;   // decision maker in IRS
float usec;           // decision maker in IRS
uint32_t cycles;      // decision maker in IRS

// Goertzel Filter variable 
#define BUFFER_SIZE 100
#define THRESHOLD_VALUE 

// Goertzel Filter variable in ADC_IRS
volatile uint16_t buffer_adc_0_count = 0;
float sinCoeff, cosCoeff, coeff;
volatile double sig_phase;
volatile double prev_sig_phase = 1;
float q1, q2, q0;
volatile double real, imag;

// Decision maker variable in PIT_IRS
volatile int start_check = 0;
volatile int bitCount = 0;
volatile float phase_shift;

// external function
void BPSKrxSetup(float, float, float, float);
void BPSKrxADCSetup();
void BPSKrxStart();
void BPSKrxStop();

// internal function
void adc_isr();
void PIT_setup();
void mod_isr();
 
#endif

/*
 object class: BPSKtx
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
void BPSKrxSetup(float freqCosCoeff, float freqSinCoeff, float freqCoeff, float bitRate, int packageBit){
    pinMode(analogInPin, INPUT);
    pinMode(digitalOutPin1, OUTPUT);
    pinMode(digitalOutPin2, OUTPUT);

    sinCoeff = freqSinCoeff;
    cosCoeff = freqCosCoeff;
    coeff = freqCoeff;
    
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
void BPSKrxADCSetup(){
    // setup ADC0 configuration
    adc->adc0->setAveraging(1);
    adc->adc0->setResolution(8);
    adc->adc0->setConversionSpeed(ADC_CONVERSION_SPEED::HIGH_SPEED);
    adc->adc0->setSamplingSpeed(ADC_SAMPLING_SPEED::HIGH_SPEED);
}

/*
 Function: BFSKrxStart()
 Description:   Starting ADC sampling and interrupts. 

 - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -
 Setting up ADC:

    Detail in datasheet page:
    Also described in Pubpub page.
*/
void BPSKrxStart(){
    // Setup ADC0 interrupt start sampling
    adc->adc0->stopQuadTimer();
    adc->adc0->startSingleRead(analogInPin);
    adc->adc0->enableInterrupts(adc_isr);
    adc->adc0->startQuadTimer(samplingFreq);
    Serial.println("ADC Sampling setup finished");
   
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
    // Serial.println(adc_val);
    
    // saving the ADC value to the Buffer
        if (buffer_adc_0_count < BUFFER_SIZE-1){
            q0 = float(adc_val) + coeff * q1 - q2;
            q2 = q1;
            q1 = q0;
            buffer_adc_0_count++;
        } else {
            // reset buffer count
            buffer_adc_0_count = 0;

            // calculate goertzel magnitude, normalized by dividing via Buffer size
            // real value
            real = q1-q2*cosCoeff;
            // imag value
            imag = q2*sinCoeff;
            // phase
            sig_phase = atan2(imag, real)*115;
            // magnitude 
            //Serial.println(sig_phase);
            // reset goertzel variable
            q1 = 0;
            q2 = 0;

        }

        //Serial.println(sig_phase);
        
//  if (abs(sig_phase - prev_sig_phase) > 1){
//    // if the change in degree is > 0.7*pi then there is a 180 phase shift
//    digitalWriteFast(digitalOutPin1, !digitalReadFast(digitalOutPin1));
//  }
//  prev_sig_phase = sig_phase;

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
  
    // the timer should be running at lower than to alise 24kHz, 
    // set the Timer Load Value Register
    PIT_LDVAL0 = cycles;
  
    // attach interrupt vector 
    attachInterruptVector(IRQ_PIT, &mod_isr);
  
    // set priority
    NVIC_SET_PRIORITY(IRQ_PIT, 255);
  
    Serial.println("PIT timer finish setup");
}

void mod_isr(){
  // reset the TFLG 
  PIT_TFLG0 = 1;

  // Test checking decision slicer rate
//  digitalWriteFast(digitalOutPin2, !digitalReadFast(digitalOutPin2));

  // taking the derivative to find the change
  phase_shift = abs(prev_sig_phase-sig_phase);

  // Test phase
  //Serial.println(phase_shift);

    // check for start bit (1)
  if (phase_shift > float(200) && start_check == 0){
    // if the change in degree is > 0.7*pi then there is a 180 phase shift
    Serial.println(phase_shift);
    start_check = 1;
  }

  // Starting transmission start print decode sig after starting bit
  // it will print the starting bit as 1
  if (start_check){
    if(phase_shift > float(200)){
      // if the change in degree is > 0.7*pi then there is a 180 phase shift
      digitalWriteFast(digitalOutPin1, 1);
      Serial.print(1);
      Serial.print(",");
    }
    else{
      digitalWriteFast(digitalOutPin1, 0);
      Serial.print(0);
      Serial.print(",");
    }
    bitCount ++;
  }

  // reset till receiver starting bit again
  if (bitCount == package_length){
    bitCount = 0;
    start_check = 0;
    Serial.println();
  }

  prev_sig_phase = sig_phase;
}
