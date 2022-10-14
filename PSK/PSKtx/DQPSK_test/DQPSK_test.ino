//

#include "Arduino.h"

#define pinOut1 5
#define pinOut2 6
#define freq 24000
#define bit_duration 10

float usec;
int32_t count;
uint32_t cycles;

volatile int32_t toggle_count;
volatile int prevPinState = 0;
volatile int32_t delay_val;

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
  
  PIT_setup();
  Serial.begin(9600);
  Serial.println(count);
  
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(led, OUTPUT);

}

int code[4] = {0, 1, 1, 0};
volatile int code_count = 0;
int code_bit_size;

void loop() {

  buttonState = digitalRead(buttonPin);
  if (buttonState != preState){
    if (buttonState == HIGH){
      digitalWrite(led, LOW);
      delay(1000);
      digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(1000);               // wait for a second
      digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
      Serial.println("pressed");

      code_count = 0;
      psk_code(4);
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
    
    // update trigger timing to 48 khz
    // disable timer and interrupt 
    PIT_TCTRL0 = 0;
    // Update Timer Load Value Register
    PIT_LDVAL0 = cycles / 2;
    // enable timer and interrupt
    PIT_TCTRL0 = 0x03;

    // toggle 
    digitalWriteFast(pinOut1, !prevPinState);
    digitalWriteFast(pinOut2, prevPinState);
    prevPinState = digitalReadFast(pinOut1);

    // increase toggle count
    toggle_count++;
  }

  // toggle for the right amount of count
  // example toggle_count >= 2 && toggle_count < 100, 2 to 99
  else if (toggle_count >= delay_val && toggle_count < count){
    
    // update trigger timing to 48 khz
    // disable timer and interrupt 
    PIT_TCTRL0 = 0;
    // Update Timer Load Value Register
    PIT_LDVAL0 = cycles;
    // enable timer and interrupt
    PIT_TCTRL0 = 0x03;
    
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
      break;
    case 0:
      delay_val = 0;
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

  // attach interrupt  vector 
  attachInterruptVector(IRQ_PIT, &mod_isr);

  // set priority
  NVIC_SET_PRIORITY(IRQ_PIT, 255);
}
