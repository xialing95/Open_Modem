#include "BFSKtx.h"

BFSKtx test((uint16_t) 28000, (uint16_t) 24000);

const int buttonPin = 2;
int led = 13;

void setup(){
  Serial.begin(9600);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(led, OUTPUT);
}

int buttonState = 0;
int preState = 0;
int code[100] = 
{1,1,0,1,1,0,0,1,1,1,\
0,1,1,0,1,0,0,1,1,1,\
1,0,1,1,1,1,1,0,1,0,\
1,0,0,0,0,1,1,0,1,0,\
0,0,1,1,0,0,0,1,1,1,\
0,1,1,0,0,0,1,0,1,0,\
1,0,1,1,1,1,1,0,0,0,\
1,0,1,0,1,0,0,0,1,0,\
0,1,1,1,1,0,1,1,0,1,\
0,0,1,1,1,0,1,0,0,0};

void loop(){  

  buttonState = digitalRead(buttonPin);

  if (buttonState != preState){
    if (buttonState == HIGH){
      digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
      delay(100);               // wait for a second
      digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
      //Serial.println("pressed");
      
      // modCode(bit frequency, wait time (ms), code array, code bit length)
      test.modCode(200, 5, code, 100);
    }
  }
  
  preState = buttonState;
}  
