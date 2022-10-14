#include "math.h"
#include "Arduino.h"

double cal_phase;
int preState;
const int buttonPin = 2;
int led = 13;
int buttonState = 0;

void setup() {
  Serial.begin(9600);
    
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(led, OUTPUT);
}

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
      cal_phase = abs(10-100);
      Serial.println(cal_phase);
      cal_phase = abs(100-10);
      Serial.println(cal_phase);
  }
 }  
  preState = buttonState;
}
