//Define Arduino UNO CPU clock
#define F_CPU 16000000L

//=======================================================
//======            Include libraries             =======
//=======================================================

//#include <avr/wdt.h>  //Watchdog Timer Library
#include "I2Cdev.h"
#include <stdfix.h>
#include <math.h>


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

//=======================================================
//======                 Makros                   =======
//=======================================================

#define ENC1_A 2
#define ENC1_B 3

#define ENC2_A 18
#define ENC2_B 19

#define ENC3_A 20
#define ENC3_B 21

#define DC1_PWM 4
#define DC2_PWM 5
#define DC3_PWM 6

#define DC1_DIR 7
#define DC2_DIR 8
#define DC3_DIR 9

#define LS1_NC 48
#define LS1_NO 49
#define LS2_NC 50
#define LS2_NO 51
#define LS3_NC 52
#define LS3_NO 53

//=======================================================
//======             GLOBAL VARIABLES             =======
//=======================================================
volatile long time1, temp1, counter1 = 0;
volatile long time2, temp2, counter2 = 0;
volatile long time3, temp3, counter3 = 0;

void setup() {
  //-----------Start Serial Communication-------------
  Serial.begin(9600);

  //-------------Define Pins--------------------------
  //Interruput pins
  pinMode(ENC1_A, INPUT_PULLUP);  // internal pullup input pin 2

  pinMode(ENC1_B, INPUT_PULLUP);  // internal pullup input pin 3

  pinMode(ENC2_A, INPUT_PULLUP);  // internal pullup input pin 18

  pinMode(ENC2_B, INPUT_PULLUP);  // internal pullup input pin 19

  pinMode(ENC3_A, INPUT_PULLUP);  // internal pullup input pin 20

  pinMode(ENC3_B, INPUT_PULLUP);  // internal pullup input pin 21

  //Limit switches
  pinMode(LS1_NC, INPUT_PULLUP);

  pinMode(LS1_NO, INPUT_PULLUP);

  pinMode(LS2_NC, INPUT_PULLUP);

  pinMode(LS2_NO, INPUT_PULLUP);

  pinMode(LS3_NC, INPUT_PULLUP);

  pinMode(LS3_NO, INPUT_PULLUP);

  //DC Motor driver
  pinMode(DC1_PWM, OUTPUT);

  pinMode(DC2_PWM, OUTPUT);

  pinMode(DC3_PWM, OUTPUT);

  pinMode(DC1_DIR, OUTPUT);

  pinMode(DC2_DIR, OUTPUT);

  pinMode(DC3_DIR, OUTPUT);

  //----------------  //Setting up interrupt---------------------------
  //Encoder1 Ch.A rising pulse from encodenren activated ai0(). AttachInterrupt 0 is DigitalPin nr 2.
  attachInterrupt(digitalPinToInterrupt(ENC1_A), ai0, RISING);

  //Encoder1 Ch.B rising pulse from encodenren activated ai1(). AttachInterrupt 1 is DigitalPin nr 3.
  attachInterrupt(digitalPinToInterrupt(ENC1_B), ai1, RISING);

  //Encoder2 Ch.A rising pulse from encodenren activated ai1(). AttachInterrupt 2 is DigitalPin nr 18.
  attachInterrupt(digitalPinToInterrupt(ENC2_A), ai2, RISING);

  //Encoder2 Ch.B rising pulse from encodenren activated ai1(). AttachInterrupt 3 is DigitalPin nr 19.
  attachInterrupt(digitalPinToInterrupt(ENC2_B), ai3, RISING);

  //Encoder3 Ch.A rising pulse from encodenren activated ai1(). AttachInterrupt 4 is DigitalPin nr 20.
  attachInterrupt(digitalPinToInterrupt(ENC3_A), ai4, RISING);

  //Encoder3 Ch.B rising pulse from encodenren activated ai1(). AttachInterrupt 5 is DigitalPin nr 21.
  attachInterrupt(digitalPinToInterrupt(ENC3_B), ai5, RISING);

  //---------------------------Referenzfahrt-------------------------------------------------
}

void loop() {
  // Send the value of counter
  if (counter1 != temp1) {
    Serial.println(counter1);
    temp1 = counter1;
  }
  if (counter2 != temp2) {
    Serial.println(counter1);
    temp2 = counter2;
  }
  if (counter3 != temp3) {
    Serial.println(counter1);
    temp3 = counter3;
  }
}

void ai0() {
  // ai0 is activated if DigitalPin ENC1_A is going from LOW to HIGH
  // Check pin ENC1_B to determine the direction
  if (digitalRead(ENC1_B) == LOW) {
    counter1++;
  } else {
    counter1--;
  }
}

void ai1() {
  // ai1 is activated if DigitalPin ENC1_B is going from LOW to HIGH
  // Check with pin ENC1_A to determine the direction
  if (digitalRead(ENC1_A) == LOW) {
    counter1--;
  } else {
    counter1++;
  }
}

void ai2() {
  // ai2 is activated if DigitalPin ENC2_A is going from LOW to HIGH
  // Check with pin ENC2_B to determine the direction
  if (digitalRead(ENC2_B) == LOW) {
    counter2--;
  } else {
    counter2++;
  }
}

void ai3() {
  // ai3 is activated if DigitalPin ENC2_B is going from LOW to HIGH
  // Check with pin ENC2_A to determine the direction
  if (digitalRead(ENC2_A) == LOW) {
    counter2--;
  } else {
    counter2++;
  }
}

void ai4() {
  // ai4 is activated if DigitalPin ENC3_A is going from LOW to HIGH
  // Check with pin ENC3_B to determine the direction
  if (digitalRead(ENC3_B) == LOW) {
    counter3--;
  } else {
    counter3++;
  }
}

void ai5() {
  // ai5 is activated if DigitalPin ENC3_B is going from LOW to HIGH
  // Check with pin ENC3_A to determine the direction
  if (digitalRead(ENC3_A) == LOW) {
    counter3--;
  } else {
    counter3++;
  }
}