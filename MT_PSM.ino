//Define Arduino UNO CPU clock
#define F_CPU 16000000L

//=======================================================
//======            Include libraries             =======
//=======================================================

//#include <avr/wdt.h>  //Watchdog Timer Library
#include "I2Cdev.h"
#include <stdfix.h>
#include <math.h>
#include "CytronMotorDriver.h"


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

// Configure the motor driver.
CytronMD motor1(PWM_DIR, DC1_PWM, DC1_DIR);  // PWM 1 = Pin 4, DIR 1 = Pin 7.
CytronMD motor2(PWM_DIR, DC2_PWM, DC2_DIR);  // PWM 2 = Pin 5, DIR 2 = Pin 8.
CytronMD motor3(PWM_DIR, DC3_PWM, DC3_DIR);  // PWM 2 = Pin 6, DIR 2 = Pin 9.
CytronMD motor[3] = { motor1, motor2, motor3 };

//Encoder constant parameters
const float res_avago = 0.36;

// Optical encoder variables
volatile long time1, temp1, counter1 = 0;
volatile long time2, temp2, counter2 = 0;
volatile long time3, temp3, counter3 = 0;

//State machine variables
char state;
bool ref1, ref2, ref3;
bool ref[3] = { ref1, ref2, ref3 };
bool ref_drive1, ref_drive2, ref_drive3;
bool refpos1, refpos2, refpos3;

int p_counter1, p_counter2, p_counter3 = 0;

int ref_counter1, ref_counter2, ref_counter3 = 0;

const int threshold = 1;

float pos1, pos2, pos3;

float ref_offset1, ref_offset2, ref_offset3;

//Transmission ration
const float trans1 = 20.00;
const float trans2 = 13.33;

//Joint values
float q1, q2, q3, q4, q5, q6, q7;

//Motor variables
int speed1, speed2, speed3 = 0;
int dir1, dir2, dir3 = 0;


void setup() {
  //-----------Set initial conditions----------------
  motor1.setSpeed(0);
  motor2.setSpeed(0);
  motor3.setSpeed(0);
  state = "Startup";
  ref1, ref2, ref3 = false;
  ref_drive1, ref_drive2, ref_drive3 = false;
  speed1, speed2, speed3 = 50;
  pos1, pos2, pos3 = 10;

  //-----------Start Serial Communication-------------
  Serial.begin(115200);

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

  //-----------------------------------------------------------------------------------------
  //------------------------------Referenzfahrt Motor1---------------------------------------
  //-----------------------------------------------------------------------------------------
  int pinstatusNC;
  int pinstatusNO;
  while (!ref1) {
    pinstatusNC = digitalRead(LS1_NC);  //If HIGH then button is pushed
    pinstatusNO = digitalRead(LS1_NO);  //If HIGH THEN button is not pushed

    //--------------------Check status-------------------------
    if (pinstatusNC == HIGH && pinstatusNO == LOW) {  //Touches Endposition
      state = 'Reference';
      refpos1 = true;
    }

    else if (pinstatusNC == LOW && pinstatusNO == HIGH && !ref_drive1) {  //Does not touch endposition
      state = 'MoveBack';
    }

    else if (pinstatusNC == LOW && pinstatusNO == LOW) {
      motor1.setSpeed(0);
      Serial.print("Limit switch 1 is broken! Emergency stop!");
      while (1) {};
    }

    else {
      motor1.setSpeed(0);
      Serial.print("Invalid state! Emergency stop!");
      while (1) {};
    }

    //--------------------State machine-----------------------
    switch (state) {
      case 'MoveBack':
        dir1 = -1;
        motor1.setSpeed(dir1 * speed1);
        if (pinstatusNC == LOW && pinstatusNO == HIGH) {
          motor1.setSpeed(0);
          refpos1 = true;
          state = 'Reference';
        }
        break;

      case 'MoveForward':
        ref_drive1 = true;  //Indicates current reference drive
        dir1 = 1;
        motor1.setSpeed(dir1 * speed1);
        //Check for end position
        if ((toAngle(counter1) / trans1) >= pos1) {
          motor1.setSpeed(0);
          state = 'MoveBack';
          delay(1000);
        }
        break;

      case 'Reference':
        motor1.setSpeed(0);

        //Phase check
        if (p_counter1 == 0) {
          speed1 = 50;
          pos1 = 10;
        } else if (p_counter1 == 1) {
          speed1 = 20;
          pos1 = 5;
        } else {
          speed1 = 10;
          pos1 = 5;
        }

        //Set reference or compare is phase!=0
        if (p_counter1 == 0 && refpos1 == true) {
          ref_counter1 = counter1;
          counter1 = 0;
        } else if (p_counter1 > 0 && refpos1 == true) {
          int dif = ref_counter1 - counter1;
          if (dif <= threshold) {
            counter1 = 0;
            ref1 = true;
            ref_drive1 = false;
          } else {
            ref_counter1 = counter1;
            counter1 = 0;
          }
        }
        //Switch phase
        p_counter1++;
        //switch case
        state = 'MoveForwards';
        delay(1000);
        break;

      default:
        // Statement(s)
        break;  // Wird nicht benötigt, wenn Statement(s) vorhanden sind
    }
  }

  //-----------------------------------------------------------------------------------------
  //------------------------------Referenzfahrt Motor2---------------------------------------
  //-----------------------------------------------------------------------------------------
  while (!ref2) {
    pinstatusNC = digitalRead(LS2_NC);  //If HIGH then button is pushed
    pinstatusNO = digitalRead(LS2_NO);  //If HIGH THEN button is not pushed

    //--------------------Check status-------------------------
    if (pinstatusNC == HIGH && pinstatusNO == LOW) {  //Touches Endposition
      state = 'Reference';
      refpos2 = true;
    }

    else if (pinstatusNC == LOW && pinstatusNO == HIGH && !ref_drive2) {  //Does not touch endposition
      state = 'MoveBack';
    }

    else if (pinstatusNC == LOW && pinstatusNO == LOW) {
      motor1.setSpeed(0);
      Serial.print("Limit switch 2 is broken! Emergency stop!");
      while (1) {};
    }

    else {
      motor1.setSpeed(0);
      Serial.print("Invalid state! Emergency stop!");
      while (1) {};
    }
  }

  //-----------------------------------------------------------------------------------------
  //------------------------------Referenzfahrt Motor3---------------------------------------
  //-----------------------------------------------------------------------------------------
  while (!ref3) {
    pinstatusNC = digitalRead(LS3_NC);  //If HIGH then button is pushed
    pinstatusNO = digitalRead(LS3_NO);  //If HIGH THEN button is not pushed

    //--------------------Check status-------------------------
    if (pinstatusNC == HIGH && pinstatusNO == LOW) {  //Touches Endposition
      state = 'Reference';
      refpos2 = true;
    }

    else if (pinstatusNC == LOW && pinstatusNO == HIGH && !ref_drive3) {  //Does not touch endposition
      state = 'MoveBack';
    }

    else if (pinstatusNC == LOW && pinstatusNO == LOW) {
      motor1.setSpeed(0);
      Serial.print("Limit switch 3 is broken! Emergency stop!");
      while (1) {};
    }

    else {
      motor1.setSpeed(0);
      Serial.print("Invalid state! Emergency stop!");
      while (1) {};
    }
  }
}

//-----------------------------------------------------------------------------------------
//------------------------------------------MAIN-------------------------------------------
//-----------------------------------------------------------------------------------------
void loop() {
  /*while (1) {
    Serial.print("Limit switch 1:\t");
    Serial.print(digitalRead(LS1_NC));
    Serial.print('\t');
    Serial.println(digitalRead(LS1_NO));

    Serial.print("Limit switch 2:\t");
    Serial.print(digitalRead(LS2_NC));
    Serial.print('\t');
    Serial.println(digitalRead(LS2_NO));

    Serial.print("Limit switch 3:\t");
    Serial.print(digitalRead(LS3_NC));
    Serial.print('\t');
    Serial.println(digitalRead(LS3_NO));
    delay(200);
  }*/

  if (counter1 != temp1) {
    Serial.println(counter1);
    temp1 = counter1;
  }
  if (counter2 != temp2) {
    Serial.println(counter2);
    temp2 = counter2;
  }
  if (counter3 != temp3) {
    Serial.println(counter3);
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
    counter2++;
  } else {
    counter2--;
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
    counter3++;
  } else {
    counter3--;
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

float toAngle(int count) {
  float q;
  q = (float)count * res_avago;
  return q;
}