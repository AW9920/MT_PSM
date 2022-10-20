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
#include <Servo.h>


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

#define SERVO1 10
#define SERVO2 11
#define SERVO3 12
#define SERVO4 13

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

//Configure Servo objects
Servo servo_roll;
Servo servo_pitch;
Servo servo_yaw1;
Servo servo_yaw2;

//Servo variables
int servo_val1, servo_val2, servo_val3, servo_val4;
int servo_off1, servo_off2, servo_off3, servo_off4;

//Encoder constant parameters
const float res_avago = 0.36;

// Optical encoder variables
volatile long time1, temp1, counter1 = 0;
volatile long time2, temp2, counter2 = 0;
volatile long time3, temp3, counter3 = 0;

//State machine variables
int state;
int home = 0;
int retrieve = 1;
int reference = 2;
int move_zero = 3;

bool ref1, ref2, ref3;
bool ref[3] = { ref1, ref2, ref3 };
bool ref_drive1, ref_drive2, ref_drive3;
bool refpos1, refpos2, refpos3;

int p_counter1 = 0, p_counter2 = 0, p_counter3 = 0;

int ref_counter1 = 0, ref_counter2 = 0, ref_counter3 = 0;

const int threshold = 1;

float pos1, pos2, pos3;  //In angular position of axis

float ref_offset1 = -40, ref_offset2 = -40, ref_offset3 = 0;

//Transmission ration
const float trans1 = 20.00;
const float trans2 = 13.33;

//Joint values
float q1, q2, q3, q4, q5, q6, q7;

//Maxon Motor variables
int speed1, speed2, speed3;
int dir1, dir2, dir3;

//PID controller variables
float kp, ki, kd;
float target_pos1, target_pos2, target_pos3;
float* target_pos[3] = { &target_pos1, &target_pos2, &target_pos3 };


void setup() {
  //-----------Set initial conditions----------------
  motor1.setSpeed(0);
  motor2.setSpeed(0);
  motor3.setSpeed(0);

  ref1 = false, ref2 = false, ref3 = false;
  ref_drive1 = false, ref_drive2 = false, ref_drive3 = false;

  //Attach pins to objects
  servo_roll.attach(SERVO1);
  servo_pitch.attach(SERVO2);
  servo_yaw1.attach(SERVO3);
  servo_yaw2.attach(SERVO4);

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
  Serial.println("Start Homing: Axis 1!");

  //Check pin status
  int pinstatusNC;
  int pinstatusNO;

  //Reference loop
  while (!ref1) {
    pinstatusNC = digitalRead(LS1_NC);  //If HIGH then button is pushed
    pinstatusNO = digitalRead(LS1_NO);  //If HIGH THEN button is not pushed
    //Debugging
    /*Serial.print("Limit switch 1:\t");
    Serial.print(digitalRead(LS1_NC));
    Serial.print('\t');
    Serial.println(digitalRead(LS1_NO));*/

    //--------------------Check status-------------------------
    if (pinstatusNC == HIGH && pinstatusNO == LOW) {  //Touches Endposition
      motor1.setSpeed(0);
      Serial.println("Reached Limit Switch");
      state = reference;
      refpos1 = true;
      ref_drive1 = true;  //Indicates current reference drive
    }

    else if (pinstatusNC == LOW && pinstatusNO == HIGH && !ref_drive1) {  //Does not touch endposition
      //Serial.println("Initial sate: Not at limit switch");
      state = home;
      refpos1 = false;
      ref_drive1 = true;  //Indicates current reference drive
    }

    else if (pinstatusNC == LOW && pinstatusNO == LOW) {
      motor1.setSpeed(0);
      Serial.print("Limit switch 1 is broken! Emergency stop!");
      ref_drive1 = false;  //Indicates current reference drive
      while (1) {};
    }

    //---------------------Phase check-------------------------
    if (p_counter1 == 0) {
      speed1 = 30;
      pos1 = 20;
    } else if (p_counter1 == 1) {
      speed1 = 30;
      pos1 = 10;
    } else {
      speed1 = 15;
      pos1 = 5;
    }

    //--------------------State machine-----------------------
    // Case 0 ---> Home (Move to LS)
    // Case 1 ---> Retrieve (Move away from LS_
    // Case 2 ---> Reference
    switch (state) {

      case 0:  //HOME
        //Serial.println("Homing!");
        dir1 = 1;
        motor1.setSpeed(dir1 * speed1);
        if (pinstatusNC == HIGH && pinstatusNO == LOW) {  //Redundant check
          motor1.setSpeed(0);
          refpos1 = true;
          state = reference;
        }
        break;

      case 1:  //RETRIEVE
        dir1 = -1;
        motor1.setSpeed(dir1 * speed1);
        Serial.print("Counter1: ");
        Serial.println(counter1);
        //Check for end position
        if (Ax1toAngle(counter1) >= pos1) {
          motor1.setSpeed(0);
          state = home;
          delay(1000);
        }
        break;

      case 2:  //REFERENCE
        motor1.setSpeed(0);
        //Set reference or compare is phase!=0
        if (p_counter1 == 0 && refpos1 == true) {
          ref_counter1 = 0;
          counter1 = 0;
        } else if (p_counter1 > 0 && refpos1 == true) {
          int dif = abs(ref_counter1 - counter1);
          if (dif <= threshold) {
            //Set counter to actual position
            counter1 = Ax1toCounts(ref_offset1);
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
        state = retrieve;
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
  Serial.println("Start Homing: Axis 2!");
  //Motor starting variables
  while (!ref2) {
    pinstatusNC = digitalRead(LS2_NC);  //If HIGH then button is pushed
    pinstatusNO = digitalRead(LS2_NO);  //If HIGH THEN button is not pushed

    //--------------------Check status-------------------------
    if (pinstatusNC == HIGH && pinstatusNO == LOW) {  //Touches Endposition
      motor2.setSpeed(0);
      Serial.println("Reached Limit Switch");
      state = reference;
      refpos2 = true;
      ref_drive2 = true;  //Indicates current reference drive
    }

    else if (pinstatusNC == LOW && pinstatusNO == HIGH && !ref_drive2) {  //Does not touch endposition
      state = home;
      refpos2 = false;
      ref_drive2 = true;  //Indicates current reference drive
    }

    else if (pinstatusNC == LOW && pinstatusNO == LOW) {
      motor2.setSpeed(0);
      Serial.print("Limit switch 2 is broken! Emergency stop!");
      while (1) {};
    }

    //---------------------Phase check-------------------------
    if (p_counter2 == 0) {
      speed2 = 30;
      pos2 = 20;
    } else if (p_counter2 == 1) {
      speed2 = 30;
      pos2 = 10;
    } else {
      speed2 = 15;
      pos2 = 5;
    }

    //--------------------State machine-----------------------
    // Case 0 ---> Home (Move to LS)
    // Case 1 ---> Retrieve (Move away from LS_
    // Case 2 ---> Reference
    switch (state) {

      case 0:  //HOME
        //Serial.println("Homing!");
        dir2 = 1;
        motor2.setSpeed(dir2 * speed2);
        if (pinstatusNC == HIGH && pinstatusNO == LOW) {  //Redundant check
          motor2.setSpeed(0);
          refpos2 = true;
          state = reference;
        }
        break;

      case 1:  //RETRIEVE
        dir2 = -1;
        motor2.setSpeed(dir2 * speed2);
        Serial.print("Counter2: ");
        Serial.println(counter2);
        //Check for end position
        if (Ax2toAngle(counter2) >= pos2) {
          motor2.setSpeed(0);
          state = home;
          delay(1000);
        }
        break;

      case 2:  //REFERENCE
        motor2.setSpeed(0);
        //Set reference or compare is phase!=0
        if (p_counter2 == 0 && refpos2 == true) {
          ref_counter2 = 0;
          counter2 = 0;
        } else if (p_counter2 > 0 && refpos2 == true) {
          int dif = abs(ref_counter2 - counter2);
          if (dif <= threshold) {
            //Set counter to actual position
            counter2 = Ax2toCounts(ref_offset2);
            ref2 = true;
            ref_drive2 = false;
          } else {
            ref_counter2 = counter2;
            counter2 = 0;
          }
        }
        //Switch phase
        p_counter2++;
        //switch case
        state = retrieve;
        delay(1000);
        break;

      default:
        // Statement(s)
        break;  // Wird nicht benötigt, wenn Statement(s) vorhanden sind
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
      motor3.setSpeed(0);
      Serial.println("Reached Limit Switch");
      state = reference;
      refpos3 = true;
      ref_drive3 = true;  //Indicates current reference drive
    }

    else if (pinstatusNC == LOW && pinstatusNO == HIGH && !ref_drive3) {  //Does not touch endposition
      state = home;
      refpos3 = false;
      ref_drive3 = true;  //Indicates current reference drive
    }

    else if (pinstatusNC == LOW && pinstatusNO == LOW) {
      motor3.setSpeed(0);
      Serial.print("Limit switch 3 is broken! Emergency stop!");
      while (1) {};
    }

    //---------------------Phase check-------------------------
    if (p_counter3 == 0) {
      speed3 = 30;
      pos3 = 20;
    } else if (p_counter3 == 1) {
      speed3 = 30;
      pos3 = 10;
    } else {
      speed3 = 15;
      pos3 = 5;
    }

    //--------------------State machine-----------------------
    // Case 0 ---> Home (Move to LS)
    // Case 1 ---> Retrieve (Move away from LS_
    // Case 2 ---> Reference
    switch (state) {

      case 0:  //HOME
        //Serial.println("Homing!");
        dir3 = 1;
        motor3.setSpeed(dir3 * speed3);
        if (pinstatusNC == HIGH && pinstatusNO == LOW) {  //Redundant check
          motor3.setSpeed(0);
          refpos3 = true;
          state = reference;
        }
        break;

      case 1:  //RETRIEVE
        dir3 = -1;
        motor3.setSpeed(dir3 * speed3);
        Serial.print("Counter3: ");
        Serial.println(counter3);
        //Check for end position
        if (Ax3toAngle(counter3) >= pos3) {
          motor3.setSpeed(0);
          state = home;
          delay(1000);
        }
        break;

      case 2:  //REFERENCE
        motor3.setSpeed(0);
        //Set reference or compare is phase!=0
        if (p_counter3 == 0 && refpos3 == true) {
          ref_counter3 = 0;
          counter3 = 0;
        } else if (p_counter3 > 0 && refpos3 == true) {
          int dif = abs(ref_counter3 - counter3);
          if (dif <= threshold) {
            //Set counter to actual position
            counter3 = Ax3toCounts(ref_offset3);
            ref3 = true;
            ref_drive3 = false;
          } else {
            ref_counter3 = counter3;
            counter3 = 0;
          }
        }
        //Switch phase
        p_counter3++;
        //switch case
        state = retrieve;
        delay(1000);
        break;

      default:
        // Statement(s)
        break;  // Wird nicht benötigt, wenn Statement(s) vorhanden sind
    }
  }
}

//-----------------------------------------------------------------------------------------
//------------------------------------------MAIN-------------------------------------------
//-----------------------------------------------------------------------------------------
void loop() {
  // Update target position
  //*Read data from Matlab over Serial link

  // Update control values of each motor; (Joint Space)
  for (int i = 0; i < 3; i++) {
    PIDupdate(target_pos[i], i);
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

float Ax1toAngle(int count) {
  float q;
  float trans = 20.000;
  q = (-1) * (float)count * res_avago / trans;
  return q;
}

float Ax2toAngle(int count) {
  float q;
  float trans = 13.333;
  q = (-1) * (float)count * res_avago / trans;
  return q;
}

float Ax3toAngle(int count) {
  float q;
  float trans;
  q = (float)count * res_avago / trans;
  return q;
}

int Ax1toCounts(float angle) {
  int q;
  float trans = 20.000;
  q = int((-1) * angle * trans / res_avago);
  return q;
}

int Ax2toCounts(float angle) {
  int q;
  float trans = 13.333;
  q = int((-1) * angle * trans / res_avago);
  return q;
}

int Ax3toCounts(float angle) {
  int q;
  float trans;
  q = int(angle * trans / res_avago);
  return q;
}