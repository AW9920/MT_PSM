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
#include <SPI.h>
#include <SD.h>
#include <util/atomic.h>


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

//=======================================================
//======                 Makros                   =======
//=======================================================

#define ENC1_A 2
#define ENC1_B 3

#define CS_SD 15

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

// #define MISO 50
// #define MOSI 51
// #define SCK  52
// #define SS   53



//#define PI 3.1415926535897932384626433832795

//=======================================================
//======             GLOBAL VARIABLES             =======
//=======================================================

// Timer variables for recording
unsigned long stepResponsetimer;
unsigned long rec_start_time;
unsigned long rec_time = 5000;
bool rec_flag = false;
bool startRec = false;

// Define SD card object
File dataFile = SD.open("datalog.txt", FILE_WRITE);

//State of Development
String refDevState = "auto";  //Uncomment if auto reference works
//String refDevState = "manual";

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
Servo servos[4] = { servo_roll, servo_pitch, servo_yaw1, servo_yaw2 };

//Servo variables
int servo_val1, servo_val2, servo_val3, servo_val4;
int servo_val[4] = { servo_val1, servo_val2, servo_val3, servo_val4 };
int servo_off1 = 99, servo_off2 = 89, servo_off3 = 92, servo_off4 = 115;
int servo_off[4] = { servo_off1, servo_off2, servo_off3, servo_off4 };

//Encoder constant parameters
const float res_avago = 0.36;

// Optical encoder variables
volatile long time1, temp1, counter1 = 0;
volatile long time2, temp2, counter2 = 0;
volatile long time3, temp3, counter3 = 0;
//Timing
unsigned long prevT = 0;
unsigned long currT;
double dt;

//Velocity computation
// float vel1, vel2, vel3;
// float vel[3] = { vel1, vel2, vel3 };
// long currCount1 = 0, currCount2 = 0, currCount3 = 0;
// long currCount[3] = { currCount1, currCount2, currCount3 };
// long preCount1 = 0, preCount2 = 0, preCount3 = 0;
// long preCount[3] = { preCount1, preCount2, preCount3 };

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
const int threshold = 10;

float pos1, pos2, pos3;  //In angular position of axis
float ref_offset1 = -25, ref_offset2 = -35, ref_offset3 = 0;

//Transmission ration
const float trans1 = 20.00;
const float trans2 = 13.33;

//Joint values
float q1, q2, q3, q4, q5, q6, q7;
float q[3] = { q1, q2, q3 };
//Uncommend when working with all joints
//float q[7] = { q1, q2, q3, q4, q5, q6, q7 };

//Maxon Motor variables
int speed1, speed2, speed3;
int dir1, dir2, dir3;

//PID controller variables
float kp, ki, kd;

float integral1, integral2, integral3;
float integral[3] = { integral1, integral2, integral3 };

float prev_e1, prev_e2, prev_e3;
float prev_e[3] = { prev_e1, prev_e2, prev_e3 };
float rate_e1, rate_e2, rate_e3;
float rate_e[3] = { rate_e1, rate_e2, rate_e3 };

float target_pos1, target_pos2, target_pos3;
float* target_pos[3] = { &target_pos1, &target_pos2, &target_pos3 };
float control_val1, control_val2, control_val3;
float control_values[3] = { control_val1, control_val2, control_val3 };
//Uncomment when working for all joints
//float target_pos1, target_pos2, target_pos3, target_pos4, target_pos5, target_pos6, target_pos7;
//float* target_pos[7] = { &target_pos1, &target_pos2, &target_pos3, &target_pos4, &target_pos5, &target_pos6, &target_pos7 };

//Receive Data
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];  // temporary array for use when parsing

// variables to hold the parsed data
char messageFromPC[numChars] = { 0 };
int integerFromPC = 0;
float floatFromPC = 0.0;

boolean newData = false;

//=======================================================
//======          Function Declaration            =======
//=======================================================
void recvWithStartEndMarkers(void);
void parseData(void);
void showParsedData(void);
void PIDupdate(float* target, int index, String mode);
void SerialPrintData(int type);
void setPwmFrequency(int pin, int divisor);
void InitSDcard(void);
void SaveData2SD(String data);

void setup() {
  //------------------------------Set system PSM frequency-----------------------------------
  // setPwmFrequency(4,1);
  // setPwmFrequency(5,1);
  // setPwmFrequency(6,1);

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
  //---------------------------------System Referencen---------------------------------------
  //-----------------------------------------------------------------------------------------

  if (refDevState == "auto") {  //Reference drive; Offset variables are determined automatically
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
        speed1 = 10;
        pos1 = 20;
      } else if (p_counter1 == 1) {
        speed1 = 10;
        pos1 = 10;
      } else {
        speed1 = 5;
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
      Serial.print("Limit switch 2:\t");
      Serial.print(digitalRead(LS1_NC));
      Serial.print('\t');
      Serial.println(digitalRead(LS1_NO));

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
        Serial.print(pinstatusNC);
        Serial.println(pinstatusNO);
        //while (1) {};
      }

      //---------------------Phase check-------------------------
      if (p_counter2 == 0) {
        speed2 = 10;
        pos2 = 20;
      } else if (p_counter2 == 1) {
        speed2 = 10;
        pos2 = 10;
      } else {
        speed2 = 5;
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
        speed3 = 40;
        pos3 = 20;
      } else if (p_counter3 == 1) {
        speed3 = 40;
        pos3 = 10;
      } else {
        speed3 = 35;
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
  } else if (refDevState == "manual") {  //Reference drive is skipped; Offset values are asigned manually
    Serial.println("Bring PSM manually to Limit Switches and comfirm state with INPUT: ");
    Serial.flush();
    // Wait for comfirmation
    while (Serial.available() == 0) {}
    // Define encoder variables
    counter1 = Ax1toCounts(-25.00);
    counter2 = Ax2toCounts(-35.00);
    counter3 = Ax3toCounts(0.00);
    Serial.println("Homed");
  }

  // -------------------------------------Home DC motors-------------------------------------
  target_pos1 = 0, target_pos2 = 0, target_pos3 = 80;
  // unsigned long startTime = millis();
  // while (true) {  //(millis() - startTime) < 5000
  //   for (int i = 0; i < 3; i++) {
  //     PIDupdate(target_pos[i], i);
  //   }
  //   //SerialPrintData(4);
  // }

  // -------------------------------------Home Servos----------------------------------------
  for (int i = 0; i < 4; i++) {
    servos[i].write(servo_off[i]);
    servo_val[i] = servo_off[i];
  }

  //while (true) {
  //SerialPrintData(2);
  //};
  while (Serial.available() > 0) {
    Serial.flush();
  }

  stepResponsetimer = millis();
}

//-----------------------------------------------------------------------------------------
//------------------------------------------MAIN-------------------------------------------
//-----------------------------------------------------------------------------------------
void loop() {
  //Create empty data string
  String dataString = "";
  int val;
  int index;

  //-------------------Compute velocity----------------



  //-------------Stepresponse eval (Evaluation)-------------
  if ((millis() - stepResponsetimer) >= 5000 && (millis() - stepResponsetimer) < 10000) {
    startRec = true;
    *target_pos[0] = 0, *target_pos[1] = 10, *target_pos[2] = 50;
    //Serial.println('Stage 1');
  } else if ((millis() - stepResponsetimer) >= 10000 && (millis() - stepResponsetimer) < 15000) {
    *target_pos[0] = 10, *target_pos[1] = 0, *target_pos[2] = 0;
    //Serial.println('Stage 2');
  } else if ((millis() - stepResponsetimer) >= 15000 && (millis() - stepResponsetimer) < 20000) {
    *target_pos[0] = -10, *target_pos[1] = -5, *target_pos[2] = 40;
    //Serial.println('Stage 3');
  } else {
    *target_pos[0] = 0, *target_pos[1] = 0, *target_pos[2] = 0;
    //Serial.println('Stage 4');
  }

  // Check for collision
  // if (digitalRead(LS1_NC) == HIGH && digitalRead(LS1_NO) == LOW) {
  //   motor[0].setSpeed(0);
  //   motor[1].setSpeed(0);
  //   motor[2].setSpeed(0);
  //   while (true) {};
  // }
  // if (digitalRead(LS2_NC) == HIGH && digitalRead(LS2_NO) == LOW) {
  //   motor[0].setSpeed(0);
  //   motor[1].setSpeed(0);
  //   motor[2].setSpeed(0);
  //   while (true) {};
  // }
  // if (digitalRead(LS3_NC) == HIGH && digitalRead(LS3_NO) == LOW) {
  //   motor[0].setSpeed(0);
  //   motor[1].setSpeed(0);
  //   motor[2].setSpeed(0);
  //   while (true) {};
  // }

  // Extract data from string and update target position
  recvWithStartEndMarkers();
  if (newData == true) {
    strcpy(tempChars, receivedChars);
    // this temporary copy is necessary to protect the original data
    //   because strtok() used in parseData() replaces the commas with \0
    parseData();
    showParsedData();
    newData = false;
    startRec = true;
  }

  // Limit target values
  target_pos1 = constrain(target_pos1, -20, 20);
  target_pos2 = constrain(target_pos2, -20, 20);
  target_pos3 = constrain(target_pos3, 0, 140);
  // target_pos4 = constrain(target_pos4, -25, 25);
  // target_pos5 = constrain(target_pos5, -25, 25);
  // target_pos6 = constrain(target_pos6, -25, 25);
  // target_pos7 = constrain(target_pos7, -25, 25);

  // Update control values of each motor; (Joint Space)
  for (int i = 0; i < 3; i++) {
    PIDupdate(target_pos[i], i, "PID");
  }

  // Update servo motors
  for (int i = 0; i < 4; i++) {
    index = i + 2;
    val = target_pos[index];
    servos[i].write(val);
  }

  //Store Data to SD card
  if ((millis() - rec_start_time) <= rec_time && rec_flag) {
    //Save current and target value to SD card
    SaveData2SD(dataString);
  } else if ((millis() - rec_start_time) >= rec_time && rec_flag) {
    dataFile.close();
    rec_flag = false;
    rec_start_time = millis();
  } else {
    rec_start_time = millis();
  }


  // Debugging
  SerialPrintData(6);
}


//-----------------------------------------------------------------------------------------
//-------------------------------------FUNCTIONS-------------------------------------------
//-----------------------------------------------------------------------------------------
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
  //float trans = 0.637;
  float D = 19.10;
  float pi = 3.1416;
  float ref = 360;
  q = (-1) * pi * D * (float)count * res_avago / ref;
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

int Ax3toCounts(float pos) {
  int q;
  //float trans = 0.637;
  float D = 19.10;
  float ref = 360;
  q = int((pos * ref) / (PI * D * res_avago * (-1)));
  return q;
}

void readJointValues() {
  Serial.flush();
  while (Serial.available() == 0) {}
  String t = Serial.readString();
  t.trim();

  //Fill array
}

void getDT(void) {
  currT = micros();
  dt = (double)(currT - prevT) / 1.0e6;
  prevT = currT;
}

// ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
//   currCount[0] = counter1;
//   currCount[1] = counter2;
//   currCount[2] = counter3;
// }

// // for (int i = 0; i < sizeof(vel) / sizeof(vel[0]); i++) {
// //   vel[i] = (currCount[i] - preCount[i])/deltaT;
// //   preCount[i] = currCount[i];
// // }