//=======================================================
//======                 Makros                   =======
//=======================================================
//Correct clock
//#define CORRECT_CLOCK 8     //Required for Arduino Mega
//#define micros() (micros() / CORRECT_CLOCK)
//#define millis() (millis() / CORRECT_CLOCK)

//Define Pins
#define ENC1_A 0
#define ENC1_B 1
#define ENC2_A 2
#define ENC2_B 3
#define ENC3_A 4
#define ENC3_B 5

#define DC1_PWM 6
#define DC2_PWM 7
#define DC3_PWM 8

#define DC1_DIR 24
#define DC2_DIR 25
#define DC3_DIR 26

#define SERVO1 9
#define SERVO2 10
#define SERVO3 11
#define SERVO4 12

#define LS1_NC 22
#define LS1_NO 23
#define LS2_NC 20
#define LS2_NO 21
#define LS3_NC 18
#define LS3_NO 19


// Math constants
#define PI 3.1415926535897932384626433832795

#define PSM_L
//#define PSM_R

//=======================================================
//======            Include libraries             =======
//=======================================================

//#include <avr/wdt.h>  //Watchdog Timer Library
//#include <Arduino.h>
#include "I2Cdev.h"
#include <stdfix.h>
#include <math.h>
#include "CytronMotorDriver.h"
#include <Servo.h>
#include <SPI.h>
#include <SD.h>
#include <util/atomic.h>
#include <Encoder.h>
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <SPI.h>
#include <SD.h>
#include <SoftwareSerial.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

//=======================================================
//======             GLOBAL VARIABLES             =======
//=======================================================

//----------Timer variables for recording
unsigned long stepResponsetimer;
unsigned long rec_start_time;
unsigned long rec_time = 5000;
bool rec_flag = false;
bool startRec = false;

//----------Define SD card object
File dataFile;

//----------State of Development
String refDevState = "auto";  //Uncomment if auto reference works
//String refDevState = "manual";

//----------Configure the motor driver.
CytronMD motor1(PWM_DIR, DC1_PWM, DC1_DIR);  // PWM 1 = Pin 4, DIR 1 = Pin 7.
CytronMD motor2(PWM_DIR, DC2_PWM, DC2_DIR);  // PWM 2 = Pin 5, DIR 2 = Pin 8.
CytronMD motor3(PWM_DIR, DC3_PWM, DC3_DIR);  // PWM 2 = Pin 6, DIR 2 = Pin 9.
CytronMD motor[3] = { motor1, motor2, motor3 };

//----------Encoder objects
Encoder Enc1(ENC1_B, ENC1_A);
Encoder Enc2(ENC2_B, ENC2_A);
Encoder Enc3(ENC3_B, ENC3_A);

//----------Configure Servo objects
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servos[4] = { servo1, servo2, servo3, servo4 };

//----------Servo variables
#ifdef PSM_R
int servo_val1, servo_val2, servo_val3, servo_val4;
int servo_val[4] = { servo_val1, servo_val2, servo_val3, servo_val4 };
int servo_off1 = 115, servo_off2 = 92, servo_off3 = 80, servo_off4 = 105;
int servo_off[4] = { servo_off1, servo_off2, servo_off3, servo_off4 };
#endif

#ifdef PSM_L
int servo_val1, servo_val2, servo_val3, servo_val4;
int servo_val[4] = { servo_val1, servo_val2, servo_val3, servo_val4 };
int servo_off1 = 100, servo_off2 = 97, servo_off3 = 90, servo_off4 = 92;
int servo_off[4] = { servo_off1, servo_off2, servo_off3, servo_off4 };
#endif

//----------Optical encoder variables
long raw_pos1, raw_pos2, raw_pos3;
long pos[3] = { raw_pos1, raw_pos2, raw_pos3 };
const float res_avago = 0.18;
//----------Sampling Time variables
float prevT = 0;
float currT;
float dt;

//----------State machine variables
int state;
int home = 0;
int retrieve = 1;
int reference = 2;
int move_zero = 3;
int dmg_cnt;

bool ref1, ref2, ref3;
bool ref[3] = { ref1, ref2, ref3 };
bool ref_drive1, ref_drive2, ref_drive3;
bool refpos1, refpos2, refpos3;

int p_counter1 = 0, p_counter2 = 0, p_counter3 = 0;
long ref_counter1 = 0, ref_counter2 = 0, ref_counter3 = 0;
const int threshold = 10;

float pos1, pos2, pos3;  //In angular position of axis
float ref_offset1 = -25, ref_offset2 = -35, ref_offset3 = 0;

//----------Transmission ration
const float trans1 = 20.00;
const float trans2 = 13.33;

//-----Enc variables
float min_res1 = 0.01, min_res2 = 0.02, min_res3 = 0.03;
float min_res[3] = { min_res1, min_res2, min_res3 };

//----------Joint values
double q1, q2, q3, q4, q5, q6, q7;
double q[3] = { q1, q2, q3 };
//----------Corrdinates
double x, y, z;
float x_m, y_m, z_m;

//----------Geometric Variables
double L_RCM = 455.99;
double L_tool = 432.5;
double d0 = -L_RCM + L_tool;

//----------Maxon Motor variables
int speed1, speed2, speed3;
int m_speed[3] = { speed1, speed2, speed3 };
int dir1, dir2, dir3;
int dir[3] = { dir1, dir2, dir3 };

//----------PID controller variables
float kp, ki, kd;
String c_mode;

float integral1, integral2, integral3;
float integral[3] = { integral1, integral2, integral3 };

float prev_e1 = 0, prev_e2 = 0, prev_e3 = 0;  //Last Error
float prev_e[3] = { prev_e1, prev_e2, prev_e3 };
float rate_e1, rate_e2, rate_e3;  //Current derivative
float rate_e[3] = { rate_e1, rate_e2, rate_e3 };
float prev_rate_e1 = 0, prev_rate_e2 = 0, prev_rate_e3 = 0;  //Last derivative
float prev_rate_e[3] = { prev_rate_e1, prev_rate_e2, prev_rate_e3 };

float target_pos1, target_pos2, target_pos3, target_pos4, target_pos5, target_pos6, target_pos7;
float* target_pos[7] = { &target_pos1, &target_pos2, &target_pos3, &target_pos4, &target_pos5, &target_pos6, &target_pos7 };
float control_val1, control_val2, control_val3;
float control_values[3] = { control_val1, control_val2, control_val3 };
float sat_control_val1, sat_control_val2, sat_control_val3;
float sat_control_values[3] = { sat_control_val1, sat_control_val2, sat_control_val3 };

bool clamp_I1 = false, clamp_I2 = false, clamp_I3 = false;
bool clamp_I[3] = { clamp_I1, clamp_I2, clamp_I3 };


//----------Receive Data
const byte numChars = 81;  //32
char receivedChars[numChars];
char tempChars[numChars];  // temporary array for use when parsing
//variables to temp hold the parsed data
char messageFromPC[numChars] = { 0 };
int integerFromPC = 0;
float floatFromPC = 0.0;
boolean newData = false;

//----------Lp filter variables
float dt_xn1;
float dt_yn1;
bool first = true;

float pre_rate_filter_e1 = 0, pre_rate_filter_e2 = 0, pre_rate_filter_e3 = 0;
float pre_rate_filter[3] = { pre_rate_filter_e1, pre_rate_filter_e2, pre_rate_filter_e3 };
float pre_filter_rate_1 = 0, pre_filter_rate_2 = 0, pre_filter_rate_3 = 0;
float pre_filter_rate[3] = { pre_filter_rate_1, pre_filter_rate_2, pre_filter_rate_3 };

//----------Position sampler
bool first_sample = true;
unsigned long sampler_start_time;
bool sampler_setup_flag;

//----------System setup
bool sys_ready = false;

//----------Read data from SD
// line buffer for 80 characters
char line[81];
bool setupFlag = true;
long SD_sampT = 6;
long SD_Timer;

//----------Latency----------------
unsigned long receive_time;
unsigned long latency;
unsigned long send_time_received;

//=======================================================
//======          Function Declaration            =======
//=======================================================
void recvWithStartEndMarkers(void);
void parseData(void);
void showParsedData(void);
void PIDupdate(float* target, int index, String mode);
void SerialPrintData(int type);
void SaveData2SD(String data);
void CheckCol(void);
void PosSampler(String mode);
void sendData(void);
void setPWMfrequency(void);

void setup() {
  //------------------------------Set system PSM frequency-----------------------------------
  setPWMfrequency();
  //-----------Set initial conditions----------------
  motor1.setSpeed(0);
  motor2.setSpeed(0);
  motor3.setSpeed(0);

  ref1 = false, ref2 = false, ref3 = false;
  ref_drive1 = false, ref_drive2 = false, ref_drive3 = false;

  //Attach pins to objects
  servos[0].attach(SERVO1);
  servos[1].attach(SERVO2);
  servos[2].attach(SERVO3);
  servos[3].attach(SERVO4);

  //-----------Start Communications-------------
  Serial.begin(115200);
  while (!Serial) {};
  Serial3.begin(115200);
  Serial.println("initialization done.");

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

  //-----------------------------------------------------------------------------------------
  //---------------------------------System Referencen---------------------------------------
  //-----------------------------------------------------------------------------------------
  // -------------------------------------Home Servos----------------------------------------
  for (int i = 0; i < 4; i++) {
    servos[i].write(servo_off[i]);
    Serial.print(servo_off[i]);
    Serial.print('\t');
  }
  // -----------------------------Home Joint 1, 2 and 3--------------------------------------
  Enc1.write(0);
  Enc2.write(0);
  Enc3.write(0);
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

      //--------------------Check status-------------------------
      if (pinstatusNC == HIGH && pinstatusNO == LOW) {  //Touches Endposition
        motor1.setSpeed(0);
        Serial.println("Reached Limit Switch");
        state = reference;
        refpos1 = true;
        ref_drive1 = true;  //Indicates current reference drive
        dmg_cnt = 0;
      }

      else if (pinstatusNC == LOW && pinstatusNO == HIGH && !ref_drive1) {  //Does not touch endposition
        Serial.println("Initial sate: Not at limit switch");
        state = home;
        refpos1 = false;
        ref_drive1 = true;  //Indicates current reference drive
        dmg_cnt = 0;
      }

      else if ((pinstatusNC == LOW && pinstatusNO == LOW)) {
        dmg_cnt++;
        if (dmg_cnt >= 10) {
          motor1.setSpeed(0);
          Serial.print("Limit switch 1 is broken! Emergency stop!");
          ref_drive1 = false;  //Indicates current reference drive
          while (1) {};
        }
      }

      //---------------------Phase check-------------------------
      if (p_counter1 == 0) {
        speed1 = 25;
        pos1 = 15;
      } else if (p_counter1 == 1) {
        speed1 = 25;
        pos1 = 10;
      } else {
        speed1 = 5;
        pos1 = -5;
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
          //Check for end position
          if (Ax1toAngle(Enc1.read()) >= pos1) {
            motor1.setSpeed(0);
            state = home;
            fixDelay(1000);
          }
          break;

        case 2:  //REFERENCE
          motor1.setSpeed(0);
          //Set reference or compare is phase!=0
          if (p_counter1 == 0 && refpos1 == true) {
            ref_counter1 = 0;
            Enc1.write(0);
            //counter1 = 0;
          } else if (p_counter1 > 0 && refpos1 == true) {
            int dif = abs(ref_counter1 - Enc1.read());
            if (dif <= threshold) {
              //Set counter to actual position
              Enc1.write(Ax1toCounts(ref_offset1));
              ref1 = true;
              ref_drive1 = false;
              Serial.print("Home! Position:");
              Serial.println(Enc1.read());
            } else {
              ref_counter1 = Enc1.read();
              Enc1.write(0);
            }
          }
          //Switch phase
          p_counter1++;
          //switch case
          state = retrieve;
          //Serial.println("1 second delay begin");
          fixDelay(1000);
          //Serial.println("1 second delay end");
          break;
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
        //Serial.println("Reached Limit Switch");
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
      }

      //---------------------Phase check-------------------------
      if (p_counter2 == 0) {
        speed2 = 20;
        pos2 = -20;
      } else if (p_counter2 == 1) {
        speed2 = 20;
        pos2 = -10;
      } else {
        speed2 = 5;
        pos2 = -5;
      }

      //--------------------State machine-----------------------
      // This statemachine references the incremental encoder. A propper flow chart can be found in the Master Thesis in Appendix
      // Case 0 ---> Home (Move to LS)
      // Case 1 ---> Retrieve (Move away from LS_
      // Case 2 ---> Reference
      switch (state) {

        case 0:  //HOME
          //Serial.println("Homing!");
          dir2 = -1;
          motor2.setSpeed(dir2 * speed2);
          if (pinstatusNC == HIGH && pinstatusNO == LOW) {  //Redundant check
            motor2.setSpeed(0);
            refpos2 = true;
            state = reference;
          }
          break;

        case 1:  //RETRIEVE
          dir2 = 1;
          motor2.setSpeed(dir2 * speed2);
          //Check for end position
          if (Ax2toAngle(Enc2.read()) >= pos2) {
            motor2.setSpeed(0);
            state = home;
          }
          break;

        case 2:  //REFERENCE
          motor2.setSpeed(0);
          //Set reference or compare is phase!=0
          if (p_counter2 == 0 && refpos2 == true) {
            ref_counter2 = 0;
            Enc2.write(0);
          } else if (p_counter2 > 0 && refpos2 == true) {
            int dif = abs(ref_counter2 - Enc2.read());
            if (dif <= threshold) {
              //Set counter to actual position
              Enc2.write(Ax2toCounts(ref_offset2));
              ref2 = true;
              ref_drive2 = false;
              Serial.print("Homed Axis2!");
            } else {
              ref_counter2 = Enc2.read();
              Enc2.write(0);
            }
          }
          //Switch phase
          p_counter2++;
          //switch case
          state = retrieve;
          fixDelay(1000);
          break;
      }
    }

    //-----------------------------------------------------------------------------------------
    //------------------------------Referenzfahrt Motor3---------------------------------------
    //-----------------------------------------------------------------------------------------
    Serial.println("Start Homing: Axis 3!");
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
        speed3 = 200;
        pos3 = 20;
      } else if (p_counter3 == 1) {
        speed3 = 200;
        pos3 = 10;
      } else {
        speed3 = 180;
        pos3 = 5;
      }

      //--------------------State machine-----------------------
      // Case 0 ---> Home (Move to LS)
      // Case 1 ---> Retrieve (Move away from LS_
      // Case 2 ---> Reference
      switch (state) {

        case 0:  //HOME
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
          //Check for end position
          if (Ax3toAngle(Enc3.read()) >= pos3) {
            motor3.setSpeed(0);
            state = home;
            fixDelay(1000);
          }
          break;

        case 2:  //REFERENCE
          motor3.setSpeed(0);
          //Set reference or compare is phase!=0
          if (p_counter3 == 0 && refpos3 == true) {
            ref_counter3 = 0;
            Enc3.write(0);
          } else if (p_counter3 > 0 && refpos3 == true) {
            int dif = abs(ref_counter3 - Enc3.read());
            if (dif <= threshold) {
              //Set counter to actual position
              Enc3.write(Ax3toCounts(ref_offset3));
              ref3 = true;
              ref_drive3 = false;
            } else {
              ref_counter3 = Enc3.read();
              Enc3.write(0);
            }
          }
          //Switch phase
          p_counter3++;
          //switch case
          state = retrieve;
          fixDelay(1000);
          break;
      }
    }
  }
  //-----------------------------------------------------------------------------------------
  //-----------------------------------Manual Homing-----------------------------------------
  //-----------------------------------------------------------------------------------------
  //This routine is called if the Homing mode is set to "manuel". I this case just bring the axis 1 - 3 to their zero pos and confirm
  //by sending a char over Serial Port
  else if (refDevState == "manual") {  //Reference drive is skipped; Offset values are asigned manually
    Serial.println("Bring PSM manually to Limit Switches and comfirm state with INPUT: ");
    Serial.flush();
    // Wait for comfirmation
    while (Serial.available() == 0) {}
    // Define encoder variables
    Enc1.write(Ax1toCounts(-25.00));
    Enc2.write(Ax2toCounts(-35.00));
    Enc3.write(Ax3toCounts(0.00));
    delay(1000);
  }

  Catch_Tool();

  RampUp_Homing();

  // ---------------------------------End Setup Routine------------------------------------
  // Tell computer system is ready
  sys_ready = true;
  //c_mode = "PID";
  //fixDelay(200);

  // Start timer
  stepResponsetimer = millis();
}

//-----------------------------------------------------------------------------------------
//------------------------------------------MAIN-------------------------------------------
//-----------------------------------------------------------------------------------------
void loop() {
  if (sys_ready) {

    // Update control values of each motor; (Joint Space)
    //LPFilter_DT();

    //Extract data from string and update target position
    recvWithStartEndMarkers();
    while (newData == true) {
      receive_time = millis();
      strcpy(tempChars, receivedChars);
      // this temporary copy is necessary to protect the original data
      //   because strtok() used in parseData() replaces the commas with \0
      parseData();
      latency = receive_time - send_time_received;
      newData = false;
    }
    //showParsedData();

    // Limit target values
    target_pos1 = constrain(target_pos1, -15, 15);
    target_pos2 = constrain(target_pos2, -15, 15);
    target_pos3 = constrain(target_pos3, 40, 120);

    //Compute joint values of Endo-Wrist
    servo_val[0] = -target_pos4 / 1.563 + (servo_off[0] - 90);
    servo_val[1] = target_pos5 / 1.019 + (servo_off[1] - 90);
    servo_val[2] = (2 * target_pos6 - target_pos7 + 1.662 * servo_val[1]) / 2.436 + (servo_off[2] - 90);
    servo_val[3] = (target_pos7 / 1.218) + servo_val[2] + (servo_off[3] - 90);
    servo_val[0] = map(servo_val[0], -90, 90, 0, 180);
    servo_val[1] = map(servo_val[1], -90, 90, 0, 180);
    servo_val[2] = map(servo_val[2], -90, 90, 0, 180);
    servo_val[3] = map(servo_val[3], -90, 90, 0, 180);
    servo_val[0] = constrain(servo_val[0], 0, 180);
    servo_val[1] = constrain(servo_val[1], 0, 180);
    servo_val[2] = constrain(servo_val[2], 0, 180);
    servo_val[3] = constrain(servo_val[3], 0, 180);

    // Update desired position for joints 1, 2 and 3
    getDT();
    for (int i = 0; i < 3; i++) {
      PIDupdate(target_pos[i], i, c_mode);
    }

    //Update servo motors
    for (int i = 0; i < 4; i++) {
      int val = servo_val[i];
      servos[i].write(val);
    }

    //Compute x,y,z position (forward kinematics)
    q[0] = Ax1toAngle(Enc1.read()) * PI / 180;  //convert to rad
    q[1] = Ax2toAngle(Enc2.read()) * PI / 180;  //convert to rad
    q[2] = Ax3toAngle(Enc3.read());
    x = -cos(q[1] - PI / 2) * (q[2] + d0);
    y = cos(q[0]) * sin(q[1] - PI / 2) * (q[2] + d0);
    z = -sin(q[0]) * sin(q[1] - PI / 2) * (q[2] + d0);

    // Debugging
    SerialPrintData(14);
  }
}

//-----------------------------------------------------------------------------------------
//-------------------------------------FUNCTIONS-------------------------------------------
//-----------------------------------------------------------------------------------------
String compData(double in_value1, double in_value2, double in_value3, byte signi) {
  char buffer[20];
  String serialData;

  dtostrf(in_value1, 0, 2, buffer);
  serialData = "<";
  serialData += buffer;
  serialData += ",";
  dtostrf(in_value2, 0, 2, buffer);
  serialData += buffer;
  serialData += ",";
  dtostrf(in_value3, 0, 2, buffer);
  serialData += buffer;
  serialData += ">";

  return serialData;
}

void fixDelay(uint32_t ms) {
  delay(ms);
}

void getDT(void) {
  //Only call once in routine
  currT = micros();
  dt = (currT - prevT) / 1.0e6;
  prevT = currT;
}

void sendData(void) {
  Serial.print(*target_pos[0], 4);
  Serial.print('/');
  Serial.print(*target_pos[1], 4);
  Serial.print('/');
  Serial.print(*target_pos[2], 4);
  Serial.print('/');
  Serial.print(Ax1toAngle(Enc1.read()), 4);
  Serial.print('/');
  Serial.print(Ax2toAngle(Enc2.read()), 4);
  Serial.print('/');
  Serial.print(Ax3toAngle(Enc3.read()), 4);
  Serial.print('/');
  Serial.print(prev_e[0], 4);
  Serial.print('/');
  Serial.print(prev_e[1], 4);
  Serial.print('/');
  Serial.print(prev_e[2], 4);
  Serial.print('/');
  Serial.print(m_speed[0]);
  Serial.print('/');
  Serial.print(m_speed[1]);
  Serial.print('/');
  Serial.println(m_speed[2]);
}

float Ax1toAngle(long count) {
  float q;
  float trans = 20.000;
  q = (-1) * (float)count * res_avago / trans;
  return q;
}

float Ax2toAngle(long count) {
  float q;
  float trans = 13.333;
  q = (-1) * (float)count * res_avago / trans;
  return q;
}

float Ax3toAngle(long count) {
  float q;
  //float trans = 0.637;
  float D = 19.10;
  float pi = 3.1416;
  float ref = 360;
  q = (1) * pi * D * (float)count * res_avago / ref;
  return q;
}

int Ax1toCounts(float angle) {
  int q;
  float trans = 20.000;
  q = int(-1.0 * angle * trans / res_avago);
  return q;
}

int Ax2toCounts(float angle) {
  int q;
  float trans = 13.333;
  q = int(-1.0 * angle * trans / res_avago);
  return q;
}

int Ax3toCounts(float pos) {
  int q;
  //float trans = 0.637;
  float D = 19.10;
  float ref = 360;
  q = int(1.0 * (pos * ref) / (PI * D * res_avago));
  return q;
}

void readJointValues() {
  Serial.flush();
  while (Serial.available() == 0) {}
  String t = Serial.readString();
  t.trim();

  //Fill array
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