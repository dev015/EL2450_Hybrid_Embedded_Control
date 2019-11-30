#include <MotorWheel.h>
#include <R2WD.h>
#include <PID_Beta6.h>
#include <PinChangeInt.h>
#include <PinChangeIntConfig.h>
#define _USE_MATH_DEFINES
#include <math.h>


#define PI 3.14159265

enum buffer_state {
  BUFFER_EMPTY,
  STATE_QUERY,
  MANUAL_CONTROL,
  POSE_DATA,
  START_GOAL,
  BUFFER_INVALID
};

const int buffer_size = 100;
const int manual_size = 16;
const int pose_size = 21;
char buffer[buffer_size];
char manual_buffer[manual_size];
char pose_buffer[pose_size];
char junk[buffer_size];

const int maxc = 500;
const int minc = -500;

/* Motor setup */
irqISR(irq1,isr1); // Intterrupt function.on the basis of the pulse, work for wheel1
MotorWheel lwheel(9,8,4,5,&irq1,REDUCTION_RATIO,int(144*PI));
irqISR(irq2,isr2);
MotorWheel rwheel(10,11,6,7,&irq2,REDUCTION_RATIO,int(144*PI));
R2WD drivetrain(&lwheel,&rwheel,WHEELSPAN);

/* Loop variables */
int left = 0;
int right = 0;
boolean ldir = DIR_ADVANCE;
boolean rdir = DIR_BACKOFF;
int x0 = 0; // cm
int y0 = 0; // cm
int x = 0; // cm
int y = 0; // cm
int theta = 0; // degree
int xg = 0; // cm
int yg = 0; // cm
/*===================*/
/* define your own variables here(copy the content of OwnVariables.c here) */
const float kPsi = 5;
const float kPsi_new = 10;
const float kW = 10;
const float kW_goal = 5;
const float pi = 3.1415;
const float p = 15; //15
float W = 0;
float V = 0;
float errorTheta = 0;
float errorTheta1 = 0;
float errorVel = 0;
float errorX0 = 0;
float errorY0 = 0;
float errorXi = 0;
float errorYi = 0;
float errorX = 0;
float errorY = 0;
float thetaGoal = 0;
float thetaGoal_new = 0;
float distSq = 0;
float errorV0 = 0;
float errorV = 0;
int ctrlCase = 0;

/*===================*/



boolean is_manual_control() {
  return (strncmp(buffer, "manual", 6) == 0);
}

boolean is_state_query() {
  return (strncmp(buffer, "state?", 6) == 0);
}

boolean is_pose_data(){
  return (strncmp(buffer, "pose", 4) == 0);
}

boolean is_start_goal(){
  return (strncmp(buffer, "startgoal", 9) == 0);
}

int read_buffer() {
  Serial.readBytesUntil(':', junk, buffer_size);
  int bytes_read = Serial.readBytesUntil(';', buffer, buffer_size);
  if (bytes_read == 0) {
    // zero bytes were read
    return BUFFER_EMPTY;
  } else if (is_manual_control()) {
    return MANUAL_CONTROL;
  } else if (is_state_query()) {
    return STATE_QUERY;
  } else if (is_pose_data()) {
    return POSE_DATA;
  } else if (is_start_goal()) {
    return START_GOAL;
  }
  else {
    return BUFFER_INVALID;
  }
}

void send_message() {
  Serial.write(130);
}

void send_done() {
  Serial.write(131);
}

void setup() {
  //TCCR0B=TCCR0B&0xf8|0x01;
  TCCR1B=TCCR1B&0xf8|0x01;    // Pin9,Pin10 PWM 31250Hz
  //TCCR2B=TCCR2B&0xf8|0x01;  // Pin3,Pin11 PWM 31250Hz
  Serial.begin(9600);
  drivetrain.PIDEnable(0.26,0.02,0,10);
  Serial.print("ready");
  send_message();
}

/* the main loop */
void loop() {
  while (Serial.available()) {
    switch (read_buffer()) {
      case BUFFER_EMPTY:
        break;
      case MANUAL_CONTROL:
        memcpy(manual_buffer, buffer, manual_size);
        sscanf(manual_buffer, "manual %d %d", &left, &right);
        Serial.print("setting manual ");
        Serial.print(left);
        Serial.print(" ");
        Serial.print(right);
        send_message();
        break;
      case POSE_DATA:
        memcpy(pose_buffer, buffer, pose_size);
        sscanf(pose_buffer, "pose %d %d %d", &x, &y, &theta);
        Serial.print("Pose x=");
        Serial.print(x, DEC);
        Serial.print(" y=");
        Serial.print(y, DEC);
        Serial.print(" theta=");
        Serial.print(theta, DEC);
        Serial.print("\n");
        send_message();
        /*=========================*/
        /*put your controller here(copy the content of Controller.c here)*/
        /*
        Case 1: Rotational Controller; V = 0;
        Case 2: V = Place Hold Condition; Rotation Control;
        Case 3: Velocity Controller; W = 0; 
        Case 4: Hybrid Controller;
        */
        ctrlCase = 4;
        errorXi = xg - x0;
        errorYi = yg - y0;
        switch (ctrlCase) {
          case 1:
            thetaGoal = atan2(errorYi,errorXi) * (180/pi);
            errorTheta = thetaGoal - theta;
            W = kPsi*errorTheta;
            V = 0;
            break;
          case 2:
            thetaGoal = atan2(errorYi,errorXi) * (180/pi);
            errorTheta = thetaGoal - theta;
            W = kPsi*errorTheta;
            errorX0 = x0 - x;
            errorY0 = y0 - y;
            errorV0 = (cos(theta*pi/180)*errorX0) + (sin(theta*pi/180)*errorY0);
            V = kW * errorV0;
            break;
          case 3:
            W = 0;
            errorX = xg - x;
            errorY = yg - y;
            errorV = (cos(theta*pi/180)*errorX) + (sin(theta*pi/180)*errorY);
            V = kW * errorV;
            break;
          case 4:
            errorX = xg - x;
            errorY = yg - y;
            thetaGoal = atan2(errorY,errorX) * (180/pi);
            errorTheta = thetaGoal - theta;
            errorTheta1 = ((x + (p*cos(theta*pi/180)) - x0)*sin(thetaGoal*pi/180)) - ((y + (p*sin(theta*pi/180)) - y0)*cos(thetaGoal*pi/180));
            errorV = (cos(theta*pi/180)*errorX) + (sin(theta*pi/180)*errorY);
            if(errorTheta >= 0.5 || errorTheta <= -0.5) {
              W = kPsi*errorTheta;
              V = 0;
              Serial.print("Loop 1 is executing");
            }
            else if(abs(errorX) > 0.5 && abs(errorY) > 0.5){
              V = kW * errorV;
              W = 0;
              /*W = kPsi_new*errorTheta1;*/
              Serial.print("Loop 2 is executing");
            }
            else {
              V = kW_goal * errorV;
              W = 0;
              Serial.print("Loop 3 is executing");
              if (errorX == 0 && errorY == 0) {
                send_done();
              }
            }
            break;
        }
        right = ((2*V) + W) / 2;
        left = ((2*V) - W) / 2;


        /*=========================*/
        break;
      case START_GOAL:
        sscanf(buffer, "startgoal %d %d %d %d", &x0, &y0, &xg, &yg);
        Serial.print("Startgoal x0=");
        Serial.print(x0, DEC);
        Serial.print(" y0=");
        Serial.print(y0, DEC);
        Serial.print(" xg=");
        Serial.print(xg, DEC);
        Serial.print(" yg=");
        Serial.print(yg, DEC);
        Serial.print("\n");
        send_message();
        /*====================*/
        /* Renew the discrete control state upon receiving new goals(copy the content of RenewControllerState.c here)*/

        /*====================*/
        break;
      case STATE_QUERY:
        Serial.print("running\n");
        send_message();
        break;
      case BUFFER_INVALID:
        left = 0;
        right = 0;
        Serial.print("got an invalid command\n");
        send_message();
        break;
    }
    // limit the control input (again in case the students did not do it)
    if (left >= maxc)
        left= maxc;
    if (left <= minc)
        left= minc;
    if (right >= maxc)
        right= maxc;
    if (right <= minc)
        right= minc;

    if (left <= 0) {
      ldir = DIR_BACKOFF;
    } else {
      ldir = DIR_ADVANCE;
    }
    if (right <= 0) {
      rdir = DIR_ADVANCE;
    } else {
      rdir = DIR_BACKOFF;
    }
    drivetrain.wheelLeftSetSpeedMMPS(abs(left), ldir);
    drivetrain.wheelRightSetSpeedMMPS(abs(right), rdir);
  }
  drivetrain.PIDRegulate();
}
