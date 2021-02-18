#include <FastAccelStepper.h>
//Stepper speed is measured as steps per second
//Normally a stepper has 200 steps per revolution

const int STEP_A_PIN = 10;
const int STEP_B_PIN = 9; 
const int DIR_A_PIN  = 12; 
const int DIR_B_PIN  = 11; 
const int STEP_EN    = 13;

FastAccelStepperEngine engine = FastAccelStepperEngine();
FastAccelStepper *stepperA = NULL;
FastAccelStepper *stepperB = NULL;

char  received_arr[19];
char  left_received_arr[8];
char  right_received_arr[8];

float    motors_vel_float[2]  = {0.0, 0.0};
unsigned int   motors_vel[2]        = {0, 0}; // ticks per microseconds
bool     motors_fw[2]         = {true, true};
bool     new_value            = false;

const byte numChars = 19;
char receivedChars[numChars];

boolean newData = false;

//----------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  Serial.println("DI");

  engine.init();
   
  stepperA = engine.stepperConnectToPin(STEP_A_PIN);
  stepperB = engine.stepperConnectToPin(STEP_B_PIN);

  stepperA->setDirectionPin(DIR_A_PIN);
  stepperA->setEnablePin(STEP_EN);
  stepperA->setAutoEnable(true);
  stepperA->setAcceleration(7500);

  stepperB->setDirectionPin(DIR_B_PIN);
  stepperB->setEnablePin(STEP_EN);
  stepperB->setAutoEnable(true);
  stepperB->setAcceleration(7500);
}

//----------------------------------------------------------------------

float i = 0;
bool decreasing = false;

void loop() {
  recvWithStartEndMarkers();
  if (newData == true) {
    if (abs(motors_vel[0]) < 20) {
      stepperA->stopMove();
    }
    else {
      stepperA->setSpeedInHz(motors_vel[0]);
      if (motors_fw[0] == true) {
        stepperA->runForward();
      }
      else {
        stepperA->runBackward();
      }
    }
    newData = false;
    
    if (abs(motors_vel[1]) < 20) {
      stepperB->stopMove();
    }
    else {
      stepperB->setSpeedInHz(motors_vel[1]);
      if (motors_fw[1] == true) {
        stepperB->runForward();
      }
      else {
        stepperB->runBackward();
      }
    }
    newData = false;
  }
}

//----------------------------------------------------------------------

void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;

  // if (Serial.available() > 0) {
  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      }
      else {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;

        memcpy(left_received_arr,  receivedChars,     8);
        memcpy(right_received_arr, receivedChars + 9, 8);
        
        motors_vel_float[0] = atof(left_received_arr);
        motors_vel[0] = abs(motors_vel_float[0]);
        if (motors_vel_float[0] > -0.001) {
          motors_fw[0]  = true;
        }
        else {
          motors_fw[0]  = false;
        }

        motors_vel_float[1] = atof(right_received_arr);
        motors_vel[1] = abs(motors_vel_float[1]);
        if (motors_vel_float[1] > -0.001) {
          motors_fw[1]  = true;
        }
        else {
          motors_fw[1]  = false;
        }
      }
    }

    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}