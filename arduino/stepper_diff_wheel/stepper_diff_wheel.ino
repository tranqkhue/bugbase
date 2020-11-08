#include <AccelStepper.h>
//Stepper speed is measured as steps per second
//Normally a stepper has 200 steps per revolution

const int STEP_EN=8;
const int STEP_A_PIN=2;
const int STEP_B_PIN=3;
const int DIR_A_PIN=5;
const int DIR_B_PIN=6;
//int const * a;
//int const * b;
//int const * c;
//int const * d;

AccelStepper stepper0(AccelStepper::DRIVER, *a, DIR_A_PIN);
AccelStepper stepper1(AccelStepper::DRIVER, *b, DIR_B_PIN);

char received_arr[19];
float motors_vel[2];// = {800,800};
const char delimiters[] = "|";
char * running_char;
char * token;
//char init_char = 'i';

void setup(){
  Serial.begin(115200);
  /*
  //--------------------------------------------------------------------
  //This section is used to read the params!
  while (true) {
    size_t num_read = Serial.readBytesUntil('\n', received_arr, 17);
    running_char    = strdup(received_arr);
    token           = strsep(&running_char, delimiters);
    if (*token == init_char){
      token      = strsep(&running_char, delimiters);
      STEP_EN    = (uint8_t)atoi(token); 
      token      = strsep(&running_char, delimiters);
      STEP_A_PIN = (uint8_t)atoi(token); 
      token      = strsep(&running_char, delimiters);
      STEP_B_PIN = (uint8_t)atoi(token); 
      token      = strsep(&running_char, delimiters);
      DIR_A_PIN  = (uint8_t)atoi(token); 
      token      = strsep(&running_char, delimiters);
      DIR_B_PIN  = (uint8_t)atoi(token); 
      break;
    }
  }
  //--------------------------------------------------------------------
  Serial.println("DI"); //Confirm having done initializing
  */

  pinMode(STEP_EN, OUTPUT);
  digitalWrite(STEP_EN, LOW);
  a = &STEP_A_PIN;
  b = &STEP_B_PIN;

  stepper0.setMaxSpeed(3300);  
  stepper1.setMaxSpeed(3300);
}
//----------------------------------------------------------------------
//Message format: +xxxx.xx|+yyyy.yy\n (velocity: steps per second)
//Total 19 bytes
 
void loop(){
  if (Serial.available() > 0) {
    size_t num_read = Serial.readBytesUntil('\n', received_arr, 17);
    running_char    = strdup(received_arr);
    token           = strsep(&running_char, delimiters);
    motors_vel[0]   = atof(token);
    token           = strsep(&running_char, delimiters);
    motors_vel[1]   = atof(token);
  }

  stepper0.setSpeed(motors_vel[0]);
  stepper1.setSpeed(motors_vel[1]);
  stepper0.runSpeed();
  stepper1.runSpeed();
  //Serial.println(motors_vel[1]);
}
