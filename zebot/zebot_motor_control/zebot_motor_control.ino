/*==========================================================================
// Author : Handson Technology
// Project : BTD7960 Motor Control Board driven by Arduino.
// Description : Speed and direction controlled by a potentiometer attached
// to analog input A0. One side pin of the potentiometer (either one) to
// ground; the other side pin to +5V
// Source-Code : BTS7960.ino
// Program: Control DC motors using BTS7960 H Bridge Driver.
//==========================================================================
*/
#define DIR_RIGHT true
#define DIR_LEFT  false
#define MAX_SPEED 128
#define MIN_SPEED 16
#define SLOW_SPEED 50
#define SERIAL_LOOP_DELAY 35 // delay between each cycle [1.Read Serial 2.Drive motors]

#define CMD_GO_FWD     119   // w
#define CMD_GO_RWD     115   // s
#define CMD_SPIN_LEFT  97    // a
#define CMD_SPIN_RIGHT 100   // d

int FWD_R_EN  = 7;  // Forward drive Right motor (R_EN)
int RWD_R_EN  = 4;  // Reverse drive Right motor (L_EN)
int FWD_R_PWM = 6;  // Forward Level or PWM (RPWM), Right motor
int RWD_R_PWM = 5;  // Reverse Level or PWM (LPWM), Right motor

int RWD_L_EN  = 8;  // Reverse drive Left motor (R_EN)
int RWD_L_PWM = 9;  // Reverse Level or PWM (LPWM), Left motor
int FWD_L_PWM = 10; // Forward Level or PWM (RPWM), Left motor
int FWD_L_EN  = 11; // Forward drive Left motor (L_EN)

int throtle;
int acceleration;

// PID variables
float Kp = (255/120);
float Ki = 1;
float Kd = 1;
int current_speed;
int I_error;
int D_error;
int lastErr;

void setup()
{
  pinMode(FWD_R_PWM, OUTPUT);
  pinMode(RWD_R_PWM, OUTPUT);
  pinMode(FWD_L_PWM, OUTPUT);
  pinMode(RWD_L_PWM, OUTPUT);
  pinMode(FWD_R_EN, OUTPUT);
  pinMode(RWD_R_EN, OUTPUT);
  pinMode(FWD_L_EN, OUTPUT);
  pinMode(RWD_L_EN, OUTPUT);

  throtle = 0;
  acceleration = 1;
  current_speed = 0;

  Serial.begin(9600);

}

void loop()
{

  byte cmd_byte;

  // check if data has been sent from the computer:
  if (Serial.available()) {
    // read the most recent byte (which will be from 0 to 255):
    cmd_byte = Serial.read();
    
    if(cmd_byte == CMD_GO_FWD){
      go_fwd(SLOW_SPEED);
    } else if(cmd_byte == CMD_GO_RWD){
      go_rwd(SLOW_SPEED);
    } else if(cmd_byte == CMD_SPIN_LEFT){
      spin(SLOW_SPEED, DIR_LEFT);
    } else if(cmd_byte == CMD_SPIN_RIGHT){
      spin(SLOW_SPEED, DIR_RIGHT);
    } else {
      release();
    }
    delay(SERIAL_LOOP_DELAY);
  } else {
     release();
  }

}

int speed_control(int desired_speed){

  if(desired_speed < MIN_SPEED){
    desired_speed = MIN_SPEED;
  }

  float err = desired_speed - current_speed;
  I_error += err; // error summation
  D_error = lastErr - err; // error variation

  float speed_f = current_speed + Kp*err + Ki*I_error + Kd*D_error;
  current_speed = constrain( (int) speed_f, MIN_SPEED, MAX_SPEED);
  return current_speed;
}

void go_fwd(int speed){
  
  digitalWrite(FWD_R_EN, HIGH);
  digitalWrite(RWD_R_EN, HIGH);
  digitalWrite(FWD_L_EN, HIGH);
  digitalWrite(RWD_L_EN, HIGH);

  int speed_pwm = speed_control(speed);

  analogWrite(RWD_R_PWM, speed_pwm);
  analogWrite(FWD_R_PWM, 0);
  analogWrite(RWD_L_PWM, 0);
  analogWrite(FWD_L_PWM, speed_pwm);
}

void go_rwd(int speed){
  
  digitalWrite(FWD_R_EN, HIGH);
  digitalWrite(RWD_R_EN, HIGH);
  digitalWrite(FWD_L_EN, HIGH);
  digitalWrite(RWD_L_EN, HIGH);
  
  int speed_pwm = speed_control(speed);

  analogWrite(RWD_R_PWM, 0);
  analogWrite(FWD_R_PWM, speed_pwm);
  analogWrite(RWD_L_PWM, speed_pwm);
  analogWrite(FWD_L_PWM, 0);
}


void spin(int speed, bool dir){
  
  digitalWrite(FWD_R_EN, HIGH);
  digitalWrite(RWD_R_EN, HIGH);
  digitalWrite(FWD_L_EN, HIGH);
  digitalWrite(RWD_L_EN, HIGH);

  int speed_pwm = speed_control(speed);

  if(dir == DIR_LEFT){
    analogWrite(RWD_R_PWM, 0);
    analogWrite(FWD_R_PWM, speed_pwm);
    analogWrite(RWD_L_PWM, 0);
    analogWrite(FWD_L_PWM, speed_pwm);
  } else {
    analogWrite(RWD_R_PWM, speed_pwm);
    analogWrite(FWD_R_PWM, 0);
    analogWrite(RWD_L_PWM, speed_pwm);
    analogWrite(FWD_L_PWM, 0);
  }
}


void brake(){
  digitalWrite(FWD_R_EN, HIGH);
  digitalWrite(RWD_R_EN, HIGH);
  digitalWrite(FWD_L_EN, HIGH);
  digitalWrite(RWD_L_EN, HIGH);
  analogWrite(RWD_R_PWM, 0);
  analogWrite(FWD_R_PWM, 0);
  analogWrite(RWD_L_PWM, 0);
  analogWrite(FWD_L_PWM, 0);  
}

void release(){
  
  digitalWrite(FWD_R_EN, LOW);
  digitalWrite(RWD_R_EN, LOW);
  digitalWrite(FWD_L_EN, LOW);
  digitalWrite(RWD_L_EN, LOW);
  analogWrite(RWD_R_PWM, 0);
  analogWrite(FWD_R_PWM, 0);
  analogWrite(RWD_L_PWM, 0);
  analogWrite(FWD_L_PWM, 0);  
}


/* Web Resources:
 https://howtomechatronics.com/tutorials/arduino/arduino-dc-motor-control-tutorial-l298n-pwm-h-bridge/
http://www.labelektronika.com/2016/09/high-current-motor-driver-Ibt-2-arduino.html
*/


