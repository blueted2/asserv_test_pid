#include <PID_v1.h>
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

#include "pins.h"
#include "pid_coefs.h"

Encoder encoder_left(ENCODER_LEFT_A, ENCODER_LEFT_B);
Encoder encoder_right(ENCODER_RIGHT_A, ENCODER_RIGHT_B);

double left_speed, right_speed, left_output, right_output;

int64_t left_pos, right_pos;

//Define Variables we'll be connecting to
double setpoint;

//Specify the links and initial tuning parameters
PID pid_left(&left_speed, &left_output, &setpoint, MOTOR_POS_P, MOTOR_SPEED_I, MOTOR_SPEED_D , DIRECT);
PID pid_right(&right_speed, &right_output, &setpoint, MOTOR_POS_P, MOTOR_SPEED_I, MOTOR_SPEED_D , DIRECT);



void setup(){

  pinMode(LEFT_WHEEL_ENABLE, OUTPUT);
  pinMode(LEFT_WHEEL_CONTROL1, OUTPUT);
  pinMode(LEFT_WHEEL_CONTROL2, OUTPUT);

  pinMode(RIGHT_WHEEL_ENABLE, OUTPUT);
  pinMode(RIGHT_WHEEL_CONTROL1, OUTPUT);
  pinMode(RIGHT_WHEEL_CONTROL2, OUTPUT);
  
  Serial.begin(9600);

  //initialize the variables we're linked to
  left_speed = 0;
  right_speed = 0;

  left_output = 0;
  right_output = 0;

  setpoint = 3;

  //turn the PID on
  pid_left.SetOutputLimits(-255, 255);
  pid_left.SetMode(AUTOMATIC);

}

uint16_t last_read = millis();

void loop(){
  uint16_t now = millis();
  double dt = (double)(now - last_read);
  long new_left_pos = encoder_left.read();
  long new_right_pos = encoder_right.read();
  
  last_read = now;

  if (dt == 0){
    left_speed = 0;
    right_speed = 0;
  }
  else
  {
    left_speed = (new_left_pos - left_pos) / dt;
    right_speed = (new_right_pos - right_pos) / dt;
  }
  

  left_pos = new_left_pos;
  right_pos = new_right_pos;

  pid_left.Compute();
  pid_right.Compute();

  if (left_output < 0)
  {
    digitalWrite(LEFT_WHEEL_CONTROL1, HIGH);
    digitalWrite(LEFT_WHEEL_CONTROL2, LOW);
  }
  else
  {
    digitalWrite(LEFT_WHEEL_CONTROL1, LOW);
    digitalWrite(LEFT_WHEEL_CONTROL2, HIGH);
  }
  analogWrite(LEFT_WHEEL_ENABLE, abs((int)left_output));

  if (right_output < 0)
  {
    digitalWrite(RIGHT_WHEEL_CONTROL1, HIGH);
    digitalWrite(RIGHT_WHEEL_CONTROL2, LOW);
  }
  else
  {
    digitalWrite(RIGHT_WHEEL_CONTROL1, LOW);
    digitalWrite(RIGHT_WHEEL_CONTROL2, HIGH);
  }
  analogWrite(RIGHT_WHEEL_ENABLE, abs((int)right_output));

  Serial.print(left_pos);
  Serial.print(" ");
  Serial.print(left_speed);
  Serial.print(" ");
  Serial.print(left_output);
  Serial.println();

  // // Serial.print("Output: ");
  // Serial.print(input / 5.0);
  // Serial.print(" ");
  // Serial.println(output / 255.0);
  
  delay(100);
}