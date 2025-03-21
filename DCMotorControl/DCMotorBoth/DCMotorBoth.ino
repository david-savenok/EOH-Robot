#include <math.h>
#define IN1 7
#define IN2 6
#define pwmPin1 3
#define encPin1 A2
#define IN3 10
#define IN4 11
#define pwmPin2 9
#define encPin2 A1

const float loopTime = 10; // every X ms, the code enters update_speed()
int timePassed = 0;
unsigned long prevMillis = 0; // last time (ms) the code entered update_speed()

float currPos1 = 360;
float desPos1 = 0;
int dir1 = 1;
int currPWM1 = 100; // current++ PWM output

float currPos2 = 0;
float desPos2 = 0;
int dir2 = 1;
int currPWM2 = 100; // current++ PWM output

int toc = 10;
int tic = 10;

float error1 = 0;
float error2 = 0;

float Ki1 = 25;
float Ki2 = 3.8;

char comma = ',';

void setup()
{
  pinMode(encPin1, INPUT);
  pinMode(pwmPin1, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(encPin2, INPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(pwmPin2, OUTPUT);

  Serial.begin(500000);
}
void loop()
{
  timePassed = millis() - prevMillis;
  if(timePassed >= loopTime){
    update_pos();
    update_error();
    //print_motor_info(); //For debugging
    prevMillis = millis();
  }
}
void print_motor_info(){
  //print current position, desired position, error, and pwm value
  Serial.println("currPos desPos error pwm");
  Serial.println(String(currPos1) + " " + String(desPos1) + " " + String(error1) + " " + String(currPWM1));
  Serial.println(String(currPos2) + " " + String(desPos2) + " " + String(error2) + " " + String (currPWM2));
}

void update_pos(){
  if (Serial.available() > 0)
  {
    String input = Serial.readString();
    int commaLoc1 = input.indexOf(comma);
    int commaLoc2 = input.indexOf(comma, commaLoc1+1);
    int position1 = (input.substring(0, commaLoc1)).toInt();
    int position2;
    if (commaLoc1 != -1){
      position2 = (input.substring(commaLoc1+1, commaLoc2)).toInt();
    }

    desPos1 = float(position1);
    while (desPos1 > 360) {desPos1 -= 360;}
    while (desPos1 < 0) {desPos1 += 360;}
    desPos2 = float(position2);
    while (desPos2 > 360) {desPos2 -= 360;}
    while (desPos2 < 0) {desPos2 += 360;}
  }
}

void update_error()
{
  tic = millis();

  readAngle(currPos1, currPos2);

  bool motor5Running = true; //End conditions
  bool motor6Running = true;
  error1 = refPos-currPos; //position error
  if (error1 > 180) error1 -= 360;  // Ensure shortest path
  if (error1 < -180) error1 += 360;
  
  error2 = refPos2-currPos2; //position error
  if (error2 > 180) error2 -= 360;  // Ensure shortest path
  if (error2 < -180) error2 += 360;

  //End conditions
  if (abs(error1) < 5) {
    digitalWrite(IN1, 1);
    digitalWrite(IN2, 1);
    currPWM1 = 0;
    motor5Running = false;
  }
  if (abs(error2) < 10) {
    digitalWrite(IN3, 1);
    digitalWrite(IN4, 1);
    currPWM2 = 0;
    motor6Running = false;
  }
  
  //Only adjust motor 5 if it hasn't hit the target yet
  if(motor5Running){
    currPWM1 = Ki1*error1;
    if (currPWM1 >= 255) {currPWM1 = 255;}
    if (currPWM1 <= -255) {currPWM1 = -255;}
    if (currPWM1 <= 0) {
      digitalWrite(IN1, 1);
      digitalWrite(IN2, 0);
      analogWrite(pwmPin1, abs(currPWM1));
      dir1 = -1;
    } else {
      digitalWrite(IN1, 0);
      digitalWrite(IN2, 1);
      analogWrite(pwmPin1,abs(currPWM1));
      dir1 = 1;
    }
  }

  //Only adjust motor 2 if it hasn't hit the target yet
  if(motor6Running){
    currPWM2 = Ki2*error2;
    if (currPWM2 >= 255) {currPWM2 = 255;}
    if (currPWM2 <= -255) {currPWM2 = -255;}
    if (currPWM2 <= 0) {
      digitalWrite(IN3, 1);
      digitalWrite(IN4, 0);
      analogWrite(pwmPin2,abs(currPWM2));
      dir2 = -1;
    } else {
      digitalWrite(IN3, 0);
      digitalWrite(IN4, 1);
      analogWrite(pwmPin2,abs(currPWM2));
      dir2 = 1;
    }
  }
  
  toc = millis();  
}

//Read encoder function
float readAngle(float &num1, float &num2){
  int raw_value = analogRead(encPin1);
  num1 = (raw_value/1023.0)*(5/3.3)*(360.0);
  int raw_value2 = analogRead(encPin2);
  num2 = (raw_value2/1023.0)*(5/3.3)*(360.0);
}