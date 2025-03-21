#include <math.h>
#define IN1 7
#define IN2 6
#define pwmPin5 3
#define encPin5 A5
#define IN3 10
#define IN4 11
#define pwmPin6 9
#define encPin6 A6

const float loopTime = 10; // every X ms, the code enters update_speed()
int timePassed = 0;
unsigned long prevMillis = 0; // last time (ms) the code entered update_speed()

float currPos5 = 360;
float desPos5 = 0;
int currPWM5 = 100; // current++ PWM output

float currPos6 = 0;
float desPos6 = 0;
int currPWM6 = 100; // current++ PWM output

int toc = 10;
int tic = 10;

float error5 = 0;
float error6 = 0;

float Ki5 = 25;
float Ki6 = 3.8;

bool motor5Running = false;
bool motor6Running = false;

char comma = ',';

void setup(){
  pinMode(encPin5, INPUT);
  pinMode(pwmPin5, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(encPin6, INPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(pwmPin6, OUTPUT);

  Serial.begin(115200);
}
void loop(){
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
  Serial.println("currPos5 desPos error pwm");
  Serial.println(String(currPos5) + " " + String(desPos5) + " " + String(error5) + " " + String(currPWM5));
  Serial.println(String(currPos6) + " " + String(desPos6) + " " + String(error6) + " " + String (currPWM6));
}

void update_pos(){
  if (Serial.available() > 0){
    String input = Serial.readString();
    int commaLoc1 = input.indexOf(comma);
    int commaLoc2 = input.indexOf(comma, commaLoc1+1);
    int position1 = (input.substring(0, commaLoc1)).toInt();
    int position2;
    if (commaLoc1 != -1){
      position2 = (input.substring(commaLoc1+1, commaLoc2)).toInt();
    }

    desPos5 = float(position1);
    while (desPos5 > 360) {desPos5 -= 360;}
    while (desPos5 < 0) {desPos5 += 360;}
    desPos6 = float(position2);
    while (desPos6 > 360) {desPos6 -= 360;}
    while (desPos6 < 0) {desPos6 += 360;}
  }
}

void update_error(){
  readAngleDC(currPos5, currPos6);
  motor5Running = true;
  motor6Running = true;
  //error = refRPM - currRPM; //velocity
  error5 = desPos5-currPos5; //position
  if (error5 > 180) error5 -= 360;  // Ensure shortest path
  if (error5 < -180) error5 += 360;
  
  error6 = desPos6-currPos6; //position
  if (error6 > 180) error6 -= 360;  // Ensure shortest path
  if (error6 < -180) error6 += 360;

  if (abs(error5) < 5) {
    digitalWrite(IN1, 1);
    digitalWrite(IN2, 1);
    currPWM5 = 0;
    motor5Running = false;
  }
  if (abs(error6) < 10) {
    digitalWrite(IN3, 1);
    digitalWrite(IN4, 1);
    currPWM6 = 0;
    motor6Running = false;
  }
  tic = millis();
  if(motor5Running){
    currPWM5 = Ki5*error5;
    if (currPWM5 >= 255) currPWM5 = 255;
    if (currPWM5 <= -255) currPWM5 = -255;
    if (currPWM5 <= 0) {
      digitalWrite(IN1, 1);
      digitalWrite(IN2, 0);
      analogWrite(pwmPin5, abs(currPWM5));
    } else {
      digitalWrite(IN1, 0);
      digitalWrite(IN2, 1);
      analogWrite(pwmPin5,abs(currPWM5));
    }
  }
  if(motor6Running){
    currPWM6 = Ki6*error6;
    if (currPWM6 >= 255) currPWM6 = 255;
    if (currPWM6 <= -255) currPWM6 = -255;
    if (currPWM6 <= 0) {
      digitalWrite(IN3, 1);
      digitalWrite(IN4, 0);
      analogWrite(pwmPin6,abs(currPWM6));
    } else {
      digitalWrite(IN3, 0);
      digitalWrite(IN4, 1);
      analogWrite(pwmPin6,abs(currPWM6));
    }  
  }
}

//Read encoder function
float readAngleDC(float &num1, float &num2){
  int raw_value = analogRead(encPin5);
  num1 = (raw_value/1023.0)*(5/3.3)*(360.0);
  int raw_value2 = analogRead(encPin6);
  num2 = (raw_value2/1023.0)*(5/3.3)*(360.0);}