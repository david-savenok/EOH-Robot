#include <math.h>
#define IN1 7
#define IN2 6
#define pwmPin 3
#define encPin A2
#define IN3 10
#define IN4 11
#define pwmPin2 9
#define encPin6 A1

const float loopTime = 10; // every X ms, the code enters update_speed()
float currRPM = 0; // current speed
float currPos = 360;
float refPos = 0;
int dir = 1;
int currPWM = 100; // current++ PWM output
int timePassed1 = 0;
unsigned long prevMillis1 = 0; // last time (ms) the code entered update_speed()

float currRPM2 = 0; // current speed
float currPos2 = 0;
float refPos2 = 0;
int dir2 = 1;
int currPWM2 = 100; // current++ PWM output

int timePassed2 = 0;
unsigned long prevMillis2 = 0; // last time (ms) the code entered update_speed()

volatile int refTime = 1;
volatile int refTime2 = 1;
int toc = 10;
int tic = 10;

float error = 0;
float error2 = 0;

float Ki = 25;
float Ki2 = 3.8;

char comma = ',';

void setup()
{
  pinMode(encPin, INPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(encPin6, INPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(pwmPin2, OUTPUT);

  Serial.begin(500000);
}
void loop()
{
  timePassed2 = millis() - prevMillis2;
  if(timePassed2 >= loopTime){
    update_pos();
    prevMillis2 = millis();
  }
  
  timePassed1 = millis() - prevMillis1;
  if(timePassed1 >= loopTime)
  {
    //print_motor_info();
    update_error();
    //print_motor_info();
    //reset cnt and time
    prevMillis1 = millis();
  }
}
void print_motor_info()
{
  //print reference RPM and current RPM+
  Serial.println("Calc_RPM currPos refPos error pwm");
  Serial.println(String(currRPM)+" "+String(currPos) + " " + String(refPos) + " " + String(error) + " " + String(currPWM));
  Serial.println(String(currRPM2)+" "+String(currPos2) + " " + String(refPos2) + " " + String(error2) + " " + String (currPWM2));
}

void update_pos()
{
  if (Serial.available() > 0)
  {
    String input = Serial.readString();
    int position2 = 0;
    int commaLoc = input.indexOf(comma);
    int position = (input.substring(0, commaLoc)).toInt();
    int commaLoc2 = input.indexOf(comma, commaLoc+1);
    if (commaLoc != -1){
      position2 = (input.substring(commaLoc+1, commaLoc2)).toInt();
    }

    refPos = float(position);
    while (refPos > 360) {refPos -= 360;}
    while (refPos < 0) {refPos += 360;}
    refPos2 = float(position2);
    while (refPos2 > 360) {refPos2 -= 360;}
    while (refPos2 < 0) {refPos2 += 360;}
  }
  
}

void update_error()
{
  readAngle(currPos, currPos2);
  currRPM = (currPos/loopTime/360.0) * dir;
  currRPM2 = (currPos2/loopTime/360.0) * dir2;
  bool cond1 = false;
  bool cond2 = false;
  //error = refRPM - currRPM; //velocity
  error = refPos-currPos; //position
  if (error > 180) error -= 360;  // Ensure shortest path
  if (error < -180) error += 360;
  
  error2 = refPos2-currPos2; //position
  if (error2 > 180) error2 -= 360;  // Ensure shortest path
  if (error2 < -180) error2 += 360;

  if (abs(error) < 5) {
    digitalWrite(IN1, 1);
    digitalWrite(IN2, 1);
    currPWM = 0;
    cond1 = true;
  }
  if (abs(error2) < 10) {
    digitalWrite(IN3, 1);
    digitalWrite(IN4, 1);
    currPWM2 = 0;
    cond2 = true;
  }
  if (cond1 && cond2){
    return;
  }
  tic = millis();

  currPWM = Ki*error;
  if (currPWM >= 255) {currPWM = 255;}
  if (currPWM <= -255) {currPWM = -255;}
  if (currPWM <= 0) {
    digitalWrite(IN1, 1);
    digitalWrite(IN2, 0);
    analogWrite(pwmPin, abs(currPWM));
    dir = -1;
  } else {
    digitalWrite(IN1, 0);
    digitalWrite(IN2, 1);
    analogWrite(pwmPin,abs(currPWM));
    dir = 1;
  }

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

  toc = millis();
  //currPWM = constrain(currPWM * Ki*error, 0, 255); // make sure this stays between 0 and 255
  //analogWrite(pwmPin, currPWM);
  //reference speed can be typed in by a user in Serial Monitor or Serial Plotter
  
}
float readAngle(float &num, float &num2){
  int raw_value = analogRead(encPin);
  num = (raw_value/1023.0)*(5/3.3)*(360.0);
  int raw_value2 = analogRead(encPin6);
  num2 = (raw_value2/1023.0)*(5/3.3)*(360.0);
}