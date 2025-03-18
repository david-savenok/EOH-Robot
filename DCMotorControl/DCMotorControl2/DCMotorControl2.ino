#include <math.h>

const int pwmPinA = 9;
const int pwmPinB = 10;
const int encPin = 2;
const float loopTime = 50; // every 100 ms, the code enters update_speed()
float refRPM = 50; // reference speed (Revolution Per Minute)
float currRPM = 0; // current speed
float currPos = 0;
float refPos = 360;
int sign = 1;
int currPWM = 100; // current++ PWM output
int timePassed = 0;
unsigned long prevMillis = 0; // last time (ms) the code entered update_speed()

int timePassed2 = 0;
unsigned long prevMillis2 = 0; // last time (ms) the code entered update_speed()

volatile int refTime = 1;
int toc = 10;
int tic = 10;

volatile long cnt = 0; // number of times Hall sensor passed the magnets
volatile long offset = 0;
float error = 0;
float Ki = 50;

char comma = ',';

void setup()
{
  pinMode(encPin, INPUT);
  pinMode(pwmPinA, OUTPUT);
  pinMode(pwmPinB, OUTPUT);
  Serial.begin(500000);
  attachInterrupt(digitalPinToInterrupt(encPin), magnet_detected, RISING);
}
void loop()
{
  timePassed2 = millis() - prevMillis2;
  if(timePassed2 >= loopTime/20)
  {
    update_pos();

    prevMillis2 = millis();
  }
  
  
  timePassed = millis() - prevMillis;
  if(timePassed >= loopTime)
  {
    print_motor_info();
    update_error();
    //reset cnt and time
    offset += cnt*sign;
    cnt = 0;
    prevMillis = millis();
  }
}
void print_motor_info()
{
  //print reference RPM and current RPM+
  Serial.println("Set_RPM Calc_RPM currPos refPos error");
  Serial.println(String(refRPM)+" "+String(currRPM)+" "+String(currPos) + " " + String(refPos) + " " + String(error) + " " +String(toc-tic) +" " +String(refTime));
}

void update_pos()
{
  if (Serial.available() > 0)
  {
    String input = Serial.readString();
    int commaLoc = input.indexOf(comma);
    int position = (input.substring(0, commaLoc)).toInt();

    int time = 0;
    if (commaLoc != -1){
      int time = (input.substring(commaLoc+1)).toInt();
      Serial.println(String(time));
    }
    

    if (position > 0)
    {
      refPos = position;
    }
    if (commaLoc != -1 && time > 0)
    {
      refTime = time;
    }
  }
}

void update_error()
{
  currRPM = cnt*(60000.0/loopTime/144.0) * sign;
  currPos =  currPos + currRPM*360.0*(loopTime/60000.0);
  
  refPos = fmod(refPos, 360);

  if(currPos >= 360){
    currPos -= 360;
  }
  else if (currPos <=0){
    currPos += 360;
  }

  //error = refRPM - currRPM; //velocity
  error = refPos-currPos; //position
  if (error > 180) error -= 360;  // Ensure shortest path
  if (error < -180) error += 360;
  if (abs(error) < 2) {
    analogWrite(pwmPinA, 0);
    analogWrite(pwmPinB, 0);
    currPWM = 0;
    return;
  }

  tic = millis();
  currPWM = Ki*error;
  Serial.println(String(currPWM));
  if (currPWM >= 255) {currPWM = 255;}
  if (currPWM <= -255) {currPWM = -255;}
  if (currPWM <= 0) {
    analogWrite(pwmPinA,abs(currPWM));
    analogWrite(pwmPinB,0);
    sign = -1;
  } else {
    analogWrite(pwmPinB,abs(currPWM));
    analogWrite(pwmPinA,0);
    sign = 1;
  }
  toc = millis();
  //currPWM = constrain(currPWM * Ki*error, 0, 255); // make sure this stays between 0 and 255
  //analogWrite(pwmPin, currPWM);
  //reference speed can be typed in by a user in Serial Monitor or Serial Plotter
  
}
void magnet_detected()
{
  cnt++;
}






