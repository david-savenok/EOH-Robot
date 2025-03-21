#include <math.h>
#define IN1 7
#define IN2 6
#define pwmPin 3
#define encPin A2

const float loopTime = 10; // every X ms, the code enters update_speed()
float currRPM = 0; // current speed
float currPos = 360;
float refPos = 360;
int dir = 1;
int currPWM = 100; // current++ PWM output
int timePassed1 = 0;
unsigned long prevMillis1 = 0; // last time (ms) the code entered update_speed()

int timePassed2 = 0;
unsigned long prevMillis2 = 0; // last time (ms) the code entered update_speed()

volatile int refTime = 1;
int toc = 10;
int tic = 10;

float error = 0;
float Ki = 50;

char comma = ',';

void setup()
{
  pinMode(encPin, INPUT);
  pinMode(pwmPin, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
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
    update_error();
    //print_motor_info();
    //reset cnt and time
    prevMillis1 = millis();
  }
}
void print_motor_info()
{
  //print reference RPM and current RPM+
  Serial.println("Calc_RPM currPos refPos error");
  Serial.println(String(currRPM)+" "+String(currPos) + " " + String(refPos) + " " + String(error) + " " +String(toc-tic) +" " +String(refTime));
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
      //Serial.println(String(time));
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
  currPos =  readAngle();
  currRPM = (currPos/loopTime/360.0) * dir;

  //error = refRPM - currRPM; //velocity
  error = refPos-currPos; //position
  if (error > 180) error -= 360;  // Ensure shortest path
  if (error < -180) error += 360;
  if (abs(error) < 5) {
    digitalWrite(IN1, 1);
    digitalWrite(IN2, 1);
    currPWM = 0;
    return;
  }

  tic = millis();
  currPWM = Ki*error;
  //Serial.println(String(currPWM));
  if (currPWM >= 255) {currPWM = 255;}
  if (currPWM <= -255) {currPWM = -255;}
  if (currPWM <= 0) {
    digitalWrite(IN1, 1);
    digitalWrite(IN2, 0);
    analogWrite(pwmPin,abs(currPWM));
    dir = -1;
  } else {
    digitalWrite(IN1, 0);
    digitalWrite(IN2, 1);
    analogWrite(pwmPin,abs(currPWM));
    dir = 1;
  }
  toc = millis();
  //currPWM = constrain(currPWM * Ki*error, 0, 255); // make sure this stays between 0 and 255
  //analogWrite(pwmPin, currPWM);
  //reference speed can be typed in by a user in Serial Monitor or Serial Plotter
  
}
float readAngle(){
  int raw_value = analogRead(encPin);
  return (raw_value/1023.0)*(5/3.3)*(360.0);  // Convert to degrees
}