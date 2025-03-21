#include <math.h>
#define IN3 10
#define IN4 11
#define pwmPin2 9
#define encPin6 A1

const float loopTime = 10; // every X ms, the code enters update_speed()
float currRPM = 0; // current speed
float currPos = 360;
float refPos = 360;
int dir = 1;
int currPWM = 100; // current++ PWM output
int timePassed = 0;
unsigned long prevMillis = 0; // last time (ms) the code entered update_speed()

int timePassed2 = 0;
unsigned long prevMillis2 = 0; // last time (ms) the code entered update_speed()

volatile int refTime = 1;
int toc = 10;
int tic = 10;

float error = 0;
float Ki = 3.8;

char comma = ',';

void setup()
{
  pinMode(encPin6, INPUT);
  pinMode(pwmPin2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.begin(500000);
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
  if(timePassed >= loopTime/2)
  {
    update_error();
    print_motor_info();
    //reset cnt and time
    prevMillis = millis();
  }
}
void print_motor_info()
{
  //print reference RPM and current RPM+
  Serial.println("currPos refPos error currPWM");
  Serial.println(String(currPos) + " " + String(refPos) + " " + String(error)+ " " + String(currPWM));
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
    

    if (abs(position) > 0)
    {
      refPos += position;
      if (refPos > 360) refPos -= 360;
      if (refPos < 0) refPos += 360;
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
  
  //error = refRPM - currRPM; //velocity
  error = refPos-currPos; //position
  if (error > 180) error -= 360;  // Ensure shortest path
  if (error < -180) error += 360;
  
  if (abs(error) < 5) {
    analogWrite(IN3, 1);
    analogWrite(IN4, 1);
    currPWM = 0;
    return;   
  }

  tic = millis();

  currPWM = Ki*error;
  if (abs(error) < 25){
    currPWM += (error/(abs(error)))*17;
  }
  //Serial.println(String(currPWM));
  if (currPWM >= 255) {currPWM = 255;}
  if (currPWM <= -255) {currPWM = -255;}


  if (currPWM <= 0) {
    digitalWrite(IN3, 1);
    digitalWrite(IN4, 0);
    analogWrite(pwmPin2,abs(currPWM));
    dir = -1;
  } else {
    digitalWrite(IN3, 0);
    digitalWrite(IN4, 1);
    analogWrite(pwmPin2,abs(currPWM));
    dir = 1;
  }

  toc = millis();
  //currPWM = constrain(currPWM * Ki*error, 0, 255); // make sure this stays between 0 and 255
  //analogWrite(pwmPin, currPWM);
  //reference speed can be typed in by a user in Serial Monitor or Serial Plotter
  
}
float readAngle(){
  int raw_value = analogRead(encPin6);
  return (raw_value/1023.0)*(5/3.3)*(360.0);  // Convert to degrees
}