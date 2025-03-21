#define STEP_PIN1 22  // Pin connected to STEP on A4988
#define DIR_PIN1 23    // Pin connected to DIR on A4988
#define STEP_PIN2 24  // Pin connected to STEP on A4988
#define DIR_PIN2 25    // Pin connected to DIR on A4988
#define STEP_PIN3 26  // Pin connected to STEP on A4988
#define DIR_PIN3 27  // Pin connected to DIR on A4988
#define STEP_PIN4 28  // Pin connected to STEP on A4988
#define DIR_PIN4 29    // Pin connected to DIR on A4988

#define Encoder1 A1
#define Encoder2 A2
#define Encoder3 A3
#define Encoder4 A4

#include <math.h>

bool motor1Running = false;  // State to track HIGH/LOW for step pulse
bool motor2Running = false;
bool motor3Running = false;  // State to track HIGH/LOW for step pulse
bool motor4Running = false;

const unsigned long clockFreq = 16000000; //Hz

int steps1;
int steps2;
int steps3;
int steps4;
int mostSteps;
short mostStepsIndex;

int count1 = 0;
int count2 = 0;
int count3 = 0;
int count4 = 0;

float desPos1; //degrees
float desPos2; //degrees
float desPos3 = 0; //degrees
float desPos4 = 0; //degrees

float curPos1; //degrees
float curPos2; //degrees
float curPos3; //degrees
float curPos4; //degrees

float theta1; //degrees
float theta2; //degrees
float theta3; //degrees
float theta4; //degrees

int rot1 = 0;
int rot2 = 0;
int rot3 = 0;
int rot4 = 0;

short steps[4];
float frequencies[4];
float timescale;

void setup() {
  // Set STEP and DIR pins as outputs
  pinMode(STEP_PIN1, OUTPUT);
  pinMode(DIR_PIN1, OUTPUT);
  pinMode(STEP_PIN2, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);
  pinMode(STEP_PIN3, OUTPUT);
  pinMode(DIR_PIN3, OUTPUT);
  pinMode(STEP_PIN4, OUTPUT);
  pinMode(DIR_PIN4, OUTPUT);
  pinMode(Encoder1, INPUT);
  pinMode(Encoder2, INPUT);
  pinMode(Encoder3, INPUT);
  pinMode(Encoder4, INPUT);

  // Set the initial direction (rotate in one direction)
  digitalWrite(DIR_PIN1, LOW);  // Set direction pin (can be HIGH or LOW)
  digitalWrite(DIR_PIN2, LOW);
  digitalWrite(DIR_PIN3, LOW);  // Set direction pin (can be HIGH or LOW)
  digitalWrite(DIR_PIN4, LOW);
  // Start serial communication for debugging
  Serial.begin(115200);

  // Disable interrupts while configuring
  noInterrupts();
  //----------------------------------------------------------------------------------------------------
  // Reset Timer1
  TCCR1A = 0;  // Normal operation, no PWM
  TCCR1B = 0;  // Reset Timer1 settings
  
  // Set Timer1 in CTC (Clear Timer on Compare Match) mode
  TCCR1B |= (1 << WGM12);  // CTC Mode
  TCCR1B |= (1 << CS10) | (1 << CS11);   // Prescaler = 8 (16 MHz timer frequency, 2MHz effective timer frequency)

  // Set OCR1A for the 500 µs HIGH pulse (1000 timer counts)
  OCR1A = 1000; //init
  OCR1B = 499;
  // Enable Timer1 Compare Match A and B interrupts
  TIMSK1 |= (1 << OCIE1A) | (1<< OCIE1B);
  //-----------------------------------------------------------------------------------------------------
  // Reset Timer3 (2)
  TCCR3A = 0;  // Normal operation, no PWM
  TCCR3B = 0;  // Reset Timer1 settings
  
  // Set Timer3 in CTC (Clear Timer on Compare Match) mode
  TCCR3B |= (1 << WGM32);  // CTC Mode
  TCCR3B |= (1<<CS30) | (1 << CS31);   // Prescaler = 8 (16 MHz timer frequency, 2MHz effective timer frequency)

  // Set OCR1A for the 500 µs HIGH pulse (1000 timer counts)
  OCR3A = 1000; //init
  OCR3B = 499;
  // Enable Timer3 Compare Match A and B interrupts
  TIMSK3 |= (1 << OCIE3A) | (1<< OCIE3B);
  //-----------------------------------------------------------------------------------------------------
  // Reset Timer4 (3)
  TCCR4A = 0;  // Normal operation, no PWM
  TCCR4B = 0;  // Reset Timer1 settings
  
  // Set Timer3 in CTC (Clear Timer on Compare Match) mode
  TCCR4B |= (1 << WGM42);  // CTC Mode
  TCCR4B |= (1<<CS40) | (1 << CS41);   // Prescaler = 8 (16 MHz timer frequency, 2MHz effective timer frequency)

  // Set OCR1A for the 500 µs HIGH pulse (1000 timer counts)
  OCR4A = 1000; //init
  OCR4B = 499;
  // Enable Timer4 Compare Match A and B interrupts
  TIMSK4 |= (1 << OCIE4A) | (1<< OCIE4B);
  //-----------------------------------------------------------------------------------------------------
    // Reset Timer3 (2)
  TCCR5A = 0;  // Normal operation, no PWM
  TCCR5B = 0;  // Reset Timer1 settings
  
  // Set Timer3 in CTC (Clear Timer on Compare Match) mode
  TCCR5B |= (1 << WGM52);  // CTC Mode
  TCCR5B |= (1<<CS50) | (1 << CS51);   // Prescaler = 8 (16 MHz timer frequency, 2MHz effective timer frequency)

  // Set OCR1A for the 500 µs HIGH pulse (1000 timer counts)
  OCR5A = 1000; //init
  OCR5B = 499;
  // Enable Timer5 Compare Match A and B interrupts
  TIMSK5 |= (1 << OCIE5A) | (1<< OCIE5B);
  //-----------------------------------------------------------------------------------------------------
  // Enable global interrupts
  interrupts();
}

//1----------------------------------------------------------------------------------------------------
ISR(TIMER1_COMPA_vect) {
  if (motor1Running) {
    digitalWrite(STEP_PIN1, HIGH);  // Generate step pulse
    count1 ++;
  }
}

ISR(TIMER1_COMPB_vect) {
  if (motor1Running) {
    digitalWrite(STEP_PIN1, LOW);   // End step pulse
  }
}

//2----------------------------------------------------------------------------------------------------
ISR(TIMER3_COMPA_vect) {
  if (motor2Running) {
    digitalWrite(STEP_PIN2, HIGH);  // Generate step pulse
    count2 ++;
  }
}

ISR(TIMER3_COMPB_vect) {
  if (motor2Running) {
    digitalWrite(STEP_PIN2, LOW);   // End step pulse
  }
}
//3----------------------------------------------------------------------------------------------------
ISR(TIMER4_COMPA_vect) {
  if (motor3Running) {
    digitalWrite(STEP_PIN3, HIGH);  // Generate step pulse
    count3 ++;
  }
}

ISR(TIMER4_COMPB_vect) {
  if (motor3Running) {
    digitalWrite(STEP_PIN3, LOW);   // End step pulse
  }
}
//4----------------------------------------------------------------------------------------------------
ISR(TIMER5_COMPA_vect) {
  if (motor4Running) {
    digitalWrite(STEP_PIN4, HIGH);  // Generate step pulse
    count4 ++;
  }
}

ISR(TIMER5_COMPB_vect) {
  if (motor4Running) {
    digitalWrite(STEP_PIN4, LOW);   // End step pulse
  }
}

//-----------------------------------------------------------------------------------------------------
void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readString(); // Read input until newline
    command.trim();  // Remove any extra spaces or newline characters
    int commaLoc1 = command.indexOf(',');
    int commaLoc2 = command.indexOf(',', commaLoc1+1);
    desPos1 = (command.substring(0, commaLoc1)).toFloat();
    desPos2 = desPos1;
    if (commaLoc1 != -1) desPos2 = (command.substring(commaLoc1+1, commaLoc2)).toFloat();
    if (!motor1Running) {
      theta1 = desPos1 - (((rot1*360.0)/20.0) + readEncoder(curPos1, A1));
      theta2 = desPos2 - (((rot2*360.0)/50.0) + readEncoder(curPos2, A2));
      theta3 = desPos3 - (((rot3*360.0)/20.0) + readEncoder(curPos3, A3));
      theta4 = desPos4 - (((rot4*360.0)/20.0) + readEncoder(curPos4, A4));
      digitalWrite(DIR_PIN1, (theta1 >= 0) ? LOW : HIGH);
      digitalWrite(DIR_PIN2, (theta2 >= 0) ? LOW : HIGH);
      digitalWrite(DIR_PIN3, (theta3 >= 0) ? LOW : HIGH);
      digitalWrite(DIR_PIN4, (theta4 >= 0) ? LOW : HIGH);//maybe change back >= -> >
      steps[0] = short((abs(theta1)*20.0)/1.8); 
      steps[1] = short((abs(theta2)*50.0)/1.8); 
      steps[2] = short((abs(theta3)*20.0)/1.8);
      steps[3] = short((abs(theta4)*20.0)/1.8);
      mostSteps = steps[0];
      mostStepsIndex = 0;
      for (int i=1; i<4; i++){
        if (steps[i] > mostSteps){
          mostSteps = steps[i];
          mostStepsIndex = i;
        }
      }
      frequencies[mostStepsIndex] = 1000;
      timescale = mostSteps/frequencies[mostStepsIndex];

      if (timescale != 0){
        for (int i = 0; i<4; i++){
          if (i != mostStepsIndex){
            frequencies[i] = steps[i]/timescale;
          }
        }
        noInterrupts();
        OCR1A = calcOCRA(frequencies[0], 32);
        OCR3A = calcOCRA(frequencies[1], 32);
        OCR4A = calcOCRA(frequencies[2], 32); 
        OCR5A = calcOCRA(frequencies[3], 32);
        interrupts();
        if (steps[0] > 0) motor1Running = true;
        if (steps[1] > 0) motor2Running = true;
        if (steps[2] > 0) motor3Running = true;
        if (steps[3] > 0) motor4Running = true;
      }
      Serial.println("Motors started.");
      rot1 += floor((theta1*20)/360);
      rot2 += floor((theta2*50)/360);
      rot3 += floor((theta3*20)/360);
      rot4 += floor((theta4*20)/360);
    }
  }
  
  if(count1 >= steps[0]){
    motor1Running = false;
    count1 = 0;
  }if(count2 >= steps[1]){
    motor2Running = false;
    count2 = 0;
  }if(count3 >= steps[2]){
    motor3Running = false;
    count3 = 0;
  }if(count4 >= steps[3]){
    motor4Running = false;
    count4 = 0;
  }
}

int calcOCRA(float freq, int prescale){
  float OCRA = ((clockFreq/(freq*prescale)) - 1);
  if(OCRA > 65535){
    OCRA = 65535;
  }
  return OCRA;
}

float readEncoder(float &currentPos, int analogPin){
  currentPos = (analogPin == A2) ? (((analogRead(analogPin)/1023.0)*(5/4.25)*(360.0))/50.0) : (((analogRead(analogPin)/1023.0)*(5/4.25)*(360.0))/20.0);
}