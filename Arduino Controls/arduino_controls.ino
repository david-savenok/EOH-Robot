#include <stdio.h>
#include <stdlib.h>
//#include <Servo.h>
#include <math.h>

using namespace std;

//-------------------DEFINE DIRECTIVES----------------------------------------

#define STEP_PIN1 22  // Pin connected to STEP on A4988
#define DIR_PIN1 23    // Pin connected to DIR on A4988
#define STEP_PIN2 24  // Pin connected to STEP on A4988
#define DIR_PIN2 25    // Pin connected to DIR on A4988
#define STEP_PIN3 26  // Pin connected to STEP on A4988
#define DIR_PIN3 27  // Pin connected to DIR on A4988
#define STEP_PIN4 28  // Pin connected to STEP on A4988
#define DIR_PIN4 29    // Pin connected to DIR on A4988
#define IN1 7
#define IN2 6
#define pwmPin5 3
#define IN3 10
#define IN4 11
#define pwmPin6 9
#define encPin1 A1
#define encPin2 A2
#define encPin3 A3
#define encPin4 A4
#define encPin5 A5
#define encPin6 A6

//-------------------STRUCTURES----------------------------------------
typedef struct Buffer {
  char name;
  char* buf;
  bool is_full = false;
} Buffer;

typedef struct Instruction {
  char instruction_type;
  char* command_start; //This needs to be fixed
  char* command_end;
} Instruction;

typedef struct State {
  float* angles;
} State;

// "{H/N}"
typedef struct StartCommand {
  char type;
} StartCommand;

// "{N/X}*{t}*{Angles}"
typedef struct EndCommand {
  char type;
  short t;
  float* angles;
} EndCommand;

// "{t}*{Angles}"
typedef struct MoveCommand {
  float* angles;
} MoveCommand;

// "{t}"
typedef struct PauseCommand {
  short t;
} PauseCommand;

// "{1/0}*{t}*{color hex}"
typedef struct LEDCommand {
  bool state;
  short t;
  int color;
} LEDCommand;

// "{1/0}"
typedef struct ClawCommand {
  bool open;
} ClawCommand;

//-------------------VARIABLE INSTANTIATIONS---------------------------
float zeroOffset[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float desiredAngles[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//Servo clawServo;
float acceptable_percent_error = 0.01; //FIGURE THIS OUT EXPERIMENTALLY TUNE
Buffer* bufA = malloc(sizeof(Buffer));
Buffer* bufB = malloc(sizeof(Buffer));
Buffer* to_fill = bufA;
Buffer* to_use = NULL;
bool signal_sent = false;
bool start_comms;

char* move_command = NULL;
bool in_token = false;

//Joint limits

float motor1LB = -360.0; //units of full rotations
float motor1UB =  360.0;

float motor2LB = -25.0;
float motor2UB =  205.0;

float motor3LB = -145.0;
float motor3UB =  145.0;

float motor4LB = -720.0;
float motor4UB =  720.0;

float motor5LB = -720.0/360.0;////EDIT FOR DCS
float motor5UB =  720.0/360.0;

float motor6LB = -720.0/360.0;
float motor6UB =  720.0/360.0;

//Stepper globals

bool checkSteppers = false;
bool steppersOff = true; 
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

float previousDesPos1 = 0.0;
float previousDesPos2 = 0.0;
float previousDesPos3 = 0.0;
float previousDesPos4 = 0.0;

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
float encZeroes[4] = {0.0, 0.0, 0.0, 0.0};

//DC Globals

const float loopTime = 10; // every X ms, the code enters update_speed()
int timePassed = 0;
unsigned long prevMillis = 0; // last time (ms) the code entered update_speed()

bool passZero5 = false;
bool passZero6 = false;

float currPos5 = 360;
float desPos5 = 0;
int currPWM5 = 100; // current++ PWM output
bool motor5Running = false;

float currPos6 = 0;
float desPos6 = 0;
int currPWM6 = 100; // current++ PWM output
bool motor6Running = false;

int toc = 0;
int tic = 0;

float error5 = 0;
float error6 = 0;

int rot5 = 0;
int rot6 = 0;

float Ki5 = 25;
float Ki6 = 3.8;

//------------------------SETUP----------------------------------------
void setup() {
  //clawServo.attach(CHOOSE A PIN PLEASE);
  Serial.begin(115200);
  //Serial1.begin(9600);
  bufA->name = 'A';
  bufB->name = 'B';
  bufA->is_full = false;
  bufB->is_full = false;
  bufA->buf = malloc(1500);
  bufB->buf = malloc(1500);
  start_comms = true;

  // Set STEP and DIR pins as outputs
  pinMode(STEP_PIN1, OUTPUT);
  pinMode(DIR_PIN1, OUTPUT);
  pinMode(STEP_PIN2, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);
  pinMode(STEP_PIN3, OUTPUT);
  pinMode(DIR_PIN3, OUTPUT);
  pinMode(STEP_PIN4, OUTPUT);
  pinMode(DIR_PIN4, OUTPUT);
  pinMode(encPin1, INPUT);
  pinMode(encPin2, INPUT);
  pinMode(encPin3, INPUT);
  pinMode(encPin4, INPUT);
  //DC pin settings
  pinMode(encPin5, INPUT);
  pinMode(encPin6, INPUT);
  pinMode(pwmPin5, OUTPUT);
  pinMode(pwmPin6, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Set the initial direction (rotate in one direction)
  digitalWrite(DIR_PIN1, LOW);  // Set stepper direction pin (can be HIGH or LOW)
  digitalWrite(DIR_PIN2, LOW);
  digitalWrite(DIR_PIN3, LOW);  // Set stepper direction pin (can be HIGH or LOW)
  digitalWrite(DIR_PIN4, LOW);

  readEncoderStepper(&curPos1, A1, 0.0);
  readEncoderStepper(&curPos2, A2, 0.0);
  readEncoderStepper(&curPos3, A3, 0.0);    
  readEncoderStepper(&curPos4, A4, 0.0);
  encZeroes[0] = curPos1;
  Serial.println(encZeroes[0]);
  encZeroes[1] = curPos2;
  encZeroes[2] = curPos3;
  encZeroes[3] = curPos4;

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

//------------------------MAIN LOOP------------------------------------
void loop() {
  if (start_comms) {
    bufA->is_full = false;
    Serial.println("A");
    start_comms = false;
  }
  if (bufA->is_full && steppersOff && !checkSteppers) {
    bool success = handleInstructionSet(&(bufA->buf));
    if (success) {
      bufA->is_full = false;
      Serial.println("A");
    }      
  }
  if (bufB->is_full && steppersOff && !checkSteppers) {
    bool success = handleInstructionSet(&(bufB->buf));
    if (success) {
      bufB->is_full = false;
      Serial.println("A");
    }
  }
}

//------------------------EVENT HANDLER---------------------------------
void serialEvent() {
  while (Serial.available() > 0) {   
    if (!to_fill->is_full) {       
      Serial.readBytesUntil('\0', to_fill->buf, 1500);
      char* terminating_char = strchr(to_fill->buf, 'Q');
      *terminating_char = '\0';  
    }  
    if (strlen(to_fill->buf) > 0) {
      if (to_fill->name == 'A') {
        to_fill->is_full = true;
        to_fill = bufB;       
      }
      else if (to_fill->name == 'B') {
        to_fill->is_full = true;
        to_fill = bufA;
      }     
    }
  } 
}


bool handleInstructionSet(char** instruction_set) {
  Instruction curr_data = parseInstructionSet(instruction_set);

  while (curr_data.command_start) { //while the instruction set is not empty, continue to execute commands
    if (curr_data.instruction_type == 'S') {
        Serial.print("S");
    }
    else if (curr_data.instruction_type == 'M') { //if the instruction is a move from point A to point B command then move the motors from A to B
        Serial.println("M");
        in_token = true;
        while (in_token) {
          (!motor1Running && !motor2Running && !motor3Running && !motor4Running) ? steppersOff = true : steppersOff = false;
          if (steppersOff && !checkSteppers) { 
            handleMoveCommand(curr_data.command_start, curr_data.command_end);
            runSteppers(); //SHOULD TAKE DESPOS1-4, AND SHOULD NOT BE GLOBAL VARIABLES || ALSO ADD A FLAG TO SIGNAL WHEN THIS IS COMPLETE AND CORRECT STEPPERS NEEDS TO RUN
          }

          // Motor stuff
          if (count1 >= steps[0]){
            motor1Running = false;
            count1 = 0;
          }
          if (count2 >= steps[1]){
            motor2Running = false;
            count2 = 0;
          }
          if (count3 >= steps[2]){
            motor3Running = false;
            count3 = 0;
          }
          if (count4 >= steps[3]){
            motor4Running = false;
            count4 = 0;
          }
          if (steppersOff && checkSteppers) {
            //Serial.println(steps[0]);
            //correctSteppers();
            checkSteppers = false;
          }
          if (timePassed >= loopTime) {
            updateDCs();
            //print_motor_info(); //For debugging
          }
        }
          //make sure to set desiredAngles after you move in case the next command is not a move command
    }
    else if (curr_data.instruction_type == 'C') {
        Serial.print("C");
    }
    else if (curr_data.instruction_type == 'L') {
        Serial.print("L");
    }
    else if (curr_data.instruction_type == 'P') {
        Serial.print("P");
    }
    else if (curr_data.instruction_type == 'E') {
        Serial.println("E");
        Serial.println("Q");
    }
    curr_data = parseInstructionSet(instruction_set);
  }
  Serial.println();
  return true;
}

//-----------------------PARSERS-----------------------------------------
Instruction parseInstructionSet(char** instruction_set) {
  Instruction result;
  //parse the instruction set and remove the first Instruction
  //returns a COMMAND, INSTRUCTION TYPE, and the truncated INSTRUCTION SET
  char instruction_type;
  result.command_start = NULL;
  size_t i = 0;
  if (*(*instruction_set + i) == '/') {
    i++;
    if (*(*instruction_set + i) == '\0') {
      result.instruction_type = '\0';
      result.command_end = NULL;
      return result;
    }
    instruction_type = *(*instruction_set + i);
    i += 2;
    result.command_start = (*instruction_set + i);
    while (*(*instruction_set + i) != '/') {
      i++;
    }
    result.command_end = (*instruction_set + i);
    *instruction_set = (*instruction_set + i);
  }
  result.instruction_type = instruction_type;
  return result;
}

MoveCommand handleMoveCommand(char* command_start, char* command_end) {
    MoveCommand result;
    //parse the A to B command 
    //Return the STARTING POINT, and an array of SEGMENTED MOVEMENTS

    // Array to hold a group of 6 floats
    float group[6];
    int groupIndex = 0;

    // Get the first token (split by comma)
    char* token = NULL;
    if (!in_token) {
      token = strtok(command_start, ",*");
    }
    else {
      token = strtok(NULL, ",*");
    }

    // Continue parsing until no more tokens are found
    while (token != NULL && token != command_end) {
      
      // Convert the token to a float
      float num = atof(token);
      group[groupIndex] = num;
      groupIndex++;

      // When we’ve collected 6 numbers, process the group
      if (groupIndex == 6) {
        // Example processing: print the group to Serial
        /*
        for (int i = 0; i < 6; i++) {
          Serial.print(group[i]);
          Serial.print(" ");
        }

        Serial.println();
        */ // New line after each group
        desPos1 = group[0];
        desPos2 = group[1];
        desPos3 = group[2];
        desPos4 = group[3];
        desPos5 = group[4];
        desPos6 = group[5];
        groupIndex = 0;   // Reset for the next group
        return result;
      }

      // Get the next token
      token = strtok(NULL, ",*"); // '*' delimits the end of the command
    }
    in_token = false;
    return result;
}

LEDCommand handleLEDCommand(char* command_start, char* command_end){

}

EndCommand handleEndCommand(char* command_start, char* command_end){

}
//---------------------------OTHER FUNCTIONS-------------------------------
void actuateClaw(int command) {
    if (command == 1) {
      //clawServo.write(CORRECT DEGREES FOR OPEN);
    }
    else {
      //clawServo.write(CORRENT DEGREES FOR CLOSED)
    }
}

void setLED(String command) {
    //set the fuckin LED idk bruh to a color index or 0 for off
}

void pause(short command, float desiredAngles) {//milliseconds
    //while (currentMillis < command) {
      //maintainDCMotors(desiredAngles);
    //}
}

void End(char type, short t, float* angles) {
    //Special move command the sends to home end either terminates or continues the program
}

/*
void setZero(float* zeroOffset) {
    //zeroOffset[0] = analogRead(FIRST ENCODER PIN)
    //zeroOffset[1] = analogRead(SECOND ENCODER PIN)    
    //zeroOffset[2] = analogRead(THIRD ENCODER PIN)    
    //zeroOffset[3] = analogRead(FOURTH ENCODER PIN)    
    //zeroOffset[4] = analogRead(FIFTH ENCODER PIN)    
    //zeroOffset[5] = analogRead(SIXTH ENCODER PIN)
}
*/

/*
void maintainDCMotors(desiredAngles) {
  //must mantain the desired position
}
*/

void correctSteppers(){
  Serial.println("called");
  readEncoderStepper(&curPos1, A1, fmod(desPos1*20.0, 360.0));
  theta1 = desPos1 - (((rot1*360.0)/20.0) + curPos1); //find relative angle to move
  theta2 = 0;//desPos2 - (((rot2*360.0)/50.0) + readEncoderStepper(&curPos2, A2)); //DOUBLE CHECK FOR FMOD
  theta3 = 0;//desPos3 - (((rot3*360.0)/20.0) + readEncoderStepper(&curPos3, A3));
  theta4 = 0;//desPos4 - (((rot4*360.0)/20.0) + readEncoderStepper(&curPos4, A4));
  if (abs(theta1) > 1.8) {
    digitalWrite(DIR_PIN1, (theta1 >= 0) ? HIGH : LOW);
    steps[0] = short((abs(theta1)*20.0)/1.8);   
  } else steps[0] = 0;
  if (abs(theta2) > 1.8) {
    digitalWrite(DIR_PIN2, (theta2 >= 0) ? HIGH : LOW);
    steps[1] = short((abs(theta2)*20.0)/1.8);   
  } else steps[1] = 0;
  if (abs(theta3) > 1.8) {
    digitalWrite(DIR_PIN3, (theta3 >= 0) ? HIGH : LOW);
    steps[2] = short((abs(theta3)*20.0)/1.8);   
  } else steps[2] = 0;
  if (abs(theta4) > 1.8) {
    digitalWrite(DIR_PIN4, (theta4 >= 0) ? HIGH : LOW);
    steps[3] = short((abs(theta4)*20.0)/1.8);   
  } else steps[3] = 0;

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
        if (steps[i] != 0){
          frequencies[i] = steps[i]/timescale;
        } else frequencies[i] = 1000;
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
  Serial.println("Motors moving to correction.");
  Serial.println(steps[0]);
  checkSteppers = false;
}

void runSteppers(){
  readEncoderStepper(&curPos1, A1, fmod(previousDesPos1*20, 360.0)/20.0);
  //Serial.println(fmod(previousDesPos1*20.0, 360.0));
  previousDesPos1 = desPos1;
  theta1 = desPos1 - (rot1*(360/20) + curPos1); //find geared angle to move
  Serial.println(curPos1);
  Serial.println(theta1);
  theta2 = desPos2 - (((rot2*360.0)/50.0) + readEncoderStepper(&curPos2, A2, fmod(desPos2*50.0, 360.0))); 
  theta3 = desPos3 - (((rot3*360.0)/20.0) + readEncoderStepper(&curPos3, A3, fmod(desPos3*20.0, 360.0)));
  theta4 = desPos4 - (((rot4*360.0)/20.0) + readEncoderStepper(&curPos4, A4, fmod(desPos4*20.0, 360.0)));

  if (theta1 > 180.0) theta1 -= 360.0;
  if (theta1 < -180.0) theta1 += 360.0;

  Serial.println(rot1);
  rot2 += floor((theta2*50.0)/360.0); //gives positive rots IS WRONG
  rot3 += floor((theta3*20.0)/360.0);
  rot4 += floor((theta4*20.0)/360.0);

  if (!((rot1*(360/20) + curPos1 + theta1) < motor1UB && (rot1*(360/20) + curPos1 + theta1) > motor1LB)) { //if outside the bound change directions to the correct point 
    Serial.println("hello ?");
    if (theta1 > 0){
      theta1 -= 360.0; 
    }
    if (theta1 < 0) {
      theta1 += 360.0;
    }
  }
  rot1 = floor((rot1*(360/20) + curPos1 + theta1)*(20/360)); //find the next rotation value

  if ((rot2 + fmod((theta2*50.0)/360.0, 360.0)) < motor2UB && (rot2 + fmod((theta2*50.0)/360.0, 360.0)) > motor2LB) { //if inside the bounds after the move, use the shortest distance, otherwise proceed as normal
    if (theta2 >  180) {
      rot2 -= floor((theta2*50.0)/360.0);
      theta2 -= 360.0; 
      rot2 += floor((theta2*50.0)/360.0);
    }
    if (theta2 < -180) {
      rot2 -= floor((theta2*50.0)/360.0);
      theta2 += 360.0;
      rot2 += floor((theta2*50.0)/360.0);
    }
  } else { 
    if (theta2 > 0){
      rot2 -= floor((theta2*50.0)/360.0);
      theta2 -= 360.0; 
      rot2 += floor((theta2*50.0)/360.0);
    }
    if (theta2 < 0) {
      rot2 -= floor((theta2*50.0)/360.0);
      theta2 += 360.0;
      rot2 += floor((theta2*50.0)/360.0);
    }
  }

  if ((rot3 + fmod((theta3*20.0)/360.0, 360.0)) < motor3UB && (rot3 + fmod((theta3*20.0)/360.0, 360.0)) > motor3LB) { //if inside the bounds after the move, use the shortest distance, otherwise proceed as normal
    if (theta3 >  180) {
      rot3 -= floor((theta3*20.0)/360.0);
      theta3 -= 360.0; 
      rot3 += floor((theta3*20.0)/360.0);
    }
    if (theta3 < -180) {
      rot3 -= floor((theta3*20.0)/360.0);
      theta3 += 360.0;
      rot3 += floor((theta3*20.0)/360.0);
    }
  } else { //if outside the bound change directions to the correct point 
    if (theta3 > 0){
      rot3 -= floor((theta3*20.0)/360.0);
      theta3 -= 360.0; 
      rot3 += floor((theta3*20.0)/360.0);
    }
    if (theta3 < 0) {
      rot3 -= floor((theta3*20.0)/360.0);
      theta3 += 360.0;
      rot3 += floor((theta3*20.0)/360.0);
    }
  }

  if ((rot4 + fmod((theta4*20.0)/360.0, 360.0)) < motor4UB && (rot4 + fmod((theta4*20.0)/360.0, 360.0)) > motor4LB) { //if inside the bounds after the move, use the shortest distance, otherwise proceed as normal
    if (theta4 >  180) {
      rot4 -= floor((theta4*20.0)/360.0);
      theta4 -= 360.0; 
      rot4 += floor((theta4*20.0)/360.0);
    }
    if (theta4 < -180) {
      rot4 -= floor((theta4*20.0)/360.0);
      theta4 += 360.0;
      rot4 += floor((theta4*20.0)/360.0);
    }
  } else { //if outside the bound change directions to the correct point 
    if (theta4 > 0){
      rot4 -= floor((theta4*20.0)/360.0);
      theta4 -= 360.0; 
      rot4 += floor((theta4*20.0)/360.0);
    }
    if (theta4 < 0) {
      rot4 -= floor((theta4*20.0)/360.0);
      theta4 += 360.0;
      rot4 += floor((theta4*20.0)/360.0);
    }
  }

  digitalWrite(DIR_PIN1, (theta1 >= 0) ? HIGH : LOW);
  digitalWrite(DIR_PIN2, (theta2 >= 0) ? HIGH : LOW);
  digitalWrite(DIR_PIN3, (theta3 >= 0) ? HIGH : LOW);
  digitalWrite(DIR_PIN4, (theta4 >= 0) ? HIGH : LOW);
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
        if (steps[i] != 0) {
          frequencies[i] = steps[i]/timescale;
        } else frequencies[i] = 1000;
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
  Serial.println(steps[0]);
  Serial.println("Motors started.");
  checkSteppers = true;
}

void updateDCs(){
  float newPos5;
  float newPos6;
  readEncoderDC(newPos5, newPos6);
  if(newPos5 != 0){
    passZero5 = ((newPos5>=0 != currPos5>=0) ? true : false); //If we switched signs, passZero is true
  }
  if(newPos6 != 0){
    passZero6 = ((newPos6>=0 != currPos6>=0) ? true : false);
  }
  
  if(passZero5){
    (newPos5>=0) ? (rot5 -= 1) : (rot5 += 1);
  }
  if(passZero6){
    (newPos6>=0) ? (rot6 -= 1) : (rot6 += 1);
  }
  currPos5 = newPos5;
  currPos6 = newPos6;
  motor5Running = true; //End conditions
  motor6Running = true;
  error5 = desPos5 - currPos5; //position error
  if (error5 > 180) error5 -= 360;  // Ensure shortest path
  if (error5 < -180) error5 += 360;
  if(error5>0){ //If we're rotating CW
    if(currPos5+error5 >= 360){ //If we pass zero
      if(rot5>0) error5 -= 360; //If passing zero is not ok, make the error the other way
    }
  }else{ //If we're rotating CCW
    if(currPos5+error5 <= 0){ //If we pass zero
      if(rot5<-1) error5 += 360; //If passing zero is not ok, make the error the other way
    }
  }

  error6 = desPos6-currPos6; //position error
  if (error6 > 180) error6 -= 360;  // Ensure shortest path
  if (error6 < -180) error6 += 360;
  if(error6>0){ //If we're rotating CW
    if(currPos6+error6 >= 360){ //If we pass zero
      if(rot6>0) error6 -= 360; //If passing zero is not ok, make the error the other way
    }
  }else{ //If we're rotating CCW
    if(currPos6+error6 <= 0){ //If we pass zero
      if(rot6<-1) error6 += 360; //If passing zero is not ok, make the error the other way
    }
  }
  //End conditions
  if (abs(error5) < 5) {
    digitalWrite(IN1, 1);
    digitalWrite(IN2, 1);
    currPWM5 = 0;
    analogWrite(pwmPin5, abs(currPWM5));
    motor5Running = false;
  }
  if (abs(error6) < 10) {
    digitalWrite(IN3, 1);
    digitalWrite(IN4, 1);
    currPWM6 = 0;
    analogWrite(pwmPin6, abs(currPWM6)); //CHECK
    motor6Running = false;
  }
  
  //Only adjust motor 5 if it hasn't hit the target yet
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

  //Only adjust motor 2 if it hasn't hit the target yet
  if(motor6Running){
    currPWM6 = Ki6*error6;
    if (currPWM6 >= 255) {currPWM6 = 255;}
    if (currPWM6 <= -255) {currPWM6 = -255;}
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

int calcOCRA(float freq, int prescale){
  float OCRA = ((clockFreq/(freq*prescale)) - 1);
  if(OCRA > 65535){
    OCRA = 65535;
  }
  return OCRA;
}

float readEncoderStepper(float* currentPos, int analogPin, float guess){
  switch (analogPin) {
      case A1:
        *currentPos = ((((analogRead(analogPin))/1023.0)*(5/3.3)*(360.0))/20.0) - encZeroes[0]; //SIGNED VALUE (AND IT SHOULD BE)
        float checkVal = (abs(*currentPos - guess)*20.0); //MAKE SURE THAT currentPos AND guess are both between 0 and 18 when checking them, BUT THEY MUST RETURN TO THEIR ACTUAL VALUE AFTER THIS
        //if (*currentPos < 0) float temp = *currentPos + 18.00; //
        if (checkVal < 1.8 || checkVal > 358.2) *currentPos = guess; //compare TEMP to guess and then edit current pos
        break;
      case A2:
        *currentPos = ((((analogRead(analogPin))/1023.0)*(5/3.3)*(360.0))/50.0) - encZeroes[1]; 
        if ((abs(*currentPos - guess)*50.0))
        break;
      case A3:
        *currentPos = ((((analogRead(analogPin))/1023.0)*(5/3.3)*(360.0))/20.0) - encZeroes[2];
        if ((abs(*currentPos - guess)*20.0))
        break;
      case A4:
        *currentPos = ((((analogRead(analogPin))/1023.0)*(5/3.3)*(360.0))/20.0) - encZeroes[3];
        if ((abs(*currentPos - guess)*20.0))
        break;  
  }
}

void print_motor_info(){
  //print current position, desired position, error, and pwm value
  Serial.println("currPos desPos error pwm");
  Serial.println(String(currPos5) + " " + String(desPos5) + " " + String(error5) + " " + String(currPWM5));
  Serial.println(String(currPos6) + " " + String(desPos6) + " " + String(error6) + " " + String (currPWM6));
}

float readEncoderDC(float &num1, float &num2){
  int raw_value = analogRead(encPin5);
  num1 = (raw_value/1023.0)*(5/3.3)*(360.0);
  int raw_value2 = analogRead(encPin6);
  num2 = (raw_value2/1023.0)*(5/3.3)*(360.0);
}