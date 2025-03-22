//command is a string filled with a certain command
//instructionType is a string explaining the type of instruction
//acceptablePercentError is the angle error percent that is acceptable
//-------------------INCLUDE STATEMENTS--------------------------------
#include <stdio.h>
#include <stdlib.h>
#include <Servo.h>
//-------------------VARIABLE INSTANTIATIONS---------------------------
float zeroOffset[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
float desiredAngles[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
Servo clawServo;
float acceptable_percent_error = 0.01; //FIGURE THIS OUT EXPERIMENTALLY TUNE
unsigned long currentMillis;
unsigned long startMillis;
//-------------------LITERALLY WHAT IS THIS----------------------------
using namespace std;

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
    short t;
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

Buffer* bufA = malloc(sizeof(Buffer));
Buffer* bufB = malloc(sizeof(Buffer));
Buffer* to_fill = bufA;
Buffer* to_use = NULL;
bool signal_sent = false;
bool start_comms;

//------------------------SETUP----------------------------------------
//bool moveMotors(vector<float> startingPoint, vector<vector<float>> segmentedMovements);

void setup() {
    //clawServo.attach(CHOOSE A PIN PLEASE);
    Serial.begin(9600);
    //Serial1.begin(9600);
    bufA->name = 'A';
    bufB->name = 'B';
    bufA->is_full = false;
    bufB->is_full = false;
    bufA->buf = malloc(1500);
    bufB->buf = malloc(1500);
    start_comms = true;
    //pins and stuff
}

//------------------------MAIN LOOP------------------------------------
void loop() {
    // put your main code here, to run repeatedly:
    /*
    String instruction_set = "";
    while (Serial.available() > 0) {
        instruction_set += Serial.readString();   
        Serial.println(instruction_set);
        Serial.flush();
    }
    bool success;
    if (instruction_set.endsWith("Q")) {
        Serial.print(instruction_set);
        //instruction_set.remove(instruction_set.lastIndexOf("Q"));
        //success = handleInstructionSet(&instruction_set);
    }
    */

  //Serial.println(bufA->name);
  //Serial.println(bufB->name);
  if (start_comms) {
    Serial.println(strlen(bufA->buf));
    bufA->is_full = false;
    Serial.println("A");
    start_comms = false;
  }
  if (bufA->is_full) {
    //bool success = handleInstructionSet(&(bufA->buf));
    bool success = true;
    if (success) {
      Serial.println(strlen(bufA->buf));
      bufA->is_full = false;
      Serial.println("A");
    }
    
    
  }
  if (bufB->is_full) {
    //bool success = handleInstructionSet(&(bufB->buf));
    bool success = true;
    if (success) {
      Serial.println(strlen(bufB->buf));
      bufB->is_full = false;
      Serial.println("A");
    }
  }
  /*
  if (!to_fill->is_full && !signal_sent) {
    Serial.println("A");
    signal_sent = true;
  }
  */
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
        //Serial.println(strlen(to_fill->buf));
        to_fill = bufB;
        //Serial.print('A');
        
      }
      else if (to_fill->name == 'B') {
        to_fill->is_full = true;
        //Serial.println(strlen(to_fill->buf));
        to_fill = bufA;
        //Serial.print('A');
      }     
    }
  } 
}


bool handleInstructionSet(char** instruction_set) {
  //String instruction_set = Serial.readString();//!!!! do this idk how
  Instruction curr_data = parseInstructionSet(instruction_set); 
  //Serial.println(curr_data.instruction_type);
  if (**instruction_set != "") {
    //Serial.println(*instruction_set);
    //Serial.println(curr_data.command);
    //return true;
  }
  //Serial.print("wowza\n");
  if (curr_data.instruction_type == 'S') { //Check if the instruction type of the first was a correct starting command
      // Initialization based on start commmand 
      // Put to zero or whatnot 
      //if (curr_data.command == 'H') {
        //setZero(zeroOffset); //reads the analog pins and sets the 0 position in terms of the encoder position.
      //}
      //Serial.println(curr_data.command_start);
      //Serial.println(**instruction_set);
      while (**instruction_set != "/") { //while the instruction set is not empty, continue to execute commands
          curr_data = parseInstructionSet(instruction_set); 
          //curr_data = parseInstructionSet(instruction_set);
          //Serial.println(*instruction_set);
          //Serial.println(curr_data.instruction_type);
          if (curr_data.instruction_type == 'M') { //if the instruction is a move from point A to point B command then move the motors from A to B
              //Serial.println(curr_data.command_start);
              //startingPoint, segmentedMovements = parseAtoB_Command(command);
              //if !moveMotors(startingPoint, segmentedMovements);
              //THROW AN ERROR something is wrong
              //Serial.println(curr_data.command_start);
              //handleMoveCommand(curr_data.command_start, curr_data.command_end);
              Serial.println("M");
              //make sure to set desiredAngles after you move in case the next command is not a move command
          }
          else if (curr_data.instruction_type == 'C') {//check for all instruction types
              //actuateClaw(curr_data.command.toInt());
              Serial.println("C");
          }
          else if (curr_data.instruction_type == 'L') { //check  all
              //setLED(array command);//fuck arrays in arduino bro
              Serial.println("L");
          }
          else if (curr_data.instruction_type == 'P') { //check  all
              //pause(curr_data.command, desiredAngles);//seconds
              Serial.println("P");
          }
          else if (curr_data.instruction_type == 'E') { //check  all
              //EndCommand parsedEnd = parseEnd(curr_data.command);
              //End(parsedEnd.type, parsedEnd.t, parsedEnd.angles);
              Serial.println("E");
              break;
          }
      }
  }
  else {
      //clearSerial();
  }
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

    // Array to hold a group of 7 floats
    float group[6];
    int groupIndex = 0;

    // Get the first token (split by comma)
    char *token = strtok(command_start, ",/");

    // Continue parsing until no more tokens are found
    while (token != NULL) {
      // Convert the token to a float
      float num = atof(token);
      group[groupIndex] = num;
      groupIndex++;

      // When we’ve collected 7 numbers, process the group
      if (groupIndex == 6) {
        // Example processing: print the group to Serial
        for (int i = 0; i < 6; i++) {
          Serial.print(group[i]);
          Serial.print(" ");
        }
        Serial.println(); // New line after each group
        groupIndex = 0;   // Reset for the next group
      }

      // Get the next token
      token = strtok(NULL, ",");
    }
    return result;
}

LEDCommand parseLEDCommand(String command){

}
EndCommand parseEndCommand(String command){

}
//---------------------------OTHER FUNCTIONS-------------------------------

/*
bool moveMotors(vector<float> startingPoint, vector<vector<float>> segmentedMovements) {
    bool success = false;
    //for ...{
      //send motor commands
    //}
    //segmentedMovements[-1] == analogRead(encoder1) Obviously this wont work this easy, but check to see if we hit the mark. If we did return true
    //segmentedMovements[-1] == analogRead(encoder1)
    //segmentedMovements[-1] == analogRead(encoder1)
    //segmentedMovements[-1] == analogRead(encoder1)
    //segmentedMovements[-1] == analogRead(encoder1)
    //segmentedMovements[-1] == analogRead(encoder1)
    return success;
}
*/

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






