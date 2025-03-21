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
String instruction_type = "";
Servo clawServo;
float acceptable_percent_error = 0.01; //FIGURE THIS OUT EXPERIMENTALLY TUNE
unsigned long currentMillis = 0;
//-------------------LITERALLY WHAT IS THIS----------------------------
using namespace std;

//-------------------STRUCTURES----------------------------------------
typedef struct Instruction {
    String command; //This needs to be fixed
    char instruction_type;
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

//------------------------SETUP----------------------------------------
//bool moveMotors(vector<float> startingPoint, vector<vector<float>> segmentedMovements);

void setup() {
    clawServo.attach(CHOOSE A PIN PLEASE);
    Serial.begin(9600);
    //pins and stuff
}

//------------------------MAIN LOOP------------------------------------
void loop() {
    // put your main code here, to run repeatedly:
    while (Serial.available() == 0) {
      delay(1000);
    }
    String instruction_set = Serial.readString();//!!!! do this idk how
    Instruction curr_data = parseInstructionSet(&instruction_set); 
    if (curr_data.instruction_type == 'S') { //Check if the instruction type of the first was a correct starting command
        // Initialization based on start commmand 
        // Put to zero or whatnot 
        if (curr_data.command == 'H') {
          setZero(zeroOffset); //reads the analog pins and sets the 0 position in terms of the encoder position.
        }
        while (instruction_set != "/") { //while the instruction set is not empty, continue to execute commands
            curr_data = parseInstructionSet(&instruction_set); //grab the newest instructions
            if (curr_data.instruction_type == "M") { //if the instruction is a move from point A to point B command then move the motors from A to B
                //startingPoint, segmentedMovements = parseAtoB_Command(command);
                //if !moveMotors(startingPoint, segmentedMovements);
                //THROW AN ERROR something is wrong
                Serial.println("M");
                //make sure to set desiredAngles after you move in case the next command is not a move command
            }
            else if (curr_data.instruction_type == "C") {//check for all instruction types
                actuateClaw(curr_data.command.toInt());
                Serial.println("C");
            }
            else if (curr_data.instruction_type == "L") { //check  all
                //setLED(array command);//fuck arrays in arduino bro
                Serial.println("L");
            }
            else if (curr_data.instruction_type == "P") { //check  all
                pause(curr_data.command, desiredAngles);//seconds
                Serial.println("P");
            }
            else if (curr_data.instruction_type == "E") { //check  all
                EndCommand parsedEnd = parseEnd(curr_data.command);
                End(parsedEnd.type, parsedEnd.t, parsedEnd.angles);
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
Instruction parseInstructionSet(String* instruction_set) {
    Instruction result;
    //parse the instruction set and remove the first Instruction
    //returns a COMMAND, INSTRUCTION TYPE, and the truncated INSTRUCTION SET
    char instruction_type;
    String command = "";
    size_t i = 0;
    if ((*instruction_set)[i] == '/') {
        i++;
        instruction_type = (*instruction_set)[i];
        i += 2;
        while ((*instruction_set)[i] != '/') {
            command += (*instruction_set)[i];
            i++;
        }
        *instruction_set = instruction_set->substring(i);
    }

    result.command = command;
    result.instruction_type = instruction_type;

    return result;
}

MoveCommand parseMoveCommand(String command) {
    MoveCommand result;
    //parse the A to B command 
    //Return the STARTING POINT, and an array of SEGMENTED MOVEMENTS
    return result;
}

LEDCommand parseLEDCommand(String command){
  notImplemented();
}
EndCommand parseEndCommand(String command){
  notImplemented();
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
      clawServo.write(CORRECT DEGREES FOR OPEN);
    }
    else {
      clawServo.write(CORRENT DEGREES FOR CLOSED)
    }
}

void setLED(String command) {
    //set the fuckin LED idk bruh to a color index or 0 for off
}

void pause(short command, float desiredAngles) {//milliseconds
    while (currentMillis < command) {
      maintainDCMotors(desiredAngles);
    }
}

void End(char type, short t, float* angles) {
    //Special move command the sends to home end either terminates or continues the program
}

void setZero(float* zeroOffset) {
    zeroOffset[0] = analogRead(FIRST ENCODER PIN)
    zeroOffset[1] = analogRead(SECOND ENCODER PIN)    
    zeroOffset[2] = analogRead(THIRD ENCODER PIN)    
    zeroOffset[3] = analogRead(FOURTH ENCODER PIN)    
    zeroOffset[4] = analogRead(FIFTH ENCODER PIN)    
    zeroOffset[5] = analogRead(SIXTH ENCODER PIN)
}

void maintainDCMotors(desiredAngles) {
  //must mantain the desired position
}









