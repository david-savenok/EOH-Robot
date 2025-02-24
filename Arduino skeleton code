//command is a string filled with a certain command
//instructionType is a string explaining the type of instruction
//acceptablePercentError is the angle error percent that is acceptable
void setup() {
  // put your setup code here, to run once:
  String instructionSet = "";
  String instructionType = "";
  float acceptablePercentError = 0.01;//FIGURE THIS OUT EXPERIMENTALLY TUNE
  //pins and stuff
}

void loop() {
  // put your main code here, to run repeatedly:
  while (checkSerial() == empty){
    delay(1000);
  }
  instructionSet = read the serial //!!!! do this idk how
  command, instructionType, instructionSet = parseInstructionSet(instructionSet); 
  if (instructionType == "start_command"){ //Check if the instruction type of the first was a correct starting command
    while (instructionSet != ""){ //while the instruction set is not empty, continue to execute commands
      command, instructionType, instructionSet = parseInstructionSet(instructionSet); //grab the newest instructions
      if instructionType == "AtoB"{ //if the instruction is a move from point A to point B command then move the motors from A to B
        startingPoint, segmentedMovements = parseAtoB_Command(command);
        if !moveMotors(startingPoint, segmentedMovements);
          //THROW AN ERROR something is wrong
      }
      else if instructionType == "actuateClaw"{//check for all instruction types
        actuateClaw(toInt(command));//toInt might not work on this
      }
      else if instructionType == "LEDColor"{ //check  all
        setLED(array command);//fuck arrays in arduino bro
      }
      else if instructionType == "pause"{ //check  all
        pause(float command);//milliseconds
      }
      else if instructionType == "STOP"{ //check  all
        STOP();//no data just STOP 
      }
    }
  }
  else {
    clearSerial();
  }
}

struct STRUCTURE {
  array COMMAND = [];//This needs to be fixed
  String INSTRUCTION TYPE = "";
  String INSTRUCTION SET = "";
}

STRUCTURE parseInstructionSet(String instructionSet){
  STRUCTURE result;
  //parse the instruction set and remove the first datablock
  //returns a COMMAND, INSTRUCTION TYPE, and the truncated INSTRUCTION SET
  return result;
}

STRUCTURE parseAtoB_Command(String command){
  STRUCTURE result;
  //parse the A to B command 
  //Return the STARTING POINT, and an array of SEGMENTED MOVEMENTS
  return result;
}

bool moveMotors(array1D startingPoint, array2D segmentedMovements){
  bool success
  for ...{
    //send motor commands
  }
  //segmentedMovements[-1] == analogRead(encoder1) Obviously this wont work this easy, but check to see if we hit the mark. If we did return true
  //segmentedMovements[-1] == analogRead(encoder1)
  //segmentedMovements[-1] == analogRead(encoder1)
  //segmentedMovements[-1] == analogRead(encoder1)
  //segmentedMovements[-1] == analogRead(encoder1)
  //segmentedMovements[-1] == analogRead(encoder1)
  return success
}

void actuateClaw(int command){
  if command == 1{
    //set servo to open
  }
  else{
    //set servo to close
  }
}
void setLED(command){
  //set the fuckin LED idk bruh to a color index or 0 for off
}

void pause(float command){//milliseconds
  //make sure to handle the DC motors :(
}

void STOP(){
  //STOP IN THE NAME OF THE LAW
}
