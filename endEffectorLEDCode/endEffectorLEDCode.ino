#define RED 2
#define GREEN 4
#define BLUE 6

void setup() {
  // put your setup code here, to run once:
  pinMode(RED, OUTPUT);
  pinMode(GREEN, OUTPUT);
  pinMode(BLUE, OUTPUT);
  Serial.begin(9600);
  
}

int redValue = 0;
int greenValue = 0;
int blueValue = 0;

/*
int targetred = 50;
int targetgreen = 255;
int targetblue = 50;
*/


int arrTarRed[6] = {50, 225, 255, 255, 0, 0};
int arrTarGreen[6] = {255, 0, 255, 255, 0, 0};
int arrTarBlue[6] = {50, 0, 0, 0, 0, 255};
int times[6] = {5, 10, 30, 30, 25, 50};
int position = 0;

void loop() {
  // put your main code here, to run repeatedly:
  if (position <= 5){
    fadeToColorsInRow();
  }
  else{
    exit(0);
  }
  Serial.println(redValue);
  Serial.println(greenValue);
  Serial.println(blueValue);

}

/*void fadeToColor(){
  //working on it
  if (redValue != targetred) {
    redValue += (targetred > redValue ) ? 1 : -1;
  }

  if (greenValue != targetgreen) {
    greenValue += (targetgreen > greenValue ) ? 1 : -1;
  }

  if (blueValue != targetblue) {
    blueValue += (targetblue > blueValue ) ? 1 : -1;
  }

  analogWrite(RED, redValue);
  analogWrite(GREEN, greenValue);
  analogWrite(BLUE, blueValue);

  delay(5);
}
*/

void fadeToColorsInRow(){
  //working on it
  if (redValue == arrTarRed[position] && greenValue == arrTarGreen[position] && blueValue == arrTarBlue[position]){
    position +=1;
  }

  if (redValue != arrTarRed[position]) {
    redValue += (arrTarRed[position] > redValue ) ? 1 : -1;
  }

  if (greenValue != arrTarGreen[position]) {
    greenValue += (arrTarGreen[position] > greenValue ) ? 1 : -1;
  }

  if (blueValue != arrTarBlue[position]) {
    blueValue += (arrTarBlue[position] > blueValue ) ? 1 : -1;
  }

  

  analogWrite(RED, redValue);
  analogWrite(GREEN, greenValue);
  analogWrite(BLUE, blueValue);

  delay(times[position]);
}
