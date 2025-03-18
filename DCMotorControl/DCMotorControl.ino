#define MOTORPOS 10
#define MOTORNEG 9
#define ENCODEVCC 11
#define PHASEA 12
#define PHASEB 13

void setup() {
  // put your setup code here, to run once:
  pinMode(MOTORPOS, OUTPUT);
  pinMode(MOTORNEG, OUTPUT);
  pinMode(ENCODEVCC, INPUT);
  pinMode(PHASEA, INPUT);
  pinMode(PHASEB, INPUT);
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  int val = analogRead(PHASEA);
  analogWrite(MOTORNEG, 255);//cw
  analogWrite(MOTORPOS, 0);

  delay(5000);

  analogWrite(MOTORNEG, 0);//ccw
  analogWrite(MOTORPOS, 255);

  delay(5000);

}


float distanceToGoal(){
  float val = digitalRead(ENCODEVCC);
  return val;
}


void receive(){


}