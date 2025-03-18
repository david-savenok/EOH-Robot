// Arduino Sketch: LightControl.ino
const int light1 = 3;  // Pin 1 for light 1 (TX pin, used as output here)
const int light2 = 4;  // Pin 2 for light 2

void setup() {
  // Set pins as outputs
  pinMode(light1, OUTPUT);
  pinMode(light2, OUTPUT);
  
  // Initialize both lights off
  digitalWrite(light1, LOW);
  digitalWrite(light2, LOW);
  
  // Start serial communication at 9600 baud
  Serial.begin(9600);
}

void loop() {
  // Check if serial data is available
  while (Serial.available() > 0) {
    char command = Serial.read();  // Read the incoming character
    
    // If 'g' is received, turn on light 1 and off light 2
    Serial.print(command);
    if (command == 'g') {
      digitalWrite(light1, HIGH);
      digitalWrite(light2, LOW);
    }
    // If 'r' is received, turn on light 2 and off light 1
    else if (command == 'r') {
      digitalWrite(light1, LOW);
      digitalWrite(light2, HIGH);
    }
  }
}