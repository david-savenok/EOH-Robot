#define STEP_PIN1 38  // Pin connected to STEP on A4988
#define DIR_PIN1 3    // Pin connected to DIR on A4988
#define STEP_PIN2 49  // Pin connected to STEP on A4988
#define DIR_PIN2 53    // Pin connected to DIR on A4988

bool motor1Running = false;  // State to track HIGH/LOW for step pulse
bool motor2Running = false;
void setup() {
  // Set STEP and DIR pins as outputs
  pinMode(STEP_PIN1, OUTPUT);
  pinMode(DIR_PIN1, OUTPUT);
  pinMode(STEP_PIN2, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);
  
  // Set the initial direction (rotate in one direction)
  digitalWrite(DIR_PIN1, LOW);  // Set direction pin (can be HIGH or LOW)
  digitalWrite(DIR_PIN2, LOW);
  // Start serial communication for debugging
  Serial.begin(115200);

  // Disable interrupts while configuring
  noInterrupts();

  // Reset Timer1
  TCCR1A = 0;  // Normal operation, no PWM
  TCCR1B = 0;  // Reset Timer1 settings
  
  // Set Timer1 in CTC (Clear Timer on Compare Match) mode
  TCCR1B |= (1 << WGM12);  // CTC Mode
  TCCR1B |= (1 << CS11);   // Prescaler = 8 (2 MHz timer frequency)

  // Set OCR1A for the 500 µs HIGH pulse (1000 timer counts)
  OCR1A = 2000;  // 500 µs HIGH
  OCR1B = 1000;

  // Enable Timer1 Compare Match A and B interrupts
  TIMSK1 |= (1 << OCIE1A) | (1<< OCIE1B);
  
  // Reset Timer3
  TCCR3A = 0;  // Normal operation, no PWM
  TCCR3B = 0;  // Reset Timer1 settings
  
  // Set Timer3 in CTC (Clear Timer on Compare Match) mode
  TCCR3B |= (1 << WGM32);  // CTC Mode
  TCCR3B |= (1 << CS31);   // Prescaler = 8 (2 MHz timer frequency)

  // Set OCR1A for the 500 µs HIGH pulse (1000 timer counts)
  OCR3A = 2000;  // 500 µs HIGH
  OCR3B = 1000;

  // Enable Timer3 Compare Match A and B interrupts
  TIMSK3 |= (1 << OCIE3A) | (1<< OCIE3B);
  // Enable global interrupts
  interrupts();
}

ISR(TIMER1_COMPA_vect) {
  if (motor1Running) {
    digitalWrite(STEP_PIN1, HIGH);  // Generate step pulse
  }
}

ISR(TIMER1_COMPB_vect) {
  if (motor1Running) {
    digitalWrite(STEP_PIN1, LOW);   // End step pulse
  }
}

ISR(TIMER3_COMPA_vect) {
  if (motor2Running) {
    digitalWrite(STEP_PIN2, HIGH);  // Generate step pulse
  }
}

ISR(TIMER3_COMPB_vect) {
  if (motor2Running) {
    digitalWrite(STEP_PIN2, LOW);   // End step pulse
  }
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');  // Read input until newline
    command.trim();  // Remove any extra spaces or newline characters
    if (command == "start1") {
      // Start motor 1
      motor1Running = true;
      Serial.println("Motor 1 started.");
    } 
    else if (command == "stop1") {
      // Stop motor 1
      motor1Running = false;
      digitalWrite(STEP_PIN1, LOW);  // Ensure the pin is LOW
      Serial.println("Motor 1 stopped.");
    }else if (command == "start2") {
      // Stop motor 2
      motor2Running = true;
      Serial.println("Motor 2 started.");
    }else if (command == "stop2") {
      // Stop motor 2
      motor2Running = false;
      digitalWrite(STEP_PIN2, LOW);  // Ensure the pin is LOW
      Serial.println("Motor 2 stopped.");
    }else if(command=="11"){
      digitalWrite(DIR_PIN1, LOW);
      OCR1A = 4000;
    }else if(command=="12"){
      digitalWrite(DIR_PIN1, LOW);
      OCR1A = 2000;
    }else if(command=="13"){
      digitalWrite(DIR_PIN1, HIGH);
      OCR1A = 4000;
    }else if(command=="14"){
      digitalWrite(DIR_PIN1, HIGH);
      OCR1A = 2000;
    }else if(command=="21"){
      digitalWrite(DIR_PIN2, LOW);
      OCR3A = 4000;
    }else if(command=="22"){
      digitalWrite(DIR_PIN2, LOW);
      OCR3A = 2000;
    }else if(command=="23"){
      digitalWrite(DIR_PIN2, HIGH);
      OCR3A = 4000;
    }else if(command=="24"){
      digitalWrite(DIR_PIN2, HIGH);
      OCR3A = 2000;
    }else{
      Serial.println("Please enter start1, stop1, start2, stop2, 11, 12, 13, 14, 21, 22, 23, or 24");
    }
  }
  
}
