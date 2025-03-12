#define STEP_PIN1 38  // Pin connected to STEP on A4988
#define DIR_PIN1 3    // Pin connected to DIR on A4988
#define STEP_PIN2 49  // Pin connected to STEP on A4988
#define DIR_PIN2 53    // Pin connected to DIR on A4988

bool motor1Running = false;  // State to track HIGH/LOW for step pulse
bool motor2Running = false;

int theta1 = 1800; //degrees
int theta2 = 18; //degrees

const unsigned long clockFreq = 16000000; //Hz

int steps1;
int steps2;

int count1 = 0;
int count2 = 0;

int calcOCRA(float freq, int prescale){
  float OCRA = ((clockFreq/(freq*prescale)) - 1);
  if(OCRA > 65535){
    OCRA = 65535;
  }
  return OCRA;
}

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
  TCCR1B |= (1 << CS10) | (1 << CS11);   // Prescaler = 8 (16 MHz timer frequency, 2MHz effective timer frequency)

  steps1 = int(theta1/1.8);
  float freq1 = 1000; //Hz
  // Set OCR1A for the 500 µs HIGH pulse (1000 timer counts)
  OCR1A = calcOCRA(freq1, 32);
  OCR1B = 499;
  // Enable Timer1 Compare Match A and B interrupts
  TIMSK1 |= (1 << OCIE1A) | (1<< OCIE1B);
  
  // Reset Timer3
  TCCR3A = 0;  // Normal operation, no PWM
  TCCR3B = 0;  // Reset Timer1 settings
  
  // Set Timer3 in CTC (Clear Timer on Compare Match) mode
  TCCR3B |= (1 << WGM32);  // CTC Mode
  TCCR3B |= (1<<CS30) | (1 << CS31);   // Prescaler = 8 (16 MHz timer frequency, 2MHz effective timer frequency)

  steps2 = int(theta2/1.8);
  float timescale = steps1*(1.0/freq1);
  float freq2 = steps2/timescale;
  // Set OCR1A for the 500 µs HIGH pulse (1000 timer counts)
  
  OCR3A = calcOCRA(freq2, 32);  // 500 µs HIGH
  OCR3B = 499;
  // Enable Timer3 Compare Match A and B interrupts
  TIMSK3 |= (1 << OCIE3A) | (1<< OCIE3B);
  // Enable global interrupts
  interrupts();
}

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

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');  // Read input until newline
    command.trim();  // Remove any extra spaces or newline characters
    if (command == "start") {
      // Start motor 1
      motor1Running = true;
      motor2Running = true;
      Serial.println("Motors started.");
    }
  }if(count1 >= steps1){
    motor1Running = false;
  }if(count2 >= steps2){
    motor2Running = false;
  }
  
}
