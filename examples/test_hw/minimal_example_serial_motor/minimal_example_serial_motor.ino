#define STEP_PIN 3    // Pin connected to stepper motor step input
#define DIR_PIN 4     // Pin connected to stepper motor direction input

const int delayPerStepMicrosec = 2500; // Delay between steps
int reference_distance = 0;            // Distance to move in steps

void setup() {
  pinMode(STEP_PIN, OUTPUT); // Stepper motor step pin
  pinMode(DIR_PIN, OUTPUT);  // Stepper motor direction pin
  
  Serial.begin(9600);       // Serial communication for input
  Serial.println("Enter the reference distance (positive or negative):");
}

void loop() {
  if (Serial.available()) {

    reference_distance = Serial.parseInt(); // Read reference distance

    // Clear any leftover characters from the serial buffer
    while (Serial.available()) {
      Serial.read();
    }
  }
    
  if (reference_distance != 0) {
    Serial.println(reference_distance);
    moveStepper(reference_distance);
    reference_distance = 0; // Reset after movement
  }
  delay(100);
}

void moveStepper(int steps) {
  bool direction = (steps > 0) ? HIGH : LOW; // Set direction
  digitalWrite(DIR_PIN, direction);

  for (int i = 0; i < abs(steps); i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(delayPerStepMicrosec / 2);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(delayPerStepMicrosec / 2);
  }
  Serial.println("Movement complete.");
}
