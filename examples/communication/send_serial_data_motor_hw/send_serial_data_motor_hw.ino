#define STEP_PIN 3    // Pin connected to stepper motor step input
#define DIR_PIN 4     // Pin connected to stepper motor direction input

const float stepPerDegree = 200.0/360.0;
const float requiredAngle = 360.0;
const int delayPerStepMicrosec = 1000; // keep between 1000 to 2400 
int reference_distance = 10;

void setup() {
  pinMode(STEP_PIN, OUTPUT); // Stepper motor step pin
  pinMode(DIR_PIN, OUTPUT);  // Stepper motor direction pin
  
  Serial.begin(9600);       // Debugging output to Serial Monitor

  Serial.println("Setup complete");
}

void loop() {  
  
  // Update threshold distance from Serial input if available
  updateReferenceDistance();
  
  if (reference_distance != 0) {
    int steps = abs(reference_distance); // Absolute steps to move
    bool direction = reference_distance > 0 ? HIGH : LOW; // Determine direction

    for (int i = 0; i < steps; i++) {
      oneStep(direction);
    }

    reference_distance = 0; // Reset after moving
    Serial.println("Movement complete.");
  }
  
}

void updateReferenceDistance() {
  if (Serial.available()) {
  
    String input = Serial.readStringUntil('\n');  // Read the input string until a newline character
    int new_distance = input.toInt();   // Convert input to integer

    if (new_distance!= 0) { // Update only if valid value
      reference_distance = new_distance;
      Serial.print("Updated threshold_distance: ");
      Serial.println(reference_distance);
    } else {
      Serial.println("Invalid input, keeping previous threshold_distance.");
    }
  }
}


void oneStep(bool motor_direction){
  
  digitalWrite(DIR_PIN, motor_direction); 
  
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(delayPerStepMicrosec/2);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(delayPerStepMicrosec/2);
  Serial.println("1 step");
}
