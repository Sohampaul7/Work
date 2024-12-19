#define STEP_PIN 3    // Pin connected to stepper motor step input
#define DIR_PIN 4     // Pin connected to stepper motor direction input


const float stepPerDegree = 200.0/360.0;
const float requiredAngle = 360.0;


void setup() {
  pinMode(STEP_PIN, OUTPUT); // Stepper motor step pin
  pinMode(DIR_PIN, OUTPUT);  // Stepper motor direction pin

  Serial.println("Setup complete");
  delay(10000);
}

void loop() {

  digitalWrite(DIR_PIN, HIGH); 
  
  for (int stp = 0; stp < requiredAngle*stepPerDegree; stp++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(3500); // Adjust speed
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(3500);
  }
  delay(1000);
  digitalWrite(DIR_PIN, LOW); 
  
  for (int stp = 0; stp < requiredAngle*stepPerDegree; stp++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(3500); // Adjust speed
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(3500);
  }
  delay(1000);

}
