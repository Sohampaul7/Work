
#define STEP_PIN 3    // Pin connected to stepper motor step input
#define DIR_PIN 4     // Pin connected to stepper motor direction input


const float stepPerDegree = 200.0/360.0;
const float requiredAngle = 360.0;
const int delayPerStepMicrosec = 1000; // keep between 1000 to 2400 

void setup() {
  pinMode(STEP_PIN, OUTPUT); // Stepper motor step pin
  pinMode(DIR_PIN, OUTPUT);  // Stepper motor direction pin

  Serial.println("Setup complete");
  delay(2000);
}

void loop() {
  doNdegrees(90, HIGH);
  delay(1000);
}


void oneStep(bool motor_direction){
  
  digitalWrite(DIR_PIN, motor_direction); 
  
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(delayPerStepMicrosec/2);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(delayPerStepMicrosec/2);

}

void doNsteps(int n, bool motor_direction){
  for (int stp=0; stp<n; stp++){
    oneStep(motor_direction);
  }
}

void doNdegrees(float degree, bool motor_direction){

  int steps = degree*stepPerDegree;
  
  for (int stp=0; stp<steps; stp++){
    oneStep(motor_direction);
  }
}
