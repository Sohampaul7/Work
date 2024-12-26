#include <AccelStepper.h>

// Define step and direction pins
#define STEP_PIN 3
#define DIR_PIN 4

// Create an instance of AccelStepper
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

void setup()
{  
  // Set the maximum speed and acceleration
  stepper.setMaxSpeed(500);   // Maximum speed in steps per second
  stepper.setAcceleration(1000); // Acceleration in steps per second^2
  
  Serial.begin(9600);       // Debugging output to Serial Monitor
  
  // Move the stepper to a target position
  stepper.moveTo(200);  // Set the target position in steps
  
  Serial.println("Setup complete");
  Serial.println(stepper.distanceToGo());
  delay(2000);
}

void loop() {
  
  
  if (stepper.distanceToGo() != 0) {
    stepper.run();
  } 
  else{
    stepper.moveTo(-200);
    Serial.println(stepper.distanceToGo());
  }
}
