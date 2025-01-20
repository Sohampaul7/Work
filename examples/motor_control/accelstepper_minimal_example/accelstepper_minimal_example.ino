#include <AccelStepper.h>

// Define step and direction pins
#define STEP_PIN 3
#define DIR_PIN 4

// Create an instance of AccelStepper
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

void setup() {
  // Initialize Serial for debugging
  Serial.begin(9600);

  // Set maximum speed and acceleration
  stepper.setMaxSpeed(500);   // Max speed in steps per second
  stepper.setAcceleration(1000); // Acceleration in steps per second^2
  
  Serial.println("Setup complete");
  delay(1000); // Delay to observe setup status

  
  // Move up by 100 steps
  stepper.move(100);
  Serial.println("Moving up");
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }
  delay(1000); // Pause at the top
  
  // Move down by 100 steps (back to the original position)
  stepper.move(-100);
  Serial.println("Moving down");
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }
  delay(1000); // Pause at the bottom
  
  // Move up by 100 steps
  stepper.move(100);
  Serial.println("Moving up");
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  }
  delay(1000); // Pause at the top
}

void loop() {
}
