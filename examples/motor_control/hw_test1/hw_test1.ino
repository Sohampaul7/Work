#include <AccelStepper.h>

// Define step and direction pins
#define STEP_PIN 3
#define DIR_PIN 4

// Create an instance of AccelStepper
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

void setup() {
  // Initialize Serial communication for debugging and input
  Serial.begin(9600);

  // Set the maximum speed and acceleration
  stepper.setMaxSpeed(1000);    // Maximum speed in steps per second
  stepper.setAcceleration(500); // Acceleration in steps per second^2

  Serial.println("Setup complete. Send target position via Serial.");
}

void loop() {
  // Check if new data is available on Serial
  if (Serial.available()) {
    int targetPosition = Serial.parseInt();  // Read the target position as an integer

    // If a valid position is received, set it as the target position
    if (targetPosition != 0 || Serial.peek() == '\n') {
      stepper.moveTo(targetPosition);       // Set the target position
      Serial.print("Moving to position: ");
      Serial.println(targetPosition);
    }
    Serial.flush(); // Clear any remaining input in the buffer
  }

  // Run the stepper motor to reach its target position
  stepper.run();
}
