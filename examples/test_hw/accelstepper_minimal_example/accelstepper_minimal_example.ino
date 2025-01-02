#include <AccelStepper.h>

#define STEP_PIN 3    // Pin connected to stepper motor step input
#define DIR_PIN 4     // Pin connected to stepper motor direction input

int reference_distance = 0; // Distance to move in steps

// Create an instance of AccelStepper
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

void setup() {
  // Initialize stepper motor pins
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  
  // Set the maximum speed and acceleration
  stepper.setMaxSpeed(500);         // Maximum speed in steps per second
  stepper.setAcceleration(1000);   // Acceleration in steps per second^2
  
  Serial.begin(9600); // Initialize serial communication
  Serial.println("Setup complete. Enter the reference distance:");
}

void loop() {
  if (Serial.available()) {
    reference_distance = Serial.parseInt(); // Read reference distance

    // Clear any leftover characters from the serial buffer
    while (Serial.available()) {
      Serial.read();
    }

    // Move the stepper motor by the reference distance
    stepper.move(reference_distance);

    // Run the stepper until the movement is completed
    while (stepper.distanceToGo() != 0) {
      stepper.run();
    }

    // Display message after completing the movement
    Serial.println("Movement completed");
    reference_distance = 0; // Reset reference distance for the next input
    Serial.println("Enter the reference distance:");
  }
}
