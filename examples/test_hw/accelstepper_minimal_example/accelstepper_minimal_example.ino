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

  stepper.setMaxSpeed(1500);
  stepper.setAcceleration(1000);
  
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
    Serial.println(stepper.distanceToGo());

    
    // Run the stepper until the movement is completed
    while (stepper.distanceToGo() != 0) {
      stepper.run();
    }
    // Debug: Check direction pin state after setting the movement
    Serial.print("DIR_PIN state after movement: ");
    Serial.println(digitalRead(DIR_PIN));

    // Display message after completing the movement
    Serial.print("Movement completed: ");
    Serial.println(reference_distance);
    reference_distance = 0; // Reset reference distance for the next input
    Serial.println("");
    Serial.println("Enter the reference distance:");
  }
}
