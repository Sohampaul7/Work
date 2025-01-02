#include <AccelStepper.h>

#define STEP_PIN 3    // Pin connected to stepper motor step input
#define DIR_PIN 4     // Pin connected to stepper motor direction input

int reference_distance = 0; // Distance to move in steps

// Create an instance of AccelStepper
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Define the range for speed and acceleration
const int minSpeed = 200;      // Minimum speed in steps per second
const int max_speed = 1000;     // Maximum speed in steps per second
const int maxAccel = 30000;     // Maximum acceleration in steps per second^2
const int maxDistance = 650;  // Maximum distance for mapping (steps)

void setup() {
  // Initialize stepper motor pins
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  
  // Set default speed and acceleration
  stepper.setMaxSpeed(minSpeed);
  stepper.setAcceleration(maxAccel);
  
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

    // Calculate absolute distance
    int abs_distance = abs(reference_distance);

    // Map distance to speed and acceleration
    int mappedSpeed = map(abs_distance, 0, maxDistance, minSpeed, max_speed);

    // Ensure values do not exceed their limits
    mappedSpeed = constrain(mappedSpeed, minSpeed, max_speed);

    // Set speed and acceleration
    stepper.setMaxSpeed(mappedSpeed);

    // Debug output
    Serial.print("Mapped Speed: ");
    Serial.println(mappedSpeed);

    // Move the stepper motor by the reference distance
    stepper.move(reference_distance);

    // Run the stepper until the movement is completed
    while (stepper.distanceToGo() != 0) {
      stepper.run();
    }

    // Display message after completing the movement
    Serial.print("Movement completed: ");
    Serial.print(reference_distance);
    Serial.println(" steps");
    reference_distance = 0; // Reset reference distance for the next input
    Serial.println("Enter the reference distance:");
  }
}
