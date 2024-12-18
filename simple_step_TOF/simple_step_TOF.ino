#define STEP_PIN 3    // Pin connected to stepper motor step input
#define DIR_PIN 4     // Pin connected to stepper motor direction input
#define ENABLE_PIN 10
#define MAX_STEPS 50  // Maximum steps for 90° rotation
#define HEADER 0x59   // Frame starting byte (0x59 for TFMini-S)
#define BUFFER_SIZE 9 // Size of the data packet

#include <Arduino.h>

// Variables for ToF sensor
uint8_t uart[BUFFER_SIZE];
int16_t dist;
uint8_t check;
int i;

// Constants
const int MIN_DISTANCE_MM = 3; // Minimum allowable distance
const int STEP_DELAY_US = 500; // Step delay in microseconds

// Stepper motor variables
int currentPositionSteps = 0; // Tracks the motor position (0 = home, MAX_STEPS = max away)

void moveStepper(int steps, bool dir) {
  digitalWrite(DIR_PIN, dir ? HIGH : LOW); // Set direction

  for (int stp = 0; stp < steps; stp++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_DELAY_US); // Adjust speed
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_DELAY_US); // Adjust speed
  }
  Serial.print("Direction: ");
  Serial.println(dir ? "Forward" : "Backward");
  Serial.print("Steps moved: ");
  Serial.println(steps);

  // Update position
  currentPositionSteps += (dir ? -steps : +steps);
  currentPositionSteps = constrain(currentPositionSteps, 0, MAX_STEPS);

  Serial.print("Updated Position: ");
  Serial.println(currentPositionSteps);
}

void setup() {
  Serial.begin(9600);       // Debugging output to Serial Monitor
  Serial2.begin(115200);    // TFMini-S is connected to Serial2 (pins 8 and 9)
  pinMode(STEP_PIN, OUTPUT); // Stepper motor step pin
  pinMode(DIR_PIN, OUTPUT);  // Stepper motor direction pin
  pinMode(LED_BUILTIN, OUTPUT); // For error indication
  pinMode(ENABLE_PIN, OUTPUT);
  
  digitalWrite(ENABLE_PIN, LOW); // Enable motor driver

  Serial.println("System Initialized...");
}

void loop() {
  // Read distance data from the sensor
  if (Serial2.available()) {
    if (Serial2.read() == HEADER) {
      uart[0] = HEADER;

      if (Serial2.read() == HEADER) {
        uart[1] = HEADER;

        for (i = 2; i < BUFFER_SIZE; i++) {
          uart[i] = Serial2.read();
        }

        // Calculate checksum
        check = uart[0] + uart[1] + uart[2] + uart[3] + uart[4] + uart[5] + uart[6] + uart[7];

        if (uart[8] == (check & 0xFF)) {
          // Calculate distance
          dist = uart[2] + uart[3] * 256;

          // Motor movement logic
          if (dist < MIN_DISTANCE_MM && currentPositionSteps < MAX_STEPS) {
            // Object is too close, move away
            moveStepper(1, false); // Move away
           // Serial.print("Object detected too close. Moving away. Current position: ");
            Serial.println(currentPositionSteps);
          } else if (dist >= MIN_DISTANCE_MM && currentPositionSteps > 0) {
            // Object is far, return to home
            moveStepper(1, true); // Move towards home
          //  Serial.print("Path clear. Returning to home. Current position: ");
          //  Serial.println(currentPositionSteps);
          }

          // Debugging output
         // Serial.print("Distance: ");
         // Serial.print(dist);
         // Serial.println(" mm");
          digitalWrite(LED_BUILTIN, HIGH); // No error
        } else {
          // Checksum error
          digitalWrite(LED_BUILTIN, LOW); // Error indication
        //  Serial.println("Checksum error!");
        }
      }
    }
  }

  delay(10); // Small delay to stabilize the loop
}
