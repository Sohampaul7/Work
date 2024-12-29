#include <AccelStepper.h>

// Define step and direction pins
#define STEP_PIN 3
#define DIR_PIN 4

#define HEADER 0x59   // Frame starting byte (0x59 for TFMini-S)
#define BUFFER_SIZE 9 // Size of the data packet

// Variables for ToF sensor
uint8_t uart[BUFFER_SIZE];
int16_t dist;
uint8_t chk;
int i;

// Create an instance of AccelStepper
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

bool setupComplete = false; // Flag to run the setup logic only once

void setup() {
  // Set the maximum speed and acceleration
  stepper.setMaxSpeed(10);    // Maximum speed in steps per second
  stepper.setAcceleration(20); // Acceleration in steps per second^2

  Serial.begin(9600);          // Debugging output to Serial Monitor
  Serial2.begin(115200);       // TFMini-S is connected to Serial2 (pins 7 and 8)

  Serial.println("Setup starting...");
}

void loop() {
  if (!setupComplete) {
    // Wait for a valid initial distance
    int initial_distance = waitForValidDistance();
    Serial.print("Initial Distance: ");
    Serial.println(initial_distance);

    // Move the stepper motor to 100 steps
    stepper.moveTo(100);
    while (stepper.distanceToGo() != 0) {
      stepper.run(); // Keep running the motor until it reaches the target
      int current_distance = waitForValidDistance();
      Serial.print("Current Distance: ");
      Serial.println(current_distance);
    }

    // Wait for a valid final distance
    int final_distance = waitForValidDistance();
    Serial.print("Final Distance: ");
    Serial.println(final_distance);

    // Calculate mm per step
    float mm_per_step = (float)(final_distance - initial_distance) / 100.0;
    Serial.print("MM per Step: ");
    Serial.println(mm_per_step);

    Serial.println("Setup complete");
    setupComplete = true; // Mark setup as done
  }
}

int waitForValidDistance() {
  int distance;
  do {
    distance = readDistance();
    if (distance == -1) {
    }
  } while (distance == -1); // Retry until a valid distance is obtained
  return distance;
}

void flushSerial() {
  while (Serial2.available()) {
    Serial2.read(); // discard incoming bytes
  }
}

int readDistance() {
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
        chk = uart[0] + uart[1] + uart[2] + uart[3] + uart[4] + uart[5] + uart[6] + uart[7];

        if (uart[8] == (chk & 0xFF)) {
          // Calculate distance in mm
          dist = (uart[2] + uart[3] * 256) * 10;
          return dist;
        }
      }
    }
  }

  // If no valid data is received, flush the buffer and return -1
  flushSerial();
  delay(1);
  return -1;
}
