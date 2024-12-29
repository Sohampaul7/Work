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

void setup() {
  // Set the maximum speed and acceleration
  stepper.setMaxSpeed(50);    // Maximum speed in steps per second
  stepper.setAcceleration(50); // Acceleration in steps per second^2

  Serial.begin(9600);          // Debugging output to Serial Monitor
  Serial2.begin(115200);       // TFMini-S is connected to Serial2 (pins 7 and 8)

  Serial.println("Setup starting...");
}

void loop() {
  static bool setupComplete = false; // Run the setup logic only once
  if (!setupComplete) {
    // Get the initial distance
    int initial_distance = waitForValidDistance();
    Serial.print("Initial Distance: ");
    Serial.println(initial_distance);

    // Move until the distance is initial + 10 mm
    int target_distance = initial_distance + 10;
    while (true) {
      int current_distance = waitForValidDistance();
      Serial.print("Current Distance: ");
      Serial.println(current_distance);
      if (current_distance >= target_distance) break;
      stepper.move(1); // Move the stepper one step at a time
      stepper.run();
    }

    Serial.println("Starting step counting...");

    // Start counting steps
    int stepCount = 0;
    target_distance = initial_distance + 30; // Stop at initial + 30 mm
    while (true) {
      int current_distance = waitForValidDistance();
      Serial.print("Current Distance: ");
      Serial.println(current_distance);

      if (current_distance >= target_distance) break;

      stepper.move(1); // Move the stepper one step at a time
      stepper.run();
      stepCount++;
    }

    Serial.print("Steps Counted: ");
    Serial.println(stepCount);

    // Calculate steps per mm
    float mmPerStep = 20.0 / stepCount; // 20 mm = 2 cm
    Serial.print("mm per step: ");
    Serial.println(mmPerStep);
    
    /*
    Steps Counted: 228
    mm per step: 0.09

    Steps Counted: 236
    mm per step: 0.08

    Steps Counted: 238
    mm per step: 0.08

    Steps Counted: 227
    mm per step: 0.09

    Steps Counted: 215
    mm per step: 0.09

    Steps Counted: 221
    mm per step: 0.09
    */
    
    Serial.println("Setup complete");
    setupComplete = true; // Mark setup as done
  }
}

int waitForValidDistance() {
  int distance;
  do {
    distance = readDistance();
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
