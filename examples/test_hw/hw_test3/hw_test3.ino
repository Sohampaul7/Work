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

bool setupComplete = false;

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
  
  calibration();
  
}

void countSteps(int& count1, int& count2, int& count3, int initial, int offset) {
    int current_distance = waitForValidDistance();
    if (current_distance >= initial + offset && current_distance < initial + offset + 10) {
        count1++;
    } else if (current_distance >= initial + offset + 10 && current_distance < initial + offset + 20) {
        count2++;
    } else if (current_distance >= initial + offset + 20 && current_distance < initial + offset + 30) {
        count3++;
    }
}

float calibration() {
    if (!setupComplete) {
        const int RANGE_OFFSET = 10;
        const int MAX_DISTANCE = 40;
        
        // Get initial distance
        int initial_distance = waitForValidDistance();
        if (initial_distance == -1) {
            Serial.println("Error: Failed to get initial distance");
            return -1.0;
        }

        Serial.print("Initial Distance: ");
        Serial.println(initial_distance);

        // Move to target distance
        int target_distance = initial_distance + RANGE_OFFSET;
        while (waitForValidDistance() < target_distance) {
            stepper.move(1);
            stepper.run();
        }

        Serial.println("Starting step counting...");
        int stepUpCount[3] = {0, 0, 0};
        int stepDownCount[3] = {0, 0, 0};

        // Step up loop
        target_distance = initial_distance + MAX_DISTANCE;
        while (waitForValidDistance() < target_distance) {
            stepper.move(1);
            stepper.run();

            // Print current distance to Serial Monitor
            int current_distance = waitForValidDistance();
            Serial.print("Stepping Up: Current Distance = ");
            Serial.println(current_distance);
            
            countSteps(stepUpCount[0], stepUpCount[1], stepUpCount[2], initial_distance, RANGE_OFFSET);
        }

        // Step down loop
        while (waitForValidDistance() > initial_distance) {
            stepper.move(-1);
            stepper.run();

            // Print current distance to Serial Monitor
            int current_distance = waitForValidDistance();
            Serial.print("Stepping Down: Current Distance = ");
            Serial.println(current_distance);
            
            countSteps(stepDownCount[0], stepDownCount[1], stepDownCount[2], initial_distance, RANGE_OFFSET);
        }

        // Calculate average steps per range
        int totalSteps = stepUpCount[0] + stepUpCount[1] + stepUpCount[2] + stepDownCount[0] + stepDownCount[1] + stepDownCount[2];
        if (totalSteps == 0) {
            Serial.println("Error: No steps counted");
            return -1.0;
        }

        float mmPerStep = RANGE_OFFSET*6.0 / totalSteps;

        // Print step counts for step-up and step-down
        Serial.println("Step Count Summary:");
        Serial.print("Step Up Count: ");
        Serial.print("Range 1 = ");
        Serial.print(stepUpCount[0]);
        Serial.print(", Range 2 = ");
        Serial.print(stepUpCount[1]);
        Serial.print(", Range 3 = ");
        Serial.println(stepUpCount[2]);

        Serial.print("Step Down Count: ");
        Serial.print("Range 1 = ");
        Serial.print(stepDownCount[0]);
        Serial.print(", Range 2 = ");
        Serial.print(stepDownCount[1]);
        Serial.print(", Range 3 = ");
        Serial.println(stepDownCount[2]);
        
        Serial.print("Average mm per Step: ");
        Serial.println(mmPerStep);

        setupComplete = true;
        return mmPerStep;
    }
    return 0.0;
}

int waitForValidDistance() {
  int distance;
  do {
    distance = readDistance();
  } while (distance == -1); // Retry until a valid distance is obtained
  return distance;
}

void flushSerial() {
  while (Serial2.available()> 0) {
    Serial2.read(); // discard incoming bytes
  }
}

bool validateChecksum(uint8_t* data, uint8_t checksum) {
    uint8_t calculated_chk = 0;
    for (int i = 0; i < BUFFER_SIZE - 1; i++) {
        calculated_chk += data[i];
    }
    return (calculated_chk & 0xFF) == checksum;
}

int readDistance() {
    if (Serial2.available() && Serial2.read() == HEADER) {
        uart[0] = HEADER;

        if (Serial2.read() == HEADER) {
            uart[1] = HEADER;

            for (i = 2; i < BUFFER_SIZE; i++) {
                uart[i] = Serial2.read();
            }

            if (validateChecksum(uart, uart[8])) {
                dist = (uart[2] + uart[3] * 256) * 10;
                return dist;
            }
        }
    }

    flushSerial();
    delay(1);
    return -1;
}
