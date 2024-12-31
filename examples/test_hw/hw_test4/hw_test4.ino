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
  stepper.setMaxSpeed(200);    // Maximum speed in steps per second
  stepper.setAcceleration(400); // Acceleration in steps per second^2

  Serial.begin(9600);          // Debugging output to Serial Monitor
  Serial2.begin(115200);       // TFMini-S is connected to Serial2 (pins 7 and 8)

  Serial.println("Setup starting...");
}

void loop() {
  
  calibration();
  
}

void countSteps(int& count1, int& count2, int& count3, int initial, int offset) {
    int current_distance = waitForValidDistance();
    if (current_distance >= initial + offset && current_distance < initial + offset + 1) {
        count1++;
    } else if (current_distance >= initial + offset + 1 && current_distance < initial + offset + 2) {
        count2++;
    } else if (current_distance >= initial + offset + 2 && current_distance < initial + offset + 3) {
        count3++;
    }
}

float calibration() {
    if (!setupComplete) {
        const int RANGE_OFFSET = 1;
        const int MAX_DISTANCE = 4;
        
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

        target_distance = initial_distance - RANGE_OFFSET;
        while (waitForValidDistance() < target_distance) {
            stepper.move(-1);
            stepper.run();
        }

        float StepPerCm = totalSteps / (RANGE_OFFSET*6.0) ;
        float StepPerCmUpRange1 = stepUpCount[0] / RANGE_OFFSET;
        float StepPerCmDownRange1 = stepDownCount[0] / RANGE_OFFSET;
        float StepPerCmUpRange2 = stepUpCount[1] / RANGE_OFFSET;
        float StepPerCmDownRange2 = stepDownCount[1] / RANGE_OFFSET;
        float StepPerCmUpRange3 = stepUpCount[2] / RANGE_OFFSET;
        float StepPerCmDownRange3 = stepDownCount[2] / RANGE_OFFSET;

        Serial.println("Step Calibration Results:");
        
        Serial.print("Average steps per cm (Overall): ");
        Serial.println(StepPerCm);
        
        Serial.println("Step-Up Calibration:");
        Serial.print(" - Range 1 : ");
        Serial.print(StepPerCmUpRange1);
        Serial.println(" step/cm");
        Serial.print(" - Range 2 : ");
        Serial.print(StepPerCmUpRange2);
        Serial.println(" step/cm");
        Serial.print(" - Range 3 : ");
        Serial.print(StepPerCmUpRange3);
        Serial.println(" step/cm");
        
        Serial.println("Step-Down Calibration:");
        Serial.print(" - Range 1 : ");
        Serial.print(StepPerCmDownRange1);
        Serial.println(" step/cm");
        Serial.print(" - Range 2 : ");
        Serial.print(StepPerCmDownRange2);
        Serial.println(" step/cm");
        Serial.print(" - Range 3 : ");
        Serial.print(StepPerCmDownRange3);
        Serial.println(" step/cm");
        Serial.print("Current Distance = ");
        Serial.println(waitForValidDistance());

        setupComplete = true;
        return StepPerCm;
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
                dist = (uart[2] + uart[3] * 256);
                return dist;
            }
        }
    }

    flushSerial();
    delay(1);
    return -1;
}
