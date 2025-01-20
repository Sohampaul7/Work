#include <AccelStepper.h>

// Define TFMini-S sensor parameters
#define HEADER 0x59   // Frame starting byte (0x59 for TFMini-S)
#define BUFFER_SIZE 9 // Data packet size

#define STEP_PIN 3    // Pin connected to stepper motor step input
#define DIR_PIN 4     // Pin connected to stepper motor direction input

int reference_distance = 0; // Distance to move in steps

// Variables for TFMini-S sensor
uint8_t uart_buffer[BUFFER_SIZE];
int16_t distance_in_cm;
uint8_t checksum;
uint8_t reference_checksum = 0xFF;
int i;
int input_distance = 10; // Target distance in cm

// Create an instance of AccelStepper
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Speed and acceleration settings
const int min_speed = 200;     // Minimum speed (steps/sec)
const int max_speed = 1000;    // Maximum speed (steps/sec)
const int maxDistance = 650;  // Maximum distance (steps)

void setup() {
  // Initialize stepper motor pins
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  
  // Set the maximum speed and acceleration
  stepper.setMaxSpeed(5000);         // Maximum speed in steps per second
  
  Serial.begin(9600); // Initialize serial communication
  Serial2.begin(115200);   // TFMini-S connected to Serial2
  delay(2000);
  Serial.println("Setup complete. Enter the reference distance:");
}

void loop() {
  // Get user input for target distance
  updateTargetDistance();

  // Read current distance from ToF sensor
  int currentDistance = readDistance();
    
  if (currentDistance != -1) {
    Serial.print("Current Distance: ");
    Serial.println(currentDistance);
  }
  
    // Move motor to adjust distance
  if (currentDistance != -1 && currentDistance != input_distance) {
    int steps = calculateSteps(currentDistance);
    stepper.move(steps);
  }

  stepper.run();

/*
  // Execute motor movement
  while (stepper.distanceToGo() != 0) {
    adjustSpeed();
    stepper.runSpeed();
  }
*/

  Serial.print("Current Distance: ");
  Serial.println(currentDistance);
}

// Convert distance difference to steps
int calculateSteps(int currentDistance) {
  const int stepsPerCm = 34; // 650 steps / 19 cm = ~34 steps/cm
  return (input_distance - currentDistance) * stepsPerCm;
}

// Update target distance from user input
void updateTargetDistance() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    int newDistance = input.toInt();
    if (newDistance > 0) {
      input_distance = newDistance;
      Serial.print("Updated target distance: ");
      Serial.println(input_distance);
    } else {
      Serial.println("Invalid input, keeping previous distance.");
    }
  }
}

int readDistance(){
  // Read distance data from the sensor
  if (Serial2.available()) {
    if (Serial2.read() == HEADER) {
      uart_buffer[0] = HEADER;

      if (Serial2.read() == HEADER) {
        uart_buffer[1] = HEADER;

        for (i = 2; i < BUFFER_SIZE; i++) {
          uart_buffer[i] = Serial2.read();
        }

        checksum = uart_buffer[0] + uart_buffer[1] + uart_buffer[2] + uart_buffer[3] + uart_buffer[4] + uart_buffer[5] + uart_buffer[6] + uart_buffer[7];

        if (uart_buffer[8] == (checksum & reference_checksum)) {
          // Calculate distance_in_cm
          return distance_in_cm = uart_buffer[2] + uart_buffer[3] * 256;
        }
      }
    }
  }
  flushSerialBuffer();
  return -1;
}

// Flush TFMini-S serial buffer
void flushSerialBuffer() {
  while (Serial2.available()) {
    Serial2.read();
  }
}

// Adjust motor speed based on remaining steps
void adjustSpeed() {
  int remainingSteps = stepper.distanceToGo();
  int mappedSpeed = (remainingSteps < 0)
                        ? map(remainingSteps, -calculateSteps(input_distance), 0, -max_speed, -min_speed)
                        : map(remainingSteps, 0, maxDistance, min_speed, max_speed);
  stepper.setSpeed(constrain(mappedSpeed, -max_speed, max_speed));
}
