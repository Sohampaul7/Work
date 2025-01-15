#include <AccelStepper.h>

// Define stepper motor pins
#define STEP_PIN 3
#define DIR_PIN 4
#define ENABLE_PIN 8 // Motor driver enable pin

// Define TFMini-S sensor parameters
#define HEADER 0x59   // Frame starting byte (0x59 for TFMini-S)
#define BUFFER_SIZE 9 // Data packet size

// Define input pins
#define BUTTON_PIN 40       // Emergency stop button pin
#define LIMIT_SWITCH_PIN 33 // Limit switch pin

// Variables for TFMini-S sensor
uint8_t uart_buffer[BUFFER_SIZE];
int16_t distance;
uint8_t checksum;
int input_distance = 10; // Target distance in cm

// Emergency and limit switch flags
volatile bool stopMotorFlag = false;
volatile bool limitSwitchFlag = false;

// Debounce timing
volatile unsigned long stopLastDebounceTime = 0;
volatile unsigned long limitSwitchLastDebounceTime = 0;
const unsigned long debounceDelay = 100; // Debounce delay in milliseconds

// Create AccelStepper instance
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Speed and acceleration settings
const int minSpeed = 200;     // Minimum speed (steps/sec)
const int maxSpeed = 1000;    // Maximum speed (steps/sec)
const int maxDistance = 650;  // Maximum distance (steps)

void setup() {
  // Motor driver setup
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW); // Enable motor driver
  stepper.setMaxSpeed(maxSpeed);

  // Serial setup
  Serial.begin(9600);      // Debug output
  Serial2.begin(115200);   // TFMini-S connected to Serial2

  // Input pin setup
  pinMode(BUTTON_PIN, INPUT);
  pinMode(LIMIT_SWITCH_PIN, INPUT);
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), stopButtonISR, RISING);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_PIN), limitSwitchISR, RISING);

  Serial.println("Setup complete");
  delay(2000);
}

void loop() {
  // Check for restart
  checkRestart();

  // Handle emergency stop or limit switch activation
  if (stopMotorFlag || limitSwitchFlag) {
    stepper.stop();
    Serial.println(stopMotorFlag ? "Emergency Stop activated. Press 'r' to restart." : "Limit switch activated. Press 'r' to restart.");
    return;
  }

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

  // Execute motor movement
  while (stepper.distanceToGo() != 0) {
    adjustSpeed();
    stepper.runSpeed();
  }
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

// Read distance from TFMini-S sensor
int readDistance() {
  if (Serial2.available()) {
    if (Serial2.read() == HEADER) {
      uart_buffer[0] = HEADER;
      if (Serial2.read() == HEADER) {
        uart_buffer[1] = HEADER;
        for (int i = 2; i < BUFFER_SIZE; i++) {
          uart_buffer[i] = Serial2.read();
        }
        checksum = 0;
        for (int i = 0; i < BUFFER_SIZE - 1; i++) {
          checksum += uart_buffer[i];
        }
        if ((checksum & 0xFF) == uart_buffer[8]) {
          distance = uart_buffer[2] + (uart_buffer[3] << 8);
          return distance;
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
                        ? map(remainingSteps, -calculateSteps(input_distance), 0, -maxSpeed, -minSpeed)
                        : map(remainingSteps, 0, maxDistance, minSpeed, maxSpeed);
  stepper.setSpeed(constrain(mappedSpeed, -maxSpeed, maxSpeed));
}

// Check for restart command
void checkRestart() {
  if (Serial.available() && Serial.read() == 'r') {
    stopMotorFlag = false;
    limitSwitchFlag = false;
    Serial.println("Motor restarted.");
  }
}

// Emergency stop InterruptServiceRoutine
void stopButtonISR() {
  if (millis() - stopLastDebounceTime > debounceDelay) {
    stopMotorFlag = true;
    stopLastDebounceTime = millis();
  }
}

// Limit switch InterruptServiceRoutine
void limitSwitchISR() {
  if (millis() - limitSwitchLastDebounceTime > debounceDelay) {
    limitSwitchFlag = true;
    limitSwitchLastDebounceTime = millis();
  }
}
