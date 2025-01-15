#include <AccelStepper.h>

// Define step and direction pins
#define STEP_PIN 3
#define DIR_PIN 4
#define ENABLE_PIN 8  // Define the enable pin of the motor driver
#define HEADER 0x59   // Frame starting byte (0x59 for TFMini-S)
#define BUFFER_SIZE 9 // Size of the data packet
#define BUTTON_PIN 40  // GPIO pin connected to emergency stop button
#define LIMITSWTICH_PIN 33  // GPIO pin connected to the limit switch

// Variables for ToF sensor
uint8_t uart_buffer[BUFFER_SIZE];
int16_t distance;
uint8_t checksum;
int i;
int input_distance = 10;

volatile bool stopMotorFlag = false; // Flag for emergency stop
volatile bool limitswitchMotorFlag = false; // Flag for limit switch
volatile unsigned long stopLastDebounceTime = 0;
volatile unsigned long limitswitchLastDebounceTime = 0;
const unsigned long debounceDelay = 100; // Debounce delay in milliseconds

// Create an instance of AccelStepper
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Define speed and acceleration ranges
const int minSpeed = 200;     // Minimum speed in steps per second
const int max_speed = 1000;   // Maximum speed in steps per second
const int maxDistance = 650;  // Maximum distance for mapping (steps)

void setup() {
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);  // Enable the motor

  // Set default speed and acceleration
  stepper.setMaxSpeed(max_speed);

  Serial.begin(9600);       // Debugging output to Serial Monitor
  Serial2.begin(115200);    // TFMini-S is connected to Serial2 (pins 7 and 8)

  pinMode(BUTTON_PIN, INPUT);
  pinMode(LIMITSWTICH_PIN, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), stopButtonPressed, RISING);
  attachInterrupt(digitalPinToInterrupt(LIMITSWTICH_PIN), limitSwitchPressed, RISING);

  Serial.println("Setup complete");
  delay(2000);
}

void loop() {
  // Non-blocking restart check
  checkRestart();

  if (stopMotorFlag || limitswitchMotorFlag) {
    stepper.stop();  // Stop the motor
    if (stopMotorFlag) {
      Serial.println("Emergency Stop activated. Press 'r' to restart.");
    } else if (limitswitchMotorFlag) {
      Serial.println("Limit switch activated. Press 'r' to restart.");
    }
    return;
  }

  inputDistance();

  int currentDistance = readDistance();
  if (currentDistance != -1) {
    Serial.print("Current Distance: ");
    Serial.println(currentDistance);
  }

  // Move motor based on distance
  if (currentDistance != -1 && currentDistance != input_distance) {
    int steps = distance_to_step(currentDistance);
    stepper.move(steps);
  }

  // Run the stepper until the movement is complete
  while (stepper.distanceToGo() != 0) {
    int remainingSteps = stepper.distanceToGo();

    // Map speed based on whether the movement is forward or reverse
    int mappedSpeed;
    if (remainingSteps < 0) {
        // Reverse movement: Map speed from -input_distance to 0
        mappedSpeed = map(remainingSteps, -distance_to_step(input_distance), 0, -max_speed, -minSpeed);
    } else {
        // Forward movement: Map speed from 0 to maxDistance
        mappedSpeed = map(remainingSteps, 0, maxDistance, minSpeed, max_speed);
    }

    // Ensure the speed values are within bounds
    mappedSpeed = constrain(mappedSpeed, -max_speed, max_speed);

    stepper.setSpeed(mappedSpeed);
    stepper.runSpeed();
  }

}

int distance_to_step(int currentDistance) {
  int stepsPerCm = 34; // 650steps/19cm=34steps/cm
  return (input_distance - currentDistance) * stepsPerCm;
}

void inputDistance() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    int new_distance = input.toInt();
    if (new_distance > 0) {
      input_distance = new_distance;
      Serial.print("Updated input distance: ");
      Serial.println(input_distance);
    } else {
      Serial.println("Invalid input, keeping previous distance.");
    }
  }
}

int readDistance() {
  if (Serial2.available()) {
    if (Serial2.read() == HEADER) {
      uart_buffer[0] = HEADER;
      if (Serial2.read() == HEADER) {
        uart_buffer[1] = HEADER;
        for (i = 2; i < BUFFER_SIZE; i++) {
          uart_buffer[i] = Serial2.read();
        }
        checksum = 0;
        for (i = 0; i < BUFFER_SIZE - 1; i++) {
          checksum += uart_buffer[i];
        }
        if ((checksum & 0xFF) == uart_buffer[8]) {
          distance = uart_buffer[2] + (uart_buffer[3] << 8);
          return distance;
        }
      }
    }
  }
  flushSerial();
  return -1;
}

void flushSerial() {
  while (Serial2.available()) {
    Serial2.read();
  }
}

void checkRestart() {
  if (Serial.available() && Serial.read() == 'r') {
    stopMotorFlag = false;
    limitswitchMotorFlag = false;
    Serial.println("Motor restarted.");
  }
}

// Interrupt Service Routines
void stopButtonPressed() {
  if (millis() - stopLastDebounceTime > debounceDelay) {
    stopMotorFlag = true;
    stopLastDebounceTime = millis();
  }
}

void limitSwitchPressed() {
  if (millis() - limitswitchLastDebounceTime > debounceDelay) {
    limitswitchMotorFlag = true;
    limitswitchLastDebounceTime = millis();
  }
}
