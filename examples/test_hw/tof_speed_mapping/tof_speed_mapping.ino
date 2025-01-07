#include <AccelStepper.h>

// Define step and direction pins
#define STEP_PIN 3
#define DIR_PIN 4
#define ENABLE_PIN 8  // Define the enable pin of the motor driver
#define HEADER 0x59   // Frame starting byte (0x59 for TFMini-S)
#define BUFFER_SIZE 9 // Size of the data packet
#define BUTTON_PIN 40  // GPIO pin connected to pushbutton
#define LIMITSWTICH_PIN 33  // GPIO pin connected to pushbutton

//TOF green to pin 7

// Variables for ToF sensor
uint8_t uart_buffer[BUFFER_SIZE];
int16_t distance;
uint8_t checksum;
int i;
int input_distance = 10;

volatile bool stopMotorFlag = false; // Flag to stop the motor
volatile bool limitswitchMotorFlag = false; // Flag to stop the motor
volatile unsigned long stopLastDebounceTime = 0; // Track the last debounce time
volatile unsigned long limitswitchLastDebounceTime = 0; // Track the last debounce time
const unsigned long debounceDelay = 100;     // Debounce delay in milliseconds

// Create an instance of AccelStepper
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Define the range for speed and acceleration
const int minSpeed = 200;      // Minimum speed in steps per second
const int max_speed = 1000;     // Maximum speed in steps per second
const int maxAccel = 30000;     // Maximum acceleration in steps per second^2
const int maxDistance = 650;  // Maximum distance for mapping (steps)

void setup()
{  
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW);  // Enable the motor
  
    // Set default speed and acceleration
  stepper.setMaxSpeed(max_speed);
  stepper.setAcceleration(maxAccel);
  
  Serial.begin(9600);       // Debugging output to Serial Monitor
  Serial2.begin(115200);    // TFMini-S is connected to Serial2 (pins 7 and 8)
  
  pinMode(BUTTON_PIN, INPUT);  // Set BUTTON_PIN as an input to read the state of the stop button
  pinMode(LIMITSWTICH_PIN, INPUT);  // Set LIMITSWTICH_PIN as an input to read the state of the limit switch
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), stopButtonPressed, RISING);  // Attach an interrupt to BUTTON_PIN that triggers the stopButtonPressed function on a RISING signal (button press)
  attachInterrupt(digitalPinToInterrupt(LIMITSWTICH_PIN), limitSwitchPressed, RISING); // Attach an interrupt to LIMITSWTICH_PIN that triggers the limitSwitchPressed function on a RISING signal (switch activation).
   
  // Move the stepper to a target position
  stepper.move(40);  // Set the target position in steps
  
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  } 
  
  // Move the stepper to a target position
  stepper.move(-40);  // Set the target position in steps
  
  while (stepper.distanceToGo() != 0) {
    stepper.run();
  } 
    
  Serial.println("Setup complete");
  Serial.println(stepper.distanceToGo());
  delay(2000);
}

void loop() {
  
  // Check for Emergency Stop activation
  if (stopMotorFlag) {
    stepper.stop();  // Stop the stepper motor
    Serial.println("Emergency Stop. Press 'r' to restart.");
    waitForRestart(); // Wait for restart command
    return; // Exit the loop function
  }

  // Check for limit switch activation
  if (limitswitchMotorFlag) {
    stepper.stop();  // Stop the stepper motor
    Serial.println("Limit switch pressed. Press 'r' to restart.");
    waitForRestart(); // Wait for restart command
    return; // Exit the loop function
  }
  
  inputDistance();  
  
  int distance = readDistance();
  
  if (distance!=-1){
    Serial.println(distance);
  }
  
  // Check if the distance exceeds the threshold and move the stepper motor accordingly
  if (distance!=-1 && distance != input_distance){
    stepper.move(input_distance - distance); // Command the stepper to move to the calculated position
  }

  // Run the stepper until the movement is completed
  while (stepper.distanceToGo() != 0) {
      // Calculate absolute distance
    int abs_distance = abs(stepper.distanceToGo());

/*
    int mappedSpeed = map(abs_distance, 0, maxDistance, minSpeed, max_speed);// Map distance to speed
    mappedSpeed = constrain(mappedSpeed, minSpeed, max_speed);// Ensure values do not exceed their limits
    stepper.setSpeed(mappedSpeed);// Set speed
    stepper.runSpeed();
*/
    stepper.run();
  }
}


void inputDistance() {
  if (Serial.available()) {

    String input = Serial.readStringUntil('\n');  // Read the input string until a newline character
    int new_distance = input.toInt();   // Convert input to integer

    if (new_distance > 0) { // Update only if valid value
      input_distance = new_distance;
      Serial.print("Updated input distance: ");
      Serial.println(input_distance);
    } else {
      Serial.println("Invalid input, keeping previous threshold_distance.");
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

        // Calculate checksumsum
        checksum = uart_buffer[0] + uart_buffer[1] + uart_buffer[2] + uart_buffer[3] + uart_buffer[4] + uart_buffer[5] + uart_buffer[6] + uart_buffer[7];

        if (uart_buffer[8] == (checksum & 0xFF)) {
          // Calculate distance
          distance = uart_buffer[2] + uart_buffer[3] * 256;
          return distance; 
        }
      }
    }
  }

  //Serial.println(distance);
  flushSerial();
  delay(1);
  return -1;
}

void flushSerial(){
  while(Serial2.available()){
    Serial2.read(); // discard incoming bytes
  }
  
}

void waitForRestart() {
  // Continuously wait for a restart signal ('r') from the Serial input
  while (true) {
    if (Serial.available() && Serial.read() == 'r') {
      stopMotorFlag = false;  // Reset emergency stop flag
      limitswitchMotorFlag = false; // Reset limit switch flag
      Serial.println("Motor restarted.");
      break;  // Exit the loop
    }
  }
}

// ISR to set the flag
void stopButtonPressed() {
  unsigned long currentTime = millis();
  if (currentTime - stopLastDebounceTime > debounceDelay) { // Check if debounce delay has passed
    stopMotorFlag = true; // Set flag to stop the motor
    stopLastDebounceTime = currentTime;
  }
}

void limitSwitchPressed() {
  unsigned long currentTime = millis();
  if (currentTime - limitswitchLastDebounceTime > debounceDelay) {  // Check if debounce delay has passed
    limitswitchMotorFlag = true; // Set flag to stop the motor
    limitswitchLastDebounceTime = currentTime;
  }
}
