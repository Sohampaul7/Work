#include <AccelStepper.h>

#define STEP_PIN 3    // Pin connected to stepper motor step input
#define DIR_PIN 4     // Pin connected to stepper motor direction input
#define HEADER 0x59   // Frame starting byte (0x59 for TFMini-S)
#define BUFFER_SIZE 9 // Size of the data packet
#define BUTTON_PIN 33  // GPIO pin connected to pushbutton
#define LIMITSWTICH_PIN 40  // GPIO pin connected to pushbutton

// Variables for ToF sensor
uint8_t uart_buffer[BUFFER_SIZE];
int16_t distance_in_cm;
uint8_t checksum;
uint8_t reference_checksum = 0xFF;
int i;
int threshold_distance = 10;

volatile bool stopMotorFlag = false; // Flag to stop the motor
volatile bool limitswitchMotorFlag = false; // Flag to stop the motor
volatile unsigned long stopLastDebounceTime = 0; // Track the last debounce time
volatile unsigned long limitswitchLastDebounceTime = 0; // Track the last debounce time
const unsigned long debounceDelay = 100;     // Debounce delay in milliseconds

// Initialize an AccelStepper object in DRIVER mode, specifying the step pin (STEP_PIN) and direction pin (DIR_PIN)
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN); //

void setup() {
  pinMode(STEP_PIN, OUTPUT); // Stepper motor step pin
  pinMode(DIR_PIN, OUTPUT);  // Stepper motor direction pin
  
  Serial.begin(9600);       // Debugging output to Serial Monitor
  Serial2.begin(115200);    // TFMini-S is connected to Serial2 (pins 7 and 8)

  pinMode(BUTTON_PIN, INPUT);  // Set BUTTON_PIN as an input to read the state of the stop button
  pinMode(LIMITSWTICH_PIN, INPUT);  // Set LIMITSWTICH_PIN as an input to read the state of the limit switch
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), stopButtonPressed, RISING);  // Attach an interrupt to BUTTON_PIN that triggers the stopButtonPressed function on a RISING signal (button press)
  attachInterrupt(digitalPinToInterrupt(LIMITSWTICH_PIN), limitSwitchPressed, RISING); // Attach an interrupt to LIMITSWTICH_PIN that triggers the limitSwitchPressed function on a RISING signal (switch activation).
   
  stepper.setMaxSpeed(500);   // Maximum speed in steps per second
  stepper.setAcceleration(1000); // Acceleration in steps per second^2

  Serial.println("Setup complete");
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
  
  // Update threshold distance from Serial input if available
  updateThresholdDistance();

  // Read the distance from the sensor
  int distance_in_cm = readDistance();  

    if (distance_in_cm!=-1){    // Only print valid distances
    // Serial.println(threshold_distance-distance_in_cm);
    Serial.println(stepper.distanceToGo());
  }

  // Check if the distance exceeds the threshold and move the stepper motor accordingly
  if (distance_in_cm!=-1 && distance_in_cm!=threshold_distance){
    stepper.move(threshold_distance-distance_in_cm); // Command the stepper to move to the calculated position
  }

  // Run the stepper motor to reach its target position
  if (stepper.distanceToGo() != 0) {
    stepper.run();  // This function must be called repeatedly in the loop to ensure smooth operation.
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

void updateThresholdDistance() {
  // Check if new data is available on the Serial input
  if (Serial.available()) {
    
    String input = Serial.readStringUntil('\n');  // Read the input string until a newline character
    int new_distance = input.toInt();   // Convert input to integer

    if (new_distance > 0) { // Update only if valid value
      threshold_distance = new_distance;
      Serial.print("Updated threshold_distance: ");
      Serial.println(threshold_distance);
    } else {
      Serial.println("Invalid input, keeping previous threshold_distance.");
    }
  }
}

void flushSerial(){
  // Clear any unread data from Serial2 input buffer
  while(Serial2.available()){
    Serial2.read(); // discard incoming bytes
  }
  
}

int readDistance(){
  // Read distance data from the sensor
  if (Serial2.available()) {  // Check if data is available on Serial2
    if (Serial2.read() == HEADER) { // Check if the first byte is the header
      uart_buffer[0] = HEADER;  // Store the header in the buffer

      if (Serial2.read() == HEADER) { // Check if the second byte is also the header
        uart_buffer[1] = HEADER;  // Store the second header in the buffer

        // Read the rest of the data into the buffer
        for (i = 2; i < BUFFER_SIZE; i++) {
          uart_buffer[i] = Serial2.read();
        }

        checksum = uart_buffer[0] + uart_buffer[1] + uart_buffer[2] + uart_buffer[3] + uart_buffer[4] + uart_buffer[5] + uart_buffer[6] + uart_buffer[7];
        
        // Verify checksum with the received checksum byte
        if (uart_buffer[8] == (checksum & reference_checksum)) {
          // Calculate distance
          distance_in_cm = uart_buffer[2] + uart_buffer[3] * 256;
          return distance_in_cm; 
        }
      }
    }
  }

  // If no valid data is received, flush the buffer and return -1
  flushSerial();
  delay(1); // Add a short delay for stability
  return -1;
}
