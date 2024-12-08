#include <SoftwareSerial.h>

#define RX_MOV 2 // Moving TOF sensor Rx pin 
#define TX_MOV 3 // Moving TOF sensor Tx pin
#define RX_FIX 4 // Fixed TOF sensor Rx pin
#define TX_FIX 5 // Fixed TOF sensor Tx pin

SoftwareSerial tfminiMove(RX_MOV, TX_MOV); // Creating a object moving for TOF sensor
SoftwareSerial tfminiFix(RX_FIX, TX_FIX);  // Creating a object fixed for TOF sensor

const int stepPin = 6; // Step pin for Stepper Motor
const int dirPin = 7; // Direction pin for Stepper Motor

const int topSw = 12; // Pin for top limit switch
const int lowSw = 13; // Pin for lower limit switch

const int bumpSw1 = 8; // Pin for 1st bump switch
const int bumpSw2 = 9; // Pin for 2nd bump switch

int16_t distance_mov; // Distance measured by moving TOF 
int16_t distance_fix; // Distance measured by fixed TOF 
int16_t distance_between;        // Actual distance in mm between TOF sensors when in topmost position
int16_t referenceDistance = 100; // Targetted distance in mm between moving TOF and ground                    (adjust this value)

float Kp = 0.10; // Proportional constant   (distance error => movement time)                                 (adjust this value)
float Ki = 0.05; // Integral constant       (sum of distance error => movement time)                          (adjust this value)
float Kd = 0.01; // Derivative constant     (change in distance error => movement time)                       (adjust this value)

// Variables for PID
int16_t error_sum = 0;        // Integral sum
int16_t last_error = 0;       // Previous error for derivative calculation

const int16_t max_error_sum = 1000; // Upper bound to prevent windup
const int16_t min_error_sum = -1000; // Lower bound to prevent windup

// Function prototypes
void calibrateMotor();                // Calibrates the motor with the limit switch and TOF 
int16_t readMovingSensorDistance();   // Gives the distance value from moving TOF
int16_t readFixedSensorDistance();    // Gives the distance value from fixed TOF
void moveUp(int16_t delay);           // Moves the stepper up acc to the movement time
void moveDown(int16_t delay);         // Moves the stepper up acc to the movement time
void limitStop();                     // Stops the activity if limit switch triggered
void bumpStop();                      // Stops the activity if bump switch triggered

void setup() {
  Serial.begin(9600);
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(topSw, INPUT);
  pinMode(lowSw, INPUT);
  pinMode(bumpSw1, INPUT);
  pinMode(bumpSw2, INPUT);

  // Initialize TFmini-S sensors
  tfminiMove.begin(115200);
  tfminiFix.begin(115200);

  calibrateMotor();

}

void loop() {
  
  // Check the limit switches and the bumper switches
  limitStop();
  bumpStop();

  // Read the values from both TOF sensors
  distance_mov = readMovingSensorDistance();
  distance_fix = readFixedSensorDistance();  

  // Remove erroneous values from both TOF sensors
  if (distance_mov == -1 || distance_fix == -1) {
    Serial.println("Error reading TOF sensors. Motor operation halted.");
    digitalWrite(stepPin, LOW); // Stop the motor
    return;
  }

  // Calculate the average error from the reference value of bot the TOF sensors
  int16_t error_mov = referenceDistance - distance_mov;
  int16_t error_fix = referenceDistance - (distance_fix - distance_between);
  
  int16_t error_avg = (error_mov + error_fix) / 2;
  
  // Calculate PID terms
  error_sum += error_avg;                        // Integral term
  if (error_sum > max_error_sum) {// Anti windup limits 
    error_sum = max_error_sum;
  } else if (error_sum < min_error_sum) {
    error_sum = min_error_sum;
  }
  int16_t d_error = error_avg - last_error;      // Derivative term
  last_error = error_avg;                        // Store error for next iteration

  unsigned long delayTime = max(abs(Kp * error_avg + Ki * error_sum + Kd * d_error), 100);

  if (abs(error_avg) < 5) { // Dead zone: Motor stays still if error_avg is within a small threshold
  Serial.println("Error within dead zone. Motor remains stationary.");
  digitalWrite(stepPin, LOW);
  } 
  else {
    if (error_avg > 0) {
      moveUp(delayTime);
    } else {
      moveDown(delayTime);
    }
  }

  Serial.print(F("Distance: "));
  Serial.print((distance_mov + (distance_fix - distance_between)) / 2);
  Serial.print(" mm, Error: ");
  Serial.print(error_avg);
  Serial.print(", Delay Time: ");
  Serial.println(delayTime);
}

// Function definitions
void calibrateMotor() {
  int16_t distance_sum = 0;
  int16_t count = 0;
  int16_t pos = 0;

  // Step 1: Move up until top limit switch is hit
  while (digitalRead(topSw) == LOW) {
    moveUp();
  }
  Serial.println("Reached top position.");
  
  while (digitalRead(lowSw) == LOW) {
    moveDown(10);
    pos++;
  }
  Serial.println("Reached bottom position.");

  moveUp(10);
  int total_steps = pos;
  
  Serial.println("Calibration complete.");
}


int16_t readMovingSensorDistance() {
  if (tfminiMove.available()) {
    uint8_t byte1 = tfminiMove.read();
    if (byte1 == 0x59) {  // Frame starting byte
      uint8_t byte2 = tfminiMove.read();
      if (byte2 == 0x59) {  // Second frame starting byte
        uint8_t low = tfminiMove.read();
        uint8_t high = tfminiMove.read();
        return (high << 8) + low;  // Combine bytes to get distance
      }
    }
  }
  delay(10);
  return -1;  // Error or no data
}

int16_t readFixedSensorDistance() {
  if (tfminiFix.available()) {
    uint8_t byte1 = tfminiFix.read();
    if (byte1 == 0x59) {  // Frame starting byte
      uint8_t byte2 = tfminiFix.read();
      if (byte2 == 0x59) {  // Second frame starting byte
        uint8_t low = tfminiFix.read();
        uint8_t high = tfminiFix.read();
        return (high << 8) + low;  // Combine bytes to get distance
      }
    }
  }
  delay(10);
  return -1;  // Error or no data
}

void moveUp(){
  if (digitalRead(topSw) == LOW) { // Ensure top switch is not triggered
    digitalWrite(dirPin, HIGH);
    digitalWrite(stepPin, HIGH);
    delay(500);
    digitalWrite(stepPin, LOW);
    delay(500);
  }
}

void moveDown(){
  if (digitalRead(lowSw) == LOW) { // Ensure bottom switch is not triggered
    digitalWrite(dirPin, LOW);
    digitalWrite(stepPin, HIGH);
    delay(500);
    digitalWrite(stepPin, LOW);
    delay(500);
  }
}

void limitStop(){
  if (digitalRead(topSw) == HIGH || digitalRead(lowSw) == HIGH) {
    Serial.println("Limit switch triggered. Motor stopped.");
    digitalWrite(stepPin, LOW);
    return;
  }
}

void bumpStop(){
  if (digitalRead(bumpSw1) == HIGH || digitalRead(bumpSw2) == HIGH) {
    Serial.println("Bump switch triggered. Motor stopped.");
    digitalWrite(stepPin, LOW);
    // Send stop command to vehicle
    return;
  }
}