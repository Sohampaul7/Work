#define STEP_PIN 3    // Pin connected to stepper motor step input
#define DIR_PIN 4     // Pin connected to stepper motor direction input
#define HEADER 0x59   // Frame starting byte (0x59 for TFMini-S)
#define BUFFER_SIZE 9 // Size of the data packet

// Variables for ToF sensor
uint8_t uart[BUFFER_SIZE];
int16_t dist;
uint8_t chk;
int i;
const float stepPerDegree = 200.0/360.0;
const float requiredAngle = 360.0;
const int delayPerStepMicrosec = 1000; // keep between 1000 to 2400 
int threshold_distance = 10;

void setup() {
  pinMode(STEP_PIN, OUTPUT); // Stepper motor step pin
  pinMode(DIR_PIN, OUTPUT);  // Stepper motor direction pin
  
  Serial.begin(9600);       // Debugging output to Serial Monitor
  Serial2.begin(115200);    // TFMini-S is connected to Serial2 (pins 7 and 8)

  Serial.println("Setup complete");
}

void loop() {  
  
  // Update threshold distance from Serial input if available
  updateThresholdDistance();
  
  int distance = readDistance();
  
  if (distance!=-1){
    Serial.print("Measured distance: ");
    Serial.println(distance);
    if (distance != threshold_distance){
      if (threshold_distance-distance>0){
        oneStep(HIGH);
      }else{
      oneStep(LOW);
      }
    }
  }
}

void updateThresholdDistance() {
  if (Serial.available()) {
    int new_distance = Serial.parseInt(); // Parse new value from Serial

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
  while(Serial2.available()){
    Serial2.read(); // discard incoming bytes
  }
  
}

int readDistance(){
  
  // Read distance data from the sensor
  if (Serial2.available()) {
    if (Serial2.read() == HEADER) {
      uart[0] = HEADER;

      if (Serial2.read() == HEADER) {
        uart[1] = HEADER;

        for (i = 2; i < BUFFER_SIZE; i++) {
          uart[i] = Serial2.read();
        }

        // Calculate chksum
        chk = uart[0] + uart[1] + uart[2] + uart[3] + uart[4] + uart[5] + uart[6] + uart[7];

        if (uart[8] == (chk & 0xFF)) {
          // Calculate distance
          dist = uart[2] + uart[3] * 256;
          return dist; 
        }
      }
    }
  }

  //Serial.println(dist);
  flushSerial();
  delay(1);
  return -1;
}

void oneStep(bool motor_direction){
  
  digitalWrite(DIR_PIN, motor_direction); 
  
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(delayPerStepMicrosec/2);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(delayPerStepMicrosec/2);
}
