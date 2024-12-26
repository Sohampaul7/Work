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

const float stepPerDegree = 200.0/360.0;
const float requiredAngle = 360.0;
const int delayPerStepMicrosec = 1000; // keep between 1000 to 2400 

// Create an instance of AccelStepper
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

void setup()
{  
  // Set the maximum speed and acceleration
  stepper.setMaxSpeed(1000);   // Maximum speed in steps per second
  stepper.setAcceleration(2000); // Acceleration in steps per second^2
  
  Serial.begin(9600);       // Debugging output to Serial Monitor
  Serial2.begin(115200);    // TFMini-S is connected to Serial2 (pins 7 and 8)
  
  Serial.println("Setup complete");
  delay(2000);
}

void loop() {
  
  int distance = readDistance();

  if (distance!=-1){
    stepper.moveTo(distance);
    Serial.println(distance);
  }
  
  if (stepper.distanceToGo() != 0) {
    stepper.run();
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
