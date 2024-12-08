#include <SoftwareSerial.h>

const int fixTOFpin = 2;
const int movTOFpin = 4;
const int dirPin = /* Set pin number */;
const int stepPin = /* Set pin number */;
const int limUp = /* Set pin number */;
const int limDown = /* Set pin number */;
const int bump1 = /* Set pin number */;
const int bump2 = /* Set pin number */;

const int thresholdHeight = /* Set threshold height */;
const float kp = /* Set proportional gain */;
const float ki = /* Set integral gain */;
const float kd = /* Set derivative gain */;

SoftwareSerial TOFfix(fixTOFpin, 3);
SoftwareSerial TOFmov(movTOFpin, 5);

int movDist = 0;
int movStr = 0;
int fixDist = 0;
int fixStr = 0;
float errorDLast = 0;
float errorILast = 0;

float groundDist = 0;
int currPos = 0;
float distPerStep = 0;

void getTOF(int pin);
void limitStop();
void bumpStop();
void moveMotor(int steps, int* currPos);
float getRosRef();  // Define this function to fetch the reference distance from ROS2

void setup() {
  Serial.begin(9600);
  TOFfix.begin(115200);
  TOFmov.begin(115200);

  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(limUp, INPUT);
  pinMode(limDown, INPUT);
  pinMode(bump1, INPUT);
  pinMode(bump2, INPUT);

  if(limUp == HIGH){    // Check if the assembly is in top position
    digitalWrite(dirPin, LOW);      // If in top position, direction of stepper motor is downward
  }
  else{
    digitalWrite(dirPin, HIGH);       // If not in top position, direction of stepper motor is upward
  }
  
  for(int i=0; i<=10; i++){  // Move it a little (10 steps), in the direction mentioned before
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);      // Step pulse width
    digitalWrite(stepPin, LOW);
    delay(500);
  }                       // This is done to bring the assembly away from the extreme positions
  
  while(limUp != HIGH){   // Now move up until the upper limit switch is triggered
    digitalWrite(dirPin, HIGH);
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);      // Step pulse width
    digitalWrite(stepPin, LOW);
    delay(500);
  }

  int count=0;              // Initialise a count to read the number of steps
  getTOF(movTOFpin);        // Get the distance of the moving TOF at the top position
  int movDistUp = movDist;  // Save this for future comparision

  getTOF(fixTOFpin);
  groundDist = fixDist;        // Get the distance of the ground from the fixed TOF
  
  if(groundDist<thresholdHeight){       // If the ground distance is lesser than threshold
    for(int i=0; i<=50; i++){           // Move it a little (50 steps), in the direction mentioned before
      digitalWrite(dirPin, LOW);
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(500);      // Step pulse width
      digitalWrite(stepPin, LOW);
      count++; 
    }
  }
  else{
    while(limDown != HIGH){   // Now move down until the lower limit switch is triggered
    digitalWrite(dirPin, HIGH);
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);      // Step pulse width
    digitalWrite(stepPin, LOW);
    count++;                // Count the number of steps required to go from the top position to bottom
    }
  }

  getTOF(movTOFpin);
  int movDistDown = movDist;
  int totalSteps = count;
  float distPerStep = (movDistUp-movDistDown)/totalSteps;
  int currPos = totalSteps;

  for(int n=totalSteps; n<totalSteps/2; n--){   // Move the assembly to middle of the stroke
    digitalWrite(dirPin, HIGH);
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(500);      // Step pulse width
    digitalWrite(stepPin, LOW);
    delayMicroseconds(500);
    currPos--;
  }

}

void loop() {
  float currDist = (movDist + fixDist - (currPos * distPerStep)) / 2;
  float reference = getRosRef();

  float errorP = reference - currDist;
  float errorD = errorDLast - errorP;
  float errorI = errorILast + errorP;

  int steps = (kp * errorP + ki * errorI + kd * errorD) / distPerStep;

  moveMotor(steps, &currPos);
  errorDLast = errorP;
  errorILast += errorP;
}

void getTOF(int pin) {
  static char i = 0;
  char j = 0;
  int checksum = 0;
  static int rx[9];

  // Set serialPort to TOFfix or TOFmov based on pin
  SoftwareSerial* serialPort = (pin == fixTOFpin) ? &TOFfix : &TOFmov;

  // Choose the correct distance and strength variables
  int* distance = (pin == fixTOFpin) ? &fixDist : &movDist;
  int* strength = (pin == fixTOFpin) ? &fixStr : &movStr;

  if (serialPort->available()) {
    rx[i] = serialPort->read();
    if (rx[0] != 0x59) {
      i = 0;
    } else if (i == 1 && rx[1] != 0x59) {
      i = 0;
    } else if (i == 8) {
      for (j = 0; j < 8; j++) {
        checksum += rx[j];
      }
      if (rx[8] == (checksum % 256)) {
        *distance = rx[2] + rx[3] * 256;
        *strength = rx[4] + rx[5] * 256;
      }
      i = 0;
    } else {
      i++;
    }
  }
}

void moveMotor(int steps, int* currPos) {
  if(steps<0){
    dirPin = LOW;
    steps = -steps;

    for(int i=0; i<steps; i++){
      limitStop();
      bumpStop();
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(500);      // Step pulse width
      digitalWrite(stepPin, LOW);
      delayMicroseconds(500);
      (*currPos)--;
    }
  }
  else{
    dirPin = HIGH;

    for(int i=0; i<steps; i++){
      limitStop();
      bumpStop();
      digitalWrite(stepPin, HIGH);
      delayMicroseconds(500);      // Step pulse width
      digitalWrite(stepPin, LOW);
      delayMicroseconds(500);
      (*currPos)++;
    }
  }
}

void limitStop() {
  if (digitalRead(limUp) == HIGH || digitalRead(limDown) == HIGH) {
    digitalWrite(stepPin, LOW);
    delay(500);
    return;
  }
}

void bumpStop() {
  if (digitalRead(bump1) == HIGH || digitalRead(bump2) == HIGH) {
    digitalWrite(stepPin, LOW);
    delay(500);
    // Send message to ROS2
    return;
  }
}

float getRosRef(){
  
}
