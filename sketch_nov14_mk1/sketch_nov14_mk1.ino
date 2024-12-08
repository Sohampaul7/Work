#include <SoftwareSerial.h>

fixTOFpin = 2;
movTOFpin = 4;

const int dirPin = ;
const int stepPin = ;

SoftwareSerial TOFfix(fixTOFpin, 3);
SoftwareSerial TOFmov(movTOFpin, 5);

int movDist = 0;
int movStr = 0;
int fixDist = 0;
int fixStr = 0;

int thresholdHeight = ; // The threshold distance of ground from fixed TOF, lesser than which the lower limit switch calibration might be dangerous

void getTOF();
void limitStop();
void bumpStop();
void moveMotor(int steps, int* currPos);

void setup() {
  Serial.begin(9600); // Initialize serial communication
  TOFfix.begin(115200); // Initialize serial communication for TOFfix
  TOFmov.begin(115200); // Initialize serial communication for TOFmov

  getTOF(fixTOFpin); // Get distance value from fixed TOF sensor
  getTOF(movTOFpin); // Get distance value from fixed TOF sensor

  if(limUp == HIGH){    // Check if the assembly is in top position
    dirPin = LOW;       // If in top position, direction of stepper motor is downward
  }
  else{
    dirPin = HIGH;       // If not in top position, direction of stepper motor is upward
  }
  for(int i=0; i<=10; i++){  // Move it a little (10 steps), in the direction mentioned before
    stepPin = HIGH; 
    delay(500);
    stepPin = LOW;
    delay(500);
  }                       // This is done to bring the assembly away from the extreme positions
  while(limUp != HIGH){   // Now move up until the upper limit switch is triggered
    dirPin = HIGH;
    stepPin = HIGH;
    delay(500);
    stepPin = LOW;
    delay(500);
  }

  int count=0;              // Initialise a count to read the number of steps
  getTOF(movTOFpin);        // Get the distance of the moving TOF at the top position
  int movDistUp = movDist;  // Save this for future comparision

  groundDist = getTOF(fixTOFpin);        // Get the distance of the ground from the fixed TOF
  
  if(groundDist<thresholdHeight){       // If the ground distance is lesser than threshold
    for(int i=0; i<=50; i++){           // Move it a little (50 steps), in the direction mentioned before
      dirPin = LOW;
      stepPin = HIGH; 
      delay(500);
      stepPin = LOW;
      delay(500);
      count++; 
  }
  else{
    while(limDown != HIGH){   // Now move down until the lower limit switch is triggered
    dirPin = LOW;
    stepPin = HIGH;
    delay(500);
    stepPin = LOW;
    delay(500);
    count++;                // Count the number of steps required to go from the top position to bottom
    }
  }

  getTOF(movTOFpin);
  int movDistDown = movDist;
  int totalSteps = count;
  float distPerStep = (movDistUp-movDistDown)/totalSteps;
  int currPos = totalSteps;

  for(int n=totalSteps; n<totalSteps/2; n--){   // Move the assembly to middle of the stroke
    dirPin = HIGH;
    stepPin = HIGH;
    delay(500);
    stepPin = LOW;
    delay(500);
    currPos--;
  }

}

void loop() {
  float currDist = (getTOF(movTOFpin)+getTOF(fixTOFpin)-(currPos*distPerStep))/2;
  float reference = getRosRef();

  errorP = reference - currDist;
  errorD = errorDLast - errorP;
  errorI = errorILast + errorP;

  steps = (kp*errorP + ki*errorI + kd*errorD)/distPerStep;

  moveMotor(steps, &currPos);
  errorDLast = errorP;
  errorILast = errorILast + errorP;
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

void moveMotor(int steps, int* currPos){
  if(steps<0){
    dirPin = LOW;
    steps = -steps;

    for(i=0; i<steps; i++){
      limitStop();
      bumpStop();
      stepPin = HIGH;
      delay(500);
      stepPin = LOW;
      delay(500);
      (*currPos)--;
    }
  }
  else{
    dirPin = HIGH;

    for(i=0; i<steps; i++){
      limitStop();
      bumpStop();
      stepPin = HIGH;
      delay(500);
      stepPin = LOW;
      delay(500);
      (*currPos)++;
    }
  }
}

void limitStop(){
  if(limUp == HIGH or limDown == HIGH){
    stepPin = LOW;
    delay(500);
    return
  }
}

void bumpStop(){
  if(bump1 == HIGH or bump2 == HIGH){
    stepPin = LOW;
    delay(500);
    // Send message to ros2
    return
  }
}
