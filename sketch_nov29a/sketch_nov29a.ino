const int dirPin = 10;
const int stepPin = 11;
const int stepNum = 200;
const int stepDelay = 800;

void setup() {
  Serial.begin(9600);

  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
}

void loop() {
  digitalWrite(dirPin, LOW);

  Serial.println("Spinning!");

  for (int i = 0; i < stepNum; i++)
  {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(stepDelay);
  }

  delay(2000);
}
