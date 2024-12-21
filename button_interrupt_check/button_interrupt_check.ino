
#define BUTTON_PIN 33  // GPIO pin connected to the button

volatile bool stopMotorFlag = false; // Flag to stop the motor

void setup() {
  Serial.begin(9600);       // Debugging output to Serial Monitor
  pinMode(BUTTON_PIN, INPUT_PULLUP);  // Configure button pin with internal pull-up resistor
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), stopButtonPressed, FALLING);  // Trigger on falling edge

  Serial.println("Setup complete");
  delay(2000);
}

void loop() {
  if (stopMotorFlag) {
    Serial.println("Motor stopped. Press 'r' to restart.");
    if (Serial.available() && Serial.read() == 'r') {
      stopMotorFlag = false;
      Serial.println("Motor restarted.");
    }
    return;
  }

}

// ISR to set the flag
void stopButtonPressed() {
  stopMotorFlag = true; // Set flag to stop the motor
}
