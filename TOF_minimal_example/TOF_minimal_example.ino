
void setup() {
  Serial.begin(9600);       // Debugging output to Serial Monitor
  Serial2.begin(115200);    // TFMini-S is connected to Serial2 (pins 7 and 8)
  
  Serial.println("System Initialized. Starting the loop.");
}

void loop() {
  Serial.println("in loop");
  if (Serial2.available()){
    Serial.println("Data available");
  }
  delay(500);
}
