#include <Arduino.h>

const int buttonPin = 10;  // Pin for the push button
bool buttonPressed = false;
bool resetRequested = false;

void setup() {
    Serial.begin(9600);  // Initialize serial communication
    pinMode(buttonPin, INPUT_PULLUP);  // Button with pull-up resistor
    
    Serial.println("Setup complete");
}

void loop() {
    // Check for incoming data from the client
    if (Serial.available() > 0 && !buttonPressed) {
        String receivedString = Serial.readStringUntil('\n');  // Read until newline
        if (receivedString.length() > 0) {  // Ensure the string is not empty
            int receivedValue = receivedString.toInt();  // Convert to integer
            Serial.print("Received value: ");
            Serial.println(receivedValue);  // Display the value
        }
    }

    // Monitor the button state and send a warning if pressed
    if (digitalRead(buttonPin) == HIGH && !buttonPressed) {
        Serial.println("BUTTON_PRESSED");
        buttonPressed = true;
    } 
    
    // Handle reset if 'r' is received
    if (Serial.available() > 0) {
        String command = Serial.readStringUntil('\n');  // Read the incoming command
        if (command == "r") {
            resetRequested = true;
        }
    }

    // Reset the button press state if 'r' is received
    if (resetRequested) {
        buttonPressed = false;
        resetRequested = false;
        Serial.println("RESET");
    }
}
