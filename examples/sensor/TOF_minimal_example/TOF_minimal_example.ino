#define HEADER 0x59   // Frame starting byte (0x59 for TFMini-S)
#define BUFFER_SIZE 9 // Size of the data packet

// Variables for ToF sensor
uint8_t uart_buffer[BUFFER_SIZE];
int16_t distance_in_cm;
uint8_t checksum;
uint8_t reference_checksum = 0xFF;
int i;

void setup() {
  Serial.begin(9600);       // Debugging output to Serial Monitor
  Serial2.begin(115200);    // TFMini-S is connected to Serial2 (pins 7 and 8)
  
  Serial.println("System Initialized. Starting the loop.");
}

void loop() {
  // Read distance data from the sensor
  if (Serial2.available()) {
    if (Serial2.read() == HEADER) {
      uart_buffer[0] = HEADER;

      if (Serial2.read() == HEADER) {
        uart_buffer[1] = HEADER;

        for (i = 2; i < BUFFER_SIZE; i++) {
          uart_buffer[i] = Serial2.read();
        }

        checksum = uart_buffer[0] + uart_buffer[1] + uart_buffer[2] + uart_buffer[3] + uart_buffer[4] + uart_buffer[5] + uart_buffer[6] + uart_buffer[7];

        if (uart_buffer[8] == (checksum & reference_checksum)) {
          // Calculate distance_in_cm
          distance_in_cm = uart_buffer[2] + uart_buffer[3] * 256;
        }
        else{
          Serial.println("Checksum error");
        }
      }
    }
  }
  Serial.println("Distance: " + String(distance_in_cm) + " cm");
  delay(5);
}
