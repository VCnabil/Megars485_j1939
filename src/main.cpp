#include <Arduino.h>
 

 
const int DE_RE_PIN = 4; // DE and RE connected to pin 9
unsigned long lastT = 0;
byte receivedData[2];

void setup() {
 
 
  pinMode(DE_RE_PIN, OUTPUT);
  digitalWrite(DE_RE_PIN, LOW); // Initialize DE/RE pin to LOW for receiving
  Serial1.begin(2000000); // Initialize Serial1 at 2 Mbps
  Serial.begin(115200); // Initialize Serial for debugging
 
   
}

// the loop function runs over and over again forever
void loop() {
if (millis() - lastT >= 50) { // Timer set to 50ms
    lastT = millis();
    
    // Send 0x54 to sensor
    digitalWrite(DE_RE_PIN, HIGH); // Set to HIGH for transmitting
    Serial1.write(0x54);
    digitalWrite(DE_RE_PIN, LOW); // Set back to LOW for receiving
  }
  // Wait to receive 2 bytes
  if (Serial1.available() >= 2) {
    //receive 1A 16 
    //        16 1A  is 5658 14bit 
    receivedData[0] = Serial1.read();
    receivedData[1] = Serial1.read();

    // Process received data
    byte b0 = receivedData[0];
    byte b1 = receivedData[1] & 0x3F;
    int rez14 = (int)(b1 << 8) | b0;
     
    // Debugging output
    Serial.print("Received bytes: ");
    Serial.print(receivedData[0], HEX);
    Serial.print(", ");
    Serial.println(receivedData[1], HEX);
    Serial.print("rez14: ");
    Serial.println(rez14);
     
  }
}
 