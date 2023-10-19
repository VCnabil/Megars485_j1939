#include <Arduino.h>
#include <mcp_can.h>
#include <SPI.h>          //Library for using SPI Communication 
const int DE_RE_PIN = 4; //
unsigned long lastT = 0;
unsigned long lastJ = 0;
int cnt=0;
byte receivedData[2];
int rez14;
int rez12;
byte rez12_b0_LOW;
byte rez12_b1_HIGH;
byte rez14_b0_LOW;
byte rez14_b1_HIGH;
MCP_CAN CAN0(10);     // Set CS to pin 10
byte data[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
void setup() {
  Serial.begin(115200);  
  pinMode(DE_RE_PIN, OUTPUT);
  digitalWrite(DE_RE_PIN, LOW); // Initialize DE/RE pin to LOW for receiving
  Serial1.begin(2000000);  
  SPI.begin(); 
  if(CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) == CAN_OK) Serial.println("MCP2515 Initialized Successfully!");
  else Serial.println("Error Initializing MCP2515...");
  CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted
}
void loop() {
if (millis() - lastT >= 50) {  
    lastT = millis();  
    digitalWrite(DE_RE_PIN, HIGH); // HIGH for transmitting
    Serial1.write(0x54);
    digitalWrite(DE_RE_PIN, LOW); // LOW for receiving
  }
  if (Serial1.available() >= 2) {
    //receive 1A 16 
    //        16 1A  is 5658 14bit 
    byte reversedLowByte =Serial1.read();
    byte reversedHighByte =Serial1.read();
    receivedData[0] = reversedLowByte;
    receivedData[1] = reversedHighByte;
    byte b0 = reversedLowByte;// 0xE4;
    byte b1temp = reversedHighByte;// 0xF9;
    byte b1 = (byte)(b1temp & 0x3F);
    rez14 = (int)(b1 << 8) | b0;
    rez12 =(int) rez14 >> 2;
    int value1 = rez12;// 0xABCD; // 16-bit integer
    byte lowByte1 = (byte)(value1 & 0xFF);
    byte highByte1 = (byte)((value1 >> 8) & 0xFF);
    rez12_b0_LOW = lowByte1;
    rez12_b1_HIGH = highByte1;
    int value2 = rez14;// 0xABCD; // 16-bit integer
    byte lowByte2 = (byte)(value2 & 0xFF);
    byte highByte2 = (byte)((value2 >> 8) & 0xFF);
    rez14_b0_LOW = lowByte2;
    rez14_b1_HIGH = highByte2;
    // // Debugging output
    // Serial.print("Received bytes: ");
    // Serial.print(receivedData[0], HEX);
    // Serial.print(", ");
    // Serial.println(receivedData[1], HEX);
    // Serial.print("rez12: ");
    // Serial.println(rez12);
    // Serial.print("rez14: ");
    // Serial.println(rez14);
  }
  if (millis() - lastJ >= 50) {  
    lastJ = millis();  
    cnt++;
    if (cnt > 254) cnt = 0;
    data[0] = rez12_b0_LOW;
    data[1] =  rez12_b1_HIGH;
    data[2] = rez14_b0_LOW;
    data[3] = rez14_b1_HIGH;
    data[6] = (byte)cnt;
    data[7] = (byte) (cnt/2);
    CAN0.sendMsgBuf(0x18FFFA00, 1, 8, data);
  }
}