
#include <Arduino.h>
#include <mcp_can.h>
#include <SPI.h>          
#include "RS485CTRL.h" 
#define PIN_RS485_RX 15
#define PIN_RS485_TX 14
unsigned long lastCanSend = 0;
unsigned long SendCanIntervalMS= 50;
int cnt=0;
MCP_CAN CAN0(10);     
byte data[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
byte Data_rec_24[2];
byte Data_rec_58[2];
byte Data_rec_54[2];
byte Data_rec_5C[2];
byte Data_12bit_24[2];
byte Data_12bit_58[2];
byte Data_12bit_54[2];
byte Data_12bit_5C[2];
int rez12_24=0;
int rez12_58=0;
int rez12_54=0;
int rez12_5C=0;
unsigned long lastPing24 = 0;
unsigned long lastPing58 = 0;
unsigned long lastPing54 = 0;
unsigned long lastPing5C = 0;
unsigned long lastPrintDebug = 0;
unsigned long PrintDebugInterval = 400;
unsigned long pingInterval = 3;  
void setup() {
  Serial.begin(115200);  
  delay(500);
   Serial3.begin(115200);
  Serial.println("RS485-Test");
  delay(200);
  SPI.begin(); 
  if(CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) == CAN_OK) Serial.println("MCP2515 Initialized Successfully!");
  else Serial.println("Error Initializing MCP2515...");
  CAN0.setMode(MCP_NORMAL);   
}
void Ping_5C() {
  if (millis() - lastPing5C >= pingInterval) {
    lastPing5C = millis();
    Serial3.write(0x5C);
    delay(1);  
    if (Serial3.available() >= 2) {
      Data_rec_5C[0] = Serial3.read();
      Data_rec_5C[1]= Serial3.read();
      byte raw_b0_LOW = Data_rec_5C[0];
      byte raw_b1_HIGH = Data_rec_5C[1];
     int rez12 =(((int)(raw_b1_HIGH & 0x3F) << 8) | raw_b0_LOW) >> 2;
      rez12_5C = rez12;
      byte rez12_b0_LOW = (byte)(rez12 & 0xFF);
      byte rez12_b1_HIGH = (byte)((rez12 >> 8) & 0xFF);
      Data_12bit_5C[0]=rez12_b0_LOW;
      Data_12bit_5C[1]=rez12_b1_HIGH;
    } else {
      Serial.println("No data received from Sensor 5C");
    }
    while (Serial3.available()) Serial3.read();  
  }
}
void Ping_54() {
  if (millis() - lastPing54 >= pingInterval) {
    lastPing54 = millis();
    Serial3.write(0x54);
    delay(1);  
    if (Serial3.available() >= 2) {
      Data_rec_54[0] = Serial3.read();
      Data_rec_54[1]= Serial3.read();
      byte raw_b0_LOW = Data_rec_54[0];
      byte raw_b1_HIGH = Data_rec_54[1];
     int rez12 =(((int)(raw_b1_HIGH & 0x3F) << 8) | raw_b0_LOW) >> 2;
      rez12_54 = rez12;
      byte rez12_b0_LOW = (byte)(rez12 & 0xFF);
      byte rez12_b1_HIGH = (byte)((rez12 >> 8) & 0xFF);
      Data_12bit_54[0]=rez12_b0_LOW;
      Data_12bit_54[1]=rez12_b1_HIGH;
    } else {
      Serial.println("No data received from Sensor 54");
    }
    while (Serial3.available()) Serial3.read();  
  }
}
void Ping_24() {
  if (millis() - lastPing24 >= pingInterval) {
    lastPing24 = millis();
    Serial3.write(0x24);
    delay(1);  
    if (Serial3.available() >= 2) {
      Data_rec_24[0] = Serial3.read();
      Data_rec_24[1]= Serial3.read();
      byte raw_b0_LOW = Data_rec_24[0];
      byte raw_b1_HIGH = Data_rec_24[1];
     int rez12 =(((int)(raw_b1_HIGH & 0x3F) << 8) | raw_b0_LOW) >> 2;
      rez12_24 = rez12;
      byte rez12_b0_LOW = (byte)(rez12 & 0xFF);
      byte rez12_b1_HIGH = (byte)((rez12 >> 8) & 0xFF);
      Data_12bit_24[0]=rez12_b0_LOW;
      Data_12bit_24[1]=rez12_b1_HIGH;
    } else {
      Serial.println("No data received from Sensor 24");
    }
    while (Serial3.available()) Serial3.read();  
  }
}
void Ping_58() {
  if (millis() - lastPing58 >= pingInterval) {
    lastPing58 = millis();
    Serial3.write(0x58);
    delay(1);  
    if (Serial3.available() >= 2) {
      Data_rec_58[0] = Serial3.read();
      Data_rec_58[1]= Serial3.read();
      byte raw_b0_LOW = Data_rec_58[0];
      byte raw_b1_HIGH = Data_rec_58[1];
      int rez12 =(((int)(raw_b1_HIGH & 0x3F) << 8) | raw_b0_LOW) >> 2;
      rez12_58 = rez12;
      byte rez12_b0_LOW = (byte)(rez12 & 0xFF);
      byte rez12_b1_HIGH = (byte)((rez12 >> 8) & 0xFF);
      Data_12bit_58[0]=rez12_b0_LOW;
      Data_12bit_58[1]=rez12_b1_HIGH;
    } else {
      Serial.println("No data received from Sensor 58");
    }
    while (Serial3.available()) Serial3.read();  
  }
}
void RunCanSend(){
   if (millis() - lastCanSend >= SendCanIntervalMS) {  
    lastCanSend = millis();  
    cnt++;
    if (cnt > 254) cnt = 0;
    data[0] = Data_12bit_58[0];
    data[1] =  Data_12bit_58[1];
    data[2] = Data_12bit_24[0];
    data[3] = Data_12bit_24[1];
    data[6] = (byte)cnt;
    data[7] = (byte) (cnt/2);
    CAN0.sendMsgBuf(0x18FFFA00, 1, 8, data);
  }
}
void RunPRintValues(){
    if (millis() - lastPrintDebug >= PrintDebugInterval) {
    lastPrintDebug = millis();
    Serial.print(" 58: " ); Serial.print(rez12_58); Serial.print("   "); Serial.print(" 24: " ); Serial.print(rez12_24); 
    Serial.print(" 54: " ); Serial.print(rez12_54); Serial.print("   "); Serial.print(" 5C: " ); Serial.print(rez12_5C); 
    Serial.println("   ");
  }
}
void loop() {
  //  Ping_54();
    Ping_24();
   // Ping_5C();
    Ping_58();
    //RunPRintValues();
    RunCanSend();
}
