#include <Arduino.h>
#include <mcp_can.h>
#include <SPI.h>
#include "RS485CTRL.h"

#define PIN_RS485_RX 15
#define PIN_RS485_TX 14
//********************************manip these
bool debug_serialRead=false;
bool debug_printValues=false;
unsigned long pingInterval = 3;
unsigned long PrintDebugInterval = 500;
unsigned long SendCanIntervalMS = 50;
//********************************DONT TOUCH THESE
unsigned long lastPing24 = 0;
unsigned long lastPing58 = 0;
unsigned long lastPing54 = 0;
unsigned long lastPing5C = 0;
unsigned long lastPrintDebug = 0;
unsigned long lastCanSend = 0;
byte data[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
int rez12_24 = 0;
int rez12_58 = 0;
int rez12_54 = 0;
int rez12_5C = 0;

MCP_CAN CAN0(10);

void initSerial() {
  Serial.begin(115200);
  delay(200);
  Serial3.begin(115200);
  Serial.println("RS485-Test");
  delay(200);
}

void initCAN() {
  SPI.begin();
  if (CAN0.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("MCP2515 Initialized Successfully!");
  } else {
    Serial.println("Error Initializing MCP2515...");
  }
  CAN0.setMode(MCP_NORMAL);
}

int processSensorData(byte data[2]) {
  int rez12 = (((int)(data[1] & 0x3F) << 8) | data[0]) >> 2;
  return rez12;
}

void handlePing(byte sensorID, int &rez12) {
  unsigned long &lastPing = (sensorID == 0x54) ? lastPing54 : (sensorID == 0x24) ? lastPing24 : (sensorID == 0x5C) ? lastPing5C : lastPing58;

  if (millis() - lastPing >= pingInterval) {
    lastPing = millis();
    Serial3.write(sensorID);
    delay(1);

    if (Serial3.available() >= 2) {
      byte Data_rec[2];
      Data_rec[0] = Serial3.read();
      Data_rec[1] = Serial3.read();
      rez12 = processSensorData(Data_rec);
    } else {
      if(debug_serialRead){
        Serial.print("No data ");
        Serial.println(sensorID, HEX);
      }
    }
    while (Serial3.available()) Serial3.read();
  }
}

void printValues() {
  if(!debug_printValues)return;
  if (millis() - lastPrintDebug >= PrintDebugInterval) {
    lastPrintDebug = millis();
    Serial.print(" 58: "); Serial.print(rez12_58); Serial.print("   ");
    Serial.print(" 24: "); Serial.print(rez12_24);
    Serial.print(" 54: "); Serial.print(rez12_54); Serial.print("   ");
    Serial.print(" 5C: "); Serial.print(rez12_5C);
    Serial.println("   ");
  }
}

void sendCAN() {
  if (millis() - lastCanSend >= SendCanIntervalMS) {
    lastCanSend = millis();

    data[0] = rez12_58 & 0xFF;
    data[1] = (rez12_58 >> 8) & 0xFF;

    data[2] = rez12_24 & 0xFF;
    data[3] = (rez12_24 >> 8) & 0xFF;

    data[4] = rez12_54 & 0xFF;
    data[5] = (rez12_54 >> 8) & 0xFF;

    data[6] = rez12_5C & 0xFF;
    data[7] = (rez12_5C >> 8) & 0xFF;

    CAN0.sendMsgBuf(0x18FFFA00, 1, 8, data);
  }
}

//*********************************setup and void loop()
void setup() {
  initSerial();
  initCAN();
}

void loop() {
  handlePing(0x54, rez12_54);
  handlePing(0x24, rez12_24);
  handlePing(0x5C, rez12_5C);
  handlePing(0x58, rez12_58);
  printValues();
  sendCAN();
}
 