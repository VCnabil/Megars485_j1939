//works as is
#include <Arduino.h>
#include <mcp_can.h>
#include <SPI.h>

#define PIN_RS485_RX 15
#define PIN_RS485_TX 14
//********************************manip these
bool debug_serialRead=false;
bool debug_printValues=false;
unsigned long pingInterval = 2;
// unsigned long pingInterval_2mbps = 1;
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
const int DE_RE_PIN = 5; //
unsigned long lastT = 0;
unsigned long lastPing14 = 0;
unsigned long lastPing18 = 0; 
 
int rez12_14 = 0;
int rez12_18 = 0;

void initSerial() {
  Serial.begin(115200);
  delay(200);
  Serial3.begin(115200);
  Serial.println("RS485-Test");
  delay(200);

    pinMode(DE_RE_PIN, OUTPUT);
  digitalWrite(DE_RE_PIN, LOW); // Initialize DE/RE pin to LOW for receiving
  Serial2.begin(2000000);  
   
  
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
    if (Serial3.available() == 2) {
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
void ping_fixed2mbps(byte sensorID, int &rez12){
    unsigned long &lastPing = (sensorID == 0x14) ? lastPing14 :  lastPing18  ;
    if (millis() - lastPing >= pingInterval) {  
        lastPing = millis();  
        digitalWrite(DE_RE_PIN, HIGH); // HIGH for transmitting
        Serial2.write(sensorID);
        digitalWrite(DE_RE_PIN, LOW); // LOW for receiving
        delay(1);
        if (Serial2.available() == 2) {
          byte Data_rec[2];
          Data_rec[0] = Serial2.read();
          Data_rec[1] = Serial2.read();
          rez12 = processSensorData(Data_rec);
        }
        while (Serial2.available()) Serial2.read();
      }
}
void handleGeneralPing(byte sensorID, int &rez12, HardwareSerial &serialPort, int deRePin = -1) {
  unsigned long &lastPing = (sensorID == 0x54) ? lastPing54 : 
                            (sensorID == 0x24) ? lastPing24 : 
                            (sensorID == 0x5C) ? lastPing5C : 
                            (sensorID == 0x14) ? lastPing14 : 
                            (sensorID == 0x58) ? lastPing58 : 
                            lastPing18;

  if (millis() - lastPing >= pingInterval) {
    lastPing = millis();
    
    if (deRePin != -1) {
      digitalWrite(deRePin, HIGH); // HIGH for transmitting
    }
    
    serialPort.write(sensorID);
    
    if (deRePin != -1) {
      digitalWrite(deRePin, LOW); // LOW for receiving
    }
    
    delay(1);
    if (serialPort.available() == 2) {
      byte Data_rec[2];
      Data_rec[0] = serialPort.read();
      Data_rec[1] = serialPort.read();
      rez12 = processSensorData(Data_rec);
    } else {
      if(debug_serialRead){
        Serial.print("No data ");
        Serial.println(sensorID, HEX);
      }
    }
    while (serialPort.available()) serialPort.read();
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
    Serial.print(" 14: "); Serial.print(rez12_14); Serial.print("   ");
    Serial.print(" 18: "); Serial.print(rez12_18);
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

    data[4] = rez12_14 & 0xFF;
    data[5] = (rez12_14 >> 8) & 0xFF;

    data[6] = rez12_18 & 0xFF;
    data[7] = (rez12_18 >> 8) & 0xFF;

    CAN0.sendMsgBuf(0x18FFFA00, 1, 8, data);
  }
}

//*********************************setup and void loop()
void setup() {
  initSerial();
  initCAN();
}

void loop() {
  handleGeneralPing(0x54, rez12_54,Serial3);
  handleGeneralPing(0x24, rez12_24,Serial3);
  handleGeneralPing(0x5C, rez12_5C,Serial3);
  handleGeneralPing(0x58, rez12_58,Serial3);
  handleGeneralPing(0x18,rez12_18,Serial2, DE_RE_PIN);
  handleGeneralPing(0x14,rez12_14,Serial2, DE_RE_PIN);
  printValues();
  sendCAN();
}
 
 