//works as is as of 11/1/2023 Wednesday
#include <Arduino.h>
#include <mcp_can.h>
 
//********************************manip these
bool debug_serialRead=false;
bool debug_printValues=false;
unsigned long pingInterval = 1;
unsigned long PrintDebugInterval = 500;
unsigned long SendCanIntervalMS = 50;
//********************************DONT TOUCH THESE
unsigned long lastPing14_A = 0;
unsigned long lastPing24_B = 0;
unsigned long lastPing34_C = 0;
unsigned long lastPing44_D = 0;
unsigned long lastPrintDebug = 0;
unsigned long lastCanSend = 0;
byte data[8] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
int rez12_14A = 0;
int rez12_24B = 0;
int rez12_34C = 0;
int rez12_44D = 0;
MCP_CAN CAN0(10);
 
void initSerial() {
  Serial.begin(115200);
  delay(200);
  Serial3.begin(38400);
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

void handleGeneralPing(byte sensorID, int &argrez12, HardwareSerial &serialPort) {
  unsigned long &lastPing = (sensorID == 0x14) ? lastPing14_A : 
                            (sensorID == 0x24) ? lastPing24_B : 
                            (sensorID == 0x34) ? lastPing34_C : 
                            (sensorID == 0x44) ? lastPing44_D : 
                            lastPing24_B;

  if (millis() - lastPing >= pingInterval) {
    lastPing = millis();
    serialPort.write(sensorID);
    delay(1);
    if (serialPort.available() == 2) {
      byte Data_rec[2];
      Data_rec[0] = serialPort.read();
      Data_rec[1] = serialPort.read();
      argrez12 = processSensorData(Data_rec);
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
    Serial.print(" 14A: "); Serial.print(rez12_14A); Serial.print(" ");
    Serial.print(" 24B: "); Serial.print(rez12_24B); Serial.print(" ");
    Serial.print(" 34C: "); Serial.print(rez12_34C); Serial.print(" ");
    Serial.print(" 44D: "); Serial.print(rez12_44D);
    Serial.println(" ");
  }
}

void sendCAN() {
  if (millis() - lastCanSend >= SendCanIntervalMS) {
    lastCanSend = millis();

    data[0] = rez12_14A & 0xFF;
    data[1] = (rez12_14A >> 8) & 0xFF;

    data[2] = rez12_24B & 0xFF;
    data[3] = (rez12_24B >> 8) & 0xFF;

    data[4] = rez12_34C & 0xFF;
    data[5] = (rez12_34C >> 8) & 0xFF;

    data[6] = rez12_44D & 0xFF;
    data[7] = (rez12_44D >> 8) & 0xFF;

    CAN0.sendMsgBuf(0x18FFFA00, 1, 8, data);
  }
}

//*********************************setup and void loop()
void setup() {
  initSerial();
  initCAN();
}

void loop() {
  handleGeneralPing(0x14, rez12_14A,Serial3);
  handleGeneralPing(0x24, rez12_24B,Serial3);
  handleGeneralPing(0x34, rez12_34C,Serial3);
  handleGeneralPing(0x44, rez12_44D,Serial3);
  printValues();
  sendCAN();
}
 
 