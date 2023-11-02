//works as is as of 11/1/2023 Wednesday Zeroing senssor 24 with button 
#include <Arduino.h>
#include <mcp_can.h>

//********************************manip these
bool debug_serialRead=false;
bool debug_printValues=false;
unsigned long pingInterval = 1;
unsigned long PrintDebugInterval = 500;
unsigned long SendCanIntervalMS = 100;
//********************************DONT TOUCH THESE Zeroing
const int buttonPin = 3;
int buttonState = 0;   
unsigned long lastRunTime_24_zero = 0;  
unsigned long lastFinishZeroing_24_Time = 0;
bool isZeroing_24=false;
const unsigned long interval_zero = 200; 
//********************************DONT TOUCH THESE Read CAN
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];
#define CAN0_INT 2
long unsigned int PGNzeroing = 0x18FF0100;
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
   pinMode(buttonPin, INPUT);//********************Zeroing
  Serial.begin(115200);
  delay(200);
  Serial3.begin(115200);
  Serial.println("RS485-Test");
  delay(200);  
}

void initCAN() {
   
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

void funcZero_24_BTN() {
  isZeroing_24=true;
  Serial3.write(0x26); //extended for 0x24
  Serial3.write(0x5E); // zero it 
  lastFinishZeroing_24_Time = millis();
}

void loop_Zeroing_BTN() {
  buttonState = digitalRead(buttonPin);
  unsigned long currentTime = millis();
  if (buttonState == HIGH && currentTime - lastRunTime_24_zero >= interval_zero) {
    funcZero_24_BTN();
    lastRunTime_24_zero = currentTime;  
  }
  if (isZeroing_24 && currentTime - lastFinishZeroing_24_Time >= interval_zero) {
    Serial.println("finished");
    isZeroing_24 = false;  // reset the flag
  }
}

void Zero_14A(){
 if(debug_printValues)Serial.println("zeroing A");
 Serial3.write(0x16); //extended for 0x14
 Serial3.write(0x5E); // zero it 
 delay(200);
}
void Zero_24B(){
 if(debug_printValues)Serial.println("zeroing B");
 Serial3.write(0x26); //extended for 0x24
 Serial3.write(0x5E); // zero it 
 delay(200);
}
void Zero_34C(){
 if(debug_printValues)Serial.println("zeroing C");
 Serial3.write(0x36); //extended for 0x34
 Serial3.write(0x5E); // zero it 
 delay(200);
}
void Zero_44D(){
 if(debug_printValues)Serial.println("zeroing D");
 Serial3.write(0x46); //extended for 0x44
 Serial3.write(0x5E); // zero it 
 delay(200);
}
//*********************************setup and void loop()
void setup() {
  initSerial();
  initCAN();
}
void loopReadCan(){
  if(!digitalRead(CAN0_INT))  // If CAN0_INT pin is low, read receive buffer
  {
    CAN0.readMsgBuf(&rxId, &len, rxBuf); 
    if((rxId & 0x80000000) == 0x80000000)     // Determine if ID is 29bit
    {
     long unsigned int recid =rxId & 0x1FFFFFFF;
    if(recid == PGNzeroing){
       if(len>=4){
        if(rxBuf[0]>0){Zero_14A();}
        if(rxBuf[1]>0){Zero_24B();}
        if(rxBuf[2]>0){Zero_34C();}
        if(rxBuf[3]>0){Zero_44D();}
       }
    }
    }
  } 
}
void loop() {
  loop_Zeroing_BTN();
  if(!isZeroing_24){
    loopReadCan();
    handleGeneralPing(0x14, rez12_14A,Serial3);
    handleGeneralPing(0x24, rez12_24B,Serial3);
    handleGeneralPing(0x34, rez12_34C,Serial3);
    handleGeneralPing(0x44, rez12_44D,Serial3);
    printValues();
    sendCAN();
  }
}