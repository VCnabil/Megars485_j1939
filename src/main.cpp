//works as is as of 11/7/2023 Tuesday using micros and loop timer! 
#include <Arduino.h>
#include <mcp_can.h>
//********************************manip these
bool debug_serialRead=false;
bool debug_printValues=false;
unsigned long pingInterval = 700;
unsigned long pingIntervalWitResponce =700;
unsigned long PrintDebugInterval = 500000;
unsigned long SendCanIntervalMS = 100000;
//********************************last debud timers printtime
unsigned long lastPrintTimeDebugtimer = 0;
unsigned long printIntervalDebugtimer = 1000000; 
unsigned long maxDurationTimerSensorsPing = 0;
unsigned long maxDurationTimerLoop = 0;
//********************************DONT TOUCH THESE Read CAN
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];
#define CAN0_INT 2
long unsigned int PGNzeroing = 0x18FF0100;
//********************************DONT TOUCH THESE
unsigned long lastPrintDebug = 0;
unsigned long lastCanSend = 0;
//********************************Zeroing without delay
const int MAX_ZERO_QUEUE = 4;
byte zeroQueue[MAX_ZERO_QUEUE];
int zeroQueueIndex = 0;
unsigned long lastZeroSensorTime = 0;
unsigned long ZeroingIntervals = 20;
MCP_CAN CAN0(10);

// Sensor struct to encapsulate sensor-related data and operations
struct Sensor {
  byte id;
  unsigned long lastPing;
  int rez12;
  HardwareSerial &serialPort;
  Sensor(byte _id, HardwareSerial &_serialPort) : id(_id), lastPing(0), rez12(0), serialPort(_serialPort) {}
  int processSensorData(byte argdata[2]) {
    int rez12 = (((int)(argdata[1] & 0x3F) << 8) | argdata[0]) >> 2;
    return rez12;
  }
  void handlePing() {
    if (micros() - lastPing >= pingInterval) {
      lastPing = micros(); 
      serialPort.write(id);
      unsigned long startDelayTime = micros();
      bool delayCompleted = false;
      while (!delayCompleted) {
        if (micros() - startDelayTime >= pingIntervalWitResponce) {
          delayCompleted = true;  
        }
      }
      if (serialPort.available() == 2) {
        byte Data_rec[2];
        Data_rec[0] = serialPort.read();
        Data_rec[1] = serialPort.read();
        rez12 = processSensorData(Data_rec);
      } else {
        if (debug_serialRead) {
          Serial.print("No data ");
          Serial.println(id, HEX);
        }
      }
      while (serialPort.available()) serialPort.read();
    }
  }

};

// Global sensor objects
Sensor sensors[] = {
  {0x14, Serial3},
  {0x24, Serial3},
  {0x34, Serial3},
  {0x44, Serial3}
};

void addToZeroQueue(byte sensorID) {
  if (zeroQueueIndex < MAX_ZERO_QUEUE) {
    zeroQueue[zeroQueueIndex++] = sensorID;
  }
}

void zeroSensor() {
  if (zeroQueueIndex > 0) {
    if (micros() - lastZeroSensorTime >= ZeroingIntervals) {
      byte sensorID = zeroQueue[0];
      if (debug_printValues) {
        Serial.print("zeroing ");
        Serial.println(sensorID, HEX);
      }
      Serial3.write(sensorID + 2);  // extended ID
      Serial3.write(0x5E);  // zero command
      lastZeroSensorTime = micros();
      for (int i = 1; i < zeroQueueIndex; i++) {
        zeroQueue[i - 1] = zeroQueue[i];
      }
      zeroQueueIndex--;
    }
  }
}

void initSerial() {
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

void sendCAN() {
  if (micros() - lastCanSend >= SendCanIntervalMS) {
    lastCanSend = micros();
    byte data[8];
    for (int i = 0; i < 4; i++) {
      data[i * 2] = sensors[i].rez12 & 0xFF;
      data[i * 2 + 1] = (sensors[i].rez12 >> 8) & 0xFF;
    }
    CAN0.sendMsgBuf(0x18FFFA00, 1, 8, data);
  }
}

void loopReadCan() {
  if (!digitalRead(CAN0_INT)) {
    CAN0.readMsgBuf(&rxId, &len, rxBuf);
    if ((rxId & 0x80000000) == 0x80000000) {
      long unsigned int recid = rxId & 0x1FFFFFFF;
      if (recid == PGNzeroing && len >= 4) {
        for (int i = 0; i < 4; i++) {
          if (rxBuf[i] > 0) {
            addToZeroQueue(sensors[i].id);
          }
        }
      }
    }
  }
}

void printValues() {
  if (!debug_printValues) return;
  if (micros() - lastPrintDebug >= PrintDebugInterval) {
    lastPrintDebug = micros();
    for (Sensor &sensor : sensors) {
      Serial.print(" ");
      Serial.print(sensor.id, HEX);
      Serial.print(": ");
      Serial.print(sensor.rez12);
    }
    Serial.println();
  }
}
//**********************************************************Setup and loop
void setup() {
  initSerial();
  initCAN();
}

void loop() {
  unsigned long startTimerLoop = micros();
  unsigned long startTimerSensorsPing = micros();

  for (Sensor &sensor : sensors) {
    sensor.handlePing();
  }
  unsigned long durationTimerSensorsPing = micros() - startTimerSensorsPing;
  if (durationTimerSensorsPing > maxDurationTimerSensorsPing) {
    maxDurationTimerSensorsPing = durationTimerSensorsPing;
  }

  printValues();
  sendCAN();
  loopReadCan();
  zeroSensor();

  unsigned long durationTimerLoop = micros() - startTimerLoop;
  if (durationTimerLoop > maxDurationTimerLoop) {
    maxDurationTimerLoop = durationTimerLoop;
  }
  if (micros() - lastPrintTimeDebugtimer >= printIntervalDebugtimer) {
    lastPrintTimeDebugtimer = micros();
    Serial.print("Max Sensor Ping Duration: "); Serial.print(maxDurationTimerSensorsPing); Serial.println(" us");
    Serial.print("Max Loop Duration: "); Serial.print(maxDurationTimerLoop); Serial.println(" us");
  }
 
}