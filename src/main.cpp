//works as is as of 11/7/2023 Tuesday using micros with NOOVERFLOW and loop timer! 
#include <Arduino.h>
#include <mcp_can.h>

//********************************manip these
bool debug_serialRead = false;
bool debug_printValues = false;
bool debug_printTimers = false;
unsigned long pingInterval_US = 6000;
unsigned long pingIntervalWithResponse_US = 784;
unsigned long PrintDebugInterval_US = 500000;
unsigned long SendCanInterval_US = 100000;
//********************************last debug timers printtime
unsigned long lastPrintTimeDebugTimer_US = 0;
unsigned long printIntervalDebugTimer_US = 1000000; 
unsigned long maxDurationTimerSensorsPing_US = 0;
unsigned long maxDurationTimerLoop_US = 0;
//********************************DONT TOUCH THESE Read CAN
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];
#define CAN0_INT 2
long unsigned int PGNzeroing = 0x18FF0100;
long unsigned int PGN_posData = 0x18FFFA00;
//********************************DONT TOUCH THESE
unsigned long lastPrintDebug_US = 0;
unsigned long lastCanSend_US = 0;
//********************************Zeroing without delay
const int MAX_ZERO_QUEUE = 4;
byte zeroQueue[MAX_ZERO_QUEUE];
int zeroQueueIndex = 0;
unsigned long lastZeroSensorTime_US= 0;
unsigned long ZeroingIntervals_US = 900;
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
    unsigned long currentMicros = micros();
    if (currentMicros - lastPing >= pingInterval_US) {
      lastPing = currentMicros; 
      serialPort.write(id);
      unsigned long startDelayTime = currentMicros;
      while (micros() - startDelayTime < pingIntervalWithResponse_US) {
        // Just wait
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
  unsigned long currentMicros = micros();
  if (zeroQueueIndex > 0 && currentMicros - lastZeroSensorTime_US >= ZeroingIntervals_US) {
    byte sensorID = zeroQueue[0];
    if (debug_printValues) {
      Serial.print("zeroing ");
      Serial.println(sensorID, HEX);
    }
    Serial3.write(sensorID + 2);  // extended ID
    Serial3.write(0x5E);  // zero command
    lastZeroSensorTime_US = currentMicros;
    for (int i = 1; i < zeroQueueIndex; i++) {
      zeroQueue[i - 1] = zeroQueue[i];
    }
    zeroQueueIndex--;
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
  unsigned long currentMicros = micros();
  if (currentMicros - lastCanSend_US >= SendCanInterval_US) {
    lastCanSend_US = currentMicros;
    byte data[8];
    for (int i = 0; i < 4; i++) {
      data[i * 2] = sensors[i].rez12 & 0xFF;
      data[i * 2 + 1] = (sensors[i].rez12 >> 8) & 0xFF;
    }
    CAN0.sendMsgBuf(PGN_posData, 1, 8, data);
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
  unsigned long currentMicros = micros();
  if (debug_printValues && currentMicros - lastPrintDebug_US >= PrintDebugInterval_US) {
    lastPrintDebug_US = currentMicros;
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
  if (durationTimerSensorsPing > maxDurationTimerSensorsPing_US) {
    maxDurationTimerSensorsPing_US = durationTimerSensorsPing;
  }

  printValues();
  sendCAN();
  loopReadCan();
  zeroSensor();

  unsigned long durationTimerLoop = micros() - startTimerLoop;
  if (durationTimerLoop > maxDurationTimerLoop_US) {
    maxDurationTimerLoop_US = durationTimerLoop;
  }
  if (micros() - lastPrintTimeDebugTimer_US >= printIntervalDebugTimer_US) {
    lastPrintTimeDebugTimer_US = micros();
    if(debug_printTimers==true){
      Serial.print("Max Sensor Ping Duration: "); Serial.print(maxDurationTimerSensorsPing_US); Serial.println(" us");
      Serial.print("Max Loop Duration: "); Serial.print(maxDurationTimerLoop_US); Serial.println(" us");
    }
  }
}
