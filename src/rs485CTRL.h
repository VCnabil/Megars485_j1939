#ifndef RS485CTRL_H
#define RS485CTRL_H

#include <Arduino.h>

class RS485CTRL {
public:
  RS485CTRL( HardwareSerial& serialPort);
  void begin(unsigned long baudRate);
  void Ping(byte sensorID);
  byte* Pingv2(byte sensorID); 
private:
  HardwareSerial& _serialPort;
  unsigned long _lastPing;
  unsigned long _pingInterval;
  byte _data[2];  // Added to store lowByte and highByte
};

#endif // RS485CTRL_H
