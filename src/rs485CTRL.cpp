#include "rs485CTRL.h"

RS485CTRL::RS485CTRL( HardwareSerial& serialPort) : _serialPort(serialPort) {
  _lastPing = 0;
  _pingInterval = 50;
}

void RS485CTRL::begin(unsigned long baudRate) {
  _serialPort.begin(baudRate);
  delay(500);
}

void RS485CTRL::Ping(byte sensorID) {
  if (millis() - _lastPing >= _pingInterval) {
    _lastPing = millis();
    _serialPort.write(sensorID);
    delay(10);
    if (_serialPort.available() >= 2) {
      byte lowByte = _serialPort.read();
      byte highByte = _serialPort.read();
      Serial.print("Received from ");
      Serial.print(sensorID, HEX);
      Serial.print(": Low Byte: 0x");
      Serial.print(lowByte, HEX);
      Serial.print(", High Byte: 0x");
      Serial.println(highByte, HEX);
    } else {
      Serial.print("No data received from Sensor ");
      Serial.println(sensorID, HEX);
    }
    while (_serialPort.available()) _serialPort.read();
  }
}
byte* RS485CTRL::Pingv2(byte sensorID) {
  if (millis() - _lastPing >= _pingInterval) {
    _lastPing = millis();
    _serialPort.write(sensorID);
  //  delay(10);
    if (_serialPort.available() >= 2) {
      _data[0] = _serialPort.read();  // lowByte
      _data[1] = _serialPort.read();  // highByte
            Serial.print("Received from ");
      Serial.print(sensorID, HEX);
      Serial.print(": Low Byte: 0x");
      Serial.print(_data[0], HEX);
      Serial.print(", High Byte: 0x");
      Serial.print(_data[1], HEX);
      Serial.print( "  ");
     // Serial.println();
    } else {
      _data[0] = 0;
      _data[1] = 0;
    }
   // while (_serialPort.available()) _serialPort.read();
  }
  return _data;
}
