#include <Arduino.h>
#include <HardwareBLESerial.h>

#ifndef PRINTS_H
#define PRINTS_H

void msgPrint(bool bleOn, HardwareBLESerial& bleSerial, const char* msg) {
    Serial.print(msg);
    if (bleOn) bleSerial.print(msg);
}

void msgPrintln(bool bleOn, HardwareBLESerial& bleSerial, const char* msg) {
    Serial.println(msg);
    if (bleOn) bleSerial.println(msg);
}

void msgPrint(bool bleOn, HardwareBLESerial& bleSerial, char msg) {
    Serial.print(msg);
    if (bleOn) bleSerial.print(msg);
}

void msgPrintln(bool bleOn, HardwareBLESerial& bleSerial, char msg) {
    Serial.println(msg);
    if (bleOn) bleSerial.println(msg);
}

void msgPrint(bool bleOn, HardwareBLESerial& bleSerial, int msg) {
    Serial.print(msg);
    if (bleOn) bleSerial.print(static_cast<int64_t>(msg));
}

void msgPrintln(bool bleOn, HardwareBLESerial& bleSerial, int msg) {
    Serial.println(msg);
    if (bleOn) bleSerial.println(static_cast<int64_t>(msg));
}

void msgPrint(bool bleOn, HardwareBLESerial& bleSerial, float msg) {
    Serial.print(msg);
    if (bleOn) bleSerial.print(msg);
}

void msgPrintln(bool bleOn, HardwareBLESerial& bleSerial, float msg) {
    Serial.println(msg);
    if (bleOn) bleSerial.println(msg);
}

#endif