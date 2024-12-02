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

void msgPrint(bool bleOn, HardwareBLESerial& bleSerial, int64_t msg) {
    Serial.print(msg);
    if (bleOn) bleSerial.print(msg);
}

void msgPrintln(bool bleOn, HardwareBLESerial& bleSerial, int64_t msg) {
    Serial.println(msg);
    if (bleOn) bleSerial.println(msg);
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