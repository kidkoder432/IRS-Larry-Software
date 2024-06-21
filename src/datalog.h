// DATA LOGGING 

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>


struct DataPoint {
    long long timestamp;    // Microseconds
    SensorReadings r;       // Sensor Readings
    Orientation o;          // Current Orientation
    float x_out, y_out;     // TVC Outputs
    float alt;              // Altitude
    int currentState;       // Current State
    float vert_vel;         // Vertical Velocity
};


void logDataPoint(DataPoint p) {
    // TODO:    
}

