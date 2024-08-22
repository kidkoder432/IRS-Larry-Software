#include <Arduino.h>
#include <SD.h>

const int chipSelect = 10; // Adjust pin according to your SD card module

void setup() {
    Serial.begin(9600);
    while (!Serial) {
        ; // wait for serial port to connect
    }

    Serial.print("Initializing SD card...");
    if (!SD.begin(chipSelect)) {
        Serial.println("initialization failed!");
        while (1);
    }
    Serial.println("initialization done.");

    // Create a file named "test.txt"
    File myFile = SD.open("test.txt", FILE_WRITE);

    if (myFile) {
        Serial.print("Writing to test.txt...");
        myFile.println("This is a line of text to write to the SD card.");
        myFile.close();
        Serial.println("done.");
    }
    else {
        Serial.println("error opening test.txt");
    }
}

void loop() {
    // Nothing to do here, just write to the SD card once in setup()
}