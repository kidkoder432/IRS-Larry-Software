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
    float kp, ki, kd;       // PID Params

    
};

bool logDataPoint(DataPoint p, SDFile dataFile) {
    if (!SD.begin(10)) {
        Serial.println("Card failed, or not present");
        return false;
    }
    
    if (!dataFile) {
        Serial.println("Couldn't open file");
        return false;
    }

    dataFile.print(p.timestamp);
    dataFile.print(",");
    dataFile.print(p.r.ax);
    dataFile.print(",");
    dataFile.print(p.r.ay);
    dataFile.print(",");
    dataFile.print(p.r.az);
    dataFile.print(",");
    dataFile.print(p.r.gx);
    dataFile.print(",");
    dataFile.print(p.r.gy);
    dataFile.print(",");
    dataFile.print(p.r.gz);
    dataFile.print(",");
    dataFile.print(p.o.yaw);
    dataFile.print(",");
    dataFile.print(p.o.pitch);
    dataFile.print(",");
    dataFile.print(p.x_out);
    dataFile.print(",");
    dataFile.print(p.y_out);
    dataFile.print(",");
    dataFile.print(p.alt);
    dataFile.print(",");
    dataFile.print(p.currentState);
    dataFile.print(",");
    dataFile.print(p.vert_vel);
    dataFile.print(",");
    dataFile.print(p.kp);
    dataFile.print(",");
    dataFile.print(p.ki);
    dataFile.print(",");
    dataFile.print(p.kd);
    dataFile.print(",");
    dataFile.print(p.b);
    dataFile.print(",");
    dataFile.print(p.dt);
    dataFile.print(",");
    dataFile.print(p.min);
    dataFile.print(",");
    dataFile.print(p.max);

    dataFile.println();

    return true;

    // TODO:    
}
void sdCardInfo() {
    Sd2Card card;
    SdVolume volume;
    SdFile root;


    Serial.print("\nInitializing SD card...");
    // we'll use the initialization code from the utility libraries
    // since we're just testing if the card is working!
    if (!card.init(SPI_HALF_SPEED, 10)) {
        Serial.println("initialization failed. Things to check:");
        Serial.println("* is a card inserted?");
        Serial.println("* is your wiring correct?");
        Serial.println("* did you change the chipSelect pin to match your shield or module?");
        while (1);
    }
    else {
        Serial.println("Wiring is correct and a card is present.");
    }
    // print the type of card
    Serial.println();
    Serial.print("Card type:         ");
    switch (card.type()) {
        case SD_CARD_TYPE_SD1:
            Serial.println("SD1");
            break;
        case SD_CARD_TYPE_SD2:
            Serial.println("SD2");
            break;
        case SD_CARD_TYPE_SDHC:
            Serial.println("SDHC");
            break;
        default:
            Serial.println("Unknown");
    }
    // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
    if (!volume.init(card)) {
        Serial.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
        while (1);
    }
    Serial.print("Clusters:          ");
    Serial.println(volume.clusterCount());
    Serial.print("Blocks x Cluster:  ");
    Serial.println(volume.blocksPerCluster());
    Serial.print("Total Blocks:      ");
    Serial.println(volume.blocksPerCluster() * volume.clusterCount());
    Serial.println();
    // print the type and size of the first FAT-type volume
    uint32_t volumesize;
    Serial.print("Volume type is:    FAT");
    Serial.println(volume.fatType(), DEC);
    volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
    volumesize *= volume.clusterCount();       // we'll have a lot of clusters
    volumesize /= 2;                           // SD card blocks are always 512 bytes (2 blocks are 1KB)
    Serial.print("Volume size (Kb):  ");
    Serial.println(volumesize);
    Serial.print("Volume size (Mb):  ");
    volumesize /= 1024;
    Serial.println(volumesize);
    Serial.print("Volume size (Gb):  ");
    Serial.println((float)volumesize / 1024.0);
    Serial.println("\nFiles found on the card (name, date and size in bytes): ");
    root.openRoot(volume);
    // list all files in the card with date and size
    root.ls(LS_R | LS_DATE | LS_SIZE);
    root.close();
}
