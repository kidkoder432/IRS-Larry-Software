// DATA LOGGING 
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>


struct DataPoint {
    long timestamp;         // Microseconds
    double DELTA_T;           // Delta Time
    SensorReadings r;       // Sensor Readings
    Vec3D o;                // Current Orientation
    double x_out, y_out;     // TVC Outputs
    double alt;              // Altitude
    int currentState;       // Current State
    double vert_vel;         // Vertical Velocity
    double px, ix, dx;       // PID Values (X)
    double py, iy, dy;       // PID Values (Y)


};




bool logStatus(const char* msg, SDFile logFile) {
    if (!logFile) {
        Serial.println("Couldn't open file");
        return false;
    }
    long long t = micros();
    unsigned long totalSeconds = t / 1000000;

    // Calculate hours, minutes, seconds, and the remaining microseconds
    unsigned long hours = totalSeconds / 3600;
    unsigned long minutes = (totalSeconds % 3600) / 60;
    unsigned long seconds = totalSeconds % 60;
    unsigned long microsPart = t % 1000000;

    // Print in hh:mm:ss.micros format
    logFile.print(hours);
    logFile.print(":");
    if (minutes < 10) logFile.print("0");  // Zero padding for minutes
    logFile.print(minutes);
    logFile.print(":");
    if (seconds < 10) logFile.print("0");  // Zero padding for seconds
    logFile.print(seconds);
    logFile.print(".");
    logFile.println(microsPart);

    logFile.print(" - ");
    logFile.println(msg);

    bool doFlush = true;
    if (millis() % 1000 < 100 && doFlush) {
        logFile.flush();
        doFlush = false;
    }
    else {
        doFlush = true;
    }
    return true;
};

bool logDataPoint(DataPoint p, SDFile dataFile) {

    if (!dataFile) {
        Serial.println("Couldn't open data file");
        return false;
    }

    dataFile.print(p.timestamp);
    dataFile.print(",");
    dataFile.print(p.DELTA_T, 6);
    dataFile.print(",");
    dataFile.print(p.r.ax, 6);
    dataFile.print(",");
    dataFile.print(p.r.ay, 6);
    dataFile.print(",");
    dataFile.print(p.r.az, 6);
    dataFile.print(",");
    dataFile.print(p.r.gx, 6);
    dataFile.print(",");
    dataFile.print(p.r.gy, 6);
    dataFile.print(",");
    dataFile.print(p.r.gz, 6);
    dataFile.print(",");
    dataFile.print(p.o.x, 6);
    dataFile.print(",");
    dataFile.print(p.o.y, 6);
    dataFile.print(",");
    dataFile.print(p.o.z, 6);
    dataFile.print(",");
    dataFile.print(p.x_out, 6);
    dataFile.print(",");
    dataFile.print(p.y_out, 6);
    dataFile.print(",");
    dataFile.print(p.alt, 6);
    dataFile.print(",");
    dataFile.print(p.currentState);
    dataFile.print(",");
    dataFile.print(p.vert_vel, 6);
    dataFile.print(",");
    dataFile.print(p.px, 6);
    dataFile.print(",");
    dataFile.print(p.ix, 6);
    dataFile.print(",");
    dataFile.print(p.dx, 6);
    dataFile.print(",");
    dataFile.print(p.py, 6);
    dataFile.print(",");
    dataFile.print(p.iy, 6);
    dataFile.print(",");
    dataFile.print(p.dy, 6);

    dataFile.println();

    bool doFlush = true;
    if (millis() % 1000 < 100 && doFlush) {
        dataFile.flush();
        doFlush = false;
    }
    else {
        doFlush = true;
    }
    Serial.println("Time,Dt,Ax,Ay,Az,Gx,Gy,Gz,Yaw,Pitch,Xout,Yout,Alt,State,Vel,Px,Ix,Dx,Py,Iy,Dy");
    Serial.print(p.timestamp, 6);
    Serial.print(",");
    Serial.print(p.DELTA_T, 6);
    Serial.print(",");
    Serial.print(p.r.ay, 6);
    Serial.print(",");
    Serial.print(p.r.az, 6);
    Serial.print(",");
    Serial.print(p.r.gx, 6);
    Serial.print(",");
    Serial.print(p.r.gy, 6);
    Serial.print(",");
    Serial.print(p.r.gz, 6);
    Serial.print(",");
    Serial.print(p.o.x, 6);
    Serial.print(",");
    Serial.print(p.o.y, 6);
    Serial.print(",");
    Serial.print(p.o.z, 6);
    Serial.print(",");
    Serial.print(p.x_out, 6);
    Serial.print(",");
    Serial.print(p.y_out, 6);
    Serial.print(",");
    Serial.print(p.alt, 6);
    Serial.print(",");
    Serial.print(p.currentState);
    Serial.print(",");
    Serial.print(p.vert_vel, 6);
    Serial.print(",");
    Serial.print(p.px, 6);
    Serial.print(",");
    Serial.print(p.ix, 6);
    Serial.print(",");
    Serial.print(p.dx, 6);
    Serial.print(",");
    Serial.print(p.py, 6);
    Serial.print(",");
    Serial.print(p.iy, 6);
    Serial.print(",");
    Serial.println(p.dy, 6);
    Serial.println();
    return true;

}

// Helper function to list files recursively
void listFiles(fs::FS& fs, const char* dirname, uint8_t levels) {
    File root = fs.open(dirname);
    if (!root) {
        Serial.println("Failed to open directory");
        return;
    }
    if (!root.isDirectory()) {
        Serial.println("Not a directory");
        return;
    }

    File file = root.openNextFile();
    while (file) {
        if (file.isDirectory()) {
            Serial.print("DIR : ");
            Serial.println(file.name());
            if (levels) {
                listFiles(fs, file.name(), levels - 1);
            }
        }
        else {
            Serial.print("FILE: ");
            Serial.print(file.name());
            Serial.print("\tSIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

void checkSDCard() {
    // Initialize SD card
    if (!SD.begin(10)) {
        Serial.println("Initialization failed. Things to check:");
        Serial.println("* Is a card inserted?");
        Serial.println("* Is your wiring correct?");
        Serial.println("* Did you set the correct CS pin?");
        return;
    }
    else {
        Serial.println("Wiring is correct and a card is present.");
    }

    // Print card type
    Serial.println();
    Serial.print("Card type:         ");
    switch (SD.cardType()) {
        case CARD_NONE:
            Serial.println("No SD card attached");
            return;
        case CARD_MMC:
            Serial.println("MMC");
            break;
        case CARD_SD:
            Serial.println("SDSC");
            break;
        case CARD_SDHC:
            Serial.println("SDHC");
            break;
        default:
            Serial.println("Unknown");
    }

    // Print volume information
    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.print("Card size (MB):    ");
    Serial.println(cardSize);

    uint64_t usedBytes = SD.usedBytes() / (1024 * 1024);
    uint64_t totalBytes = SD.totalBytes() / (1024 * 1024);
    Serial.print("Used space (MB):   ");
    Serial.println(usedBytes);
    Serial.print("Total space (MB):  ");
    Serial.println(totalBytes);

    // List files on the card
    Serial.println("\nFiles found on the card (name and size): ");
    listFiles(SD, "/", 0);
}



// void sdCardInfo() {
//     Sd2Card card;
//     SdVolume volume;
//     SdFile root;

//     Serial.print("\nInitializing SD card...");
//     // we'll use the initialization code from the utility libraries
//     // since we're just testing if the card is working!
//     if (!card.init(SPI_HALF_SPEED, 10)) {
//         Serial.println("initialization failed. Things to check:");
//         Serial.println("* is a card inserted?");
//         Serial.println("* is your wiring correct?");
//         Serial.println("* did you change the chipSelect pin to match your shield or module?");
//         while (1);
//     }
//     else {
//         Serial.println("Wiring is correct and a card is present.");
//     }
//     // print the type of card
//     Serial.println();
//     Serial.print("Card type:         ");
//     switch (card.type()) {
//         case SD_CARD_TYPE_SD1:
//             Serial.println("SD1");
//             break;
//         case SD_CARD_TYPE_SD2:
//             Serial.println("SD2");
//             break;
//         case SD_CARD_TYPE_SDHC:
//             Serial.println("SDHC");
//             break;
//         default:
//             Serial.println("Unknown");
//     }
//     // Now we will try to open the 'volume'/'partition' - it should be FAT16 or FAT32
//     if (!volume.init(card)) {
//         Serial.println("Could not find FAT16/FAT32 partition.\nMake sure you've formatted the card");
//         while (1);
//     }
//     Serial.print("Clusters:          ");
//     Serial.println(volume.clusterCount());
//     Serial.print("Blocks x Cluster:  ");
//     Serial.println(volume.blocksPerCluster());
//     Serial.print("Total Blocks:      ");
//     Serial.println(volume.blocksPerCluster() * volume.clusterCount());
//     Serial.println();
//     // print the type and size of the first FAT-type volume
//     uint32_t volumesize;
//     Serial.print("Volume type is:    FAT");
//     Serial.println(volume.fatType(), DEC);
//     volumesize = volume.blocksPerCluster();    // clusters are collections of blocks
//     volumesize *= volume.clusterCount();       // we'll have a lot of clusters
//     volumesize /= 2;                           // SD card blocks are always 512 bytes (2 blocks are 1KB)
//     Serial.print("Volume size (Kb):  ");
//     Serial.println(volumesize);
//     Serial.print("Volume size (Mb):  ");
//     volumesize /= 1024;
//     Serial.println(volumesize);
//     Serial.print("Volume size (Gb):  ");
//     Serial.println((double)volumesize / 1024.0);
//     Serial.println("\nFiles found on the card (name, date and size in bytes): ");
//     root.openRoot(volume);
//     // list all files in the card with date and size
//     root.ls(LS_R | LS_DATE | LS_SIZE);
//     root.close();
// }