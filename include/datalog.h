// DATA LOGGING 
#include <Arduino.h>
#include <SPI.h>
#include <SdFat.h>

#define DEBUG 0
struct DataPoint {
    long timestamp;         // Milliseconds
    float DELTA_T;         // Delta Time
    SensorReadings r;       // Sensor Readings
    Vec3D o;                // Current Orientation
    float x_out, y_out;    // TVC Outputs
    float alt;             // Altitude
    int currentState;       // Current State
    float vert_vel;        // Vertical Velocity
    float px, ix, dx;      // PID Values (X)
    float py, iy, dy;      // PID Values (Y)


};




bool logStatus(const char* msg, ExFile& logFile) {
    if (!logFile.isOpen()) {
        Serial.println("Couldn't open log file");
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
    logFile.print(microsPart);

    logFile.print(" - ");
    logFile.println(msg);

    logFile.sync();
    return true;
};

bool logDataBin(DataPoint p, ExFile& dataFile) {

    if (!dataFile.isOpen()) {
        Serial.println("Couldn't open data file");
        return false;
    }

    dataFile.write((byte*)&p, sizeof(DataPoint));

    return true;

}
bool logDataPoint(DataPoint p, ExFile& dataFile) {

    if (!dataFile.isOpen()) {
        Serial.println("Couldn't open data file");
        return false;
    }

    dataFile.print(p.timestamp);
    dataFile.print(",");
    dataFile.print(p.DELTA_T, 3);
    dataFile.print(",");
    dataFile.print(p.r.ax, 3);
    dataFile.print(",");
    dataFile.print(p.r.ay, 3);
    dataFile.print(",");
    dataFile.print(p.r.az, 3);
    dataFile.print(",");
    dataFile.print(p.r.gx, 3);
    dataFile.print(",");
    dataFile.print(p.r.gy, 3);
    dataFile.print(",");
    dataFile.print(p.r.gz, 3);
    dataFile.print(",");
    dataFile.print(p.o.x, 3);
    dataFile.print(",");
    dataFile.print(p.o.y, 3);
    dataFile.print(",");
    dataFile.print(p.o.z, 3);
    dataFile.print(",");
    dataFile.print(p.x_out, 3);
    dataFile.print(",");
    dataFile.print(p.y_out, 3);
    dataFile.print(",");
    dataFile.print(p.alt, 3);
    dataFile.print(",");
    dataFile.print(p.currentState);
    dataFile.print(",");
    dataFile.print(p.vert_vel, 3);
    dataFile.print(",");
    dataFile.print(p.px, 3);
    dataFile.print(",");
    dataFile.print(p.ix, 3);
    dataFile.print(",");
    dataFile.print(p.dx, 3);
    dataFile.print(",");
    dataFile.print(p.py, 3);
    dataFile.print(",");
    dataFile.print(p.iy, 3);
    dataFile.print(",");
    dataFile.print(p.dy, 3);
    dataFile.println();

#if DEBUG
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
#endif
    return true;

}

// Helper function to list files recursively
// void listFiles(fs::FS& fs, const char* dirname, uint8_t levels) {
//     File root = fs.open(dirname);
//     if (!root) {
//         Serial.println("Failed to open directory");
//         return;
//     }
//     if (!root.isDirectory()) {
//         Serial.println("Not a directory");
//         return;
//     }

//     File file = root.openNextFile();
//     while (file) {
//         if (file.isDirectory()) {
//             Serial.print("DIR : ");
//             Serial.println(file.name());
//             if (levels) {
//                 listFiles(fs, file.name(), levels - 1);
//             }
//         }
//         else {
//             Serial.print("FILE: ");
//             Serial.print(file.name());
//             Serial.print("\tSIZE: ");
//             Serial.println(file.size());
//         }
//         file = root.openNextFile();
//     }
// }

// void checkSDCard() {
//     // Initialize SD card
//     if (!SD.begin(10)) {
//         Serial.println("Initialization failed. Things to check:");
//         Serial.println("* Is a card inserted?");
//         Serial.println("* Is your wiring correct?");
//         Serial.println("* Did you set the correct CS pin?");
//         return;
//     }
//     else {
//         Serial.println("Wiring is correct and a card is present.");
//     }

//     // Print card type
//     Serial.println();
//     Serial.print("Card type:         ");
//     switch (SD.cardType()) {
//         case CARD_NONE:
//             Serial.println("No SD card attached");
//             return;
//         case CARD_MMC:
//             Serial.println("MMC");
//             break;
//         case CARD_SD:
//             Serial.println("SDSC");
//             break;
//         case CARD_SDHC:
//             Serial.println("SDHC");
//             break;
//         default:
//             Serial.println("Unknown");
//     }

//     // Print volume information
//     uint64_t cardSize = SD.cardSize() / (1024 * 1024);
//     Serial.print("Card size (MB):    ");
//     Serial.println(cardSize);

//     uint64_t usedBytes = SD.usedBytes() / (1024 * 1024);
//     uint64_t totalBytes = SD.totalBytes() / (1024 * 1024);
//     Serial.print("Used space (MB):   ");
//     Serial.println(usedBytes);
//     Serial.print("Total space (MB):  ");
//     Serial.println(totalBytes);

//     // List files on the card
//     Serial.println("\nFiles found on the card (name and size): ");
//     listFiles(SD, "/", 0);
// }



void sdCardInfo(SdExFat& sd) {


    // Print card type
    uint8_t cardType = sd.card()->type();
    Serial.print("Card Type: ");
    switch (cardType) {
        case SD_CARD_TYPE_SD1: Serial.println("SD1"); break;
        case SD_CARD_TYPE_SD2: Serial.println("SD2"); break;
        case SD_CARD_TYPE_SDHC: Serial.println("SDHC/SDXC"); break;
        default: Serial.println("Unknown"); break;
    }

    // Retrieve sector-based information
    uint64_t sectorCount = sd.card()->sectorCount();
    uint64_t sectorsPerCluster = sd.vol()->sectorsPerCluster();  // 1 block = 1 sector in SdFat
    uint64_t freeSectorCount = (uint64_t)sd.vol()->freeClusterCount() * sectorsPerCluster;

    // Print sector information
    Serial.print("Total Sectors: ");
    Serial.println(sectorCount);

    Serial.print("Sectors Per Cluster: ");
    Serial.println(sectorsPerCluster);

    Serial.print("Free Sectors: ");
    Serial.println(freeSectorCount);

    Serial.print("Used Sectors: ");
    Serial.println(sectorCount - freeSectorCount);

    // Calculate card size (512 bytes per sector)
    uint64_t cardSize = sectorCount * 512;  // Convert sectors to bytes
    Serial.print("Card Size: ");
    Serial.print(cardSize / (1024 * 1024));  // Convert bytes to MB
    Serial.println(" MB");

    // Calculate free space
    uint64_t freeSpace = freeSectorCount * 512;  // Convert sectors to bytes
    Serial.print("Free Space: ");
    Serial.print(freeSpace / (1024 * 1024));  // Convert bytes to MB
    Serial.println(" MB");

    // List files and directories
    Serial.println("Listing files on the card:");
    sd.ls(LS_R | LS_DATE | LS_SIZE);  // Recursive, show date and size
}
