// DATA LOGGING 
#include <Arduino.h>
#include <SPI.h>
#if USE_RP2040
#include <LittleFS.h>
#include <VFS.h>
#endif
#include <SdFat.h>

#define DEBUG 0
#define RP2040_FS_SIZE_KB 1024

// <l12f3h2x8f
struct DataPoint {
    int timestamp;          // Milliseconds
    float DELTA_T;          // Delta Time
    SensorReadings r;       // Sensor Readings
    Vec3D o;                // Current Orientation
    float x_out, y_out;     // TVC Outputs
    short x_act, y_act;     // TVC Readings
    short currentState;     // Current State
    float alt;              // Altitude
    float vert_vel;         // Vertical Velocity
    float px, ix, dx;       // PID Values (X)
    float py, iy, dy;       // PID Values (Y)
    DataPoint() = default;
    bool isEmpty = true;


};

union DataPointBin {
    DataPoint p = DataPoint();
    unsigned char dataBytes[sizeof(DataPoint) - 4];

    DataPointBin() = default;
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

bool logDataPointBin(DataPoint p, ExFile& dataFile) {

    if (!dataFile.isOpen()) {
        Serial.println("Couldn't open data file");
        return false;
    }

    DataPointBin pBin;
    pBin.p = p;

    dataFile.write(pBin.dataBytes, sizeof(DataPoint) - 4);

    return true;

}

bool logDataRaw(uint8_t data[], int size, ExFile& dataFile) {

    if (!dataFile.isOpen()) {
        Serial.println("Couldn't open data file");
        return false;
    }

    dataFile.write(data, size);
    dataFile.sync();

    
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

#if USE_RP2040

bool logDataRaw(uint8_t data[], int size, FILE *dataFile) {
    if (!dataFile) {
        Serial.println("Couldn't open flash file");
        return false;
    }

    if (!fwrite(data, size, 1, dataFile)) {
        Serial.println("Failed to write to flash file");
        return false;
    }

    fflush(dataFile);

    return true;
}

bool logDataPointBin(DataPoint p, FILE *dataFile) {

    DataPointBin pBin;
    pBin.p = p;

    if (!dataFile) {
        Serial.println("Couldn't open flash file");
        return false;
    }

    if (!fwrite(pBin.dataBytes, sizeof(DataPoint) - 4, 1, dataFile)) {
        Serial.println("Failed to write to flash file");
        return false;
    }
    
    fflush(dataFile);

    return true;
}

bool writeSD(FILE *flashFile, ExFile& dataFile) {
    if (!dataFile.isOpen()) {
        Serial.println("Couldn't open data file");
        return false;
    }

    if (!flashFile) {
        Serial.println("Couldn't open flash file");
        return false;
    }

    char buf[512];
    while (!feof(flashFile)) {
        int bytesRead = fread(buf, 1, sizeof(buf), flashFile);
        if (bytesRead > 0) {
            if (!dataFile.write(buf, bytesRead)) {
                Serial.println("Failed to write to data file");
                return false;
            }
            dataFile.sync();
        }
    }

    return true;

}

#endif