// DATA LOGGING 
#include <Arduino.h>
#include <SPI.h>
#include <SdFat.h>

#undef SPI_DRIVER_SELECT
#undef USE_SPI_ARRAY_TRANSFER
#undef SD_MAX_INIT_RATE_KHZ

#define SPI_DRIVER_SELECT 1
#define USE_SPI_ARRAY_TRANSFER 1
#define SD_MAX_INIT_RATE_KHZ 20000

#define RING_BUFFER_SIZE 10


#define DEBUG 0

const char* DATA_HEADER = "Time,Dt,Ax,Ay,Az,Gx,Gy,Gz,Roll,Pitch,Yaw,TvcX,TvcY,State,Alt,Vel,Px,Ix,Dx,Py,Iy,Dy";

// <l12fh2x8f
struct DataPoint {
    int timestamp;          // Milliseconds
    float DELTA_T;          // Delta Time
    SensorReadings r;       // Sensor Readings
    Vec3D o;                // Current Orientation
    float x_out, y_out;     // TVC Outputs
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

bool logDataRaw(uint8_t data[], long size, ExFile& dataFile) {

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

    return true;

}

void sdCardInfo(SdExFat& sd) {

    // Retrieve card type and sector-based information
    SdCard* card = sd.card();
    ExFatVolume* vol = sd.vol();

    uint8_t cardType = card->type();
    uint64_t sectorCount = card->sectorCount();
    uint64_t sectorsPerCluster = vol->sectorsPerCluster();
    uint64_t freeClusterCount = vol->freeClusterCount();
    uint64_t freeSectorCount = freeClusterCount * sectorsPerCluster;

    // Print card type
    Serial.print("Card Type: ");
    Serial.println((cardType == SD_CARD_TYPE_SD1) ? "SD1" :
        (cardType == SD_CARD_TYPE_SD2) ? "SD2" :
        (cardType == SD_CARD_TYPE_SDHC) ? "SDHC/SDXC" : "Unknown");

    // Print sector and space details
    constexpr uint64_t SECTOR_SIZE = 512;
    constexpr uint64_t MB_DIVISOR = 1024 * 1024;

    Serial.print("Total Sectors: ");  Serial.println(sectorCount);
    Serial.print("Sectors Per Cluster: ");  Serial.println(sectorsPerCluster);
    Serial.print("Free Sectors: ");  Serial.println(freeSectorCount);
    Serial.print("Used Sectors: ");  Serial.println(sectorCount - freeSectorCount);

    Serial.print("Card Size: ");
    Serial.print((sectorCount * SECTOR_SIZE) / MB_DIVISOR);
    Serial.println(" MB");

    Serial.print("Free Space: ");
    Serial.print((freeSectorCount * SECTOR_SIZE) / MB_DIVISOR);
    Serial.println(" MB");

    // List files and directories
    Serial.println("Listing files on the card:");
    sd.ls(LS_R | LS_DATE | LS_SIZE);
}


class DataLogger {
public:
    DataLogger() : logFile(nullptr) {} // Start with no file

    // Call this after your other class successfully opens the file
    void setTargetFile(ExFile* file) {
        this->logFile = file;
    }

    void addPoint(DataPoint p) {
        int nextHead = (producePtr + 1) % BUFFER_SIZE;
        if (nextHead != consumePtr) {
            ringBuffer[producePtr] = p;
            producePtr = nextHead;
        }
    }

    bool logNext() {
        // Only write if we have a valid file and data is ready
        if (numPending() > RING_BUFFER_SIZE) {
            DataPoint arr[RING_BUFFER_SIZE];
            unsigned char bytes[(RING_BUFFER_SIZE) * (sizeof(DataPoint) - 4)];
            for (int i = 0; i < RING_BUFFER_SIZE; i++) {
                if (arr[i].isEmpty) {
                    continue;
                }
                DataPointBin pBin;
                pBin.p = arr[i];
                memcpy(&bytes[i * (sizeof(DataPoint) - 4)], pBin.dataBytes, sizeof(DataPoint) - 4);
            }

            logDataRaw(bytes, (RING_BUFFER_SIZE) * (sizeof(DataPoint) - 4), *logFile);
            return true;
        }
        return false;
    }

    int numPending() {
        return (producePtr - consumePtr + BUFFER_SIZE) % BUFFER_SIZE;
    }

    void logAllPoints() {
        bool more = true;
        while (more) {
            more = logNext();
        }

    }

private:
    ExFile* logFile; // Pointer allows the file to be "assigned" later
    static const int BUFFER_SIZE = 512;
    DataPoint ringBuffer[BUFFER_SIZE];
    volatile int producePtr = 0;
    volatile int consumePtr = 0;
};