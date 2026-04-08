// DATA LOGGING 
#include <Arduino.h>
#include <SPI.h>
#include <SdFat.h>

// #undef SPI_DRIVER_SELECT
// #undef USE_SPI_ARRAY_TRANSFER
// #undef SD_MAX_INIT_RATE_KHZ

// #define SPI_DRIVER_SELECT 1
// #define USE_SPI_ARRAY_TRANSFER 1
// #define SD_MAX_INIT_RATE_KHZ 20000

#define RING_BUFFER_SIZE 10


#define DEBUG 0

const char* DATA_HEADER = "time,ax,ay,az,gx,gy,gz,ox,oy,oz,x_out,y_out,state,alt,vert_vel,px,ix,dx,py,iy,dy,dt\n";

// <L9f12H
struct DataPoint {
    uint32_t timestamp;     // 4

    // IMU
    SensorReadings r;       // 24

    // Orientation
    Vec3D o;                // 12

    // Outputs
    int16_t x_out, y_out;   // 4

    // State
    int16_t state;          // 2

    // Altitude + velocity
    int16_t alt;            // 2
    int16_t vert_vel;       // 2

    // PID X
    int16_t px, ix, dx;     // 6

    // PID Y
    int16_t py, iy, dy;     // 6

    // Optional: delta time (scaled)
    int16_t dt;             // 2

    uint8_t isEmpty;        // 1
    // padding - 3 bytes, total 68
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

    // logFile.sync();
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
    // dataFile.sync();

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

    bool logNext(bool dummy = false) {        
        // Only write if we have a valid file and data is ready
        if (numPending() > RING_BUFFER_SIZE) {
            DataPoint arr[RING_BUFFER_SIZE];

            for (int i = 0; i < RING_BUFFER_SIZE; i++) {
                arr[i] = ringBuffer[consumePtr];
                consumePtr = (consumePtr + 1) % BUFFER_SIZE;
            }

            unsigned char bytes[(RING_BUFFER_SIZE) * (sizeof(DataPoint) - 4)];
            for (int i = 0; i < RING_BUFFER_SIZE; i++) {
                if (arr[i].isEmpty) {
                    continue;
                }
                DataPointBin pBin;
                pBin.p = arr[i];
                memcpy(&bytes[i * (sizeof(DataPoint) - 4)], pBin.dataBytes, sizeof(DataPoint) - 4);
            }
            if (dummy) {
                unsigned long long start = micros();
                while (micros() - start < 2000) {}
            } else {
                logDataRaw(bytes, (RING_BUFFER_SIZE) * (sizeof(DataPoint) - 4), *logFile);
            }
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