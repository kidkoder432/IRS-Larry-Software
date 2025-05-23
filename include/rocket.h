#include <Arduino.h>

#define USE_FLASH 0

#if USE_RP2040
#include <WiFiNINA.h>
#elif USE_BLE_SENSE
#include <alt.h>
#include <SparkFun_BMI270_Arduino_Library.h>
#endif

#include <orientation.h>


#include <Servo.h>
#include <pins.h>
#include <tvc.h>
#include <leds.h>
#include <pyro.h>
#include <buzzer.h>
#include <chutes.h>

#include <config.h>
#include <datalog.h>

#if USE_BLE
#include <prints.h>
#include <HardwareBLESerial.h>
#endif

#define CHECK_EMPTY_LOG 0

float mag3(float x, float y, float z) { return sqrt(x * x + y * y + z * z); }
float absx(float x) { return x < 0 ? -x : x; }

enum DataLogSpeed {
    DLS_SLOW,
    DLS_MEDIUM,
    DLS_FAST
};


enum FlightStates {
    FS_SETUP,
    FS_READY,
    FS_LAUNCHING,
    FS_ARMED,
    FS_THRUST,
    FS_COASTING,
    FS_LANDING,
    FS_TOUCHDOWN,
    FS_ABORT,
    FS_TEST,
    FS_IDLE,
    FS_SHUTDOWN
};

class Rocket {

private:  // Member variables and internal functions
#if USE_BLE 
    HardwareBLESerial& bleSerial = HardwareBLESerial::getInstance();
#endif

    SensorReadings readings;
    GyroBiases biases;
    float pressureOffset = 0;

    Quaternion attitude;
    Vec3D dir;
    float yaw, pitch, roll;
    float vertVel, altitude;

    float x_out, y_out;

    bool doLog = true;
    bool doBatchLog = true;
    static const int BUFFER_SIZE = 256;
    DataPoint dataArr[BUFFER_SIZE];
    int loopCount = 0;

    bool ledsOn = true;

    SdExFat sd;
    ExFile dataFile;
    ExFile logFile;
    Config config;
    DataLogSpeed logSpeed = DLS_SLOW;
    int bufferCount = 0;

#if USE_RP2040
    LittleFS_MBED* fs = new LittleFS_MBED;
    FILE* flashFile;
#endif

    FlightStates currentState = FS_SETUP;
    unsigned long lastLoopTime;

public: // Public functions

    bool bleOn = true;
    float deltaTime = 0;
    bool useCompl = true;


    PyroChannel pyro1_motor = PyroChannel(PYRO_1_LANDING_MOTOR_PIN, 2000L, false, false);
    PyroChannel pyro2_land = PyroChannel(PYRO_2_LANDING_LEGS_PIN, 2000L, false, false);

    Parachute parachute = Parachute(PYRO_2_LANDING_LEGS_PIN);

    TVC tvc;

    // Handle catastrophic failures
    void HALT_AND_CATCH_FIRE() {

        while (true) {
            flash(COLOR_RED, 400);
            playAbortSound();
            delay(2);
        }
    }

    void HALT_AND_CATCH_FIRE(Color color) {
        while (true) {
            flash(color);
            playAbortSound();
            delay(2);

        }
    }

    void HALT_DONE() {
        playShutdownSound();
        while (true) {
            playLocatorSound();
            flash(COLOR_GREEN, COLOR_LIGHTBLUE, 1600);
            delay(2);
        }
    }

    // ---- Setup functions ---- //

    // Initialize Serial
    bool initSerial() {
        Serial.begin(115200);

        return true;
    }

    // Initialize BLE
#if USE_BLE 
    bool initBle() {
        if (!bleSerial.beginAndSetupBLE("Rocket Controller")) {
            printMessage("Failed to initialize BLE!");
            HALT_AND_CATCH_FIRE();
            return false;
        }
        return true;
    }
#endif
    // Initialize SD Card
    bool initSD() {
        if (!sd.begin(10, SPI_FULL_SPEED)) {
            printMessage("Failed to initialize SD card!");
            HALT_AND_CATCH_FIRE();
            return false;
        }
        return true;
    }

    // Log information about the SD Card
    void getSdInfo() {
        sdCardInfo(sd);
    }

    // Initialize log files
    bool initLogs() {
        if (!logFile.open("log.txt", O_WRITE | O_CREAT)) {
            printMessage("Failed to open log file!");
            HALT_AND_CATCH_FIRE();
            return false;
        }
    #if CHECK_EMPTY_LOG
        if (logFile.size() > 0) {
            printMessage("Log file is not empty!");
            HALT_AND_CATCH_FIRE(COLOR_YELLOW);
        }
    #endif
        logFile.truncate(0);
        logStatus("Log file initialized", logFile);
        logFile.sync();
        if (!dataFile.open("data.bin", O_WRITE | O_CREAT)) {
            printMessage("Failed to open data file!");
            HALT_AND_CATCH_FIRE();
            return false;
        }
    #if CHECK_EMPTY_LOG
        if (dataFile.size() > 0) {
            printMessage("Data file is not empty!");
            HALT_AND_CATCH_FIRE(COLOR_YELLOW);
        }
    #endif
        dataFile.truncate(0);
        dataFile.println(DATA_HEADER);
        dataFile.sync();
        setLogSpeed(DLS_SLOW);
        return true;
    }

#if USE_FLASH
    bool initFlash() {
        if (!fs->init()) {
            printMessage("Failed to initialize LittleFS!");
            HALT_AND_CATCH_FIRE();
        }
        flashFile = fopen("/littlefs/data.bin", "wb");
        if (!flashFile) {
            printMessage("Failed to open flash file!");
            HALT_AND_CATCH_FIRE();
        }
        return true;
    }
#endif

    void setLogSpeed(DataLogSpeed _logSpeed) {
        logSpeed = _logSpeed;
        switch (logSpeed) {
            case DLS_SLOW:
                printMessage("Setting log speed to LOW (10 Hz)");
                break;
            case DLS_MEDIUM:
                printMessage("Setting log speed to MEDIUM (33 Hz)");
                break;
            case DLS_FAST:
                printMessage("Setting log speed to FAST (100 Hz)");
                break;
        }
    }

    // Initialize configuration
    bool initConfig() {
        config = readConfig();
        logStatus("Config read successfully", logFile);
        doBatchLog = config["DATA_LOG_BATCH"] > 0;
        printConfig(config);
        return true;
    }

    // Initialize sensors
    bool setupSensors() {
        return initSensors();
    }

    // Calibrate sensors and log results
    bool calibrateAndLog() {
        printMessage("Calibrating sensors");
        logStatus("Calibrating Sensors", logFile);
        biases = calibrateSensors(config);
        char buf[64];
        snprintf(buf, sizeof(buf), "bx = %f, by = %f, bz = %f", biases.bx, biases.by, biases.bz);
        logStatus(buf, logFile);

        printMessage("Sensors calibrated");
        updateTime(false);
        return true;
    }

    // Initialize angles
    bool initAngles() {
        if (config["INIT_ACCEL"] > 0) {
            printMessage("Initializing angles using accelerometer");
            readSensors(readings, biases);
            float totalYaw = 0;
            float totalPitch = 0;

            for (int i = 0; i < 20; ++i) {
                readSensors(readings, biases);
                totalYaw -= atan2(readings.ax, sqrt(readings.az * readings.az + readings.ay * readings.ay));
                totalPitch -= atan2(readings.az, readings.ay);
            }

            yaw = totalYaw / 20;
            pitch = totalPitch / 20;
            dir = Vec3D(pitch * 180 / PI, 0, yaw * 180 / PI);


            // --- Initialize the attitude quaternion ---
            // We use the calculated average User Pitch, User Roll, and User Yaw (all in RADIANS).
            // It's understood that your `Quaternion::from_euler_rotation(arg1, arg2, arg3)` function
            // expects its arguments to directly correspond to rotations around the body's own X, Y, and Z axes,
            // respectively, based on your rocket's coordinate system.
            //   arg1 (function's "roll" param)  -> Rotation around Body X-axis -> Our avgUserPitchRad
            //   arg2 (function's "pitch" param) -> Rotation around Body Y-axis -> Our avgUserRollRad
            //   arg3 (function's "yaw" param)   -> Rotation around Body Z-axis -> Our avgUserYawRad
            attitude = Quaternion::from_euler_rotation(pitch, 0, yaw);
        }
        else {
            printMessage("Using default angles");
            dir = Vec3D(0, 0, 0);
            attitude = Quaternion();
        }

        Serial.print("rpy init: ");
        Serial.print(dir.x);
        Serial.print(", ");
        Serial.print(dir.y);
        Serial.print(", ");
        Serial.println(dir.z);
        return true;
    }

    // Initialize LEDs
    bool initLeds() {
        pinMode(LEDR, OUTPUT);
        pinMode(LEDG, OUTPUT);
        pinMode(LEDB, OUTPUT);
        showColor(COLOR_OFF);
        return true;
    }

    // Initialize TVC system
    bool initTvc() {
        tvc.setup(deltaTime, config);
        tvc.lock();
        logStatus("TVC initialized", logFile);
        return true;
    }

    // Initialize Pyros
    bool initPyros() {

        bool p1os = config["PYRO_1_ONE_SHOT"] > 0;
        bool p2os = config["PYRO_2_ONE_SHOT"] > 0;

        long p1ft = (long)max(2000L, (long)round(config["PYRO_1_FIRE_TIME"]));
        long p2ft = (long)max(2000L, (long)round(config["PYRO_2_FIRE_TIME"]));

        pyro1_motor = PyroChannel(PYRO_LANDING_MOTOR_IGNITION, p1ft, false, p1os);
        pyro2_land = PyroChannel(PYRO_LANDING_LEGS_DEPLOY, p2ft, false, p2os);

        pyro1_motor.begin();
        pyro2_land.begin();
        pyro1_motor.arm();
        pyro2_land.arm();
        return true;
    }

    // Inititalize buzzer
    bool initBuzzer() {
        pinMode(BUZZER_PIN, OUTPUT);
        return true;
    }

    // Initialize parachutes
    bool initChutes() {
        parachute.config(config);
        parachute.arm();
        return true;
    }

    // Finish setup
    bool finishSetup() {
        lastLoopTime = millis();
        logStatus("Setup complete", logFile);
        setState(FS_READY);
        // playStartupSound();
        return true;
    }

    // ---- Update functions ---- //

    // Update BLE communication
#if USE_BLE
    void updateBle() {
        bleSerial.poll();
    }
#endif

    // Update TVC control
    void updateTvc() {
        Vec2D tvc_out = tvc.update(dir, deltaTime);
        x_out = tvc_out.x;
        y_out = tvc_out.y;
    }

    // Update sensor readings
    void updateSensors() {
        readSensors(readings, biases);
    }

    // Update angles using sensor readings
    void updateAngles() {

        if (useCompl) {
            digitalWrite(LED_BUILTIN, HIGH);
            attitude = get_angles_compl_quat(config["COMP_FILTER_ALPHA_GYRO"], deltaTime, readings, attitude);
        }
        else {
            digitalWrite(LED_BUILTIN, LOW);
            attitude = get_angles_quat(readings, attitude, deltaTime);
        }

        dir = quaternion_to_user_euler(attitude);

        if (config["FLIP_DIR_X"] > 0) dir.x = -dir.x;
        if (config["FLIP_DIR_Y"] > 0) dir.y = -dir.y;
        if (config["FLIP_DIR_Z"] > 0) dir.z = -dir.z;

        pitch = dir.x;
        roll = dir.y;
        yaw = dir.z;


        // Normalize angles to the range [-180, 180]
        if (yaw > 180) yaw -= 360;
        if (yaw < -180) yaw += 360;
        if (pitch > 180) pitch -= 360;
        if (pitch < -180) pitch += 360;
    }

    // Update altitude and vertical velocity
    void updateAltVel() {
    #if USE_BLE_SENSE
        altitude = getAltitude(config["PRESSURE_REF"], pressureOffset);
    #else
        altitude = 0;
    #endif
        vertVel -= (readings.ay - 1) * 9.80665 * deltaTime;
    }

    // Play buzzer heartbeat tone
    void updateBuzzer() {
        if (millis() % 1500 < 50) {
            playConstantTone(415, 200);
        }
    }

    // Update pyros
    void updatePyros() {
        pyro1_motor.update();
        pyro2_land.update();
    }

    void updateChutes() {
        if (parachute.update()) {
            logMessage("Parachute deployed");
        }
    }

    void fireChutes() {
        parachute.deployTimer();
    }

    void updateAngleLeds() {
        if (!ledsOn) {
            showColor(COLOR_OFF);
            return;
        }

        Color color = COLOR_GREEN;  // Default color

        if (abs(yaw) >= 15 && abs(pitch) >= 15) {
            color = COLOR_PURPLE;
        }
        else if (abs(yaw) >= 15) {
            color = COLOR_RED;
        }
        else if (abs(pitch) >= 15) {
            color = COLOR_LIGHTBLUE;
        }

        showColor(color); // Single function call
    }


    void updateStateLeds() {
        switch (currentState) {
            case FS_READY:
                flash(COLOR_BLUE, 1000);
                break;
            case FS_THRUST:
                showColor(COLOR_GREEN);
                break;
            case FS_COASTING:
                flash(COLOR_GREEN, 600);
                break;
            case FS_LANDING:
                showColor(COLOR_YELLOW);
                break;
            case FS_TOUCHDOWN:
                showColor(COLOR_BLUE);
                break;
            case FS_ABORT:
                showColor(COLOR_RED);
                break;
        }
    }

    DataPoint getDataPoint() {
        DataPoint p;
        p.timestamp = millis();
        p.DELTA_T = deltaTime;
        p.r = readings;
        p.o = Vec3D(roll, pitch, yaw);
        p.x_out = x_out;
        p.y_out = y_out;
        p.alt = altitude; // getAltitude(config["PRESSURE_REF"], pressureOffset);
        p.currentState = static_cast<short>(currentState);
        p.vert_vel = vertVel;
        p.px = tvc.pid_x.p;
        p.ix = tvc.pid_x.i;
        p.dx = tvc.pid_x.d;
        p.py = tvc.pid_y.p;
        p.iy = tvc.pid_y.i;
        p.dy = tvc.pid_y.d;
        p.isEmpty = false;

        return p;
    }

#if USE_FLASH
    bool saveFlashFile(FILE* flashFile, ExFile& dataFile) {
        uint8_t buffer[88];
        // char header[] = "Time,Dt,ax,ay,az,gx,gy,gz,roll,pitch,yaw,x_out,y_out,x_act,y_act,alt,state,vel,px,ix,dx,py,iy,dy\n";
        // dataFile.write(header, strlen(header));

        dataFile.open("data.bin", O_WRITE | O_CREAT);
        dataFile.sync();
        dataFile.seekSet(dataFile.size());
        Serial.println(dataFile.size());

        fclose(flashFile);
        flashFile = fopen("/littlefs/data.bin", "rb");
        if (!flashFile) {
            printMessage("Couldn't open flash file");
            logMessage("Couldn't open flash file");
            return false;
        }
        long totalSize = 0;
        while (true) {
            size_t bytesRead = fread(buffer, 1, sizeof(buffer), flashFile);
            if (feof(flashFile)) break;
            if (bytesRead > 0) {
                size_t bytesWritten = dataFile.write(buffer, bytesRead);

                if (bytesWritten != bytesRead) {
                    printMessage("Error writing to data file");
                    logMessage("Error writing to data file");
                    return false;
                }
                totalSize += bytesRead;
                Serial.println(dataFile.size());
            }
            else {
                printMessage("Error reading from flash file");
                logMessage("Error reading from flash file");
                return false;
            }
        }

        fclose(flashFile);
        flashFile = fopen("/littlefs/data.bin", "wb");
        dataFile.sync();
        return true;

    }
#endif

    void updateDataLog() {

        if (!doLog) return;

        if (logSpeed == DLS_SLOW) {
            if (loopCount % 10 != 0) return;
        }
        else if (logSpeed == DLS_MEDIUM) {
            if (loopCount % 3 != 0) return;
        }
        else if (logSpeed == DLS_FAST) {
            if (loopCount % 1 != 0) return;
        }


        DataPoint d = getDataPoint();

        if (doBatchLog) {
            dataArr[bufferCount] = d;
            bufferCount++;
        }


        if (bufferCount >= BUFFER_SIZE) {
            logDataBatchOneShot(dataArr, BUFFER_SIZE);
            bufferCount = 0;

            for (int i = 0; i < BUFFER_SIZE; i++) {
                dataArr[i] = DataPoint();
            }
        }
    }

    // Log data to the SD card in a batch
    void logDataBatch(const DataPoint dataArr[], int bufferSize) {
        for (int i = 0; i < bufferSize; i++) {
            if (dataArr[i].isEmpty) {
                continue;
            }
        #if USE_FLASH
            logDataPointBin(dataArr[i], flashFile);
        #else
            logDataPointBin(dataArr[i], dataFile);
        #endif
        }
    #if USE_FLASH
        fflush(flashFile);
    #else
        dataFile.sync();
    #endif
    }


    void logDataBatchOneShot(const DataPoint dataArr[], int bufferSize) {
        unsigned char bytes[(bufferCount) * (sizeof(DataPoint) - 4)];
        for (int i = 0; i < bufferCount; i++) {
            if (dataArr[i].isEmpty) {
                continue;
            }
            DataPointBin pBin;
            pBin.p = dataArr[i];
            memcpy(&bytes[i * (sizeof(DataPoint) - 4)], pBin.dataBytes, sizeof(DataPoint) - 4);
        }
    #if USE_FLASH
        printFilesystemInfo();

        if (getFreeSpace() < 30 * 1024) {
            printMessage("Out of space on flash");
            saveFlashFile(flashFile, dataFile);
        }
    #endif


    #if USE_FLASH
        logDataRaw(bytes, (bufferCount + 1) * (sizeof(DataPoint) - 4), flashFile);
    #else
        logDataRaw(bytes, (bufferCount) * (sizeof(DataPoint) - 4), dataFile);
    #endif
    }

    // Log a single data point
    void logPoint(DataPoint p) {
    #if USE_FLASH
        logDataPointBin(p, flashFile);
    #else
        logDataPointBin(p, dataFile);
    #endif
    }

    // Update loop timing
    void updateTime(bool inc = true) {
        while (millis() - lastLoopTime < 10) {}
        deltaTime = (millis() - lastLoopTime) / 1000.0;
        lastLoopTime = millis();
        if (inc) loopCount++;

        loopCount %= 1000;

    }

    // ---- Control functions ---- //

    // Enable or disable data logging
    void setDataLog(bool _doLog) {

        if (_doLog == doLog) return;
        doLog = _doLog;
        if (doLog) {

            printMessage("Data Logging Enabled, opening logs");
            initSD();
            initLogs();
        }
        else {
            // log the last bits of data and close the file and card
            printMessage("Data Logging Disabled, saving logs");

            // force log
            if (doBatchLog) {
                logDataBatchOneShot(dataArr, BUFFER_SIZE);
            }
            else {
                logPoint(getDataPoint());
            }

        #if USE_FLASH
            if (fflush(flashFile)) {
                printMessage("Error syncing flash file");
            }
            if (!saveFlashFile(flashFile, dataFile)) {
                printMessage("Failed to save flash file to SD card");
            }
        #endif

            if (!cleanupLogs()) {
                printMessage("Failed to save logs!");
                cleanupSD();
                HALT_AND_CATCH_FIRE();
            }
            cleanupSD();
            printMessage("Logs saved successfully");

        }
    }

    void toggleDataLog() {
        setDataLog(!doLog);
    }

    // Enable or disable LEDs
    void setLeds(bool ledsOn) {
        this->ledsOn = ledsOn;
    }

    void toggleLeds() {
        setLeds(!ledsOn);
    }

    void setState(FlightStates state) {
        currentState = state;
    }

    void enableCompl() {
        useCompl = true;
    }

    void disableCompl() {
        useCompl = false;
    }


    // Reset angles to initial state
    void resetAngles() {

        logStatus("Resetting Angles", logFile);

        initAngles();
    }

    // Fire pyro 1
    void firePyro1() {
        pyro1_motor.fire();
        logStatus("Pyro 1 fired", logFile);
    }

    // Fire pyro 2
    void firePyro2() {
        pyro2_land.fire();
        logStatus("Pyro 2 fired", logFile);
    }

    void logMessage(const char* message) {
        char buf[256];
        snprintf(buf, sizeof(buf), "LOG: %s", message);
        Serial.println(buf);
        logStatus(message, logFile);
    }

    void printMessage(auto message) {
    #if USE_BLE 
        msgPrintln(bleOn, bleSerial, message);
    #else
        Serial.println(message);
    #endif
    }

    void printMessage(auto message, bool ln) {
        if (ln) {
        #if USE_BLE
            msgPrintln(bleOn, bleSerial, message);
        #else
            Serial.println(message);
        #endif
        }
        else {
        #if USE_BLE
            msgPrint(bleOn, bleSerial, message);
            msgPrint(bleOn, bleSerial, " ");
        #else
            Serial.print(message);
            Serial.print(" ");
        #endif
        }
    }

    // --- Getters --- //

#if USE_BLE 
    HardwareBLESerial& getBle() { return bleSerial; }
#endif
    Vec3D getDir() { return dir; }
    Quaternion getAttitude() { return attitude; }
    const SensorReadings& getReadings() { return readings; }
    float getAlt() { return altitude; }
    FlightStates getState() { return currentState; }
    float getConfigValue(const char* key) { return config[key]; }


    bool cleanupLogs() {
        logMessage("Cleaning up logs...");
    #if USE_FLASH 
        printMessage("Closing flash file...");
        if (fclose(flashFile)) {
            printMessage("Error closing flash file");
            return false;
        }
    #endif

        printMessage("Closing data file...");
        if (!dataFile.sync()) {
            printMessage("Error syncing data file");
            return false;
        }
        if (!dataFile.close()) {
            printMessage("Error closing data file");
            return false;
        }

        printMessage("Closing log file...");
        if (!logFile.sync()) {
            printMessage("Error syncing log file");
            return false;
        }
        if (!logFile.close()) {
            printMessage("Error closing log file");
            return false;
        }

        return true;
    }

    void cleanupSD() {
        sd.end();
    }

    void fullCleanup() {
        printMessage("Shutting down...");
        logMessage("Shutting down...");
        printMessage("Disarming pyros...");
        logMessage("Disarming pyros...");
        pyro1_motor.disarm();
        pyro2_land.disarm();

        stopTone();

        printMessage("Shutting down TVC...");
        logMessage("Shutting down TVC...");
        tvc.abort();

        printMessage("Shutting down sensors...");
        logMessage("Shutting down sensors...");

        printMessage("Cleaning up logs...");
        // force log
        if (doBatchLog) {
            logDataBatchOneShot(dataArr, BUFFER_SIZE);
        }
        else {
            logPoint(getDataPoint());
        }

        cleanupLogs();
        cleanupSD();

        printMessage("All systems shut down. Please reset the board. ");
    #if USE_BLE 
        bleSerial.end();
    #endif
        Serial.end();


    }

    void abort() {
        parachute.deploy();
        printMessage("Disarming pyros...");
        logMessage("Disarming pyros...");
        pyro1_motor.disarm();
        pyro2_land.disarm();

        stopTone();

        printMessage("Shutting down TVC...");
        logMessage("Shutting down TVC...");
        tvc.abort();

    }

    void finish() {
        setState(FS_SHUTDOWN);
        fullCleanup();
        HALT_DONE();
    }
};