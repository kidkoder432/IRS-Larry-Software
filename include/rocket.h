#include <Arduino.h>

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

    Quaternion attitude;
    Vec3D dir;
    float yaw, pitch, roll;
    float vertVel, altitude;

    float x_out, y_out;

    bool doLog = true;
    static const int BUFFER_SIZE = 512;
    int loopCount = 0;

    bool ledsOn = true;

    SdExFat sd;
    ExFile dataFile;
    ExFile logFile;
    Config config;
    DataLogger dataLogger;
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
    bool tvcPermaLocked = false;
#if USE_BLE_SENSE 
    Altimeter altimeter;
#endif

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
        if (!sd.begin(SdSpiConfig(10, DEDICATED_SPI, SPI_FULL_SPEED))) {
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
        logFile.preAllocate(65536);

        logMessage("Log file initialized");
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
        dataFile.preAllocate(8388608);
        dataFile.println(DATA_HEADER);
        dataFile.sync();
        setLogSpeed(DLS_FAST);
        dataLogger.setTargetFile(&dataFile);
        return true;
    }

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
        logMessage("Config read successfully");
        tvcPermaLocked = config["TVC_PERMA_LOCKED"] > 0;
        printConfig(config);
        return true;
    }

    // Initialize sensors
    bool setupSensors() {
        initIMU();
        delay(1000);
    #if USE_BLE_SENSE
        if (!altimeter.begin()) {
            printMessage("Failed to initialize baro!");
            HALT_AND_CATCH_FIRE(COLOR_PINK);

        }
        Wire1.setClock(400000);

    #endif
        return true;
    }

    // Calibrate sensors and log results
    bool calibrateAndLog() {
        printMessage("Calibrating sensors");
        logMessage("Calibrating Sensors");
        biases = calibrateSensors(config);
        char buf[64];
        snprintf(buf, sizeof(buf), "bx = %f, by = %f, bz = %f", biases.bx, biases.by, biases.bz);
        logMessage(buf);

        altimeter.calibrate();

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
                Vec3D accelAngles = get_angles_accel(readings);
                totalYaw += accelAngles.z;
                totalPitch += accelAngles.y;
            }

            yaw = totalYaw / 20;
            pitch = totalPitch / 20;
            dir = Vec3D(0, pitch * 180 / PI, yaw * 180 / PI);

            attitude = from_euler_xyz(0, pitch, yaw);
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
        logMessage("TVC initialized");
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
        logMessage("Setup complete");
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
        Vec2D tvc_out;
        if (tvcPermaLocked) {
            tvc_out = tvc.lock();
        }
        else {
            tvc_out = tvc.update(dir, attitude, deltaTime);
        }
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

        dir = quaternion_to_euler(attitude);

        if (config["FLIP_DIR_X"] > 0) dir.x = -dir.x;
        if (config["FLIP_DIR_Y"] > 0) dir.y = -dir.y;
        if (config["FLIP_DIR_Z"] > 0) dir.z = -dir.z;

        roll = dir.x;
        pitch = dir.y;
        yaw = dir.z;


        // Normalize angles to the range [-180, 180]
        if (yaw > 180) yaw -= 360;
        if (yaw < -180) yaw += 360;
        if (pitch > 180) pitch -= 360;
        if (pitch < -180) pitch += 360;
    }

    // Update altitude and vertical velocity
#if USE_BLE_SENSE
    void updateAltVel() {
        altimeter.update(readings, attitude, deltaTime);
        altitude = altimeter.getAltitude();
        vertVel = altimeter.getVelocity();
    }
#else
    void updateAltVel() {
        altitude = 0; // No altitude sensor
        vertVel = 0; // No vertical velocity sensor
    }
#endif

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
            case FS_ARMED:
                showColor(COLOR_WHITE);
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
        p.timestamp = (uint32_t)millis();

        p.r = readings;
        p.o = dir;

        p.x_out = (int16_t)(x_out * 100.0f);
        p.y_out = (int16_t)(y_out * 100.0f);

        p.state = (int16_t)currentState;
        p.alt = (int16_t)(altitude * 100.0f);
        p.vert_vel = (int16_t)(vertVel * 100.0f);

        p.px = (int16_t)(tvc.pid_x.p * 100.0f);
        p.ix = (int16_t)(tvc.pid_x.i * 100.0f);
        p.dx = (int16_t)(tvc.pid_x.d * 100.0f);

        p.py = (int16_t)(tvc.pid_y.p * 100.0f);
        p.iy = (int16_t)(tvc.pid_y.i * 100.0f);
        p.dy = (int16_t)(tvc.pid_y.d * 100.0f);

        p.dt = (int16_t)(deltaTime * 1000.0f);
        p.isEmpty = (uint8_t)false;
        return p;
    }

    void getNewData() {
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

        dataLogger.addPoint(getDataPoint());

    }

    void logNextData() {

        if (!doLog) return;

        dataLogger.logNext();

    }

    // Log data to the SD card in a batch
    void logDataBatch(const DataPoint dataArr[], int bufferSize) {
        for (int i = 0; i < bufferSize; i++) {
            if (dataArr[i].isEmpty) {
                continue;
            }

            logDataPointBin(dataArr[i], dataFile);
        }
        dataFile.sync();
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

        logDataRaw(bytes, (bufferCount) * (sizeof(DataPoint) - 4), dataFile);
    }

    // Log a single data point
    void logPoint(DataPoint p) {
        logDataPointBin(p, dataFile);
    }

    // Update loop timing
    void updateTime(bool inc = true) {
        deltaTime = (micros() - lastLoopTime) / 1000000.0;
        lastLoopTime = micros();
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
            dataLogger.logAllPoints();

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

        logMessage("Resetting Angles");

        initAngles();
    }

    // Fire pyro 1
    void firePyro1() {
        pyro1_motor.fire();
        logMessage("Pyro 1 fired");
    }

    // Fire pyro 2
    void firePyro2() {
        pyro2_land.fire();
        logMessage("Pyro 2 fired");
    }

    void logMessage(const char* message) {
        logStatus(message, logFile);
    }

    void printMessage(const char* message) {
    #if USE_BLE 
        msgPrintln(bleOn, bleSerial, message);
    #else
        Serial.println(message);
    #endif
        logMessage(message);
    }

    template <typename T>
    void printMessage(T message) {
    #if USE_BLE 
        msgPrintln(bleOn, bleSerial, message);
    #else
        Serial.println(message);
    #endif
        if constexpr (std::is_same<std::decay_t<T>, const char*>::value) {
            logMessage(message);
        }
    }

    template <typename T>
    void printMessage(T message, bool ln) {
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

        printMessage("Closing data file...");
        dataFile.truncate();
        if (!dataFile.sync()) {
            printMessage("Error syncing data file");
            return false;
        }
        if (!dataFile.close()) {
            printMessage("Error closing data file");
            return false;
        }

        printMessage("Closing log file...");
        logFile.truncate();
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
        dataLogger.logAllPoints();

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

    bool heartbeat(bool state) {
        if (loopCount % 40 == 0) {
            loopCount = 0;
            if (state) {
                showColor(COLOR_GREEN);
            }
            else {
                showColor(COLOR_OFF);
            }
            return !state;
        }
        else {
            return state;
        }
    }

    int getPending() {
        return dataLogger.numPending();
    }
};