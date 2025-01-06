#include <Arduino.h>

#if !USE_RP2040
#include <alt.h>
#endif

#include <SparkFun_BMI270_Arduino_Library.h>
#include <orientation.h>

#if USE_RP2040
#include <WiFiNINA.h>
#endif

#include <Servo.h>
#include <pins.h>
#include <tvc.h>
#include <leds.h>
#include <pyro.h>
#include <buzzer.h>

#include <config.h>
#include <datalog.h>

#if USE_BLE
#include <prints.h>
#include <HardwareBLESerial.h>
#endif

float mag3(float x, float y, float z) { return sqrt(x * x + y * y + z * z); }

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
    bool useCompl = true;
    float yaw, pitch, roll;
    float vertVel, altitude;

    float x_out, y_out;

    bool doLog = true;
    bool doBatchLog = true;
    static const int BUFFER_SIZE = 100;
    DataPoint dataArr[BUFFER_SIZE];
    int loopCount = 1;

    bool ledsOn = true;

    SdExFat sd;
    ExFile dataFile;
    ExFile logFile;
    Config config;

    int currentState = 0;
    unsigned long lastLoopTime;

public: // Public functions

    bool bleOn = true;
    float deltaTime = 0;

    PyroChannel pyro1_motor = PyroChannel(PYRO_1_LANDING_MOTOR_PIN, 6000L, false, false);
    PyroChannel pyro2_land = PyroChannel(PYRO_2_LANDING_LEGS_PIN, 6000L, false, false);

    TVC tvc;

    // Handle catastrophic failures
    void HALT_AND_CATCH_FIRE() {

        while (true) {
            flash(COLOR_RED, 400);
            playAbortSound();

        }
    }

    void HALT_AND_CATCH_FIRE(Color color) {
        while (true) {
            flash(color);
        }
    }

    void HALT_DONE() {
        playShutdownSound();
        while (true) {
            playLocatorSound();
            flash(COLOR_GREEN, COLOR_LIGHTBLUE, 1600);
        }
    }

    // ---- Setup functions ---- //

    // General setup function
    bool setup() {
        if (!initSerial()) return false;
    #if USE_BLE 
        if (!initBle()) return false;
    #endif
        if (!initSD()) return false;
        if (!initLogs()) return false;
        if (!initConfig()) return false;
        if (!setupSensors()) return false;
        if (!initAngles()) return false;
        if (!initLeds()) return false;
        if (!initTvc()) return false;
        if (!initPyros()) return false;
        if (!initBuzzer()) return false;
        return finishSetup();
    }

    // Initialize Serial
    bool initSerial() {
        Serial.begin(115200);

        return !(!Serial);
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
        if (!logFile.open("log.txt", O_TRUNC | O_WRITE | O_CREAT)) {
            printMessage("Failed to open log file!");
            HALT_AND_CATCH_FIRE();
            return false;
        }
        logStatus("Log file initialized", logFile);

        if (!dataFile.open("data.bin", O_TRUNC | O_WRITE | O_CREAT)) {
            printMessage("Failed to open data file!");
            HALT_AND_CATCH_FIRE();
            return false;
        }
        dataFile.println("Timestamp,Delta Time,Ax,Ay,Az,Gx,Gy,Gz,Yaw,Pitch,Roll,TvcX,TvcY,Alt,State,Vel,Px,Ix,Dx,Py,Iy,Dy");
        dataFile.sync();
        return true;
    }

    // Initialize configuration
    bool initConfig() {
        config = readConfig();
        logStatus("Config read successfully", logFile);
        doBatchLog = config["DATA_LOG_BATCH"] > 0;
        return true;
    }

    // Initialize sensors
    bool setupSensors() {
        initSensors();
    #if !USE_RP2040
        logStatus("Calibrating Barometer", logFile);
        pressureOffset = calculateOffset(config["PRESSURE_REF"]);
        logStatus("Barometer calibrated", logFile);
    #endif
        return true;
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
        return true;
    }

    // Initialize angles
    bool initAngles() {
        if (config["INIT_ACCEL"] > 0) {
            printMessage("Initializing angles using accelerometer");
            readSensors(readings, biases);
            yaw = atan2(readings.ax, sqrt(readings.az * readings.az + readings.ay * readings.ay));
            pitch = atan2(readings.az, readings.ay);
            dir = Vec3D(0, -pitch, -yaw);
            attitude = Quaternion::from_euler_rotation(yaw, pitch, 0);
        }
        else {
            printMessage("Using default angles");
            dir = Vec3D(0, 0, 0);
            attitude = Quaternion();
        }
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
        tvc.begin(deltaTime);
        tvc.lock();
        tvc.configure(config);
        logStatus("TVC initialized", logFile);
        return true;
    }

    // Initialize Pyros
    bool initPyros() {

        bool p1os = config["PYRO_1_ONE_SHOT"] > 0;
        bool p2os = config["PYRO_2_ONE_SHOT"] > 0;

        long p1ft = (long)max(2000L, (long)round(config["PYRO_1_FIRE_TIME"]));
        long p2ft = (long)max(2000L, (long)round(config["PYRO_2_FIRE_TIME"]));

        // pyro1_motor = PyroChannel(PYRO_LANDING_MOTOR_IGNITION, p1ft, false, p1os);
        // pyro2_land = PyroChannel(PYRO_LANDING_LEGS_DEPLOY, p2ft, false, p2os);

        pyro1_motor.begin();
        pyro2_land.begin();
        pyro1_motor.arm();
        pyro2_land.
            arm();
        return true;
    }

    // Inititalize buzzer
    bool initBuzzer() {
        pinMode(BUZZER_PIN, OUTPUT);
        return true;
    }

    // Finish setup
    bool finishSetup() {
        lastLoopTime = millis();
        logStatus("Setup complete", logFile);
        playStartupSound();
        return true;
    }

    // ---- Update functions ---- //

    // General update function
    void update() {
    #if USE_BLE 
        updateBle();
    #endif
        updateSensors();
        updateAngles();
        updateAltVel();
        updateState();
        updateTvc();
        updatePyros();
        updateBuzzer();
        updateAngleLeds();
        updateDataLog();

        updateTime();
    }

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

        dir = get_angles_quat(readings, attitude, deltaTime);

        if (config["FLIP_DIR_X"] > 0) dir.x = -dir.x;
        if (config["FLIP_DIR_Y"] > 0) dir.y = -dir.y;
        if (config["FLIP_DIR_Z"] > 0) dir.z = -dir.z;

        if (useCompl) {
            Vec2D dir_c = get_angles_complementary(config["COMP_FILTER_ALPHA_GYRO"], deltaTime, readings, yaw, pitch);
            yaw = dir_c.x;
            pitch = dir_c.y;
        }
        else {
            yaw = dir.z;
            pitch = dir.y;
        }

        roll = dir.x;

        dir = Vec3D(yaw, pitch, roll);

        // Normalize angles to the range [-180, 180]
        if (yaw > 180) yaw -= 360;
        if (yaw < -180) yaw += 360;
        if (pitch > 180) pitch -= 360;
        if (pitch < -180) pitch += 360;
    }

    // Update altitude and vertical velocity
    void updateAltVel() {
    #if !USE_RP2040
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

    // Update current state of the system
    void updateState() {
        // ABORT: State -> 127
        if (abs(yaw) >= 35 || abs(pitch) >= 35) {
            logMessage("ERR: ABORT - UNSTABLE");
            currentState = 127;
        }

        // TOUCHDOWN: State 3 -> 5
        if (currentState == 2 && mag3(readings.ax, readings.ay, readings.az) <= 1.5 && altitude <= 2) {
            currentState = 4;
            logMessage("Touchdown");
        }

        // STAGE 1 BURNOUT: State 1 -> 2
        if (currentState == 1 && mag3(readings.ax, readings.ay, readings.az) <= 1.1) {
            currentState = 2;
            logMessage("Stage 1 burnout");
            logMessage("Coasting");
        }

        // LAUNCH: State 0 -> 1
        if (currentState == 0 && mag3(readings.ax, readings.ay, readings.az) >= 1.5) {
            currentState = 1;
            logMessage("Launch");
        }
    }

    // Update pyros
    void updatePyros() {
        pyro1_motor.update();
        pyro2_land.update();
    }

    void updateAngleLeds() {
        if (ledsOn) {
            if (abs(yaw) >= 15) {
                showColor(COLOR_RED);
                if (abs(pitch) >= 15) {
                    showColor(COLOR_PURPLE);
                }
            }
            else {
                showColor(COLOR_GREEN);
                if (abs(pitch) >= 15) {
                    showColor(COLOR_LIGHTBLUE);
                }
            }
        }
        else {
            showColor(COLOR_OFF);
        }
    }

    void updateStateLeds() {
        switch (currentState) {
            case 0:
                flash(COLOR_BLUE, 1000);
                break;
            case 1:
                showColor(COLOR_GREEN);
                break;
            case 2:
                flash(COLOR_GREEN, 600);
                break;
            case 3:
                showColor(COLOR_YELLOW);
                break;
            case 4:
                showColor(COLOR_BLUE);
                break;
            case 127:
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
        p.currentState = currentState;
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

    void updateDataLog() {

        if (!doLog) return;

        DataPoint d = getDataPoint();

        if (doBatchLog)
            dataArr[loopCount - 1] = d;
        if (loopCount >= BUFFER_SIZE) {
            logDataBatch(dataArr, BUFFER_SIZE);
            loopCount = 0;

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
            logDataPointBin(dataArr[i], dataFile);
        }
    }

    // Log a single data point
    void logPoint(DataPoint p) {
        logDataPointBin(p, dataFile);
    }

    // Update loop timing
    void updateTime() {
        deltaTime = (millis() - lastLoopTime) / 1000.0;
        lastLoopTime = millis();
        loopCount++;

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
                logDataBatch(dataArr, BUFFER_SIZE);
            }
            else {
                logPoint(getDataPoint());
            }

            cleanupLogs();
            cleanupSD();

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

    void setState(int state) {
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
    int getCurrentState() { return currentState; }
    float getConfigValue(std::string key) { return config[key]; }


    void cleanupLogs() {
        dataFile.sync();
        dataFile.close();
        logMessage("Cleaning up logs...");
        logFile.sync();
        logFile.close();
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

        printMessage("Shutting down TVC...");
        logMessage("Shutting down TVC...");
        tvc.abort();

        printMessage("Shutting down sensors...");
        logMessage("Shutting down sensors...");

        printMessage("Cleaning up logs...");
        cleanupLogs();
        cleanupSD();

        printMessage("All systems shut down. Please reset the board. ");
    #if USE_BLE 
        bleSerial.end();
    #endif
        Serial.end();


    }

    void abort() {
        fullCleanup();
        HALT_AND_CATCH_FIRE();

    }

    void finish() {
        fullCleanup();
        HALT_DONE();
    }
};