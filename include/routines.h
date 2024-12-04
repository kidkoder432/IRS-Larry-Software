#include <Arduino.h>
#include <Serial.h>
#include <Servo.h>

#include <SparkFun_BMI270_Arduino_Library.h>
#include <orientation.h>

#include <tvc.h>
#include <leds.h>
#include <pyro.h>

#include <alt.h>
#include <Arduino_LPS22HB.h>

#include <config.h>
#include <datalog.h>
#include <prints.h>
#include <HardwareBLESerial.h>

float mag3(float x, float y, float z) { return sqrt(x * x + y * y + z * z); }

class Rocket {
public:

    // Handle catastrophic failures
    void HALT_AND_CATCH_FIRE() {
        while (true) {
            flash(COLOR_RED, 400);
        }
    }

    void HALT_AND_CATCH_FIRE(Color color) {
        while (true) {
            flash(color);
        }
    }

    void HALT_DONE() {
        while (true) {
            flash(COLOR_GREEN, COLOR_LIGHTBLUE, 800);
        }
    }

    // ---- Setup functions ---- //

    // General setup function
    bool setup() {
        if (!initSerial()) return false;
        if (!initBle()) return false;
        if (!initSd()) return false;
        if (!initLogs()) return false;
        if (!initConfig()) return false;
        if (!setupSensors()) return false;
        if (!initAngles()) return false;
        if (!initLeds()) return false;
        if (!initTvc()) return false;
        if (!initPyros()) return false;
        return finishSetup();
    }

    // Initialize Serial
    bool initSerial() {
        Serial.begin(115200);
        return true;
    }

    // Initialize BLE
    bool initBle() {
        if (!bleSerial.beginAndSetupBLE("Rocket Controller")) {
            msgPrintln(bleOn, bleSerial, "Failed to initialize BLE!");
            HALT_AND_CATCH_FIRE();
            return false;
        }
        return true;
    }

    // Initialize SD Card
    bool initSd() {
        if (!sd.begin(10, SPI_FULL_SPEED)) {
            msgPrintln(bleOn, bleSerial, "Failed to initialize SD card!");
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
            msgPrintln(bleOn, bleSerial, "Failed to open log file!");
            HALT_AND_CATCH_FIRE();
            return false;
        }
        logStatus("Log file initialized", logFile);

        if (!dataFile.open("data.bin", O_TRUNC | O_WRITE | O_CREAT)) {
            msgPrintln(bleOn, bleSerial, "Failed to open data file!");
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
        return true;
    }

    // Initialize sensors
    bool setupSensors() {
        initSensors();
        logStatus("Calibrating Barometer", logFile);
        pressureOffset = calculateOffset(config["PRESSURE_REF"]);
        logStatus("Barometer calibrated", logFile);

        return true;
    }

    // Calibrate sensors and log results
    bool calibrateAndLog() {
        msgPrintln(bleOn, bleSerial, "Calibrating sensors");
        logStatus("Calibrating Sensors", logFile);
        biases = calibrateSensors(config);

        char buf[64];
        snprintf(buf, sizeof(buf), "bx = %f, by = %f, bz = %f", biases.bx, biases.by, biases.bz);
        logStatus(buf, logFile);

        msgPrintln(bleOn, bleSerial, "Sensors calibrated");
        return true;
    }

    // Initialize angles
    bool initAngles() {
        if (config["INIT_ACCEL"] > 0) {
            msgPrintln(bleOn, bleSerial, "Initializing angles using accelerometer");
            readSensors(readings, biases);
            yaw = atan2(readings.ax, sqrt(readings.az * readings.az + readings.ay * readings.ay));
            pitch = atan2(readings.az, readings.ay);
            dir = Vec3D(0, -pitch, -yaw);
            attitude = Quaternion::from_euler_rotation(yaw, 0, pitch);
        }
        else {
            msgPrintln(bleOn, bleSerial, "Using default angles");
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
        pyro1_motor.begin();
        pyro2_land.begin();
        pyro1_motor.arm();
        pyro2_land.arm();
        return true;
    }

    // Finish setup
    bool finishSetup() {
        lastLoopTime = millis();
        logStatus("Setup complete", logFile);
        return true;
    }

    // ---- Update functions ---- //

    // General update function
    void update() {
        updateBle();
        updateSensors();
        updateAngles();
        updateAltVel();
        updateState();
        updateTvc();
        updatePyros();
        logData();

        updateTime();
    }

    // Update BLE communication
    void updateBle() {
        bleSerial.poll();
    }

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

        yaw = dir.z;
        pitch = dir.y;
        roll = dir.x;

        // Normalize angles to the range [-180, 180]
        if (yaw > 180) yaw -= 360;
        if (yaw < -180) yaw += 360;
        if (pitch > 180) pitch -= 360;
        if (pitch < -180) pitch += 360;
    }

    // Update altitude and vertical velocity
    void updateAltVel() {
        altitude = getAltitude(config["PRESSURE_REF"], pressureOffset);
        vertVel -= readings.ay * 9.80665 * deltaTime;
    }

    // Update current state of the system
    void updateState() {
        // ABORT: State -> 127
        if (abs(yaw) >= 45 || abs(pitch) >= 45) {
            logStatus("ERR: ABORT - UNSTABLE", logFile);
            currentState = 127;
        }

        // TOUCHDOWN: State 3 -> 5
        if (currentState == 3 && mag3(readings.ax, readings.ay, readings.az) <= 1.5) {
            currentState = 5;
        }

        // STAGE 1 BURNOUT: State 1 -> 3
        if (currentState == 1 && mag3(readings.ax, readings.ay, readings.az) <= 1.1) {
            currentState = 3;
        }

        // LAUNCH: State 0 -> 1
        if (currentState == 0 && mag3(readings.ax, readings.ay, readings.az) >= 1.5) {
            currentState = 1;
        }
    }

    // Update pyros
    void updatePyros() {
        pyro1_motor.update();
        pyro2_land.update();
    }

    void updateLeds() {
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

    // Log data to the SD card
    void logData() {
        if (!doLog) return;

        DataPoint p;
        p.timestamp = millis();
        p.DELTA_T = deltaTime;
        p.r = readings;
        p.o = Vec3D(roll, pitch, yaw);
        p.x_out = x_out;
        p.y_out = y_out;
        p.alt = getAltitude(config["PRESSURE_REF"], pressureOffset);
        p.currentState = currentState;
        p.vert_vel = vertVel;
        p.px = tvc.pid_x.p;
        p.ix = tvc.pid_x.i;
        p.dx = tvc.pid_x.d;
        p.py = tvc.pid_y.p;
        p.iy = tvc.pid_y.i;
        p.dy = tvc.pid_y.d;

        logDataBin(p, dataFile);
    }

    // Update loop timing
    void updateTime() {
        deltaTime = (millis() - lastLoopTime) / 1000.0;
        lastLoopTime = millis();

    }

    // ---- Control functions ---- //

    // Enable or disable TVC
    void setTvc(bool locked) {
        if (locked) {
            msgPrintln(bleOn, bleSerial, "Locking TVC");
            logStatus("Locking TVC", logFile);
            tvc.lock();
        }
        else {
            msgPrintln(bleOn, bleSerial, "Unlocking TVC");
            logStatus("Unlocking TVC", logFile);
            tvc.pid_x.reset();
            tvc.pid_y.reset();
            tvc.unlock();
        }
    }

    // Enable or disable data logging
    void setDataLog(bool doLog) {
        this->doLog = doLog;
        if (doLog) {
            initLogs();
            logStatus("Data logging enabled", logFile);
        }
        else {
            dataFile.close();
            logStatus("Data logging disabled", logFile);
            logFile.close();
        }
    }

    // Enable or disable LEDs
    void setLeds(bool ledsOn) {
        this->ledsOn = ledsOn;
    }

    // Reset angles to initial state
    void resetAngles() {

        logStatus("Resetting Angles", logFile);

        if (config["INIT_ACCEL"] > 0) {
            msgPrintln(bleOn, bleSerial, "Resetting angles using accelerometer");
            readSensors(readings, biases);
            yaw = atan2(readings.ax, sqrt(readings.az * readings.az + readings.ay * readings.ay));
            pitch = atan2(readings.az, readings.ay);
            dir = Vec3D(0, -pitch, -yaw);
            attitude = Quaternion::from_euler_rotation(yaw, 0, pitch);
        }
        else {
            msgPrintln(bleOn, bleSerial, "Using default angles");
            dir = Vec3D(0, 0, 0);
            attitude = Quaternion();
        }
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
        msgPrintln(bleOn, bleSerial, message);
    }

    void printMessage(auto message, bool ln) {
        if (ln) msgPrintln(bleOn, bleSerial, message);
        else msgPrint(bleOn, bleSerial, message);
    }

    HardwareBLESerial& getBle() { return bleSerial; }
    Vec3D getDir() { return dir; }
    SensorReadings getReadings() { return readings; }
    float getAlt() { return altitude; }

    void cleanupLogs() {
        dataFile.close();
        logMessage("Cleaning up...");
        logFile.close();
    }

    void fullCleanup() {
        printMessage("Shutting down...");
        pyro1_motor.disarm();
        pyro2_land.disarm();
        cleanupLogs();
        sd.end();
        bleSerial.end();
        tvc.abort();
        Serial.end();
        HALT_DONE();


    }

    HardwareBLESerial& bleSerial = HardwareBLESerial::getInstance();
    bool bleOn = true;

    SensorReadings readings;
    Biases biases;
    float pressureOffset = 0;

    Quaternion attitude;
    Vec3D dir;
    float yaw, pitch, roll;
    float vertVel, altitude;

    TVC tvc;
    float x_out, y_out;

    bool doLog = true;
    bool ledsOn = true;

    SdExFat sd;
    ExFile dataFile;
    ExFile logFile;
    Config config;

    PyroChannel pyro1_motor = PyroChannel(PYRO_LANDING_MOTOR_IGNITION, 1000L, false, true);
    PyroChannel pyro2_land = PyroChannel(PYRO_LANDING_LEGS_DEPLOY, 1000L, false, true);

    int currentState = 0;
    unsigned long lastLoopTime;
    float deltaTime = 0;
};