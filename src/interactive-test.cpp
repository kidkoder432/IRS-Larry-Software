// Test all components and features

#include <rocket_dummy.h>
#include <NRF52_MBED_TimerInterrupt.h>

Rocket rocket;

#if USE_BLE 
HardwareBLESerial& bleSerial = rocket.getBle();
#endif

NRF52_MBED_Timer ITimer(NRF_TIMER_3);


char receivedChar;
bool newCommand = false;
bool dirOutLock = true;
bool sensorOutLock = true;
bool experimentMode = false;

long currentMs;

const char HELP_STR[] =
R"(Input commands to test the following:
U: Unlock the TVC
L: Lock the TVC
K: Toggle LEDs
C: Calibrate the Gyro
O: Print Orientation
A: Print Altitude
R: Print Sensor Readings
D: Toggle Data Logging
X: Cancel Printing
G: Activate Pyro 1 (Motor Ignition)
T: Deploy Parachute (3 Second Timer)
S: SD Card Info
Q: Reset Angles
P: Show Performance Metrics
B: Toggle Complementary Filter
E: Toggle Experiment/Test Mode
Z: Switch Bluetooth to USB (override)
H: Help)";


// Global timing struct
struct PerfStats {
    unsigned long sensorUs;
    unsigned long anglesUs;
    unsigned long tvcUs;
    unsigned long totalUs;
    unsigned long sdUs;
    unsigned long isr_to_run;  // how long between ISR firing and code actually running
    unsigned long worstTotal;
    unsigned long worstSensor;
} perf;

volatile unsigned long isrFiredAt = 0;

volatile bool state = false;
volatile bool sdFree = true;
volatile unsigned long long lastLoopTime = 0;

void loopHandler() {
    sdFree = false;
    isrFiredAt = micros();
    state = rocket.heartbeat(state);
}

void recvOneChar() {
    if (Serial.available() > 0) {
        receivedChar = Serial.read();
        receivedChar = toupper(receivedChar);
        // rocket.printMessage(receivedChar);
        newCommand = true;
    }
#if USE_BLE 
    if (bleSerial.available() > 0) {
        receivedChar = bleSerial.read();
        receivedChar = toupper(receivedChar);
        // rocket.printMessage(receivedChar);
        newCommand = true;
    }
#endif
    if (receivedChar == '\n' || receivedChar == '\r') {
        newCommand = false;
    }
}

void setup() {

    // Setup basic interfaces
    rocket.initSerial();
    delay(2000);

    rocket.initBuzzer();
    rocket.printMessage("Buzzer initialized!");

    rocket.initLeds();
    rocket.printMessage("LEDs initialized!");

    // Setup SD card, config and data logging
    rocket.initSD();
    rocket.printMessage("SD card initialized!");
    rocket.getSdInfo();
    rocket.initLogs();
#if USE_FLASH
    rocket.initFlash();
#endif
    rocket.printMessage("Data logging initialized!");
    rocket.initConfig();
    rocket.printMessage("Config initialized!");

    // Init sensors and angles
    rocket.setupSensors();
    rocket.calibrateAndLog();

#if USE_BLE
    rocket.initBle();
    int waits = 0;
    bool override = false;
    while (!bleSerial && waits < 30 && !override) {
        flash(COLOR_BLUE);
        delay(2);
        recvOneChar();
        if (newCommand) {
            switch (receivedChar) {
                case 'Z':
                    rocket.bleOn = false;
                    override = true;
                    break;
                default:
                    override = false;
                    break;
            }
        }

        if (millis() % 1000 < 40) {
            rocket.printMessage("BLE not connected, waiting...");
            waits++;
            delay(50);
            showColor(COLOR_OFF);

        }
    }

    if (bleSerial) {
        rocket.bleOn = true;
        rocket.printMessage("HardwareBLESerial central device connected!");
        rocket.logMessage("HardwareBLESerial central device connected!");
    }
    else {
        rocket.bleOn = false;
        rocket.printMessage("BLE not connected! Using USB serial...");
        rocket.logMessage("WARN: BLE not connected! Using USB serial...");

    }
#endif

    rocket.initAngles();
    rocket.printMessage("Sensors and angles initialized!");

    // init TVC
    rocket.initTvc();
    rocket.printMessage("TVC initialized!");

    // init pyros and complete only after other inits succeed
    rocket.initPyros();
    rocket.printMessage("Pyros initialized!");

    rocket.initChutes();
    rocket.printMessage("Chutes initialized!");

    rocket.finishSetup();

    // IMPORTANT: You must call this for MBED timers to initialize
    if (ITimer.attachInterruptInterval(10000, loopHandler)) {
        rocket.printMessage("Starting ITimer OK, interval = 10ms");
    }
    else {
        rocket.printMessage("Can't set ITimer. Select another timer or interval");
        rocket.HALT_AND_CATCH_FIRE();
    }


    rocket.printMessage("Setup complete!");
    rocket.printMessage("Welcome to the Tax Collector Test Suite");
    rocket.printMessage("This script tests all components and features of the rocket,");
    rocket.printMessage("including sensors, LEDs, TVC, and pyro channels.");
    rocket.printMessage(HELP_STR);
}

void loop() {

    recvOneChar();

    if (newCommand == true) {
        switch (receivedChar) {
            case 'U':
                rocket.printMessage("Unlocking TVC");
                rocket.logMessage("Unlocking TVC");
                rocket.tvc.unlock();
                break;
            case 'L':
                rocket.printMessage("Locking TVC");
                rocket.logMessage("Locking TVC");
                rocket.tvc.lock();
                break;
            case 'K':
                rocket.printMessage("Toggling LEDs");
                rocket.toggleLeds();
                break;
            case 'C':
                rocket.calibrateAndLog();
                break;
            case 'O':
                rocket.printMessage("Orientation: ");
                sensorOutLock = true;
                dirOutLock = false;
                break;
            case 'A':
                rocket.updateAltVel();
                rocket.printMessage("Altitude: ", false);
                rocket.printMessage(rocket.getAlt());
                break;
            case 'R':
                rocket.printMessage("Sensor Readings: ");
                dirOutLock = true;
                sensorOutLock = false;
                break;
            case 'X':
                sensorOutLock = true;
                dirOutLock = true;
                break;
            case 'G':
                rocket.printMessage("Pyro 1: Motor Ignition");
                rocket.logMessage("Pyro 1: Motor Ignition");
                rocket.firePyro1();
                break;
            case 'T':
                rocket.printMessage("Firing parachute");
                rocket.logMessage("Firing parachute");
                rocket.parachute.deployTimer();
                break;
            case 'S':
                rocket.printMessage("Reading SD Card Info");
                rocket.getSdInfo();
                break;
            case 'Q':
                rocket.printMessage("Resetting Angles");
                rocket.resetAngles();
                break;
            case 'H':
                rocket.printMessage(HELP_STR);
                break;

            case 'D':
                rocket.toggleDataLog();
                break;
            case 'P':
                char buf[256];
                snprintf(buf, sizeof(buf),
                    "sensor=%lu  angles=%lu  tvc=%lu  total=%lu  sd=%lu  latency=%lu  worst=%lu (all us)",
                    perf.sensorUs, perf.anglesUs, perf.tvcUs,
                    perf.totalUs, perf.sdUs, perf.isr_to_run, perf.worstTotal);
                rocket.printMessage(buf);

                snprintf(buf, sizeof(buf), "Loop rate: %0.2f Hz", 1000000.0 / (perf.totalUs + perf.sdUs));
                rocket.printMessage(buf);

                rocket.printMessage("Pending data points: ", false);
                rocket.printMessage(rocket.getPending());

                break;

            case 'Y':
                rocket.printMessage("Running test routine");
                rocket.logMessage("Running test routine");
                rocket.tvc.testRoutine();
                rocket.printMessage("Test routine complete!");
                rocket.logMessage("Test routine complete!");
                break;

            case 'E':
                experimentMode = !experimentMode;
                if (experimentMode) {
                    rocket.setState(FS_TEST);
                    rocket.setLogSpeed(DLS_FAST);
                    rocket.printMessage("Experiment Mode ON");
                    rocket.logMessage("Experiment Mode ON - Running Test");
                    rocket.calibrateAndLog();
                    rocket.initAngles();
                    rocket.disableCompl();
                    rocket.printMessage("Unlocking TVC");
                    rocket.logMessage("Unlocking TVC");
                    rocket.tvc.unlock();

                    rocket.printMessage("Firing Engine");
                    rocket.logMessage("Firing Engine");
                    rocket.firePyro1();

                    rocket.printMessage("Releasing from stand");
                    rocket.logMessage("Releasing from stand");
                    rocket.firePyro2();
                    currentMs = millis();
                    rocket.setDataLog(true);
                }
                else {
                    rocket.setState(FS_ABORT);
                    rocket.printMessage("Experiment Mode ABORTED");
                    rocket.logMessage("Experiment Mode ABORTED");
                    rocket.printMessage("Locking TVC");
                    rocket.logMessage("Locking TVC");
                    rocket.tvc.lock();

                    rocket.printMessage("Disarming Pyros");
                    rocket.logMessage("Disarming Pyros");
                    rocket.pyro1_motor.disarm();
                    rocket.pyro2_land.disarm();
                }
                break;
            case 'B':
                if (rocket.useCompl) {
                    rocket.printMessage("Disabling Complementary Filter");
                    rocket.disableCompl();

                }
                else {
                    rocket.printMessage("Enabling Complementary Filter");
                    rocket.enableCompl();
                }
                break;
            default:
                rocket.printMessage("Invalid command");
                break;
        }
        newCommand = false;
    }

    if (((millis() - currentMs) > 7000) && (experimentMode == true)) {
        experimentMode = false;
        rocket.setState(FS_IDLE);
        rocket.setLogSpeed(DLS_SLOW);
        rocket.printMessage("Experiment Mode OFF - Test Complete");
        rocket.logMessage("Experiment Mode OFF - Test Complete");
        rocket.printMessage("Locking TVC");
        rocket.logMessage("Locking TVC");
        rocket.tvc.lock();

        rocket.printMessage("Disarming Pyros");
        rocket.logMessage("Disarming Pyros");
        rocket.pyro1_motor.disarm();
        rocket.pyro2_land.disarm();

        currentMs = millis();
        rocket.enableCompl();
    }

    if (!sensorOutLock) {
        SensorReadings readings = rocket.getReadings();
        rocket.printMessage(readings.ax, false);
        rocket.printMessage(" ", false);
        rocket.printMessage(readings.ay, false);
        rocket.printMessage(" ", false);
        rocket.printMessage(readings.az, false);

        rocket.printMessage(readings.gx, false);
        rocket.printMessage(" ", false);
        rocket.printMessage(readings.gy, false);
        rocket.printMessage(" ", false);
        rocket.printMessage(readings.gz);
    }

    if (!dirOutLock) {
        Vec3D dir = rocket.getDir();
        float roll = dir.x;
        float pitch = dir.y;
        float yaw = dir.z;
        rocket.printMessage(roll, false);
        rocket.printMessage(" ", false);
        rocket.printMessage(pitch, false);
        rocket.printMessage(" ", false);
        rocket.printMessage(yaw);
    }

#if USE_BLE
    if (rocket.bleOn) rocket.updateBle();
#endif

    if (!sdFree) {

        // constrain to 100hz
        rocket.updateTime();

        // Update spatial data
        rocket.updateAltVel();

        rocket.updateChutes();

        sdFree = true;

        unsigned long t0 = micros();
        perf.isr_to_run = t0 - isrFiredAt;  // ISR latency

        rocket.updateSensors();
        unsigned long t1 = micros();

        rocket.updateAngles();
        unsigned long t2 = micros();

        rocket.updateTvc();
        unsigned long t3 = micros();

        rocket.updatePyros();
        rocket.getNewData();
        unsigned long t4 = micros();

        perf.sensorUs = t1 - t0;
        perf.anglesUs = t2 - t1;
        perf.tvcUs = t3 - t2;
        perf.totalUs = t4 - t0;

        if (perf.totalUs > perf.worstTotal)  perf.worstTotal = perf.totalUs;
        if (perf.sensorUs > perf.worstSensor) perf.worstSensor = perf.sensorUs;
    }
    else {
        unsigned long tSD = micros();
        rocket.logNextData();
        perf.sdUs = micros() - tSD;
    }

}

