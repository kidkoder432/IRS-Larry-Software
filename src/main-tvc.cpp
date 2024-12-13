// Test all components and features

#include <rocket.h>

#define SD_ATTACHED 1
#define LOG_DATA_BATCH 1

Rocket rocket;

char receivedChar;
bool newCommand = false;

HardwareBLESerial& bleSerial = rocket.getBle();

void recvOneChar() {
    if (Serial.available() > 0) {
        receivedChar = Serial.read();
        receivedChar = toupper(receivedChar);
        // rocket.printMessage(receivedChar);
        newCommand = true;
    }
    if (bleSerial.available() > 0) {
        receivedChar = bleSerial.read();
        receivedChar = toupper(receivedChar);
        // rocket.printMessage(receivedChar);
        newCommand = true;
    }
    if (receivedChar == '\n' || receivedChar == '\r') {
        newCommand = false;
    }
}

bool isLaunched = false;

char HELP_STR[] =
R"(Welcome!
This is the LAUNCH CONTROL SOFTWARE for the TAX COLLECTOR"
Use this software to control launch, run specific routines, or abort the launch.
Commands:
L: Launch
A: Abort
C: Calibrate Sensors
)";

void setup() {

    // Setup basic interfaces
    rocket.initSerial();
    delay(2000);


#if SD_ATTACHED
    // Setup SD card, config and data logging
    rocket.initSD();
    rocket.printMessage("SD card initialized!");
    rocket.getSdInfo();
    rocket.initLogs();
    rocket.printMessage("Data logging initialized!");
    rocket.initConfig();
    rocket.printMessage("Config initialized!");

    rocket.initBle();
    int waits = 0;
    bool override = false;
    while (!bleSerial && waits < 30 && !override) {
        flash(COLOR_BLUE);
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

    // Init sensors and angles
    rocket.setupSensors();
    rocket.calibrateAndLog();
    rocket.initAngles();
    rocket.printMessage("Sensors and angles initialized!");

    // init hardware
    rocket.initLeds();
    rocket.printMessage("LEDs initialized!");
    rocket.initTvc();
    rocket.printMessage("TVC initialized!");

    // init pyros and complete only after other inits succeed
    rocket.initPyros();
    rocket.printMessage("Pyros initialized!");
    rocket.finishSetup();

    rocket.printMessage("Setup complete!");
    rocket.printMessage(HELP_STR);
}

void loop() {

    if (rocket.bleOn) rocket.updateBle();

    rocket.updateTvc();
    rocket.updateSensors();
    rocket.updateAngles();
    rocket.updateAltVel();
    rocket.updateState();
    rocket.updatePyros();
    rocket.updateDataLog();

    if (!isLaunched) {
        recvOneChar();
        rocket.updateAngleLeds();

        if (newCommand == true) {
            switch (receivedChar) {
                case 'L': {

                    rocket.printMessage("Launch sequence initiated!");
                    rocket.logMessage("Launch sequence initiated!");
                    rocket.printMessage("Performing pre-launch tasks.");
                    rocket.logMessage("Performing pre-launch tasks.");
                    rocket.printMessage("Calibrating Sensors");
                    rocket.calibrateAndLog();

                    int launchSecs = 21;
                    newCommand = false;
                    while (launchSecs > 0) {

                        if (launchSecs > 12) flash(COLOR_YELLOW, 1000);
                        else if (launchSecs > 5) flash(COLOR_RED, 1000);
                        else flash(COLOR_RED, 500);
                        recvOneChar();
                        if (newCommand) {
                            rocket.printMessage("Launch aborted!");
                            rocket.logMessage("Launch aborted!");
                            newCommand = false;

                            rocket.finish();

                        }

                        if (millis() % 1000 < 40) {
                            rocket.printMessage("Launching in T - ", false);
                            rocket.printMessage((int64_t) launchSecs, false);
                            rocket.printMessage(" seconds! Press any key + ENTER to abort the launch.");                        launchSecs--;
                            delay(41);

                        }
                    }

                    rocket.printMessage("Launching!");
                    rocket.logMessage("Launching!");
                    rocket.disableCompl();

                    rocket.printMessage("Firing Engine");
                    rocket.logMessage("Firing Engine");
                    rocket.firePyro1();

                    rocket.setDataLog(true);

                    rocket.printMessage("Ignition Complete");
                    rocket.logMessage("Ignition Complete");

                    isLaunched = true;
                    break;
                } case 'A':
                    rocket.printMessage("Abort sequence initiated!");
                    rocket.logMessage("Abort sequence initiated!");
                    rocket.abort();
                    break;
                case 'C':
                    rocket.printMessage("Calibrating Sensors");
                    rocket.calibrateAndLog();
                    break;
            }
        }
    }

    else {
        rocket.updateStateLeds();
        switch (rocket.getCurrentState()) {
            case 0:     // Pad-Idle
                rocket.tvc.lock();
                break;
            case 1:     // Powered Ascent
                rocket.tvc.unlock();
                break;
            case 2:     // Coast
                rocket.tvc.lock();
                break;
            case 3:     // Powered Descent
                rocket.tvc.unlock();
                break;
            case 4:     // Touchdown
                rocket.logMessage("Touchdown confirmed. We are safe on Earth!");
                rocket.finish();
                break;
            case 127:   // Abort
                rocket.tvc.lock();
                rocket.logMessage("Mission abort!");
                rocket.abort();
                break;

        }

    }
}