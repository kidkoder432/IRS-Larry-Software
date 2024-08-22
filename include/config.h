// Read config file from SD card

#include <Arduino.h>
#include <SDConfig.h>
#include <stdlib.h>

const int chipSelect = 10; // Adjust pin according to your SD card module

struct Config {

    float PRESSURE_0 = 101.325;
    float Kp = 0.0, Ki = 0.0, Kd = 0.0, N = 0.0;
    int FILTER_KALMAN = false;

};

Config readConfig() {
    Config config;

    int maxLineLength = 127;
    SDConfig cfg;
    // Open the configuration file.
    if (!cfg.begin("config.cfg", maxLineLength)) {
        Serial.println("Failed to open configuration file: ");
        return config;
    }
    // Read each setting from the file.
    while (cfg.readNextSetting()) {
        // Put a nameIs() block here for each setting you have.

        if (cfg.nameIs("PRESSURE_0")) {
            config.PRESSURE_0 = atof(cfg.getValue());
        }
        else if (cfg.nameIs("Kp")) {
            config.Kp = atof(cfg.getValue());
        }
        else if (cfg.nameIs("Ki")) {
            config.Ki = atof(cfg.getValue());
        }
        else if (cfg.nameIs("Kd")) {
            config.Kd = atof(cfg.getValue());;
        }
        else if (cfg.nameIs("N")) {
            config.N = atof(cfg.getValue());
        }
        else if (cfg.nameIs("FILTER_KALMAN")) {
            config.FILTER_KALMAN = cfg.getIntValue();
        }
    }
    // clean up
    cfg.end();

    return config;

}