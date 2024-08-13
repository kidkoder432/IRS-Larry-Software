// Read config file from SD card

#include <Arduino.h>
#include <SdConfigFile.h>

const int chipSelect = 10; // Adjust pin according to your SD card module

struct Config {

    float PRESSURE_0;
    float Kp, Ki, Kd;
    bool FILTER_KALMAN;

};

Config readConfig() {
    Config config;

    SdConfigFile configFile(10);

    while (configFile.read("config.cfg")) {
        configFile.get("PRESSURE_0", config.PRESSURE_0);
        configFile.get("Kp", config.Kp);
        configFile.get("Ki", config.Ki);
        configFile.get("Kd", config.Kd);
        configFile.get("FILTER_KALMAN", config.FILTER_KALMAN);
    }

    return config;

}