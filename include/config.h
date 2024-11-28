// Read config file from SD card

#include <Arduino.h>
#include <SdFat.h>
#include <SDConfig.h>
#include <stdlib.h>
#include <unordered_map>

#ifndef CONFIG_H
#define CONFIG_H

const int chipSelect = 10; // Adjust pin according to your SD card module

typedef std::unordered_map<const char*, double> Config2;

void toLowercase(char* str) {
    for (int i = 0; str[i] != '\0'; i++) {
        str[i] = tolower(str[i]);
    }
}

void toUppercase(char* str) {
    for (int i = 0; str[i] != '\0'; i++) {
        str[i] = toupper(str[i]);
    }
}

String strip(const String& str) {
    int startIndex = 0;
    int endIndex = str.length() - 1;

    // Find the first non-whitespace character from the left
    while (startIndex <= endIndex && isspace(str[startIndex])) {
        startIndex++;
    }

    // Find the first non-whitespace character from the right
    while (endIndex >= startIndex && isspace(str[endIndex])) {
        endIndex--;
    }

    // Return the stripped string
    return str.substring(startIndex, endIndex + 1);
}

Config2 defaultConfig = {
    {"PRESSURE_0", 100.816},
    {"Kp", 1.0},
    {"Ki", 0.4},
    {"Kd", 0.1},
    {"N", 50},
    {"FILTER_KALMAN", true}
};

struct Config {

    double PRESSURE_0 = 101.325;
    double Kp = 0.0, Ki = 0.0, Kd = 0.0, N = 0.0;
    int FILTER_KALMAN = false;

};

Config2 readConfig() {
    Config2 config;

    int maxLineLength = 127;
    SDConfig cfg;
    // Open the configuration file.
    if (!cfg.begin("config.cfg", maxLineLength)) {
        Serial.println("Failed to open configuration file: ");
        return defaultConfig;
    }
    // Read each setting from the file.
    while (cfg.readNextSetting()) {

        const char* name = cfg.getName();
        const char* value = cfg.getValue();

        if (strcmp(value, "true") == 0) {
            config[name] = 1.0;

        }
        else if (strcmp(value, "false") == 0) {
            config[name] = 0.0;
        }
        else {
            config[name] = atof(value);
        }


        
    }
    // clean up
    cfg.end();

    return config;

}

struct Config {

    double PRESSURE_0 = 101.325;
    double Kp = 0.0, Ki = 0.0, Kd = 0.0, N = 0.0;
    int FILTER_KALMAN = false;

};

Config readConfig_old() {
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

Config2 readConfig_TEST() {
    Config2 config;

    ExFile configFile;
    if (!configFile.open("config.cfg")) {
        Serial.println("Failed to open config file");
        return defaultConfig;
    }

    while (configFile.available()) {

        if (configFile.peek() == '#') {
            Serial.print("Comment. Next char is: ");
            configFile.readStringUntil('\n');
            Serial.println((char) configFile.peek());
            continue;
        }
        else {

            char key[32];
            strip(configFile.readStringUntil('=')).toCharArray(key, 32);
            char value[32];
            strip(configFile.readStringUntil('\n')).toCharArray(value, 32);

            Serial.println(key);

            toLowercase(value);


            if (strcmp(value, "true") == 0) {
                config[key] = true;
            }
            else if (strcmp(value, "false") == 0) {
                config[key] = false;
            }
            else {
                config[key] = atof(value);
            }
        }
    }

    Serial.println("Printing config:");
    for (auto& elem : defaultConfig) {
        Serial.print(elem.first);
        Serial.print("=");
        Serial.println(elem.second);
    }

    configFile.close();
    return config;

}

#endif

