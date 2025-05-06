// Read config file from SD card

#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <SdFat.h>
#include <SDConfig.h>
#include <stdlib.h>
#include <unordered_map>

const int chipSelect = 10; // Adjust pin according to your SD card module

struct cstr_hash {
    size_t operator()(const char* s) const {
        size_t h = 0;
        while (*s) h = (h * 131) + *s++;
        return h;
    }
};

struct cstr_equal {
    bool operator()(const char* a, const char* b) const {
        return strcmp(a, b) == 0;
    }
};

typedef std::unordered_map<const char*, float, cstr_hash, cstr_equal> Config;

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

Config defaultConfig = {
    {"PRESSURE_REF", 100.816},
    {"Kp", 1.0},
    {"Ki", 0.4},
    {"Kd", 0.1},
    {"N", 50},
    {"FILTER_KALMAN", true}
};

Config readConfig() {
    Config config;

    int maxLineLength = 127;
    SDConfig cfg;
    // Open the configuration file.
    if (!cfg.begin("config.cfg", maxLineLength)) {
        Serial.println("Failed to open configuration file!");
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
            Serial.println(name);
            config[name] = atof(value);
        }



    }
    // clean up
    cfg.end();

    return config;

}

void printConfig(Config& config) {
    for (auto& entry : config) {
        Serial.print(entry.first);
        Serial.print(": ");
        Serial.println(entry.second);
    }
}

// bool saveConfig(SdExFat& sd, Config& config) {
//     ExFile newConfig;
//     if (!newConfig.open("new_config.cfg", O_CREAT | O_WRITE | O_TRUNC)) {
//         Serial.println("Failed to open new config file");
//         return false;
//     }

//     ExFile oldConfig;
//     oldConfig.open("config.cfg", O_READ);
//     while (oldConfig.available()) {
//         if (oldConfig.peek() == '#') {
//             String line = oldConfig.readStringUntil('\n');
//             newConfig.println(line);
//         }
//         else if (oldConfig.readStringUntil('\n') == "") {
//             newConfig.println();
//         }
//         else {
//             String key = oldConfig.readStringUntil('=');
//             key.trim();
//             newConfig.print(key);
//             newConfig.print(" = ");
//             newConfig.println(config[std::string(key.c_str())]);
//             oldConfig.readStringUntil('\n');
//         }
//     }

//     newConfig.close();
//     oldConfig.close();

//     // sd.rename("config.cfg", "old_config.cfg");
//     sd.remove("config.cfg");
//     sd.rename("new_config.cfg", "config.cfg");


//     return true;
// }

#endif

