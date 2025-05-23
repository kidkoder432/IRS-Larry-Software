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

// typedef std::unordered_map<const char*, float, cstr_hash, cstr_equal> Config;
typedef std::unordered_map<std::string, float> Config;

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
    // PID Gains
    {"Kp", 4.09469141},
    {"Ki", 0.000847052453},
    {"Kd", 0.484600093},
    {"N", 70.0},

    // Altitude Pressure Reference (kPa)
    {"PRESSURE_REF", 100.816},

    // TVC Limits/Centers (degrees)
    {"XMIN", 76.0},
    {"XDEF", 100.0},
    {"XMAX", 124.0},
    {"YMIN", 73.0},
    {"YDEF", 97.0},
    {"YMAX", 121.0},

    // TVC Output Flip
    {"FLIP_X", false},
    {"FLIP_Y", false},

    // TVC Roll compensation
    {"ROLL_COMP", true},

    // Angle Output Flip
    {"FLIP_DIR_X", false},
    {"FLIP_DIR_Y", false},
    {"FLIP_DIR_Z", false},

    // Calibration Routines
    {"DO_CRT", true},
    {"DO_ACCEL_FOC", true},
    {"DO_GYRO_FOC", false},

    // Use accelerometer for initial angle estimates
    {"INIT_ACCEL", true},

    // Datalog buffer
    {"DATA_LOG_BATCH", true},

    // Complementary filter parameter
    {"COMP_FILTER_ALPHA_GYRO", 0.95},

    // Altitude filter parameter
    {"ALT_FILTER_ALPHA_ACCEL", 0.95},

    // Pyro channel parameters
    {"PYRO_1_ONE_SHOT", false},
    {"PYRO_2_ONE_SHOT", false},
    {"PYRO_1_FIRE_TIME", 2500.0},
    {"PYRO_2_FIRE_TIME", 2500.0},
    {"PARACHUTE_FIRE_TIME", 2500.0},
    {"PARACHUTE_ONE_SHOT", false},
    {"PARACHUTE_BURNOUT_DELAY", 3000.0},
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

        std::string name = std::string(cfg.getName());
        const char* value = cfg.getValue();

        if (strcmp(value, "true") == 0) {
            config[name] = 1.0;

        }
        else if (strcmp(value, "false") == 0) {
            config[name] = 0.0;
        }
        else {
            Serial.println(name.c_str());
            config[name] = atof(value);
        }



    }
    // clean up
    cfg.end();

    return config;

}

void printConfig(Config& config) {
    for (auto& entry : config) {
        Serial.print(entry.first.c_str());
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

