// This code has been refactored to be compatible with Arduino by removing
// dependencies on the C++ Standard Template Library (STL) like std::vector,
// std::string, and std::stringstream, which can be memory-intensive on
// microcontrollers.

#include <rocket.h>
#include <stdlib.h> // Required for atof()
#include <string.h> // Required for strtok(), strncmp(), strlen()

// Helper function, remains unchanged.
float isBetween(float x, float a, float b) { return a <= x && x <= b; }

// Global rocket object instance.
Rocket rocket;

// --- Globals for Serial Communication ---
// Using a fixed-size C-style character array instead of std::string to avoid dynamic
// memory allocation, which can lead to heap fragmentation on Arduino.
const unsigned int MAX_BUFFER_SIZE = 128;
char serialBuffer[MAX_BUFFER_SIZE];
byte bufferIndex = 0;

/**
 * @brief Reads a single line from the Serial port into a global buffer.
 * This function is non-blocking. It reads available characters until a newline
 * character is found, indicating a complete command.
 * * @return True if a complete line has been received, false otherwise.
 * The complete, null-terminated line is stored in the global `serialBuffer`.
 */
bool recvOneLine() {
    while (Serial.available()) {
        char c = Serial.read();
        // Serial.print(c);
        // Check for the newline character, which signifies the end of a command.
        if (c == '\n') {
            serialBuffer[bufferIndex] = '\0'; // Null-terminate the string.
            bufferIndex = 0; // Reset index for the next command.
            return true; // A complete line is ready.
        }
        // Ignore carriage return characters.
        else if (c != '\r') {
            // Add the character to the buffer if there's space.
            if (bufferIndex < MAX_BUFFER_SIZE - 1) {
                serialBuffer[bufferIndex++] = c;
            }
            // If the buffer is full, we discard further characters to prevent overflow.
            // The command will likely be invalid, but this prevents a crash.
        }
    }
    return false; // No complete line has been received yet.
}
/**
 * @brief Checks if a C-style string starts with a given prefix (case insensitive).
 * Replaces the functionality of std::string::startsWith.
 * @param base The string to check.
 * @param prefix The prefix to look for.
 * @return True if the string starts with the prefix, false otherwise.
 */
bool startsWith(const char* base, const char* prefix) {
    while (*prefix) {
        if (tolower(*base) != tolower(*prefix)) {
            return false;
        }
        base++;
        prefix++;
    }
    return true;
}

void printTvc() {
    Vec2D tvcOut = rocket.tvc.getAngle();
    rocket.printMessage("TVC X: ", false);
    rocket.printMessage(tvcOut.x, false);
    rocket.printMessage(" Y: ", false);
    rocket.printMessage(tvcOut.y);

    Vec2D tvcRead = rocket.tvc.read();
    rocket.printMessage("TVC Read X: ", false);
    rocket.printMessage(tvcRead.x, false);
    rocket.printMessage(" Y: ", false);
    rocket.printMessage(tvcRead.y);
}

// This welcome message remains unchanged.
char HELP_STR[] =
R"(Welcome!
This is the SOFTWARE-IN-THE-LOOP program for the TAX COLLECTOR
Use this software to test the TVC's functionality using SITL.
)";

void setup() {
    // Setup sequence is logical and remains unchanged.
    rocket.initSerial();
    delay(2000);

    rocket.initLeds();
    rocket.printMessage("LEDs initialized!");

    rocket.initSD();
    rocket.printMessage("SD card initialized!");
    rocket.getSdInfo();
    rocket.initLogs();
    rocket.printMessage("Data logging initialized!");

    rocket.initConfig();
    rocket.printMessage("Config initialized!");

    rocket.initTvc();
    rocket.printMessage("TVC initialized!");

    rocket.finishSetup();

    rocket.printMessage("Setup complete!");
    rocket.printMessage(HELP_STR);

    delay(1000);
    rocket.tvc.reset(); // Reset TVC to default angles.
}

void loop() {
    // Constrain the loop to a consistent frequency.
    rocket.updateTime();

    // Check if a new command has been received over Serial.
    if (recvOneLine()) {
        // If the command is an SITL command...
        if (startsWith(serialBuffer, "SITL - ")) {
            // Serial.print("Received SITL command: ");
            // Serial.println(serialBuffer);

            // --- C-style String Parsing ---
            // The command format is "SITL - <angle> - <x>"
            // We use strtok() to parse the string in-place. strtok() modifies the
            // original string by replacing delimiters with null terminators ('\0').

            // Get a pointer to the start of the data, after "SITL - ".
            char* data = serialBuffer + 7;

            // Get the first token (the angle value). The delimiter is " - ".
            char* angleStr = strtok(data, " - ");
            // Subsequent calls to strtok() with NULL as the first argument
            // continue parsing the same string. This gets the second token (the x value).
            char* xStr = strtok(NULL, " - ");

            // Convert the string tokens to floats using atof().
            // If a token is NULL (e.g., the command was incomplete), atof() on a NULL
            // pointer is undefined behavior, so we check first. atof returns 0.0 if the
            // string is not a valid number.
            float angle = angleStr ? atof(angleStr) : 0.0f;
            float x = xStr ? atof(xStr) : 0.0f;

            // Update the rocket simulation with the new values.
            rocket.updateAngles_SITL(angle);
            rocket.updateTvc_SITL(x);

        }
        else if (startsWith(serialBuffer, "RST")) {
            rocket.tvc.reset();
        }
        else if (startsWith(serialBuffer, "TEST")) {
            rocket.tvc.testRoutine();
        }
        else if (startsWith(serialBuffer, "TVC")) {
            printTvc();
        }
        else {
            rocket.printMessage("Unknown command: ");
            rocket.printMessage(serialBuffer);
        }
        serialBuffer[0] = '\0'; // Clear the buffer for the next command.
        bufferIndex = 0; // Reset the index for the next command.
    }
    
}
