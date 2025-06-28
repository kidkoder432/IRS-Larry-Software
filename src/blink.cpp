#include <Arduino.h>

void setup() {
    // Initialize Serial communication at 115200 baud rate
    Serial.begin(115200);
    // while (!Serial) {
    //     ; // Wait for Serial to be ready
    // }
    
    // Print a welcome message
    Serial.println("Blink example started");

    pinMode(LED_BUILTIN, OUTPUT); // Set the built-in LED pin as output
    pinMode(8, OUTPUT); // Set pin 6 as output (if needed for other purposes)
    pinMode(5, OUTPUT); // Set pin 7 as output (if needed for other purposes)
}

void loop() {
    // Blink the built-in LED
    digitalWrite(LED_BUILTIN, HIGH); // Turn the LED on
    digitalWrite(8, HIGH); // Turn pin 6 on (if needed)
    digitalWrite(5, HIGH); // Turn pin 5 on (if needed)
    delay(1000); // Wait for 1 second
    digitalWrite(LED_BUILTIN, LOW); // Turn the LED off
    digitalWrite(8, LOW); // Turn pin 6 off (if needed)
    digitalWrite(5, LOW); // Turn pin 5 off (if needed)
    delay(1000); // Wait for 1 second
}