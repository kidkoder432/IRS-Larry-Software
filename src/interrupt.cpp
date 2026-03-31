#include <NRF52_MBED_TimerInterrupt.h> // Use .h extension
#include <Arduino.h>

// NRF_TIMER_3 is usually the safest choice on Nano 33 BLE
NRF52_MBED_Timer ITimer(NRF_TIMER_3);

volatile long counter = 0;

void FastLoopHandler() {
    counter++;
}

void doStuff() {
    digitalWrite(LED_BUILTIN, 1);
    long start = millis();
    while (millis() - start < 100);
    digitalWrite(LED_BUILTIN, 0);
}

void printPriorities() {
    // Check your flight control timer (you're using TIMER3)
    int timerPriority = NVIC_GetPriority(TIMER3_IRQn);

    // Check the I2C (Wire) and SPI (SD Card) priorities
    // These often share the same peripheral ID on the nRF52840
    int i2cPriority = NVIC_GetPriority(SPIM1_SPIS1_TWIM1_TWIS1_SPI1_TWI1_IRQn);
    int spiPriority = NVIC_GetPriority(SPIM2_SPIS2_SPI2_IRQn);

    Serial.println("--- Interrupt Priorities (0 is highest) ---");
    Serial.print("Flight Timer (TIMER3): "); Serial.println(timerPriority);
    Serial.print("I2C Sensors (TWI):     "); Serial.println(i2cPriority);
    Serial.print("SD Card (SPI):         "); Serial.println(spiPriority);

    Serial.print("TWI1 (I2C):           "); Serial.println(NVIC_GetPriority(SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0_IRQn));
    Serial.print("SPI3 (SD Card):       "); Serial.println(NVIC_GetPriority(SPIM3_IRQn));
}

void setup() {
    Serial.begin(115200);
    while (!Serial) {};
    delay(3000);

    pinMode(LED_BUILTIN, OUTPUT);
    printPriorities();


    // IMPORTANT: You must call this for MBED timers to initialize
    if (ITimer.attachInterruptInterval(10000, FastLoopHandler)) {
        Serial.println("Starting ITimer OK, interval = 10ms");
    }
    else {
        Serial.println("Can't set ITimer. Select another timer or interval");
    }

}

void loop() {
    // Print every 1 second to avoid crashing the Serial buffer
    static unsigned long lastPrint = millis();
    if (millis() - lastPrint > 1000) {

        // FIX: Cast to float to avoid the "Zero" result
        float rate = ((float)counter / (float)millis()) * 1000.0f;

        Serial.print("Counter: ");
        Serial.print(counter);
        Serial.print(" | Freq: ");
        Serial.print(rate);
        Serial.println(" Hz");

        lastPrint = millis();
    }

    if (millis() % 1000 < 10) {
        doStuff();
    }
}