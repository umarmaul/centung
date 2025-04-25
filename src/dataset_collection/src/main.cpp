#include <Arduino.h>

#include "include/Config.h"
#include "include/HX711Handler.h"
#include "include/MPU6050Handler.h"
#include "include/NetworkHandler.h"
#include "include/Utilities.h"

// Shared data
volatile float weight = 0.0;
volatile float smoothedRoll = 0.0;
volatile float smoothedPitch = 0.0;
SemaphoreHandle_t sensorMutex = NULL;

void setup() {
    Serial.begin(115200);
    Serial.println("Starting ESP32 Centung Project...");

    // Initialize pins
    pinMode(PIN_BUTTON, INPUT_PULLUP);
    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, LOW);
    digitalWrite(PIN_BUZZER, LOW);

    // Initialize I2C
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(400000);

    // Create RTOS objects
    sensorMutex = xSemaphoreCreateMutex();
    SemaphoreHandle_t hx711Semaphore = xSemaphoreCreateBinary();
    SemaphoreHandle_t buttonSemaphore = xSemaphoreCreateBinary();

    if (!sensorMutex || !hx711Semaphore || !buttonSemaphore) {
        Serial.println("Failed to create RTOS objects!");
        while (1);
    }

    // Initialize handlers
    HX711Handler hx711(PIN_HX711_DATA, PIN_HX711_CLOCK, hx711Semaphore, sensorMutex);
    MPU6050Handler mpu(sensorMutex);
    NetworkHandler network(buttonSemaphore, sensorMutex);

    // Start components
    hx711.begin();
    mpu.begin();
    mpu.calibrate();
    network.connectWiFi();

    // Attach interrupts
    attachInterrupt(digitalPinToInterrupt(PIN_HX711_DATA), HX711Handler::dataReadyISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(PIN_BUTTON), NetworkHandler::buttonISR, FALLING);

    // Start tasks
    hx711.startTask();
    mpu.startTask();
    network.startTask();

    Serial.println("Setup complete.");
    digitalWrite(PIN_LED, HIGH);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}