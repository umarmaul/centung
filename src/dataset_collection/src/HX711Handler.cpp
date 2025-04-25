#include "include/HX711Handler.h"
#include <Arduino.h>
#include "include/Config.h"
#include "include/Utilities.h"

HX711Handler::HX711Handler(int dataPin, int clockPin, SemaphoreHandle_t dataSemaphore, SemaphoreHandle_t mutex)
    : loadCell(dataPin, clockPin), dataSemaphore(dataSemaphore), mutex(mutex), taskHandle(NULL) {}

void HX711Handler::begin() {
    loadCell.begin();
    Serial.println("Starting HX711 Tare...");
    loadCell.start(HX711_STABILIZING_TIME, true);
    if (loadCell.getTareTimeoutFlag() || loadCell.getSignalTimeoutFlag()) {
        Serial.println("HX711 Timeout! Check wiring.");
        while (1);
    }
    loadCell.setCalFactor(CALIBRATION_VALUE);
    Serial.println("HX711 Tare complete.");
}

void HX711Handler::startTask() {
    xTaskCreatePinnedToCore(taskWrapper, "HX711 Task", 4096, this, 3, &taskHandle, 1);
}

void HX711Handler::taskWrapper(void* pvParameters) {
    static_cast<HX711Handler*>(pvParameters)->task();
}

void HX711Handler::task() {
    Serial.println("HX711 Task started.");
    float currentWeight;
    while (true) {
        if (xSemaphoreTake(dataSemaphore, portMAX_DELAY) == pdTRUE) {
            if (loadCell.update()) {
                currentWeight = loadCell.getData();
                currentWeight = round(currentWeight * 100) / 100.0;
                if (currentWeight < 0) currentWeight = 0;
                if (xSemaphoreTake(mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    weight = currentWeight;
                    xSemaphoreGive(mutex);
                } else {
                    Serial.println("HX711 Task: Failed to get mutex!");
                }
            }
        }
    }
}

void HX711Handler::dataReadyISR() {
    BaseType_t higherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(sensorMutex, &higherPriorityTaskWoken);
    if (higherPriorityTaskWoken) portYIELD_FROM_ISR();
}