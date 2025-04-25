#include "include/NetworkHandler.h"
#include <Arduino.h>
#include "include/Config.h"
#include "include/Utilities.h"

NetworkHandler::NetworkHandler(SemaphoreHandle_t buttonSemaphore, SemaphoreHandle_t mutex)
    : buttonSemaphore(buttonSemaphore), mutex(mutex), taskHandle(NULL), lastButtonTime(0) {}

void NetworkHandler::connectWiFi() {
    Serial.print("Connecting to WiFi: ");
    WiFi.begin(SSID, PASSWORD);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nConnected to WiFi");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\nFailed to connect to WiFi.");
    }
}

void NetworkHandler::startTask() {
    xTaskCreatePinnedToCore(taskWrapper, "HTTP Task", 8192, this, 1, &taskHandle, 0);
}

void NetworkHandler::taskWrapper(void* pvParameters) {
    static_cast<NetworkHandler*>(pvParameters)->task();
}

void NetworkHandler::task() {
    Serial.println("HTTP Task started.");
    float localWeight, localRoll, localPitch;
    while (true) {
        if (xSemaphoreTake(buttonSemaphore, portMAX_DELAY) == pdTRUE) {
            Serial.println("Button pressed!");
            digitalWrite(PIN_BUZZER, HIGH);
            vTaskDelay(pdMS_TO_TICKS(200));
            digitalWrite(PIN_BUZZER, LOW);
            if (xSemaphoreTake(mutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                localWeight = weight;
                localRoll = smoothedRoll;
                localPitch = smoothedPitch;
                xSemaphoreGive(mutex);
            } else {
                Serial.println("HTTP Task: Failed to get mutex!");
                continue;
            }
            Serial.printf("Data: Weight: %.2f, Roll: %.2f, Pitch: %.2f\n", localWeight, localRoll, localPitch);
            if (WiFi.status() == WL_CONNECTED) {
                HTTPClient http;
                bool success = false;
                if (http.begin(SERVER_URL)) {
                    http.addHeader("Content-Type", "application/json");
                    http.addHeader("x-device-name", DEVICE_NAME);
                    http.addHeader("x-device-password", DEVICE_PASSWORD);
                    String payload = "{\"weight\":" + String(localWeight, 2) + "," + "\"roll\":" + String(localRoll, 2) + "," + "\"pitch\":" + String(localPitch, 2) + "," + "\"profile_code\":\"" + String(PROFILE_CODE) + "\"}";
                    Serial.println("Sending: " + payload);
                    int code = http.POST(payload);
                    if (code > 0) {
                        Serial.println("HTTP Code: " + String(code));
                        String response = http.getString();
                        Serial.println("Response: " + response);
                        if (code == 200 || code == 201) success = true;
                    } else {
                        Serial.println("HTTP Error: " + http.errorToString(code));
                    }
                    http.end();
                } else {
                    Serial.println("HTTP begin failed!");
                }
                buzzerFeedback(success);
            } else {
                Serial.println("WiFi disconnected.");
                buzzerFeedback(false);
            }
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}

void NetworkHandler::buttonISR() {
    static unsigned long lastInterruptTime = 0;
    unsigned long currentTime = millis();
    if (currentTime - lastInterruptTime > BUTTON_DEBOUNCE_DELAY) {
        lastInterruptTime = currentTime;
        BaseType_t higherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(sensorMutex, &higherPriorityTaskWoken);
        if (higherPriorityTaskWoken) portYIELD_FROM_ISR();
    }
}