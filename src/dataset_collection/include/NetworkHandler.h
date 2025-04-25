#ifndef NETWORK_HANDLER_H
#define NETWORK_HANDLER_H

#include <HTTPClient.h>
#include <WiFi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

class NetworkHandler {
   public:
    NetworkHandler(SemaphoreHandle_t buttonSemaphore, SemaphoreHandle_t mutex);
    void connectWiFi();
    void startTask();
    static void taskWrapper(void* pvParameters);
    static void buttonISR();

   private:
    SemaphoreHandle_t buttonSemaphore;
    SemaphoreHandle_t mutex;
    TaskHandle_t taskHandle;
    unsigned long lastButtonTime;
    void task();
};

#endif