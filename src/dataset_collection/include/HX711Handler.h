#ifndef HX711_HANDLER_H
#define HX711_HANDLER_H

#include <HX711_ADC.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

class HX711Handler {
   public:
    HX711Handler(int dataPin, int clockPin, SemaphoreHandle_t dataSemaphore, SemaphoreHandle_t mutex);
    void begin();
    void startTask();
    static void taskWrapper(void* pvParameters);
    static void dataReadyISR();

   private:
    HX711_ADC loadCell;
    SemaphoreHandle_t dataSemaphore;
    SemaphoreHandle_t mutex;
    TaskHandle_t taskHandle;
    void task();
};

#endif