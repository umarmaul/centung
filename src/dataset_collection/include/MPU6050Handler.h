#ifndef MPU6050_HANDLER_H
#define MPU6050_HANDLER_H

#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

class MPU6050Handler {
   public:
    MPU6050Handler(SemaphoreHandle_t mutex);
    void begin();
    void calibrate();
    void startTask();
    static void taskWrapper(void* pvParameters);

   private:
    SemaphoreHandle_t mutex;
    TaskHandle_t taskHandle;
    float gyroBiasRoll, gyroBiasPitch, gyroBiasYaw;
    float rollWindow[10];  // SMOOTHING_WINDOW_SIZE
    float pitchWindow[10];
    int smoothingIndex;
    void task();
    void readSensor(float& rateRoll, float& ratePitch, float& rateYaw, float& angleRoll, float& anglePitch);
    void kalman1D(float& state, float& uncertainty, float rate, float measurement);
    void smoothAngles(float kalmanRoll, float kalmanPitch);
};

#endif