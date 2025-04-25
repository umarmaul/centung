#ifndef UTILITIES_H
#define UTILITIES_H

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

extern volatile float weight;
extern volatile float smoothedRoll;
extern volatile float smoothedPitch;
extern SemaphoreHandle_t sensorMutex;

void buzzerFeedback(bool success);

#endif