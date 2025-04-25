#include "include/MPU6050Handler.h"
#include <Arduino.h>

#include "include/Config.h"
#include "include/Utilities.h"

MPU6050Handler::MPU6050Handler(SemaphoreHandle_t mutex)
    : mutex(mutex), taskHandle(NULL), gyroBiasRoll(0), gyroBiasPitch(0), gyroBiasYaw(0), smoothingIndex(0) {
    for (int i = 0; i < SMOOTHING_WINDOW_SIZE; i++) {
        rollWindow[i] = 0;
        pitchWindow[i] = 0;
    }
}

void MPU6050Handler::begin() {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission(true);
    delay(100);
    Serial.println("MPU6050 Initialized.");
}

void MPU6050Handler::calibrate() {
    Serial.println("Calibrating MPU6050...");
    digitalWrite(PIN_LED, LOW);
    float rollSum = 0, pitchSum = 0, yawSum = 0;
    float dummyAngleRoll, dummyAnglePitch;
    for (int i = 0; i < MPU_CALIBRATION_COUNT; i++) {
        float r, p, y;
        readSensor(r, p, y, dummyAngleRoll, dummyAnglePitch);
        rollSum += r;
        pitchSum += p;
        yawSum += y;
        delay(3);
    }
    gyroBiasRoll = rollSum / MPU_CALIBRATION_COUNT;
    gyroBiasPitch = pitchSum / MPU_CALIBRATION_COUNT;
    gyroBiasYaw = yawSum / MPU_CALIBRATION_COUNT;
    Serial.println("Calibration Complete.");
    digitalWrite(PIN_LED, HIGH);
}

void MPU6050Handler::startTask() {
    xTaskCreatePinnedToCore(taskWrapper, "MPU Task", 4096, this, 2, &taskHandle, 1);
}

void MPU6050Handler::taskWrapper(void* pvParameters) {
    static_cast<MPU6050Handler*>(pvParameters)->task();
}

void MPU6050Handler::task() {
    Serial.println("MPU Task started.");
    float rateRoll, ratePitch, rateYaw, angleRoll, anglePitch;
    float kalmanRoll = 0, kalmanPitch = 0;
    float uncertaintyRoll = KALMAN_UNCERTAINTY_ROLL;
    float uncertaintyPitch = KALMAN_UNCERTAINTY_PITCH;
    readSensor(rateRoll, ratePitch, rateYaw, angleRoll, anglePitch);
    kalmanRoll = angleRoll;
    kalmanPitch = anglePitch;
    for (int i = 0; i < SMOOTHING_WINDOW_SIZE; i++) {
        rollWindow[i] = kalmanRoll;
        pitchWindow[i] = kalmanPitch;
    }
    TickType_t lastWakeTime = xTaskGetTickCount();
    const TickType_t frequency = pdMS_TO_TICKS(4);
    while (true) {
        readSensor(rateRoll, ratePitch, rateYaw, angleRoll, anglePitch);
        rateRoll -= gyroBiasRoll;
        ratePitch -= gyroBiasPitch;
        kalman1D(kalmanRoll, uncertaintyRoll, rateRoll, angleRoll);
        kalman1D(kalmanPitch, uncertaintyPitch, ratePitch, anglePitch);
        smoothAngles(kalmanRoll, kalmanPitch);
        vTaskDelayUntil(&lastWakeTime, frequency);
    }
}

void MPU6050Handler::readSensor(float& rateRoll, float& ratePitch, float& rateYaw, float& angleRoll, float& anglePitch) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, (uint8_t)14, (uint8_t)true);
    int16_t accX = Wire.read() << 8 | Wire.read();
    int16_t accY = Wire.read() << 8 | Wire.read();
    int16_t accZ = Wire.read() << 8 | Wire.read();
    Wire.read() << 8 | Wire.read();  // Skip temperature
    int16_t gyroX = Wire.read() << 8 | Wire.read();
    int16_t gyroY = Wire.read() << 8 | Wire.read();
    int16_t gyroZ = Wire.read() << 8 | Wire.read();
    float accX_g = (float)accX / MPU_ACCEL_SCALE;
    float accY_g = (float)accY / MPU_ACCEL_SCALE;
    float accZ_g = (float)accZ / MPU_ACCEL_SCALE;
    rateRoll = (float)gyroX / MPU_GYRO_SCALE;
    ratePitch = (float)gyroY / MPU_GYRO_SCALE;
    rateYaw = (float)gyroZ / MPU_GYRO_SCALE;
    angleRoll = atan2(accY_g, sqrt(accX_g * accX_g + accZ_g * accZ_g)) * (180.0 / PI);
    anglePitch = -atan2(accX_g, sqrt(accY_g * accY_g + accZ_g * accZ_g)) * (180.0 / PI);
}

void MPU6050Handler::kalman1D(float& state, float& uncertainty, float rate, float measurement) {
    float dt = 0.004;
    state += dt * rate;
    uncertainty += dt * dt * 4 * 4;
    float R = 3 * 3;
    float gain = uncertainty / (uncertainty + R);
    state += gain * (measurement - state);
    uncertainty = (1 - gain) * uncertainty;
}

void MPU6050Handler::smoothAngles(float kalmanRoll, float kalmanPitch) {
    rollWindow[smoothingIndex] = kalmanRoll;
    pitchWindow[smoothingIndex] = kalmanPitch;
    smoothingIndex = (smoothingIndex + 1) % SMOOTHING_WINDOW_SIZE;
    float rollSum = 0, pitchSum = 0;
    for (int i = 0; i < SMOOTHING_WINDOW_SIZE; i++) {
        rollSum += rollWindow[i];
        pitchSum += pitchWindow[i];
    }
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        smoothedRoll = rollSum / SMOOTHING_WINDOW_SIZE;
        smoothedPitch = pitchSum / SMOOTHING_WINDOW_SIZE;
        xSemaphoreGive(mutex);
    }
}