#ifndef CONFIG_H
#define CONFIG_H

// WiFi Credentials
static const char* SSID = "Gedung MRPQ";
static const char* PASSWORD = "umar1234";

// API Server
static const char* SERVER_URL = "http://168.231.119.233:5000/research/create";
static const char* DEVICE_NAME = "prototype";
static const char* DEVICE_PASSWORD = "prototype";
static const char* PROFILE_CODE = "prototype";

// Pin Definitions
static const int PIN_SDA = 43;
static const int PIN_SCL = 44;
static const int PIN_BUTTON = 21;
static const int PIN_HX711_DATA = 18;
static const int PIN_HX711_CLOCK = 17;
static const int PIN_BUZZER = 3;
static const int PIN_LED = 15;

// HX711 Settings
static const int CAL_VAL_EEPROM_ADDR = 0;
static const int TARE_OFFSET_EEPROM_ADDR = 20;
static const float CALIBRATION_VALUE = 1306.5337;
static const unsigned long HX711_STABILIZING_TIME = 2000;

// MPU6050 Settings
static const uint8_t MPU6050_ADDR = 0x68;
static const float MPU_ACCEL_SCALE = 4096.0;  // ±8g
static const float MPU_GYRO_SCALE = 65.5;     // ±500°/s
static const int MPU_CALIBRATION_COUNT = 2000;

// Kalman Filter Settings
static const float KALMAN_UNCERTAINTY_ROLL = 2 * 2;
static const float KALMAN_UNCERTAINTY_PITCH = 2 * 2;

// Smoothing Settings
static const int SMOOTHING_WINDOW_SIZE = 10;

// RTOS Settings
static const unsigned long BUTTON_DEBOUNCE_DELAY = 200;

#endif