#include <Arduino.h>

#include "config.h"
#include "library.h"

// Profile Variables
String profile1_code = PROFILE_CODE_1;
String profile2_code = PROFILE_CODE_2;
String profile3_code = PROFILE_CODE_3;
String profile4_code = PROFILE_CODE_4;

// Hardware Instances
HX711_ADC LoadCell(PIN_HX711_DATA, PIN_HX711_CLOCK);
WiFiManager wm;
TFT_eSPI tft = TFT_eSPI();
OneButton button1(PUSH_BUTTON_1, true, true);
OneButton button2(PUSH_BUTTON_2, true, true);
OneButton button3(PUSH_BUTTON_3, true, true);
OneButton button4(PUSH_BUTTON_4, true, true);

// Sprite Instances
TFT_eSprite loading_screen = TFT_eSprite(&tft);
TFT_eSprite progress_bar = TFT_eSprite(&tft);
TFT_eSprite select_profile = TFT_eSprite(&tft);
TFT_eSprite text_profile = TFT_eSprite(&tft);
TFT_eSprite tunggu_nimbang = TFT_eSprite(&tft);
TFT_eSprite timbangan_screen = TFT_eSprite(&tft);
TFT_eSprite arc = TFT_eSprite(&tft);
TFT_eSprite text_timbangan = TFT_eSprite(&tft);
TFT_eSprite pairing_mode = TFT_eSprite(&tft);
TFT_eSprite text_paring = TFT_eSprite(&tft);
TFT_eSprite cek_data = TFT_eSprite(&tft);
TFT_eSprite calib_screen = TFT_eSprite(&tft);
TFT_eSprite calib_text = TFT_eSprite(&tft);
TFT_eSprite position_screen = TFT_eSprite(&tft);
TFT_eSprite position_text = TFT_eSprite(&tft);
TFT_eSprite alarm_screen = TFT_eSprite(&tft);
TFT_eSprite alarm_text = TFT_eSprite(&tft);

// --- RTOS Handles ---
TaskHandle_t h_taskSensorRead = NULL;
TaskHandle_t h_taskButtonCheck = NULL;
TaskHandle_t h_taskDisplayUpdate = NULL;
TaskHandle_t h_taskMainLogic = NULL;
TaskHandle_t h_taskTimeSync = NULL;
TaskHandle_t h_setupTask = NULL;  // Handle for the main setup task

SemaphoreHandle_t xSerialMutex = NULL;
SemaphoreHandle_t xSensorDataMutex = NULL;
SemaphoreHandle_t xStateMutex = NULL;
SemaphoreHandle_t xI2CMutex = NULL;
SemaphoreHandle_t xDataReadySemaphore = NULL;  // For HX711 ISR

QueueHandle_t xButtonQueue, xHttpQueue;

// --- Forward Declarations ---
void setupEEPROM();
void setupScreen();
void setupLoadCell();
void readMPU6050(float &RateRoll, float &RatePitch, float &RateYaw, float &AngleRoll, float &AnglePitch);
void setupMPU6050();

// Task Functions
void taskWiFiManagerCode(void *pvParameters);
void taskButtonCheckCode(void *pvParameters);

// Thread-safe Serial Print
void safePrintln(String msg);
void safePrint(String msg);

// ISR Function
void IRAM_ATTR resetESP();
void IRAM_ATTR dataReadyISR();

void setup() {
    Serial.begin(115200);
    delay(5000);
    Serial.println("Starting Centung...");

    Wire.begin(PIN_IIC_SDA, PIN_IIC_SCL);
    Wire.setClock(400000);

    pinMode(PIN_POWER_ON, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PIN_BAT_VOLT, INPUT);

    digitalWrite(PIN_POWER_ON, HIGH);

    rtc_gpio_pullup_en((gpio_num_t)PIN_HX711_DATA);
    rtc_gpio_hold_en((gpio_num_t)PIN_HX711_DATA);

    rtc_gpio_pullup_en((gpio_num_t)PUSH_BUTTON_3);
    rtc_gpio_hold_en((gpio_num_t)PUSH_BUTTON_3);
    esp_sleep_enable_ext0_wakeup((gpio_num_t)PUSH_BUTTON_3, 0);

    // attachInterrupt(digitalPinToInterrupt(PUSH_BUTTON_3), resetESP, FALLING);
    attachInterrupt(digitalPinToInterrupt(PIN_HX711_DATA), dataReadyISR, FALLING);

    // Store the handle of the current task (setup runs in this task initially)
    h_setupTask = xTaskGetCurrentTaskHandle();

    // --- Create Mutexes and Semaphores FIRST ---
    xSerialMutex = xSemaphoreCreateMutex();
    xSensorDataMutex = xSemaphoreCreateMutex();
    xStateMutex = xSemaphoreCreateMutex();
    xI2CMutex = xSemaphoreCreateMutex();
    xDataReadySemaphore = xSemaphoreCreateBinary();

    xButtonQueue = xQueueCreate(10, sizeof(ButtonEvent));

    if (!xSerialMutex || !xSensorDataMutex || !xStateMutex || !xI2CMutex || !xDataReadySemaphore) {
        safePrintln("FATAL: Failed to create RTOS synchronization objects!");
        while (1);
    }
    safePrintln("RTOS Objects Created.");

    // --- Initialize Peripherals ---
    setupEEPROM();
    setupScreen();
    setupLoadCell();
    setupMPU6050();
    safePrintln("Peripherals Initialized.");

    // --- Start WiFi Manager Task ---
    xTaskCreatePinnedToCore(taskWiFiManagerCode, "WiFi Setup Task", 8192, NULL, 1, NULL, 0);
    safePrintln("WiFi Setup Task created. Waiting for notification...");

    // Wait here until WiFi is connected by the taskWiFiManagerCode
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait indefinitely for notification

    if (WiFi.status() == WL_CONNECTED) {
        safePrintln("WiFi Connection Established by Task.");
        xTaskCreatePinnedToCore(taskButtonCheckCode, "ButtonCheck", 2048, NULL, 2, &h_taskButtonCheck, 1);
    } else {
        safePrintln("FATAL: WiFi Connection Failed in Task!");
    }
}

void loop() {
}

void setupEEPROM() {
    EEPROM.begin(512);
    EEPROM.get(calVal_eepromAdress, calibrationValue);
    if (isnan(calibrationValue)) {
        calibrationValue = 1306.5337;
    }
    safePrint("Calibration value: ");
    safePrintln(String(calibrationValue));

    EEPROM.get(tareOffsetVal_eepromAdress, calibrationOffset);
    if (isnan(calibrationOffset)) {
        calibrationOffset = 8434937.00;
    }
    safePrint("Zero offset value: ");
    safePrintln(String(calibrationOffset));
}

void setupScreen() {
    ledcSetup(0, 10000, 8);
    ledcAttachPin(38, 0);
    ledcWrite(0, brightness);

    tft.init();
    tft.setRotation(3);
    tft.setSwapBytes(true);
    tft.fillScreen(TFT_WHITE);
    tft.setTextColor(TFT_BLACK, 0xE73C);
    tft.pushImage(0, 0, 320, 170, loading);

    loading_screen.createSprite(320, 170);
    loading_screen.setSwapBytes(true);
    progress_bar.createSprite(320, 170);

    select_profile.createSprite(320, 170);
    select_profile.setSwapBytes(true);
    text_profile.createSprite(320, 170);
    text_profile.setTextColor(TFT_BLACK, PROFILE_BACKGROUND);

    tunggu_nimbang.createSprite(320, 170);
    tunggu_nimbang.setSwapBytes(true);
    timbangan_screen.createSprite(320, 170);
    timbangan_screen.setSwapBytes(true);
    arc.createSprite(320, 170);
    text_timbangan.createSprite(320, 170);
    text_timbangan.setTextColor(TFT_BLACK, 0xE73C);

    pairing_mode.createSprite(320, 170);
    pairing_mode.setSwapBytes(true);
    text_paring.createSprite(320, 170);
    text_paring.setTextColor(TFT_BLACK, 0xE73C);

    calib_screen.createSprite(320, 170);
    calib_screen.setSwapBytes(true);
    calib_text.createSprite(320, 170);
    calib_text.setTextColor(TFT_BLACK, 0xE73C);

    position_screen.createSprite(320, 170);
    position_screen.setSwapBytes(true);
    position_text.createSprite(320, 170);
    position_text.setTextColor(TFT_BLACK, 0xE73C);

    cek_data.createSprite(320, 170);
    cek_data.setTextDatum(4);
    cek_data.setTextColor(TFT_WHITE, TFT_BLACK);

    alarm_screen.createSprite(320, 170);
    alarm_screen.setSwapBytes(true);
    alarm_text.createSprite(320, 170);
    alarm_text.setTextColor(TFT_BLACK, 0xE73C);
}

void setupLoadCell() {
    LoadCell.begin();
    LoadCell.start(stabilizingtime, _tare);

    if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag()) {
        safePrintln("Timeout, check MCU>HX711 wiring and pin designations");
        esp_deep_sleep_start();
    } else {
        LoadCell.setCalFactor(calibrationValue);  // set calibration value (float)
        safePrintln("Startup is complete");
    }
}

void readMPU6050(float &RateRoll, float &RatePitch, float &RateYaw, float &AngleRoll, float &AnglePitch) {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x3B);  // Starting with ACCEL_XOUT_H register
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, (uint8_t)14, (uint8_t)true);  // Request 14 bytes

    // Baca data Accelerometer dan Gyroscope
    // Format: AXH, AXL, AYH, AYL, AZH, AZL, TempH, TempL, GXH, GXL, GYH, GYL, GZH, GZL
    int16_t AccXLSB = Wire.read() << 8 | Wire.read();
    int16_t AccYLSB = Wire.read() << 8 | Wire.read();
    int16_t AccZLSB = Wire.read() << 8 | Wire.read();
    Wire.read() << 8 | Wire.read();  // Skip temperature
    int16_t GyroX = Wire.read() << 8 | Wire.read();
    int16_t GyroY = Wire.read() << 8 | Wire.read();
    int16_t GyroZ = Wire.read() << 8 | Wire.read();

    // Konversi ke nilai fisik (g dan deg/s)
    // Perhatikan tanda negatif jika orientasi sensor terbalik
    float AccX = (float)AccXLSB / MPU_ACCEL_SCALE;
    float AccY = (float)AccYLSB / MPU_ACCEL_SCALE;
    float AccZ = (float)AccZLSB / MPU_ACCEL_SCALE;

    // Define AngleRoll as physical roll (originally AnglePitch)
    AngleRoll = -atan2(AccX, sqrt(AccY * AccY + AccZ * AccZ)) * (180.0 / PI);
    // Define AnglePitch as physical pitch (originally AngleRoll)
    AnglePitch = atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * (180.0 / PI);

    // Define RateRoll as physical roll rate (originally RatePitch)
    RateRoll = (float)GyroY / MPU_GYRO_SCALE;
    // Define RatePitch as physical pitch rate (originally RateRoll)
    RatePitch = (float)GyroX / MPU_GYRO_SCALE;
    RateYaw = (float)GyroZ / MPU_GYRO_SCALE;
}

void setupMPU6050() {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0x00);  // Set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
    delay(100);  // Tunggu sensor stabil
    safePrintln("MPU6050 Initialized.");

    safePrintln("Calibrating MPU6050 Gyro... Keep it stable.");

    float rollSum = 0, pitchSum = 0, yawSum = 0;
    float dummyAngleRoll, dummyAnglePitch;  // Variabel sementara

    for (int i = 0; i < MPU_CALIBRATION_COUNT; i++) {
        float r, p, y;
        readMPU6050(r, p, y, dummyAngleRoll, dummyAnglePitch);  // Hanya butuh rate untuk kalibrasi bias
        rollSum += r;
        pitchSum += p;
        yawSum += y;
        delay(3);  // Delay kecil antar pembacaan
    }

    GyroBiasRoll = rollSum / MPU_CALIBRATION_COUNT;
    GyroBiasPitch = pitchSum / MPU_CALIBRATION_COUNT;
    GyroBiasYaw = yawSum / MPU_CALIBRATION_COUNT;

    safePrintln("Calibration Complete:");
    safePrint("Gyro Bias Roll: ");
    safePrintln(String(GyroBiasRoll));
    safePrint("Gyro Bias Pitch: ");
    safePrintln(String(GyroBiasPitch));
    safePrint("Gyro Bias Yaw: ");
    safePrintln(String(GyroBiasYaw));
}

void setupButtonCallbacks() {
    button1.attachClick([]() {
        safePrintln("Button 1 Clicked");
        ButtonEvent event = {1, 0};
        xQueueSend(xButtonQueue, &event, 0);
    });
    button1.attachLongPressStart([]() {
        ButtonEvent event = {1, 1};
        xQueueSend(xButtonQueue, &event, 0);
    });
    button1.attachDoubleClick([]() {
        ButtonEvent event = {1, 2};
        xQueueSend(xButtonQueue, &event, 0);
    });
    button2.attachClick([]() {
        safePrintln("Button 2 Clicked");
        ButtonEvent event = {2, 0};
        xQueueSend(xButtonQueue, &event, 0);
    });
    button2.attachLongPressStart([]() {
        ButtonEvent event = {2, 1};
        xQueueSend(xButtonQueue, &event, 0);
    });
    button2.attachDoubleClick([]() {
        ButtonEvent event = {2, 2};
        xQueueSend(xButtonQueue, &event, 0);
    });
    button3.attachClick([]() {
        safePrintln("Button 3 Clicked");
        ButtonEvent event = {3, 0};
        xQueueSend(xButtonQueue, &event, 0);
    });
    button3.attachLongPressStart([]() {
        ButtonEvent event = {3, 1};
        xQueueSend(xButtonQueue, &event, 0);
        safePrintln("entering deep sleep...");
        pinMode(PIN_POWER_ON, OUTPUT);
        pinMode(PIN_LCD_BL, OUTPUT);
        digitalWrite(PIN_POWER_ON, LOW);
        digitalWrite(PIN_LCD_BL, LOW);
        delay(1000);
        esp_deep_sleep_start();
    });
    button3.attachDoubleClick([]() {
        ButtonEvent event = {3, 2};
        xQueueSend(xButtonQueue, &event, 0);
    });
    button4.attachClick([]() {
        safePrintln("Button 4 Clicked");
        ButtonEvent event = {4, 0};
        xQueueSend(xButtonQueue, &event, 0);
    });
    button4.attachLongPressStart([]() {
        ButtonEvent event = {4, 1};
        xQueueSend(xButtonQueue, &event, 0);
    });
    button4.attachDoubleClick([]() {
        ButtonEvent event = {4, 2};
        xQueueSend(xButtonQueue, &event, 0);
    });
}

void taskWiFiManagerCode(void *pvParameters) {
    safePrintln("taskWiFiManager executing.");
    wm.setConnectTimeout(CONNECT_TIMEOUT);
    wm.setConfigPortalTimeout(PORTAL_TIMEOUT);

    bool wifiConnected = wm.autoConnect("CENTUNG");

    if (!wifiConnected && !Ping.ping(remote_ip)) {
        safePrintln("WiFiManager Failed to Connect.");
        Serial.flush();
        wm.setConfigPortalTimeout(PORTAL_TIMEOUT);
        if (!wm.startConfigPortal("CENTUNG")) {
            safePrintln("failed to connect and hit timeout");
            esp_deep_sleep_start();
        } else {
            safePrintln("WiFiManager Connected!");
            delay(100);  // Small delay to stabilize WiFi connection
            ESP.restart();
        }
    } else {
        safePrintln("WiFiManager Connected!");
        delay(100);  // Small delay to stabilize WiFi connection
    }

    // Notify the setup() task that WiFi setup attempt is complete
    if (h_setupTask != NULL) {
        xTaskNotifyGive(h_setupTask);
        safePrintln("Notification sent to setup task.");
    } else {
        safePrintln("Error: Could not get handle for setup task to notify.");
    }

    // Flush Serial output and wait briefly to ensure all messages are printed
    Serial.flush();
    delay(50);

    vTaskDelete(NULL);  // Delete self
}

void taskButtonCheckCode(void *pvParameters) {
    safePrintln("taskButtonCheck started.");
    setupButtonCallbacks();
    while (1) {
        button1.tick();
        button2.tick();
        button3.tick();
        button4.tick();
        vTaskDelay(pdMS_TO_TICKS(10));  // Check buttons every 10ms
    }
}

void IRAM_ATTR resetESP() {
    ESP.restart();
}

void IRAM_ATTR dataReadyISR() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xDataReadySemaphore, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

void safePrintln(String msg) {
    if (xSerialMutex == NULL) {
        Serial.println("Error: Serial Mutex is NULL!");
        return;
    }
    if (xSemaphoreTake(xSerialMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        Serial.println(msg);
        Serial.flush();  // Ensure immediate output
        xSemaphoreGive(xSerialMutex);
    } else {
        Serial.println("Error: Could not take Serial Mutex for : " + msg);
    }
}

void safePrint(String msg) {
    if (xSerialMutex == NULL) {
        Serial.println("Error: Serial Mutex is NULL!");
        return;
    }
    if (xSemaphoreTake(xSerialMutex, pdMS_TO_TICKS(1000)) == pdTRUE) {
        Serial.print(msg);
        Serial.flush();  // Ensure immediate output
        xSemaphoreGive(xSerialMutex);
    } else {
        Serial.println("Error: Could not take Serial Mutex for : " + msg);
    }
}

// ================== BATAS SUCI ================

// // --- RTOS Handles ---
// TaskHandle_t h_taskSensorRead = NULL;
// TaskHandle_t h_taskButtonCheck = NULL;
// TaskHandle_t h_taskDisplayUpdate = NULL;
// TaskHandle_t h_taskFirebaseComms = NULL;
// TaskHandle_t h_taskMainLogic = NULL;
// TaskHandle_t h_taskTimeSync = NULL;
// TaskHandle_t h_setupTask = NULL;  // Handle for the main setup task

// SemaphoreHandle_t xSerialMutex = NULL;
// SemaphoreHandle_t xFirebaseMutex = NULL;
// SemaphoreHandle_t xSensorDataMutex = NULL;
// SemaphoreHandle_t xStateMutex = NULL;
// SemaphoreHandle_t xI2CMutex = NULL;
// SemaphoreHandle_t xDataReadySemaphore = NULL;  // For HX711 ISR

// // --- Forward Declarations ---
// void IRAM_ATTR dataReadyISR();
// void setupWiFi();
// void setupFirebase();
// void setupLoadcell();
// void setupMPU6050();  // Modified
// void setupButtons();
// void setupDisplay();
// void loadCalibration();
// void saveCalibration();
// void performTare();
// void getLocalTimeRTC();
// bool connectFirebase();

// // Kalman related functions
// void readMPU6050Data(float &RateRoll, float &RatePitch, float &RateYaw, float &AngleRollAcc, float &AnglePitchAcc);  // Modified signature
// void calibrateMPU6050GyroBias();                                                                                     // Renamed for clarity
// void kalmanUpdate(float newAngle, float newRate, float &kalmanAngle, float &kalmanBias, float P[2][2], float dt);    // Modified Kalman function for 2-state (angle, bias)
// void smoothAngles(float &smoothedRoll, float &smoothedPitch);                                                        // Modified signature

// // Task Functions
// void taskSensorReadCode(void *pvParameters);
// void taskButtonCheckCode(void *pvParameters);
// void taskDisplayUpdateCode(void *pvParameters);
// void taskFirebaseCommsCode(void *pvParameters);
// void taskMainLogicCode(void *pvParameters);
// void taskTimeSyncCode(void *pvParameters);
// void taskWiFiManagerCode(void *pvParameters);

// // Helper Functions
// String extractDocumentId(String documentName);
// void deleteDocument(String documentId);
// void get_query();

// // Thread-safe Serial Print
// void safePrintln(String msg) {
//     if (xSemaphoreTake(xSerialMutex, portMAX_DELAY) == pdTRUE) {
//         Serial.println(msg);
//         xSemaphoreGive(xSerialMutex);
//     }
// }
// void safePrint(String msg) {
//     if (xSemaphoreTake(xSerialMutex, portMAX_DELAY) == pdTRUE) {
//         Serial.print(msg);
//         xSemaphoreGive(xSerialMutex);
//     }
// }

// // =========================================================================
// // SETUP - Initializes hardware, RTOS objects, and starts tasks
// // =========================================================================
// void setup() {
//     Serial.begin(115200);
//     while (!Serial);

//     Serial.println("\nStarting Smart Rice Dispenser - RTOS Kalman Version");

//     EEPROM.begin(512);
//     // Initialize I2C using pins from pin_config.h
//     Wire.begin(PIN_IIC_SDA, PIN_IIC_SCL);
//     Wire.setClock(400000);  // Use fast I2C clock speed

//     // Store the handle of the current task (setup runs in this task initially)
//     h_setupTask = xTaskGetCurrentTaskHandle();

//     // --- Create Mutexes and Semaphores FIRST ---
//     xSerialMutex = xSemaphoreCreateMutex();
//     xFirebaseMutex = xSemaphoreCreateMutex();
//     xSensorDataMutex = xSemaphoreCreateMutex();
//     xStateMutex = xSemaphoreCreateMutex();
//     xI2CMutex = xSemaphoreCreateMutex();
//     xDataReadySemaphore = xSemaphoreCreateBinary();

//     if (!xSerialMutex || !xFirebaseMutex || !xSensorDataMutex || !xStateMutex || !xI2CMutex || !xDataReadySemaphore) {
//         Serial.println("FATAL: Failed to create RTOS synchronization objects!");
//         while (1);
//     }

//     safePrintln("RTOS Objects Created.");

//     // --- Initialize Peripherals ---
//     setupDisplay();
//     setupLoadcell();
//     setupMPU6050();  // Includes MPU init and blocking calibration
//     setupButtons();

//     safePrintln("Peripherals Initialized.");

//     // --- Start WiFi Manager Task ---
//     xTaskCreatePinnedToCore(
//         taskWiFiManagerCode, "WiFi Setup Task", 8192, NULL, 1, NULL, 0);

//     safePrintln("WiFi Setup Task created. Waiting for notification...");

//     // Wait here until WiFi is connected by the taskWiFiManagerCode
//     ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait indefinitely for notification

//     if (WiFi.status() == WL_CONNECTED) {
//         safePrintln("WiFi Connection Established by Task.");
//         setupFirebase();
//         safePrintln("Firebase Configured.");

//         // --- Start Core Application Tasks (Pin to Core 1 if possible) ---
//         safePrintln("Creating Application Tasks...");
//         // Increase stack for sensor task if Kalman/smoothing adds significant load
//         xTaskCreatePinnedToCore(taskSensorReadCode, "SensorRead", 6144, NULL, 3, &h_taskSensorRead, 1);
//         xTaskCreatePinnedToCore(taskButtonCheckCode, "ButtonCheck", 2048, NULL, 2, &h_taskButtonCheck, 1);
//         xTaskCreatePinnedToCore(taskDisplayUpdateCode, "DisplayUpdate", 4096, NULL, 2, &h_taskDisplayUpdate, 1);
//         xTaskCreatePinnedToCore(taskFirebaseCommsCode, "FirebaseComms", 8192, NULL, 1, &h_taskFirebaseComms, 1);
//         xTaskCreatePinnedToCore(taskMainLogicCode, "MainLogic", 4096, NULL, 2, &h_taskMainLogic, 1);
//         xTaskCreatePinnedToCore(taskTimeSyncCode, "TimeSync", 4096, NULL, 1, &h_taskTimeSync, 1);

//         safePrintln("All Application Tasks Created.");

//     } else {
//         safePrintln("FATAL: WiFi Connection Failed in Task!");
//         while (1) {
//             vTaskDelay(1000 / portTICK_PERIOD_MS);
//         }
//     }
// }

// // =========================================================================
// // LOOP - Unused
// // =========================================================================
// void loop() {
//     vTaskDelay(1000 / portTICK_PERIOD_MS);
// }

// // =========================================================================
// // WiFi Manager Task (Unchanged from previous RTOS version)
// // =========================================================================
// void taskWiFiManagerCode(void *pvParameters) {
//     safePrintln("taskWiFiManager executing.");
//     wm.setConnectTimeout(20);
//     wm.setConfigPortalTimeout(180);

//     bool wifiConnected = wm.autoConnect("SmartRiceDispenserAP");

//     if (!wifiConnected) {
//         safePrintln("WiFiManager Failed to Connect.");
//     } else {
//         safePrintln("WiFiManager Connected!");
//         safePrint("IP Address: ");
//         safePrintln(WiFi.localIP().toString());
//     }

//     // Notify the setup() task that WiFi setup attempt is complete
//     if (h_setupTask != NULL) {
//         xTaskNotifyGive(h_setupTask);
//         safePrintln("Notification sent to setup task.");
//     } else {
//         safePrintln("Error: Could not get handle for setup task to notify.");
//     }

//     vTaskDelete(NULL);  // Delete self
// }

// // =========================================================================
// // Sensor Reading Task (Modified for Kalman Filter)
// // =========================================================================
// void taskSensorReadCode(void *pvParameters) {
//     safePrintln("taskSensorRead started.");
//     TickType_t xLastWakeTime;
//     const TickType_t xFrequency = pdMS_TO_TICKS((int)(KalmanDt * 1000));  // Task frequency based on KalmanDt (e.g., 4ms)

//     float rateRoll, ratePitch, rateYaw;         // Local variables for raw rates
//     float angleRollAcc, anglePitchAcc;          // Local variables for accel angles
//     float smoothedRoll = 0, smoothedPitch = 0;  // Local for smoothed output

//     // Initialize the xLastWakeTime variable with the current time.
//     xLastWakeTime = xTaskGetTickCount();

//     while (1) {
//         // --- Read Load Cell (Same as before) ---
//         if (xSemaphoreTake(xDataReadySemaphore, pdMS_TO_TICKS(10)) == pdTRUE) {  // Short wait for semaphore
//             if (LoadCell.update()) {
//                 float weight = LoadCell.getData();
//                 if (xSemaphoreTake(xSensorDataMutex, portMAX_DELAY) == pdTRUE) {
//                     currentWeight_kg = weight / 1000.0;
//                     xSemaphoreGive(xSensorDataMutex);
//                 }
//             } else {
//                 safePrintln("Error: Load cell update failed after ISR signal.");
//             }
//         }  // No else needed, we continue even if LC data is missed

//         // --- Read MPU6050 & Apply Kalman Filter ---
//         // Protect I2C access
//         if (xSemaphoreTake(xI2CMutex, portMAX_DELAY) == pdTRUE) {
//             // Read raw accel/gyro data and calculate accel angles
//             readMPU6050Data(rateRoll, ratePitch, rateYaw, angleRollAcc, anglePitchAcc);
//             xSemaphoreGive(xI2CMutex);  // Release I2C Mutex *before* calculations

//             // Apply gyro bias correction
//             rateRoll -= GyroBiasRoll;
//             ratePitch -= GyroBiasPitch;
//             // rateYaw -= GyroBiasYaw; // Yaw is not used in this filter

//             // Apply Kalman filter update
//             // void kalmanUpdate(float newAngle, float newRate, float &kalmanAngle, float &kalmanBias, float P[2][2], float dt)
//             kalmanUpdate(angleRollAcc, rateRoll, KalmanAngleRoll, KalmanBiasRoll, KalmanPRoll, KalmanDt);
//             kalmanUpdate(anglePitchAcc, ratePitch, KalmanAnglePitch, KalmanBiasPitch, KalmanPPitch, KalmanDt);

//             // Optional: Apply Smoothing
//             smoothAngles(smoothedRoll, smoothedPitch);  // Updates smoothedRoll, smoothedPitch vars

//             // --- Update Shared Sensor Variables ---
//             if (xSemaphoreTake(xSensorDataMutex, portMAX_DELAY) == pdTRUE) {
//                 // Use smoothed values if smoothing is enabled, otherwise use direct Kalman output
//                 // roll = KalmanAngleRoll;
//                 // pitch = KalmanAnglePitch;
//                 roll = smoothedRoll;    // Use smoothed values
//                 pitch = smoothedPitch;  // Use smoothed values
//                                         // yaw = 0; // Yaw not calculated here
//                                         // temperature = ???; // Need to add temp reading in readMPU6050Data if required
//                 xSemaphoreGive(xSensorDataMutex);
//             }
//             // Debug print (optional, use mutex)
//             // safePrintln("R: " + String(roll, 2) + ", P: " + String(pitch, 2));

//         } else {
//             safePrintln("Error: Could not get I2C Mutex for MPU read.");
//             // Skip MPU reading/update for this cycle
//         }

//         // Wait for the next cycle. Ensures consistent timing for Kalman filter.
//         vTaskDelayUntil(&xLastWakeTime, xFrequency);
//     }
// }

// // =========================================================================
// // Button Check Task (Unchanged)
// // =========================================================================
// void taskButtonCheckCode(void *pvParameters) {
//     safePrintln("taskButtonCheck started.");
//     button1.attachClick([]() {
//         safePrintln("Button 1 Clicked");
//         if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
//             currentProfile = 1;
//             waitUntilClick = 0;
//             xSemaphoreGive(xStateMutex);
//         }
//     });
//     button2.attachClick([]() { safePrintln("Button 2 Clicked"); /* Signal MainLogic */ });
//     button3.attachClick([]() { safePrintln("Button 3 Clicked"); /* Signal MainLogic */ });
//     button4.attachClick([]() { safePrintln("Button 4 Clicked"); /* Signal MainLogic */ });
//     button4.attachLongPressStart([]() { safePrintln("Button 4 Long Press Start - TARE"); performTare(); });
//     button4.attachDoubleClick([]() { safePrintln("Button 4 Double Click - RESTART"); ESP.restart(); });

//     while (1) {
//         button1.tick();
//         button2.tick();
//         button3.tick();
//         button4.tick();
//         vTaskDelay(pdMS_TO_TICKS(20));  // Check buttons every 20ms
//     }
// }

// // =========================================================================
// // Display Update Task (Unchanged, reads 'roll' and 'pitch')
// // =========================================================================
// void taskDisplayUpdateCode(void *pvParameters) {
//     safePrintln("taskDisplayUpdate started.");
//     float displayedWeight = -1.0;
//     int displayedState = -1;
//     float displayedRoll = -999, displayedPitch = -999;  // Track displayed angles

//     while (1) {
//         // Read shared data
//         float weightToShow;
//         int stateToShow;
//         String profileNameToShow = "None";
//         double currentIntake = 0;
//         double maxIntake = 0;
//         float rollToShow, pitchToShow;

//         if (xSemaphoreTake(xSensorDataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
//             weightToShow = currentWeight_kg;
//             rollToShow = roll;    // Get filtered roll
//             pitchToShow = pitch;  // Get filtered pitch
//             xSemaphoreGive(xSensorDataMutex);
//         } else {
//             continue;
//         }

//         if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
//             stateToShow = currentState;
//             switch (currentProfile) {
//                 case 1:
//                     profileNameToShow = pengguna1;
//                     currentIntake = KNasi_profile1;
//                     maxIntake = KNasi1;
//                     break;
//                 case 2:
//                     profileNameToShow = pengguna2;
//                     currentIntake = KNasi_profile2;
//                     maxIntake = KNasi2;
//                     break;
//                 case 3:
//                     profileNameToShow = pengguna3;
//                     currentIntake = KNasi_profile3;
//                     maxIntake = KNasi3;
//                     break;
//                 case 4:
//                     profileNameToShow = pengguna4;
//                     currentIntake = KNasi_profile4;
//                     maxIntake = KNasi4;
//                     break;
//                 default:
//                     profileNameToShow = "Select User";
//                     break;
//             }
//             xSemaphoreGive(xStateMutex);
//         } else {
//             continue;
//         }

//         // --- Update Display Logic ---
//         // Update weight
//         if (abs(weightToShow - displayedWeight) > 0.01) {
//             tft.fillRect(10, 50, 150, 20, TFT_BLACK);
//             tft.setCursor(10, 50);
//             tft.print("Weight: ");
//             tft.print(weightToShow, 2);
//             tft.print(" kg");
//             displayedWeight = weightToShow;
//         }

//         // Update state/profile info
//         if (stateToShow != displayedState) {
//             tft.fillRect(10, 80, 200, 20, TFT_BLACK);
//             tft.setCursor(10, 80);
//             tft.print("State: ");
//             tft.print(stateToShow);

//             tft.fillRect(10, 110, 300, 40, TFT_BLACK);
//             tft.setCursor(10, 110);
//             tft.print("User: ");
//             tft.print(profileNameToShow);
//             tft.setCursor(10, 130);
//             tft.print("Intake: ");
//             tft.print(currentIntake, 0);
//             tft.print(" / ");
//             tft.print(maxIntake, 0);
//             tft.print(" g");

//             displayedState = stateToShow;
//         }

//         // Update Roll/Pitch if changed significantly
//         if (abs(rollToShow - displayedRoll) > 0.5 || abs(pitchToShow - displayedPitch) > 0.5) {
//             tft.fillRect(180, 10, 140, 40, TFT_BLACK);  // Clear area
//             tft.setCursor(180, 10);
//             tft.print("R:");
//             tft.print(rollToShow, 1);
//             tft.setCursor(180, 30);
//             tft.print("P:");
//             tft.print(pitchToShow, 1);
//             displayedRoll = rollToShow;
//             displayedPitch = pitchToShow;
//         }

//         vTaskDelay(pdMS_TO_TICKS(100));
//     }
// }

// // =========================================================================
// // Firebase Communication Task (Unchanged)
// // =========================================================================
// void taskFirebaseCommsCode(void *pvParameters) {
//     // --- Code is identical to the previous RTOS version ---
//     // It reads state and sensor data using mutexes when logging.
//     safePrintln("taskFirebaseComms started.");
//     bool initialQueryDone = false;
//     unsigned long lastLogTime = 0;
//     const unsigned long logInterval = 5 * 60 * 1000;
//     bool firebaseReady = false;

//     while (1) {
//         // Ensure Firebase is connected and ready
//         if (!Firebase.ready() || WiFi.status() != WL_CONNECTED) {
//             safePrintln("Firebase/WiFi not ready, attempting connection...");
//             firebaseReady = connectFirebase();
//             if (!firebaseReady) {
//                 vTaskDelay(pdMS_TO_TICKS(15000));
//                 continue;
//             }
//             safePrintln("Firebase Connection Successful.");
//             initialQueryDone = false;
//         } else {
//             firebaseReady = true;
//         }

//         // Perform Initial Data Query
//         if (firebaseReady && !initialQueryDone) {
//             safePrintln("Performing initial Firebase query...");
//             if (xSemaphoreTake(xFirebaseMutex, portMAX_DELAY) == pdTRUE) {
//                 get_query();
//                 xSemaphoreGive(xFirebaseMutex);
//                 safePrintln("Initial query complete.");
//                 if (xSemaphoreTake(xStateMutex, portMAX_DELAY) == pdTRUE) {
//                     // Copy fetched data
//                     done_query = 1;
//                     xSemaphoreGive(xStateMutex);
//                 }
//                 initialQueryDone = true;
//             } else {
//                 safePrintln("Error: Could not get Firebase Mutex for initial query.");
//             }
//         }

//         // Periodic Data Logging
//         unsigned long now = millis();
//         if (firebaseReady && initialQueryDone && (now - lastLogTime > logInterval)) {
//             safePrintln("Logging data to Firebase...");
//             if (xSemaphoreTake(xFirebaseMutex, portMAX_DELAY) == pdTRUE) {
//                 // 1. Read necessary data safely
//                 float weightToLog = 0;
//                 int currentProfileToLog = 0;
//                 double intakeToLog = 0;  // Example, determine actual value to log
//                 float rollToLog = 0, pitchToLog = 0;

//                 if (xSemaphoreTake(xSensorDataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
//                     weightToLog = currentWeight_kg;  // Log current weight maybe? Or last dispensed amount?
//                     rollToLog = roll;
//                     pitchToLog = pitch;
//                     xSemaphoreGive(xSensorDataMutex);
//                 }
//                 if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
//                     currentProfileToLog = currentProfile;
//                     // intakeToLog = ? // Determine what value represents the intake to log here
//                     xSemaphoreGive(xStateMutex);
//                 }

//                 // 2. Construct Firebase Path and JSON data
//                 String path = "/log_centung";
//                 FirebaseJson content;
//                 getLocalTimeRTC();
//                 String timestamp = String(year) + "-" + String(month) + "-" + String(day) + "T" +
//                                    String(timeHour) + ":" + String(timeMin) + ":" + String(timeSec) + "Z";

//                 content.set("fields/device_id/stringValue", devId);  // Use actual device ID
//                 content.set("fields/timestamp/timestampValue", timestamp);
//                 content.set("fields/berat_nasi/doubleValue", intakeToLog);  // LOG THE ACTUAL DISPENSED AMOUNT
//                 content.set("fields/profile/integerValue", currentProfileToLog);
//                 content.set("fields/roll/doubleValue", rollToLog);    // Log angle
//                 content.set("fields/pitch/doubleValue", pitchToLog);  // Log angle

//                 // 3. Send data
//                 String documentPath = "";
//                 safePrintln("Creating Firestore Document...");
//                 bool success = Firebase.Firestore.createDocument(&fbdo, FIREBASE_PROJECT_ID, "", path.c_str(), content.raw());

//                 if (success) {
//                     safePrintln("Log sent successfully. Response: " + fbdo.payload());
//                     lastLogTime = now;
//                 } else {
//                     safePrintln("Error sending log: " + fbdo.errorReason());
//                 }

//                 xSemaphoreGive(xFirebaseMutex);
//             } else {
//                 safePrintln("Error: Could not get Firebase Mutex for logging.");
//             }
//         }
//         vTaskDelay(pdMS_TO_TICKS(1000));
//     }
// }

// // =========================================================================
// // Main Application Logic Task (Unchanged, uses 'roll' and 'pitch')
// // =========================================================================
// void taskMainLogicCode(void *pvParameters) {
//     // --- Code is identical to the previous RTOS version ---
//     // It reads 'roll' and 'pitch' from the shared sensor data using mutexes
//     // to make state decisions.
//     safePrintln("taskMainLogic started.");
//     int localState = 0;
//     int previousState = -1;
//     unsigned long stateEntryTime = 0;

//     while (1) {
//         // Read Inputs
//         float currentWeight;
//         float currentRoll, currentPitch;  // Will get Kalman filtered values
//         int buttonEvent = 0;
//         bool firebaseDataUpdated = false;

//         if (xSemaphoreTake(xSensorDataMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
//             currentWeight = currentWeight_kg;
//             currentRoll = roll;
//             currentPitch = pitch;
//             xSemaphoreGive(xSensorDataMutex);
//         } else {
//             vTaskDelay(pdMS_TO_TICKS(10));
//             continue;
//         }

//         if (xSemaphoreTake(xStateMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
//             localState = currentState;
//             // Check button/firebase flags/queues
//             xSemaphoreGive(xStateMutex);
//         } else {
//             vTaskDelay(pdMS_TO_TICKS(10));
//             continue;
//         }

//         // State Machine Logic
//         unsigned long timeInState = millis() - stateEntryTime;

//         if (localState != previousState) {
//             safePrintln("Entering State: " + String(localState));
//             stateEntryTime = millis();
//             previousState = localState;
//             if (xSemaphoreTake(xStateMutex, portMAX_DELAY) == pdTRUE) {
//                 waitUntilClick = (localState == 0);
//                 xSemaphoreGive(xStateMutex);
//             }
//         }

//         switch (localState) {
//             case 0:  // Idle / Select Profile State
//                 if (currentProfile != 0) {
//                     localState = 1;
//                 }
//                 break;

//             case 1:  // Ready / Waiting for Rice Dispense
//                 // Use filtered angles for tilt detection
//                 if (abs(currentPitch) > 30 || abs(currentRoll) > 30) {  // Adjust threshold if needed
//                     safePrintln("Tilt detected, possible dispense");
//                     localState = 2;
//                 }
//                 if (buttonEvent == 4) {
//                     localState = 0;
//                     currentProfile = 0;
//                 }
//                 break;

//             case 2:  // Dispensing / Measuring
//                 // Use filtered angles to detect stability
//                 if (abs(currentPitch) < 10 && abs(currentRoll) < 10 && timeInState > 500) {  // Adjust threshold
//                     safePrintln("Dispense complete (stable)");
//                     if (xSemaphoreTake(xStateMutex, portMAX_DELAY) == pdTRUE) {
//                         // Calculate dispensed amount (needs weight tracking logic)
//                         // Signal firebase task to log
//                         // firebaseLogRequest = true; etc.
//                         xSemaphoreGive(xStateMutex);
//                     }
//                     localState = 1;
//                 }
//                 break;
//             // ... other states ...
//             default:
//                 safePrintln("Warning: Unknown state: " + String(localState));
//                 localState = 0;
//                 break;
//         }

//         if (localState != previousState) {
//             if (xSemaphoreTake(xStateMutex, portMAX_DELAY) == pdTRUE) {
//                 currentState = localState;
//                 xSemaphoreGive(xStateMutex);
//             }
//         }

//         vTaskDelay(pdMS_TO_TICKS(50));
//     }
// }

// // =========================================================================
// // Time Synchronization Task (Unchanged)
// // =========================================================================
// void taskTimeSyncCode(void *pvParameters) {
//     // --- Code is identical to the previous RTOS version ---
//     safePrintln("taskTimeSync started.");
//     const long gmtOffset_sec = 7 * 3600;
//     const int daylightOffset_sec = 0;
//     const char *ntpServer = "pool.ntp.org";

//     while (WiFi.status() != WL_CONNECTED) {
//         safePrintln("TimeSync: Waiting for WiFi...");
//         vTaskDelay(pdMS_TO_TICKS(5000));
//     }

//     safePrintln("TimeSync: Configuring NTP...");
//     configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

//     while (1) {
//         getLocalTimeRTC();
//         vTaskDelay(pdMS_TO_TICKS(60 * 60 * 1000));  // Sync hourly
//         safePrintln("TimeSync: Resyncing NTP...");
//         configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
//     }
// }

// // =========================================================================
// // Initialization Functions (Called from setup)
// // =========================================================================

// // --- Display Setup (Unchanged) ---
// void setupDisplay() { /* ... Same as before ... */
//     safePrintln("Initializing Display...");
//     tft.init();
//     tft.setRotation(1);
//     tft.fillScreen(TFT_BLACK);
//     tft.setTextColor(TFT_WHITE, TFT_BLACK);
//     tft.setTextSize(2);
//     tft.setCursor(10, 10);
//     tft.println("Starting...");
//     safePrintln("Display Initialized.");
// }

// // --- Load Cell Setup (Unchanged) ---
// void setupLoadcell() { /* ... Same as before ... */
//     safePrintln("Initializing Load Cell...");
//     LoadCell.begin();
//     LoadCell.start(stabilizingtime, true);
//     loadCalibration();
//     if (LoadCell.getTareTimeoutFlag()) {
//         safePrintln("Load Cell Tare Timeout!");
//     } else {
//         safePrintln("Load Cell Tare successful.");
//     }
//     LoadCell.setCalFactor(calibrationValue);
//     safePrintln("Load Cell Calibration Factor set to: " + String(calibrationValue));
//     attachInterrupt(digitalPinToInterrupt(PIN_HX711_DATA), dataReadyISR, FALLING);
//     safePrintln("Load Cell Initialized and ISR attached.");
// }

// // --- MPU6050 Setup (Modified for Kalman/Direct Wire) ---
// void setupMPU6050() {
//     safePrintln("Initializing MPU6050...");
//     // Protect I2C access during initialization
//     if (xSemaphoreTake(xI2CMutex, portMAX_DELAY) == pdTRUE) {
//         // Wake up MPU6050
//         Wire.beginTransmission(MPU6050_ADDR);
//         Wire.write(PWR_MGMT_1_REG);  // Power management register
//         Wire.write(0x00);            // Write 0 to wake up
//         if (Wire.endTransmission() != 0) {
//             safePrintln("FATAL: MPU6050 Wake command failed!");
//             xSemaphoreGive(xI2CMutex);  // Release mutex before halting
//             while (1) vTaskDelay(100);
//         }
//         safePrintln("MPU6050 Wake command sent.");
//         delay(100);  // Allow time for sensor to stabilize after wake

//         // Optional: Set Gyro and Accel Full Scale Range (if different from default)
//         // Example: Set Gyro to +/- 500 deg/s (0x08), Accel to +/- 8g (0x10)
//         Wire.beginTransmission(MPU6050_ADDR);
//         Wire.write(GYRO_CONFIG_REG);
//         Wire.write(0x08);  // Check datasheet for values corresponding to scale factors used
//         Wire.endTransmission();
//         delay(10);
//         Wire.beginTransmission(MPU6050_ADDR);
//         Wire.write(ACCEL_CONFIG_REG);
//         Wire.write(0x10);  // Check datasheet for values corresponding to scale factors used
//         Wire.endTransmission();
//         delay(10);
//         safePrintln("MPU6050 Ranges configured.");

//         // --- Perform Gyro Bias Calibration ---
//         // IMPORTANT: This blocks setup() for CalibrationCount * 3 milliseconds!
//         calibrateMPU6050GyroBias();  // Uses blocking delay inside

//         // Initialize smoothing arrays
//         for (int i = 0; i < smoothingWindow; i++) {
//             rollWindow[i] = 0;
//             pitchWindow[i] = 0;
//         }
//         smoothingIndex = 0;
//         safePrintln("Smoothing arrays initialized.");

//         safePrintln("MPU6050 Initialized and Calibrated.");
//         xSemaphoreGive(xI2CMutex);
//     } else {
//         safePrintln("FATAL: Could not get I2C Mutex for MPU6050 Init!");
//         while (1) {
//             vTaskDelay(100 / portTICK_PERIOD_MS);
//         };
//     }
// }

// // --- Button Setup (Unchanged) ---
// void setupButtons() { /* ... Same as before ... */
//     safePrintln("Initializing Buttons...");
//     safePrintln("Buttons Initialized.");
// }

// // --- Firebase Setup (Unchanged) ---
// void setupFirebase() { /* ... Same as before ... */
//     safePrintln("Configuring Firebase...");
//     // Removed config.project_id assignment as FirebaseConfig has no such member.
//     config.database_url = String(FIREBASE_HOST);
//     config.signer.test_mode = true;  // Use if using Database Secret
//     auth.user.email = "";
//     auth.user.password = "";
//     Firebase.begin(&config, &auth);
//     Firebase.reconnectWiFi(true);
//     // fbdo.setResponseTimeout(1000 * 60); // Removed: setResponseTimeout is deprecated.
//     //    Firebase.setwriteSizeLimit(fbdo, "tiny");
//     safePrintln("Firebase Configured.");
// }

// // =========================================================================
// // Kalman Filter & MPU6050 Helper Functions
// // =========================================================================

// // Function to read accelerometer and gyroscope data via I2C
// // Modifies the passed reference variables
// void readMPU6050Data(float &RateRoll, float &RatePitch, float &RateYaw, float &AngleRollAcc, float &AnglePitchAcc) {
//     // Assumes I2C Mutex is already taken by caller

//     Wire.beginTransmission(MPU6050_ADDR);
//     Wire.write(ACCEL_XOUT_H_REG);  // Starting register for Accel X High byte
//     Wire.endTransmission(false);   // Send repeated start

//     if (Wire.requestFrom(MPU6050_ADDR, 14) == 14) {  // Request 14 bytes (Accel + Temp + Gyro)
//         int16_t AccXLSB = Wire.read() << 8 | Wire.read();
//         int16_t AccYLSB = Wire.read() << 8 | Wire.read();
//         int16_t AccZLSB = Wire.read() << 8 | Wire.read();
//         // Skip Temperature bytes
//         Wire.read();
//         Wire.read();
//         int16_t GyroX = Wire.read() << 8 | Wire.read();
//         int16_t GyroY = Wire.read() << 8 | Wire.read();
//         int16_t GyroZ = Wire.read() << 8 | Wire.read();

//         // Scale Gyro values to degrees/second
//         RateRoll = (float)GyroX / GyroScaleFactor;
//         RatePitch = (float)GyroY / GyroScaleFactor;
//         RateYaw = (float)GyroZ / GyroScaleFactor;

//         // Scale Accel values to g's
//         float AccX = (float)AccXLSB / AccelScaleFactor;
//         float AccY = (float)AccYLSB / AccelScaleFactor;
//         float AccZ = (float)AccZLSB / AccelScaleFactor;

//         // Calculate Roll and Pitch angles from Accelerometer data
//         // Avoid division by zero or sqrt of zero if AccZ is near zero (sensor flat)
//         // Protect against edge case where vector is straight up/down Z axis
//         if (AccZ != 0.0) {
//             AngleRollAcc = atan2(AccY, AccZ) * (180.0 / PI);
//             // Calculate pitch using atan2 for stability around +-90 degrees
//             AnglePitchAcc = atan2(-AccX, sqrt(AccY * AccY + AccZ * AccZ)) * (180.0 / PI);
//         } else {
//             // Handle edge case - e.g., if AccX is positive, pitch is -90, else +90
//             AngleRollAcc = (AccY > 0) ? 90.0 : -90.0;  // Or determine based on AccY if desired
//             AnglePitchAcc = (AccX > 0) ? -90.0 : 90.0;
//         }
//         // Note: atan2(y, x) preferred over atan(y/x)

//     } else {
//         safePrintln("Error reading MPU6050 data (bytes mismatch)");
//         // Keep previous values or set to zero?
//         RateRoll = RatePitch = RateYaw = 0;
//         AngleRollAcc = AnglePitchAcc = 0;  // Or keep last known good angle?
//     }
// }

// // Calibrate Gyro Bias (Called during setup - blocking)
// void calibrateMPU6050GyroBias() {
//     // Assumes I2C Mutex is already taken by caller
//     safePrintln("Starting Gyro Calibration...");
//     float rollSum = 0, pitchSum = 0, yawSum = 0;
//     float tempRateRoll, tempRatePitch, tempRateYaw, tempAngleRoll, tempAnglePitch;  // Dummy vars

//     for (int i = 0; i < CalibrationCount; i++) {
//         readMPU6050Data(tempRateRoll, tempRatePitch, tempRateYaw, tempAngleRoll, tempAnglePitch);
//         rollSum += tempRateRoll;
//         pitchSum += tempRatePitch;
//         yawSum += tempRateYaw;
//         if (i % 100 == 0) safePrint(".");  // Progress indicator
//         delay(3);                          // Use blocking delay here as RTOS scheduler might not be reliable yet
//                                            // OR move calibration to the sensor task after startup using vTaskDelay.
//     }

//     GyroBiasRoll = rollSum / CalibrationCount;
//     GyroBiasPitch = pitchSum / CalibrationCount;
//     GyroBiasYaw = yawSum / CalibrationCount;

//     safePrintln("\nCalibration complete.");
//     safePrintln(" Bias Roll: " + String(GyroBiasRoll, 4));
//     safePrintln(" Bias Pitch: " + String(GyroBiasPitch, 4));
//     safePrintln(" Bias Yaw: " + String(GyroBiasYaw, 4));
// }

// // Kalman filter update function (2-state: angle, bias)
// void kalmanUpdate(float newAngle, float newRate, float &kalmanAngle, float &kalmanBias, float P[2][2], float dt) {
//     // Predict step
//     // Update angle estimation based on gyro rate and estimated bias
//     kalmanAngle += dt * (newRate - kalmanBias);

//     // Update error covariance matrix prediction
//     P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + KalmanQAngle);
//     P[0][1] -= dt * P[1][1];
//     P[1][0] -= dt * P[1][1];
//     P[1][1] += KalmanQBias * dt;

//     // Update step (incorporate accelerometer measurement)
//     float S = P[0][0] + KalmanRMeasure;  // Innovation covariance (denominator of gain)
//     float K[2];                          // Kalman gain vector
//     K[0] = P[0][0] / S;
//     K[1] = P[1][0] / S;

//     float y = newAngle - kalmanAngle;  // Angle difference (innovation)

//     // Update angle and bias estimates
//     kalmanAngle += K[0] * y;
//     kalmanBias += K[1] * y;

//     // Update error covariance matrix
//     float P00_temp = P[0][0];
//     float P01_temp = P[0][1];

//     P[0][0] -= K[0] * P00_temp;
//     P[0][1] -= K[0] * P01_temp;
//     P[1][0] -= K[1] * P00_temp;
//     P[1][1] -= K[1] * P01_temp;
// }

// // Smoothing function (Simple Moving Average)
// void smoothAngles(float &smoothedRoll, float &smoothedPitch) {
//     // Uses global KalmanAngleRoll, KalmanAnglePitch as input

//     rollWindow[smoothingIndex] = KalmanAngleRoll;
//     pitchWindow[smoothingIndex] = KalmanAnglePitch;

//     smoothingIndex = (smoothingIndex + 1) % smoothingWindow;

//     float rollSum = 0, pitchSum = 0;
//     for (int i = 0; i < smoothingWindow; i++) {
//         rollSum += rollWindow[i];
//         pitchSum += pitchWindow[i];
//     }

//     smoothedRoll = rollSum / smoothingWindow;
//     smoothedPitch = pitchSum / smoothingWindow;
// }

// // =========================================================================
// // Other Helper Functions (Unchanged)
// // =========================================================================
// void loadCalibration() { /* ... Same as before ... */
//     safePrintln("Loading calibration from EEPROM...");
//     EEPROM.get(calVal_eepromAdress, calibrationValue);
//     EEPROM.get(tareOffsetVal_eepromAdress, tareOffset);
//     if (isnan(calibrationValue) || calibrationValue == 0.0) {
//         calibrationValue = 1.0;
//         safePrintln("Invalid calibration value in EEPROM, using default.");
//     }
//     if (isnan(tareOffset)) {
//         tareOffset = 0;
//         safePrintln("Invalid tare offset in EEPROM, using default.");
//     }
//     safePrintln("Calibration Value: " + String(calibrationValue));
//     safePrintln("Tare Offset: " + String(tareOffset));
// }
// void saveCalibration() { /* ... Same as before ... */
//     safePrintln("Saving calibration to EEPROM...");
//     safePrintln("Value: " + String(calibrationValue));
//     safePrintln("Offset: " + String(tareOffset));
//     EEPROM.put(calVal_eepromAdress, calibrationValue);
//     EEPROM.put(tareOffsetVal_eepromAdress, tareOffset);
//     if (EEPROM.commit()) {
//         safePrintln("EEPROM commit successful.");
//     } else {
//         safePrintln("Error: EEPROM commit failed.");
//     }
// }
// void performTare() { /* ... Same as before ... */
//     safePrintln("Performing Tare operation...");
//     LoadCell.tareNoDelay();
//     needsTare = false;
//     safePrintln("Tare command issued.");
// }
// void getLocalTimeRTC() { /* ... Same as before (using simpler strftime) ... */
//     struct tm timeinfo;
//     if (!getLocalTime(&timeinfo)) {
//         safePrintln("Warning: Failed to obtain time");
//         return;
//     }
//     strftime(timeHour, 3, "%H", &timeinfo);
//     strftime(timeMin, 3, "%M", &timeinfo);
//     strftime(timeSec, 3, "%S", &timeinfo);
//     strftime(day, 3, "%d", &timeinfo);
//     strftime(month, 3, "%m", &timeinfo);
//     strftime(year, 5, "%Y", &timeinfo);
//     strftime(timeWeekDay, 4, "%u", &timeinfo);
//     dayInWeek = atoi(timeWeekDay);
// }
// bool connectFirebase() { /* ... Same as before ... */
//     unsigned long startAttemptTime = millis();
//     bool connected = false;
//     if (WiFi.status() != WL_CONNECTED) {
//         safePrintln("Firebase Connect: WiFi disconnected.");
//         return false;
//     }
//     if (xSemaphoreTake(xFirebaseMutex, pdMS_TO_TICKS(5000)) == pdTRUE) {
//         safePrintln("Firebase Connect: Attempting...");
//         Firebase.begin(&config, &auth);  // Re-init might be needed
//         while (!Firebase.ready() && (millis() - startAttemptTime < 15000)) {
//             safePrint(".");
//             vTaskDelay(pdMS_TO_TICKS(500));
//         }
//         if (Firebase.ready()) {
//             safePrintln("\nFirebase Connected/Ready.");
//             connected = true;
//         } else {
//             safePrintln("\nFirebase Connection Failed. Error: " + fbdo.errorReason());
//             connected = false;
//         }
//         xSemaphoreGive(xFirebaseMutex);
//     } else {
//         safePrintln("Firebase Connect: Timed out waiting for mutex.");
//         connected = false;
//     }
//     return connected;
// }
// // --- Firebase Query Function (To be called from Firebase Task) ---
// // Make sure this uses the fbdo object passed to it or accesses it via mutex
// void get_query() {
//     // IMPORTANT: Ensure xFirebaseMutex is TAKEN before calling this function
//     //            and GIVEN after it finishes.

//     safePrintln("Executing get_query()...");
//     FirebaseJson queryJson;     // Use a local JSON object
//     FirebaseJson logQueryJson;  // Use a local JSON object for log queries
//     // FirebaseJson resultJson; // Not needed if just using fbdo.payload()

//     // --- Query Setup Common Part ---
//     queryJson.set("select/fields/[0]/fieldPath", "device_id");
//     queryJson.set("select/fields/[1]/fieldPath", "nama_lengkap");
//     queryJson.set("select/fields/[2]/fieldPath", "KNasi");
//     queryJson.set("from/collectionId", "profile");
//     queryJson.set("from/allDescendants", false);
//     queryJson.set("limit", 1);  // Limit to 1 result per query

//     // --- Function to process profile query result ---
//     auto processProfileResult = [&](const String &payload, String &outNama, double &outKNasi) {
//         DynamicJsonDocument doc(1024);  // Adjust size as needed
//         DeserializationError error = deserializeJson(doc, payload);
//         if (error) {
//             safePrintln("Error deserializing profile JSON: " + String(error.c_str()));
//             safePrintln("Payload: " + payload);
//             outNama = "Error";
//             outKNasi = 0;
//             return;
//         }
//         if (doc.is<JsonArray>() && doc.size() > 0 && doc[0].containsKey("document")) {
//             outNama = doc[0]["document"]["fields"]["nama_lengkap"]["stringValue"].as<String>();
//             // Use doubleValue for numbers from Firestore unless specifically integer
//             outKNasi = doc[0]["document"]["fields"]["KNasi"]["doubleValue"].as<double>();
//             if (outNama == "null" || outNama.isEmpty()) {
//                 outNama = "Unknown";  // Default if name is missing/null
//             }
//         } else {
//             safePrintln("Profile query result format unexpected or empty.");
//             safePrintln("Payload: " + payload);
//             outNama = "Not Found";
//             outKNasi = 0;
//         }
//     };

//     // --- Function to process log query result ---
//     auto processLogResult = [&](const String &payload, double &outTotalBerat) {
//         outTotalBerat = 0;
//         // Increase size if many logs expected per day, decrease if fewer
//         DynamicJsonDocument doc(8192);  // Increased size guess
//         DeserializationError error = deserializeJson(doc, payload);
//         if (error) {
//             safePrintln("Error deserializing log JSON: " + String(error.c_str()));
//             safePrintln("Payload: " + payload);  // Print payload on error
//             return;
//         }
//         if (doc.is<JsonArray>()) {
//             int count = 0;
//             for (JsonObject item : doc.as<JsonArray>()) {
//                 if (item.containsKey("document") && item["document"].containsKey("fields") && item["document"]["fields"].containsKey("berat_nasi")) {
//                     outTotalBerat += item["document"]["fields"]["berat_nasi"]["doubleValue"].as<double>();
//                     count++;
//                 }
//             }
//             safePrintln("Logs processed for period: " + String(count));  // Log how many were actually processed
//         } else {
//             safePrintln("Log query result format unexpected (not array).");
//             safePrintln("Payload: " + payload);  // Print payload
//             if (doc.containsKey("error")) {
//                 safePrintln("Firestore Error: " + doc["error"]["message"].as<String>());
//             }
//         }
//     };

//     // --- Get Time for Log Query ---
//     getLocalTimeRTC();  // Ensure time is current
//     char dateBuffer[20];
//     snprintf(dateBuffer, sizeof(dateBuffer), "%s-%02d-%02d", year, month, day);
//     String startTimestampUTC = String(dateBuffer) + "T17:00:00.00Z";  // Yesterday 5 PM UTC is Today midnight WIB
//     safePrintln("Querying logs since: " + startTimestampUTC);

//     // --- Query Profile 1 ---
//     safePrintln("Querying Profile 1 (ID: " + kodeUnik1 + ")");
//     queryJson.set("where/fieldFilter/field/fieldPath", "device_id");
//     queryJson.set("where/fieldFilter/op", "EQUAL");
//     queryJson.set("where/fieldFilter/value/stringValue", kodeUnik1);
//     // *** FIXED HERE ***
//     if (Firebase.Firestore.runQuery(&fbdo, FIREBASE_PROJECT_ID, "", "/", &queryJson)) {
//         processProfileResult(fbdo.payload(), pengguna1, KNasi1);
//         safePrintln(" P1: Name=" + pengguna1 + ", Max=" + String(KNasi1));
//     } else {
//         safePrintln(" P1 Query Error: " + fbdo.errorReason());
//         pengguna1 = "Error";
//         KNasi1 = 0;
//     }

//     // --- Query Logs for Profile 1 ---
//     logQueryJson.clear();  // Clear before building new query
//     logQueryJson.set("select/fields/[0]/fieldPath", "berat_nasi");
//     logQueryJson.set("from/collectionId", "log_centung");
//     logQueryJson.set("where/compositeFilter/op", "AND");
//     logQueryJson.set("where/compositeFilter/filters/[0]/fieldFilter/field/fieldPath", "device_id");
//     logQueryJson.set("where/compositeFilter/filters/[0]/fieldFilter/op", "EQUAL");
//     logQueryJson.set("where/compositeFilter/filters/[0]/fieldFilter/value/stringValue", kodeUnik1);
//     logQueryJson.set("where/compositeFilter/filters/[1]/fieldFilter/field/fieldPath", "timestamp");
//     logQueryJson.set("where/compositeFilter/filters/[1]/fieldFilter/op", "GREATER_THAN_OR_EQUAL");
//     logQueryJson.set("where/compositeFilter/filters/[1]/fieldFilter/value/timestampValue", startTimestampUTC);

//     // *** FIXED HERE ***
//     if (Firebase.Firestore.runQuery(&fbdo, FIREBASE_PROJECT_ID, "", "/", &logQueryJson)) {
//         processLogResult(fbdo.payload(), KNasi_profile1);
//         safePrintln(" P1 Logs: Intake=" + String(KNasi_profile1));
//     } else {
//         safePrintln(" P1 Log Query Error: " + fbdo.errorReason());
//         KNasi_profile1 = 0;
//     }

//     // --- Query Profile 2 ---
//     safePrintln("Querying Profile 2 (ID: " + kodeUnik2 + ")");
//     queryJson.set("where/fieldFilter/value/stringValue", kodeUnik2);  // Just change the ID
//     // *** FIXED HERE ***
//     if (Firebase.Firestore.runQuery(&fbdo, FIREBASE_PROJECT_ID, "", "/", &queryJson)) {
//         processProfileResult(fbdo.payload(), pengguna2, KNasi2);
//         safePrintln(" P2: Name=" + pengguna2 + ", Max=" + String(KNasi2));
//     } else {
//         safePrintln(" P2 Query Error: " + fbdo.errorReason());
//         pengguna2 = "Error";
//         KNasi2 = 0;
//     }

//     // --- Query Logs for Profile 2 ---
//     logQueryJson.set("where/compositeFilter/filters/[0]/fieldFilter/value/stringValue", kodeUnik2);  // Change ID
//     // *** FIXED HERE ***
//     if (Firebase.Firestore.runQuery(&fbdo, FIREBASE_PROJECT_ID, "", "/", &logQueryJson)) {
//         processLogResult(fbdo.payload(), KNasi_profile2);
//         safePrintln(" P2 Logs: Intake=" + String(KNasi_profile2));
//     } else {
//         safePrintln(" P2 Log Query Error: " + fbdo.errorReason());
//         KNasi_profile2 = 0;
//     }

//     // --- Query Profile 3 ---
//     safePrintln("Querying Profile 3 (ID: " + kodeUnik3 + ")");
//     queryJson.set("where/fieldFilter/value/stringValue", kodeUnik3);
//     // *** FIXED HERE ***
//     if (Firebase.Firestore.runQuery(&fbdo, FIREBASE_PROJECT_ID, "", "/", &queryJson)) {
//         processProfileResult(fbdo.payload(), pengguna3, KNasi3);
//         safePrintln(" P3: Name=" + pengguna3 + ", Max=" + String(KNasi3));
//     } else {
//         safePrintln(" P3 Query Error: " + fbdo.errorReason());
//         pengguna3 = "Error";
//         KNasi3 = 0;
//     }

//     // --- Query Logs for Profile 3 ---
//     logQueryJson.set("where/compositeFilter/filters/[0]/fieldFilter/value/stringValue", kodeUnik3);  // Change ID
//     // *** FIXED HERE ***
//     if (Firebase.Firestore.runQuery(&fbdo, FIREBASE_PROJECT_ID, "", "/", &logQueryJson)) {
//         processLogResult(fbdo.payload(), KNasi_profile3);
//         safePrintln(" P3 Logs: Intake=" + String(KNasi_profile3));
//     } else {
//         safePrintln(" P3 Log Query Error: " + fbdo.errorReason());
//         KNasi_profile3 = 0;
//     }

//     // --- Query Profile 4 ---
//     safePrintln("Querying Profile 4 (ID: " + kodeUnik4 + ")");
//     queryJson.set("where/fieldFilter/value/stringValue", kodeUnik4);
//     // *** FIXED HERE ***
//     if (Firebase.Firestore.runQuery(&fbdo, FIREBASE_PROJECT_ID, "", "/", &queryJson)) {
//         processProfileResult(fbdo.payload(), pengguna4, KNasi4);
//         safePrintln(" P4: Name=" + pengguna4 + ", Max=" + String(KNasi4));
//     } else {
//         safePrintln(" P4 Query Error: " + fbdo.errorReason());
//         pengguna4 = "Error";
//         KNasi4 = 0;
//     }

//     // --- Query Logs for Profile 4 ---
//     logQueryJson.set("where/compositeFilter/filters/[0]/fieldFilter/value/stringValue", kodeUnik4);  // Change ID
//     // *** FIXED HERE ***
//     if (Firebase.Firestore.runQuery(&fbdo, FIREBASE_PROJECT_ID, "", "/", &logQueryJson)) {
//         processLogResult(fbdo.payload(), KNasi_profile4);
//         safePrintln(" P4 Logs: Intake=" + String(KNasi_profile4));
//     } else {
//         safePrintln(" P4 Log Query Error: " + fbdo.errorReason());
//         KNasi_profile4 = 0;
//     }

//     safePrintln("get_query() finished.");
// }

// // =========================================================================
// // ISRs (Unchanged)
// // =========================================================================
// void IRAM_ATTR dataReadyISR() {
//     BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//     xSemaphoreGiveFromISR(xDataReadySemaphore, &xHigherPriorityTaskWoken);
//     if (xHigherPriorityTaskWoken) {
//         portYIELD_FROM_ISR();
//     }
// }