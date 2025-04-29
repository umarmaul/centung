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
TFT_eSprite text_init = TFT_eSprite(&tft);
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

SemaphoreHandle_t xSensorDataMutex = NULL;
SemaphoreHandle_t xStateMutex = NULL;
SemaphoreHandle_t xI2CMutex = NULL;
SemaphoreHandle_t xDataReadySemaphore = NULL;  // For HX711 ISR

QueueHandle_t xButtonQueue, xHttpQueue;

// Shared Variables
AppState currentState = STATE_PROFILE_SELECTION;
SensorData sensorData;

// --- Forward Declarations ---
void setupEEPROM();
void setupScreen();
void setupLoadCell();
void readMPU6050(float &RateRoll, float &RatePitch, float &RateYaw, float &AngleRoll, float &AnglePitch);
void setupMPU6050();

// Task Functions
void taskWiFiManagerCode(void *pvParameters);
void taskButtonCheckCode(void *pvParameters);
void taskDeepSleepCode(void *pvParameters);
void taskMainLogicCode(void *pvParameters);

// ISR Function
void IRAM_ATTR resetESP();
void IRAM_ATTR dataReadyISR();

String getStateName(AppState state) {
    switch (state) {
        case STATE_PROFILE_SELECTION:
            return "STATE_PROFILE_SELECTION";
        case STATE_CHECK_ORIENTATION:
            return "STATE_CHECK_ORIENTATION";
        case STATE_MEASURING:
            return "STATE_MEASURING";
        case STATE_CALIBRATION:
            return "STATE_CALIBRATION";
        case STATE_CHECK_WEIGHT:
            return "STATE_CHECK_WEIGHT";
        case STATE_DELETE_DOC:
            return "STATE_DELETE_DOC";
        case STATE_PAIRING:
            return "STATE_PAIRING";
        case STATE_ALARM:
            return "STATE_ALARM";
        case STATE_ERROR:
            return "STATE_ERROR";
        default:
            return "Unknown";
    }
}

void setup() {
    Wire.begin(PIN_IIC_SDA, PIN_IIC_SCL);
    Wire.setClock(400000);

    Serial.begin(115200);

    pinMode(PIN_POWER_ON, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PIN_BAT_VOLT, INPUT);

    digitalWrite(PIN_POWER_ON, HIGH);

    rtc_gpio_pullup_en((gpio_num_t)PIN_HX711_DATA);
    rtc_gpio_hold_en((gpio_num_t)PIN_HX711_DATA);

    rtc_gpio_pullup_en((gpio_num_t)PUSH_BUTTON_3);
    rtc_gpio_hold_en((gpio_num_t)PUSH_BUTTON_3);
    esp_sleep_enable_ext0_wakeup((gpio_num_t)PUSH_BUTTON_3, 0);

    attachInterrupt(digitalPinToInterrupt(PUSH_BUTTON_3), resetESP, FALLING);
    attachInterrupt(digitalPinToInterrupt(PIN_HX711_DATA), dataReadyISR, FALLING);

    // Store the handle of the current task (setup runs in this task initially)
    h_setupTask = xTaskGetCurrentTaskHandle();

    // --- Create Mutexes and Semaphores FIRST ---
    xSensorDataMutex = xSemaphoreCreateMutex();
    xStateMutex = xSemaphoreCreateMutex();
    xI2CMutex = xSemaphoreCreateMutex();
    xDataReadySemaphore = xSemaphoreCreateBinary();

    xButtonQueue = xQueueCreate(10, sizeof(ButtonEvent));

    Serial.println("RTOS Objects Created.");

    // --- Initialize Peripherals ---
    setupEEPROM();
    setupScreen();
    setupLoadCell();
    setupMPU6050();
    Serial.println("Peripherals Initialized.");

    // --- Start WiFi Manager Task ---
    xTaskCreatePinnedToCore(taskWiFiManagerCode, "WiFi Setup Task", 8192, NULL, 1, NULL, 0);

    // Wait here until WiFi is connected by the taskWiFiManagerCode
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait indefinitely for notification

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("WiFi Connection Established by Task.");
        xTaskCreatePinnedToCore(taskButtonCheckCode, "ButtonCheck", 2048, NULL, 2, &h_taskButtonCheck, 1);
        xTaskCreatePinnedToCore(taskDeepSleepCode, "DeepSleep", 2048, NULL, 3, NULL, 1);
        xTaskCreatePinnedToCore(taskMainLogicCode, "MainTask", 8192, NULL, 1, &h_taskMainLogic, 1);
    } else {
        Serial.println("FATAL: WiFi Connection Failed in Task!");
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

    EEPROM.get(tareOffsetVal_eepromAdress, calibrationOffset);
    if (isnan(calibrationOffset)) {
        calibrationOffset = 8434937.00;
    }
}

void setupScreen() {
    ledcSetup(0, 10000, 8);
    ledcAttachPin(38, 0);
    ledcWrite(0, brightness);

    tft.init();
    tft.setRotation(3);
    tft.setSwapBytes(true);
    tft.fillScreen(TFT_WHITE);
    tft.pushImage(0, 0, 320, 170, loading);

    loading_screen.createSprite(320, 170);
    loading_screen.setSwapBytes(true);
    progress_bar.createSprite(320, 170);
    text_init.createSprite(320, 170);
    text_init.setTextColor(TFT_BLACK, BACKGROUND);

    select_profile.createSprite(320, 170);
    select_profile.setSwapBytes(true);
    text_profile.createSprite(320, 170);
    text_profile.setTextColor(TFT_BLACK, BACKGROUND);

    tunggu_nimbang.createSprite(320, 170);
    tunggu_nimbang.setSwapBytes(true);
    timbangan_screen.createSprite(320, 170);
    timbangan_screen.setSwapBytes(true);
    arc.createSprite(320, 170);
    text_timbangan.createSprite(320, 170);
    text_timbangan.setTextColor(TFT_BLACK, BACKGROUND);

    pairing_mode.createSprite(320, 170);
    pairing_mode.setSwapBytes(true);
    text_paring.createSprite(320, 170);
    text_paring.setTextColor(TFT_BLACK, BACKGROUND);

    calib_screen.createSprite(320, 170);
    calib_screen.setSwapBytes(true);
    calib_text.createSprite(320, 170);
    calib_text.setTextColor(TFT_BLACK, BACKGROUND);

    position_screen.createSprite(320, 170);
    position_screen.setSwapBytes(true);
    position_text.createSprite(320, 170);
    position_text.setTextColor(TFT_BLACK, BACKGROUND);

    cek_data.createSprite(320, 170);
    cek_data.setTextDatum(4);
    cek_data.setTextColor(TFT_WHITE, TFT_BLACK);

    alarm_screen.createSprite(320, 170);
    alarm_screen.setSwapBytes(true);
    alarm_text.createSprite(320, 170);
    alarm_text.setTextColor(TFT_BLACK, BACKGROUND);
}

void initTextScreen(String message) {
    loading_screen.pushImage(0, 0, 320, 170, loading);
    text_init.fillSprite(BACKGROUND);
    text_init.drawCentreString(message, 160, 128, 1);
    text_init.pushToSprite(&loading_screen, 0, 0, BACKGROUND);
    loading_screen.pushSprite(0, 0);
}

void setupLoadCell() {
    initTextScreen("Initializing Load Cell...");
    LoadCell.begin();
    LoadCell.start(stabilizingtime, _tare);

    if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag()) {
        Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
        esp_deep_sleep_start();
    } else {
        LoadCell.setCalFactor(calibrationValue);  // set calibration value (float)
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
    initTextScreen("Initializing MPU6050...");
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0x00);  // Set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
    delay(100);  // Tunggu sensor stabil

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
}

void setupButtonCallbacks() {
    button1.attachClick([]() {
        Serial.println("Button 1 Clicked");
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
        Serial.println("Button 2 Clicked");
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
        Serial.println("Button 3 Clicked");
        ButtonEvent event = {3, 0};
        xQueueSend(xButtonQueue, &event, 0);
    });
    button3.attachLongPressStop([]() {
        ButtonEvent event = {3, 1};
        xQueueSend(xButtonQueue, &event, 0);
    });
    button3.attachDoubleClick([]() {
        ButtonEvent event = {3, 2};
        xQueueSend(xButtonQueue, &event, 0);
    });
    button4.attachClick([]() {
        Serial.println("Button 4 Clicked");
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

void progressBarScreen() {
    int blocks = 0;
    while (blocks < 20) {
        loading_screen.pushImage(0, 0, 320, 170, loading);
        progress_bar.fillSprite(BACKGROUND);
        blocks++;
        progress_bar.drawRoundRect(98, 122, 124, 16, 3, TFT_BLACK);

        for (int i = 0; i < blocks; i++) {
            progress_bar.fillRect(i * 5 + (98 + 2) + (i * 1), 122 + 4, 5, 10, TFT_BLACK);
        }

        progress_bar.pushToSprite(&loading_screen, 0, 0, BACKGROUND);
        loading_screen.pushSprite(0, 0);
    }
}

void taskWiFiManagerCode(void *pvParameters) {
    wm.setConnectTimeout(CONNECT_TIMEOUT);
    wm.setConfigPortalTimeout(PORTAL_TIMEOUT);
    String message = "Connecting to WiFi: " + String(DEVICE_NAME);
    initTextScreen(message);

    bool wifiConnected = wm.autoConnect(DEVICE_NAME);

    if (!wifiConnected) {
        Serial.println("failed to connect and hit timeout");
        esp_deep_sleep_start();
    } else {
        if (!Ping.ping(REMOTE_IP)) {
            wm.setConfigPortalTimeout(PORTAL_TIMEOUT);
            if (!wm.startConfigPortal(DEVICE_NAME)) {
                Serial.println("No internet and hit timeout");
                esp_deep_sleep_start();
            } else {
                Serial.println("Connected and sleep");
                esp_deep_sleep_start();
            }
        }
    }

    progressBarScreen();

    // Notify the setup() task that WiFi setup attempt is complete
    if (h_setupTask != NULL) {
        xTaskNotifyGive(h_setupTask);
    } else {
        Serial.println("Error: Could not get handle for setup task to notify.");
    }

    vTaskDelete(NULL);  // Delete self
}

void taskButtonCheckCode(void *pvParameters) {
    detachInterrupt(digitalPinToInterrupt(PUSH_BUTTON_3));
    setupButtonCallbacks();
    while (1) {
        button1.tick();
        button2.tick();
        button3.tick();
        button4.tick();
        vTaskDelay(pdMS_TO_TICKS(10));  // Check buttons every 10ms
    }
}

void taskDeepSleepCode(void *pvParameters) {
    while (1) {
        ButtonEvent event;
        if (xQueueReceive(xButtonQueue, &event, portMAX_DELAY) == pdTRUE) {
            if (event.buttonId == 3 && event.eventType == 1) {
                Serial.println("Deep Sleep triggered by Button 3 long press.");

                // Suspend tasks safely
                if (h_taskSensorRead != NULL) vTaskSuspend(h_taskSensorRead);
                if (h_taskButtonCheck != NULL) vTaskSuspend(h_taskButtonCheck);
                if (h_taskDisplayUpdate != NULL) vTaskSuspend(h_taskDisplayUpdate);
                if (h_taskMainLogic != NULL) vTaskSuspend(h_taskMainLogic);
                if (h_taskTimeSync != NULL) vTaskSuspend(h_taskTimeSync);

                // Disable peripherals
                LoadCell.powerDown();
                Wire.end();
                digitalWrite(PIN_LCD_BL, LOW);
                digitalWrite(PIN_POWER_ON, LOW);

                // Detach all interrupts
                detachInterrupt(digitalPinToInterrupt(PIN_HX711_DATA));

                // Flush serial
                Serial.end();

                // Enter deep sleep
                Serial.println("Entering deep sleep...");
                esp_deep_sleep_start();
            }
        }
    }
}

void taskMainLogicCode(void *pvParameters) {
    while (1) {
        ButtonEvent event;
        HttpCommand data;
        if (xQueueReceive(xButtonQueue, &event, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (xSemaphoreTake(xStateMutex, portMAX_DELAY) == pdTRUE) {
                switch (currentState) {
                    case STATE_PROFILE_SELECTION:
                        initTextScreen(getStateName(currentState));
                        if (event.eventType == 0) {  // Click
                            if (event.buttonId == 1)
                                data.profile_code = profile1_code;
                            else if (event.buttonId == 2)
                                data.profile_code = profile2_code;
                            else if (event.buttonId == 3)
                                data.profile_code = profile3_code;
                            else if (event.buttonId == 4)
                                data.profile_code = profile4_code;
                            Serial.println(data.profile_code);
                            currentState = STATE_CHECK_ORIENTATION;
                        }
                        break;
                    case STATE_CHECK_ORIENTATION:
                        initTextScreen(getStateName(currentState));
                        break;
                    case STATE_MEASURING:
                        initTextScreen(getStateName(currentState));
                        break;
                    case STATE_ALARM:
                        initTextScreen(getStateName(currentState));
                        break;
                }
                xSemaphoreGive(xStateMutex);
            }
        }
    }
}

void IRAM_ATTR resetESP() {
    detachInterrupt(digitalPinToInterrupt(PUSH_BUTTON_3));
    detachInterrupt(digitalPinToInterrupt(PIN_HX711_DATA));

    ESP.restart();
}

void IRAM_ATTR dataReadyISR() {
    xSemaphoreGiveFromISR(xDataReadySemaphore, NULL);
}