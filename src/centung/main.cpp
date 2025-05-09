#include <Arduino.h>

#include "config.h"
#include "library.h"

// Profile Variables
String profile1_code = PROFILE_CODE_1;
String profile2_code = PROFILE_CODE_2;
String profile3_code = PROFILE_CODE_3;
String profile4_code = PROFILE_CODE_4;
String profileNames[4] = {"Profile 1", "Profile 2", "Profile 3", "Profile 4"};  // Updated by HttpRequest task
String selectedProfile;

// Hardware Instances
HX711_ADC LoadCell(PIN_HX711_DATA, PIN_HX711_CLOCK);
WiFiManager wm;
TFT_eSPI tft = TFT_eSPI();
OneButton button1(PUSH_BUTTON_3, true, true);
OneButton button2(PUSH_BUTTON_1, true, true);
OneButton button3(PUSH_BUTTON_4, true, true);
OneButton button4(PUSH_BUTTON_2, true, true);

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

// RTOS Handles
TaskHandle_t h_taskSensorRead = NULL;
TaskHandle_t h_taskButtonCheck = NULL;
TaskHandle_t h_taskDisplayUpdate = NULL;
TaskHandle_t h_taskMainLogic = NULL;
TaskHandle_t h_taskHttpRequest = NULL;
TaskHandle_t h_taskDeepSleep = NULL;
TaskHandle_t h_setupTask = NULL;

SemaphoreHandle_t xSensorDataMutex = NULL;
SemaphoreHandle_t xStateMutex = NULL;
SemaphoreHandle_t xI2CMutex = NULL;
SemaphoreHandle_t xDataReadySemaphore = NULL;  // For HX711 ISR

QueueHandle_t xButtonQueue = NULL;
QueueHandle_t xHttpQueue = NULL;

// Shared Variables
AppState currentState = STATE_PROFILE_SELECTION;
SensorData sensorData = {0.0, 0.0, 0.0};
float totalNasi[4] = {0.0, 0.0, 0.0, 0.0};  // Updated by HttpRequest task
float maxNasi[4] = {0.0, 0.0, 0.0, 0.0};    // Updated by HttpRequest task

// --- Forward Declarations ---
void setupEEPROM();
void setupScreen();
void setupLoadCell();
void setupMPU6050();
void readMPU6050(float &RateRoll, float &RatePitch, float &RateYaw, float &AngleRoll, float &AnglePitch);
void kalman_1d(float &KalmanState, float &KalmanUncertainty, float Rate, float Measurement);
void smoothAngles(float KalmanAngleRoll, float KalmanAnglePitch);

// Task Functions
void taskSensorReadCode(void *pvParameters);
void taskButtonCheckCode(void *pvParameters);
void taskDisplayUpdateCode(void *pvParameters);
void taskMainLogicCode(void *pvParameters);
void taskHttpRequestCode(void *pvParameters);
void taskDeepSleepCode(void *pvParameters);
void taskWiFiManagerCode(void *pvParameters);

// ISR Function
void IRAM_ATTR resetESP();
void IRAM_ATTR dataReadyISR();

// Utility Functions
String getStateName(AppState state);

void setup() {
    Serial.begin(115200);
    Wire.begin(PIN_IIC_SDA, PIN_IIC_SCL);
    Wire.setClock(400000);

    pinMode(PIN_POWER_ON, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PIN_BAT_VOLT, INPUT);

    digitalWrite(PIN_POWER_ON, HIGH);
    digitalWrite(PIN_BUZZER, LOW);

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
    xHttpQueue = xQueueCreate(10, sizeof(HttpCommand));

    if (!xSensorDataMutex || !xStateMutex || !xI2CMutex || !xDataReadySemaphore || !xButtonQueue || !xHttpQueue) {
        Serial.println("RTOS Object Creation Failed!");
        esp_deep_sleep_start();
    }

    // --- Initialize Peripherals ---
    setupEEPROM();
    setupScreen();
    setupLoadCell();
    setupMPU6050();
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

    // --- Start WiFi Manager Task ---
    xTaskCreatePinnedToCore(taskWiFiManagerCode, "WiFi Setup Task", 8192, NULL, 1, NULL, 0);

    // Wait here until WiFi is connected by the taskWiFiManagerCode
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);  // Wait indefinitely for notification

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("Calibration Value: " + String(calibrationValue));
        Serial.println("Calibration Offset: " + String(calibrationOffset));
        xTaskCreatePinnedToCore(taskButtonCheckCode, "ButtonCheck", 2048, NULL, 1, &h_taskButtonCheck, 1);
        xTaskCreatePinnedToCore(taskSensorReadCode, "SensorRead", 4096, NULL, 2, &h_taskSensorRead, 1);
        xTaskCreatePinnedToCore(taskHttpRequestCode, "HttpRequest", 8192, NULL, 3, &h_taskHttpRequest, 1);
        xTaskCreatePinnedToCore(taskDisplayUpdateCode, "DisplayUpdate", 4096, NULL, 1, &h_taskDisplayUpdate, 0);
        xTaskCreatePinnedToCore(taskMainLogicCode, "MainLogic", 8192, NULL, 2, &h_taskMainLogic, 0);
        xTaskCreatePinnedToCore(taskDeepSleepCode, "DeepSleep", 2048, NULL, 3, &h_taskDeepSleep, 0);
    } else {
        Serial.println("WiFi Connection Failed!");
        esp_deep_sleep_start();
    }
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));  // Empty loop
}

void setupEEPROM() {
    EEPROM.begin(512);
    EEPROM.get(calVal_eepromAdress, calibrationValue);
    if (isnan(calibrationValue)) {
        calibrationValue = 1435.05;
    }

    EEPROM.get(tareOffsetVal_eepromAdress, calibrationOffset);
    if (isnan(calibrationOffset)) {
        calibrationOffset = 8434937.00;
    }
}

void setupScreen() {
    ledcSetup(0, 10000, 8);
    ledcAttachPin(PIN_LCD_BL, 0);
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
        LoadCell.setCalFactor(calibrationValue);
    }
}

void setupMPU6050() {
    initTextScreen("Initializing MPU6050...");
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission(true);
    delay(100);

    float rollSum = 0, pitchSum = 0, yawSum = 0;
    float dummyAngleRoll, dummyAnglePitch;
    for (int i = 0; i < MPU_CALIBRATION_COUNT; i++) {
        float r, p, y;
        readMPU6050(r, p, y, dummyAngleRoll, dummyAnglePitch);
        rollSum += r;
        pitchSum += p;
        yawSum += y;
        delay(3);
    }
    GyroBiasRoll = rollSum / MPU_CALIBRATION_COUNT;
    GyroBiasPitch = pitchSum / MPU_CALIBRATION_COUNT;
    GyroBiasYaw = yawSum / MPU_CALIBRATION_COUNT;

    // Initialize Kalman and smoothing
    readMPU6050(rollSum, pitchSum, yawSum, dummyAngleRoll, dummyAnglePitch);
    KalmanAngleRoll = dummyAngleRoll;
    KalmanAnglePitch = dummyAnglePitch;
    for (int i = 0; i < SMOOTHING_WINDOW_SIZE; i++) {
        rollWindow[i] = KalmanAngleRoll;
        pitchWindow[i] = KalmanAnglePitch;
    }
}

void readMPU6050(float &RateRoll, float &RatePitch, float &RateYaw, float &AngleRoll, float &AnglePitch) {
    if (xSemaphoreTake(xI2CMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        Wire.beginTransmission(MPU6050_ADDR);
        Wire.write(0x3B);
        Wire.endTransmission(false);
        Wire.requestFrom(MPU6050_ADDR, (uint8_t)14, (uint8_t)true);

        int16_t AccXLSB = Wire.read() << 8 | Wire.read();
        int16_t AccYLSB = Wire.read() << 8 | Wire.read();
        int16_t AccZLSB = Wire.read() << 8 | Wire.read();
        Wire.read() << 8 | Wire.read();  // Skip temperature
        int16_t GyroX = Wire.read() << 8 | Wire.read();
        int16_t GyroY = Wire.read() << 8 | Wire.read();
        int16_t GyroZ = Wire.read() << 8 | Wire.read();

        float AccX = (float)AccXLSB / MPU_ACCEL_SCALE;
        float AccY = (float)AccYLSB / MPU_ACCEL_SCALE;
        float AccZ = (float)AccZLSB / MPU_ACCEL_SCALE;

        AngleRoll = -atan2(AccX, sqrt(AccY * AccY + AccZ * AccZ)) * (180.0 / PI);
        AnglePitch = atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * (180.0 / PI);

        RateRoll = (float)GyroY / MPU_GYRO_SCALE;
        RatePitch = (float)GyroX / MPU_GYRO_SCALE;
        RateYaw = (float)GyroZ / MPU_GYRO_SCALE;

        xSemaphoreGive(xI2CMutex);
    }
}

void kalman_1d(float &KalmanState, float &KalmanUncertainty, float Rate, float Measurement) {
    float dt = 0.004;  // 4ms loop
    KalmanState += dt * Rate;
    KalmanUncertainty += dt * dt * 4 * 4;  // Process noise
    float R = 3 * 3;                       // Measurement noise
    float KalmanGain = KalmanUncertainty / (KalmanUncertainty + R);
    KalmanState += KalmanGain * (Measurement - KalmanState);
    KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
}

void smoothAngles(float KalmanAngleRoll, float KalmanAnglePitch) {
    rollWindow[smoothingIndex] = KalmanAngleRoll;
    pitchWindow[smoothingIndex] = KalmanAnglePitch;
    smoothingIndex = (smoothingIndex + 1) % SMOOTHING_WINDOW_SIZE;

    float rollSum = 0, pitchSum = 0;
    for (int i = 0; i < SMOOTHING_WINDOW_SIZE; i++) {
        rollSum += rollWindow[i];
        pitchSum += pitchWindow[i];
    }

    if (xSemaphoreTake(xSensorDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        sensorData.roll = rollSum / SMOOTHING_WINDOW_SIZE;
        sensorData.pitch = pitchSum / SMOOTHING_WINDOW_SIZE;
        xSemaphoreGive(xSensorDataMutex);
    }
}

void setupButtonCallbacks() {
    button1.attachClick([]() {
        ButtonEvent event = {1, 0};
        xQueueSend(xButtonQueue, &event, 0);
    });
    button1.attachLongPressStop([]() {
        ButtonEvent event = {1, 1};
        xQueueSend(xButtonQueue, &event, 0);
        xTaskNotifyGive(h_taskDeepSleep);
    });
    button1.attachDoubleClick([]() {
        ButtonEvent event = {1, 2};
        xQueueSend(xButtonQueue, &event, 0);
    });
    button2.attachClick([]() {
        ButtonEvent event = {2, 0};
        xQueueSend(xButtonQueue, &event, 0);
    });
    button2.attachLongPressStop([]() {
        ButtonEvent event = {2, 1};
        xQueueSend(xButtonQueue, &event, 0);
    });
    button2.attachDoubleClick([]() {
        ButtonEvent event = {2, 2};
        xQueueSend(xButtonQueue, &event, 0);
    });
    button3.attachClick([]() {
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
        ButtonEvent event = {4, 0};
        xQueueSend(xButtonQueue, &event, 0);
    });
    button4.attachLongPressStop([]() {
        ButtonEvent event = {4, 1};
        xQueueSend(xButtonQueue, &event, 0);
    });
    button4.attachDoubleClick([]() {
        ButtonEvent event = {4, 2};
        xQueueSend(xButtonQueue, &event, 0);
    });
}

void progressBarScreen() {
    for (int blocks = 0; blocks <= 20; blocks++) {
        loading_screen.pushImage(0, 0, 320, 170, loading);
        progress_bar.fillSprite(BACKGROUND);
        progress_bar.drawRoundRect(98, 122, 124, 16, 3, TFT_BLACK);
        for (int i = 0; i < blocks; i++) {
            progress_bar.fillRect(i * 5 + 100 + i, 126, 5, 8, TFT_BLACK);
        }
        progress_bar.pushToSprite(&loading_screen, 0, 0, BACKGROUND);
        loading_screen.pushSprite(0, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void updateTimeVars() {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        return;
    }

    strftime(timeHour, 3, "%H", &timeinfo);
    strftime(timeMin, 3, "%M", &timeinfo);
    strftime(timeSec, 3, "%S", &timeinfo);
    strftime(timeWeekDay, 10, "%A", &timeinfo);

    String InWeek = String(timeWeekDay);
    for (int i = 0; i < 7; i++) {
        if (InWeek == SDay[i])
            dayInWeek = i;
    }

    strftime(day, 3, "%d", &timeinfo);
    strftime(month, 10, "%B", &timeinfo);
    strftime(year, 5, "%Y", &timeinfo);

    dayInMonth = String(day).toInt();

    for (int i = 0; i < 12; i++) {
        if (String(month) == Months[i]) {
            daysInMonth = mm[i];
            bulan = i + 1;
            tanggal = dayInMonth - 1;
            if (tanggal == 0) {
                tanggal = mm[i - 1];
                if (bulan == 1) {
                    tanggal = 31;
                    bulan = 12;
                    String(year) = "2023";
                }
                bulan = i;
            }
        }
    }

    int j = dayInWeek;
    for (int i = dayInMonth; i > 0; i--) {
        firstDay = j;
        j--;

        if (j == -1)
            j = 6;
    }
}

void taskWiFiManagerCode(void *pvParameters) {
    wm.setConnectTimeout(CONNECT_TIMEOUT);
    wm.setConfigPortalTimeout(PORTAL_TIMEOUT);
    initTextScreen("Connecting to WiFi: " + String(DEVICE_NAME));

    bool wifiConnected = wm.autoConnect(DEVICE_NAME);

    if (!wifiConnected) {
        Serial.println("WiFi Connection Timeout!");
        esp_deep_sleep_start();
    } else if (!Ping.ping(REMOTE_IP)) {
        if (!wm.startConfigPortal(DEVICE_NAME)) {
            Serial.println("No Internet!");
            esp_deep_sleep_start();
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

void taskSensorReadCode(void *pvParameters) {
    float RateRoll, RatePitch, RateYaw, AngleRoll, AnglePitch;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(4);  // 4ms loop

    for (;;) {
        // Read HX711
        if (xSemaphoreTake(xDataReadySemaphore, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (LoadCell.update()) {
                float weight = LoadCell.getData();
                weight = round(weight * 100) / 100.0;
                if (weight < 0) weight = 0;
                if (xSemaphoreTake(xSensorDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                    sensorData.weight = weight;
                    xSemaphoreGive(xSensorDataMutex);
                }
            }
        }

        // Read MPU6050 with Kalman Filter
        readMPU6050(RateRoll, RatePitch, RateYaw, AngleRoll, AnglePitch);
        RateRoll -= GyroBiasRoll;
        RatePitch -= GyroBiasPitch;
        kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
        kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
        smoothAngles(KalmanAngleRoll, KalmanAnglePitch);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void taskButtonCheckCode(void *pvParameters) {
    detachInterrupt(digitalPinToInterrupt(PUSH_BUTTON_3));
    setupButtonCallbacks();
    for (;;) {
        button1.tick();
        button2.tick();
        button3.tick();
        button4.tick();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void taskDisplayUpdateCode(void *pvParameters) {
    for (;;) {
        float localWeight, localRoll, localPitch, localTotalNasi, localMaxNasi;
        AppState localState;
        String localProfile;

        if (xSemaphoreTake(xSensorDataMutex, portMAX_DELAY) == pdTRUE) {
            localWeight = sensorData.weight;
            localRoll = sensorData.roll;
            localPitch = sensorData.pitch;
            xSemaphoreGive(xSensorDataMutex);
        }

        if (xSemaphoreTake(xStateMutex, portMAX_DELAY) == pdTRUE) {
            localState = currentState;
            localProfile = selectedProfile;
            if (selectedProfile == profile1_code) {
                localTotalNasi = totalNasi[0];
                localMaxNasi = maxNasi[0];
            } else if (selectedProfile == profile2_code) {
                localTotalNasi = totalNasi[1];
                localMaxNasi = maxNasi[1];
            } else if (selectedProfile == profile3_code) {
                localTotalNasi = totalNasi[2];
                localMaxNasi = maxNasi[2];
            } else if (selectedProfile == profile4_code) {
                localTotalNasi = totalNasi[3];
                localMaxNasi = maxNasi[3];
            }
            xSemaphoreGive(xStateMutex);
        }

        // Calculate battery percentage
        float volt = analogRead(PIN_BAT_VOLT) * 3.3 / 4095.0 * 2;
        float batteryPercentage = ((volt - 3.0) / (4.2 - 3.0)) * 100;
        if (batteryPercentage > 100) batteryPercentage = 100;
        if (batteryPercentage < 0) batteryPercentage = 0;
        int end_angle = (localWeight * 180 / 300) + 90;

        switch (localState) {
            case STATE_PROFILE_SELECTION: {
                select_profile.pushImage(0, 0, 320, 170, profile);
                text_profile.fillSprite(BACKGROUND);
                text_profile.drawCentreString(String(batteryPercentage, 0), 246, 5, 1);
                text_profile.drawCentreString(profileNames[0], 90, 27, 2);
                text_profile.drawCentreString(profileNames[1], 250, 27, 2);
                text_profile.drawCentreString(profileNames[2], 90, 125, 2);
                text_profile.drawCentreString(profileNames[3], 250, 125, 2);
                text_profile.pushToSprite(&select_profile, 0, 0, BACKGROUND);
                select_profile.pushSprite(0, 0);
                break;
            }
            case STATE_MEASURING: {
                if (localTotalNasi > localMaxNasi && !_isAware) {
                    currentState = STATE_ALARM;
                }

                timbangan_screen.pushImage(0, 0, 320, 170, timbangan);
                arc.fillSprite(BACKGROUND);
                text_timbangan.fillSprite(BACKGROUND);

                arc.drawSmoothArc(160, 153, 90, 80, 89, end_angle, TFT_CYAN, BACKGROUND, true);
                text_timbangan.drawCentreString(String(localWeight), 160, 90, 6);
                text_timbangan.drawCentreString("Back", 20, 15, 2);
                text_timbangan.drawCentreString("Save", 300, 15, 2);
                text_timbangan.drawCentreString(localProfile, 160, 5, 2);
                text_timbangan.drawCentreString("Tare", 300, 150, 2);
                text_timbangan.drawCentreString("Gyro", 20, 150, 2);
                text_timbangan.drawCentreString(String(batteryPercentage, 0), 246, 5, 1);
                arc.pushToSprite(&timbangan_screen, 0, 0, BACKGROUND);
                text_timbangan.pushToSprite(&timbangan_screen, 0, 0, BACKGROUND);
                timbangan_screen.pushSprite(0, 0);

                break;
            }
            case STATE_GYRO_TEST: {
                position_screen.pushImage(0, 0, 320, 170, datar);
                position_text.fillSprite(BACKGROUND);
                position_text.drawCentreString("Back", 20, 15, 2);
                position_text.drawCentreString(String(batteryPercentage, 0), 246, 5, 1);
                position_text.drawCentreString(String(localRoll), 90, 95, 4);
                position_text.drawCentreString(String(localPitch), 225, 95, 4);
                position_text.pushToSprite(&position_screen, 0, 0, BACKGROUND);
                position_screen.pushSprite(0, 0);
                break;
            }
            case STATE_CALIBRATION: {
                calib_screen.pushImage(0, 0, 320, 170, kotak);
                calib_text.fillSprite(BACKGROUND);
                calib_text.drawCentreString("Calibration Mode", 160, 55, 4);
                calib_text.drawCentreString("Tare first, then put 20g weight", 160, 90, 2);
                calib_text.drawCentreString("Back", 55, 20, 2);
                calib_text.drawCentreString("Reset", 55, 135, 2);
                calib_text.drawCentreString("Start", 265, 20, 2);
                calib_text.drawCentreString("Tare", 265, 135, 2);
                calib_text.drawCentreString(String(batteryPercentage, 0), 246, 5, 1);
                calib_text.pushToSprite(&calib_screen, 0, 0, BACKGROUND);
                calib_screen.pushSprite(0, 0);
                break;
            }
            case STATE_HISTORY: {
                updateTimeVars();

                cek_data.fillSprite(TFT_BLACK);

                cek_data.drawRoundRect(2, 2, 118, 166, 5, TFT_WHITE);  /// border
                cek_data.fillRoundRect(10, 61, 100, 24, 5, COLOR_2);
                cek_data.fillRoundRect(10, 110, 100, 24, 5, COLOR_2);

                cek_data.setTextColor(TFT_WHITE, COLOR_2);
                cek_data.drawCentreString(String(localTotalNasi, 1) + String(" gram"), 62, 64, 2);
                cek_data.drawCentreString(String(localMaxNasi, 1) + " gram", 62, 113, 2);

                cek_data.setTextColor(TFT_WHITE, TFT_BLACK);

                caw = 24;
                cay = 70;
                cax = 142;
                cah = 15;

                cek_data.setTextDatum(4);
                for (int j = 0; j < 7; j++) {
                    cek_data.drawString(Day[j], cax + (j * caw), cay, 2);
                }

                cek_data.drawCentreString("Telah dikonsumsi", 62, 40, 2);
                cek_data.drawCentreString("Maksimum", 62, 90, 2);

                int broj = 1;
                int w = 0;
                bool started = 0;

                for (int i = 0; i < 6; i++)
                    for (int j = 0; j < 7; j++) {
                        if (w == firstDay)
                            started = 1;
                        if (started == 1 && broj <= daysInMonth) {
                            if (broj == dayInMonth)
                                cek_data.setTextColor(TFT_BLACK, TFT_WHITE);
                            else
                                cek_data.setTextColor(TFT_ORANGE, TFT_BLACK);
                            cek_data.drawString(String(broj), cax + (j * caw), cay + cah + (cah * i), 2);
                            broj++;
                        }
                        w++;
                    }

                cek_data.setTextDatum(0);
                cek_data.setTextColor(TFT_WHITE, TFT_BLACK);
                cek_data.setFreeFont(&Orbitron_Light_32);
                cek_data.drawString(String(timeHour) + ":" + String(timeMin), 130, -6);
                cek_data.setFreeFont(&Orbitron_Light_24);

                cek_data.setTextColor(0xD399, TFT_BLACK);
                cek_data.drawString(String(timeSec), 250, -4);

                cek_data.setTextColor(0x35F9, TFT_BLACK);
                cek_data.setFreeFont(&FreeSans9pt7b);
                cek_data.drawString(String(month) + "  " + String(dayInMonth), 130, 32);

                cek_data.setTextColor(GRAY, TFT_BLACK);
                cek_data.setTextFont(0);
                cek_data.drawString("BATTERY:", 250, 34);

                cek_data.drawString(String(batteryPercentage, 0) + "%", 250, 46);
                cek_data.drawRoundRect(304, 30, 12, 136, 2, TFT_SILVER);

                seg = brightness / 24;
                for (int i = 0; i < seg; i++)
                    cek_data.fillRect(308, 150 - (i * 13), 4, 11, 0x35F9);
                cek_data.drawLine(cax - 10, cay - 10, cax + 152, cay - 10, GRAY);

                cek_data.pushImage(298, 0, 26, 26, bright);
                cek_data.pushSprite(0, 0);
                break;
            }
            case STATE_PAIRING: {
                pairing_mode.pushImage(0, 0, 320, 170, kotak);
                text_paring.fillSprite(BACKGROUND);
                text_paring.drawCentreString("Pairing Mode", 160, 55, 4);
                text_paring.drawCentreString("WiFi: " + String(DEVICE_NAME), 160, 90, 4);
                text_paring.drawCentreString("Back", 55, 20, 2);
                text_paring.drawCentreString("Start", 265, 20, 2);
                text_paring.drawCentreString(String(batteryPercentage, 0), 246, 5, 1);
                text_paring.pushToSprite(&pairing_mode, 0, 0, BACKGROUND);
                pairing_mode.pushSprite(0, 0);
                break;
            }
            case STATE_ALARM: {
                alarm_screen.pushImage(0, 0, 320, 170, alarmScreen);
                alarm_text.fillSprite(BACKGROUND);
                alarm_text.drawCentreString("Simpan", 258, 35, 2);
                alarm_text.drawCentreString("Hapus", 258, 128, 2);
                alarm_text.drawCentreString(String(batteryPercentage, 0), 246, 5, 1);
                alarm_text.pushToSprite(&alarm_screen, 0, 0, BACKGROUND);
                alarm_screen.pushSprite(0, 0);
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

void taskMainLogicCode(void *pvParameters) {
    HttpCommand httpCmd;
    for (;;) {
        ButtonEvent event;
        if (xQueueReceive(xButtonQueue, &event, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (xSemaphoreTake(xStateMutex, portMAX_DELAY) == pdTRUE) {
                switch (currentState) {
                    case STATE_PROFILE_SELECTION: {
                        if (event.eventType == 0) {  // Click
                            if (event.buttonId == 1) {
                                selectedProfile = profile1_code;
                            } else if (event.buttonId == 2) {
                                selectedProfile = profile2_code;
                            } else if (event.buttonId == 3) {
                                selectedProfile = profile3_code;
                            } else if (event.buttonId == 4) {
                                selectedProfile = profile4_code;
                            }
                            currentState = STATE_MEASURING;
                        } else if (event.eventType == 1 && event.buttonId == 3) {  // press stop
                            currentState = STATE_PAIRING;
                        }
                        break;
                    }
                    case STATE_MEASURING: {
                        if (event.eventType == 0) {  // Click
                            if (event.buttonId == 2) {
                                vTaskDelay(pdMS_TO_TICKS(1000));  // Wait for stabilization
                                if (xSemaphoreTake(xSensorDataMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
                                    httpCmd = {0, sensorData.weight, sensorData.roll, sensorData.pitch, selectedProfile};
                                    xSemaphoreGive(xSensorDataMutex);
                                }
                                xQueueSend(xHttpQueue, &httpCmd, 0);
                            } else if (event.buttonId == 1) {
                                currentState = STATE_PROFILE_SELECTION;
                            } else if (event.buttonId == 4) {
                                LoadCell.update();
                                LoadCell.tareNoDelay();
                            } else if (event.buttonId == 3) {
                                currentState = STATE_GYRO_TEST;
                            }
                        } else if (event.eventType == 1) {  // press stop
                            if (event.buttonId == 2) {
                                currentState = STATE_HISTORY;
                            } else if (event.buttonId == 3) {
                                currentState = STATE_PAIRING;
                            } else if (event.buttonId == 4) {
                                currentState = STATE_CALIBRATION;
                            }
                        }
                        break;
                    }
                    case STATE_GYRO_TEST: {
                        if (event.eventType == 0) {
                            if (event.buttonId == 1) {
                                currentState = STATE_MEASURING;
                            }
                        }
                        break;
                    }
                    case STATE_HISTORY: {
                        if (event.eventType == 0) {  // click
                            if (event.buttonId == 1) {
                                currentState = STATE_MEASURING;
                            } else if (event.buttonId == 2) {
                                if (brightness < 240) {
                                    brightness += 24;
                                    ledcSetup(0, 10000, 8);
                                    ledcAttachPin(38, 0);
                                    ledcWrite(0, brightness);
                                }
                            } else if (event.buttonId == 3) {
                                httpCmd = {1, sensorData.weight, sensorData.roll, sensorData.pitch, selectedProfile};
                                xQueueSend(xHttpQueue, &httpCmd, 0);
                            } else if (event.buttonId == 4) {
                                if (brightness > 24) {
                                    brightness -= 24;
                                    ledcSetup(0, 10000, 8);
                                    ledcAttachPin(38, 0);
                                    ledcWrite(0, brightness);
                                }
                            }
                        } else if (event.eventType == 1 && event.buttonId == 3) {
                            httpCmd = {2, sensorData.weight, sensorData.roll, sensorData.pitch, selectedProfile};
                            xQueueSend(xHttpQueue, &httpCmd, 0);
                        }
                        break;
                    }
                    case STATE_PAIRING: {
                        if (event.eventType == 0) {  // click
                            if (event.buttonId == 1) {
                                currentState = STATE_PROFILE_SELECTION;
                            } else if (event.buttonId == 2) {
                                wm.setConfigPortalTimeout(PORTAL_TIMEOUT);
                                if (!wm.startConfigPortal(DEVICE_NAME)) {
                                    Serial.println("failed to connect and hit timeout");
                                    esp_deep_sleep_start();
                                } else {
                                    pairing_mode.pushImage(0, 0, 320, 170, kotak);
                                    text_paring.fillSprite(BACKGROUND);
                                    text_paring.drawCentreString("Pairing Mode", 160, 55, 4);
                                    text_paring.drawCentreString("Connected!", 160, 90, 4);
                                    text_paring.pushToSprite(&pairing_mode, 0, 0, BACKGROUND);
                                    pairing_mode.pushSprite(0, 0);
                                    vTaskDelay(pdMS_TO_TICKS(2000));
                                    digitalWrite(PIN_BUZZER, HIGH);
                                    vTaskDelay(pdMS_TO_TICKS(100));
                                    digitalWrite(PIN_BUZZER, LOW);
                                    esp_deep_sleep_start();
                                }
                            }
                        }
                        break;
                    }
                    case STATE_CALIBRATION: {
                        if (event.eventType == 0) {  // click
                            if (event.buttonId == 1) {
                                currentState = STATE_MEASURING;
                            } else if (event.buttonId == 2) {
                                LoadCell.update();
                                float known_mass = 20.0;
                                vTaskDelay(pdMS_TO_TICKS(2000));

                                LoadCell.refreshDataSet();
                                calibrationValue = LoadCell.getNewCalibration(known_mass);

                                EEPROM.begin(512);
                                EEPROM.put(calVal_eepromAdress, calibrationValue);
                                EEPROM.commit();

                                LoadCell.setCalFactor(calibrationValue);

                                digitalWrite(PIN_BUZZER, HIGH);
                                vTaskDelay(pdMS_TO_TICKS(100));
                                digitalWrite(PIN_BUZZER, LOW);
                            } else if (event.buttonId == 3) {
                                calibrationValue = 1435.05;
                                EEPROM.begin(512);
                                EEPROM.put(calVal_eepromAdress, calibrationValue);
                                EEPROM.commit();

                                calibrationOffset = 8434937.00;
                                EEPROM.begin(512);
                                EEPROM.put(tareOffsetVal_eepromAdress, calibrationOffset);
                                EEPROM.commit();

                                LoadCell.setTareOffset(calibrationOffset);
                                LoadCell.setCalFactor(calibrationValue);

                                digitalWrite(PIN_BUZZER, HIGH);
                                vTaskDelay(pdMS_TO_TICKS(100));
                                digitalWrite(PIN_BUZZER, LOW);
                            } else if (event.buttonId == 4) {
                                LoadCell.update();
                                LoadCell.tareNoDelay();

                                calibrationOffset = 8434937.00;
                                EEPROM.begin(512);
                                EEPROM.put(tareOffsetVal_eepromAdress, calibrationOffset);
                                EEPROM.commit();

                                LoadCell.setTareOffset(calibrationOffset);

                                LoadCell.setCalFactor(1.0);
                                digitalWrite(PIN_BUZZER, HIGH);
                                vTaskDelay(pdMS_TO_TICKS(100));
                                digitalWrite(PIN_BUZZER, LOW);
                            }
                        }
                        break;
                    }
                    case STATE_ALARM: {
                        if (event.eventType == 0) {  // click
                            if (event.buttonId == 2) {
                                _isAware = true;
                                currentState = STATE_MEASURING;
                            } else if (event.buttonId == 4) {
                                _isAware = true;
                                httpCmd = {1, sensorData.weight, sensorData.roll, sensorData.pitch, selectedProfile};
                                xQueueSend(xHttpQueue, &httpCmd, 0);
                                currentState = STATE_MEASURING;
                            }
                        }
                        break;
                    }
                }
                xSemaphoreGive(xStateMutex);
            } else {
                Serial.println("Failed to acquire mutex");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void taskHttpRequestCode(void *pvParameters) {
    HTTPClient http;
    String baseUrl = SOCKET_ADDRESS;
    bool _isFetched = false;
    for (;;) {
        // Periodic profile names and logs fetch
        if (WiFi.status() == WL_CONNECTED && (currentState == STATE_PROFILE_SELECTION || currentState == STATE_HISTORY)) {
            // Fetch profile names
            http.begin(baseUrl + "profile/get_name?profile1=" + profile1_code + "&profile2=" + profile2_code + "&profile3=" + profile3_code + "&profile4=" + profile4_code);
            http.addHeader("x-device-name", DEVICE_NAME);
            http.addHeader("x-device-password", DEVICE_PASSWORD);
            int code = http.GET();
            if (code == 200) {
                String payload = http.getString();
                JsonDocument doc;
                deserializeJson(doc, payload);
                JsonObject data = doc["data"];
                for (int i = 0; i < 4; i++) {
                    String key = "name_profile" + String(i + 1);
                    profileNames[i] = data[key] | profileNames[i];
                }
            } else {
                Serial.println("Failed to fetch profile names: " + String(code));
            }
            http.end();

            // Fetch Daily logs
            http.begin(baseUrl + "log/get_logs?profile1=" + profile1_code + "&profile2=" + profile2_code + "&profile3=" + profile3_code + "&profile4=" + profile4_code);
            http.addHeader("x-device-name", DEVICE_NAME);
            http.addHeader("x-device-password", DEVICE_PASSWORD);
            code = http.GET();
            if (code == 200) {
                String payload = http.getString();
                JsonDocument doc;
                deserializeJson(doc, payload);
                JsonObject data = doc["data"];
                totalNasi[0] = data["weight_" + profile1_code] | 0.0;
                totalNasi[1] = data["weight_" + profile2_code] | 0.0;
                totalNasi[2] = data["weight_" + profile3_code] | 0.0;
                totalNasi[3] = data["weight_" + profile4_code] | 0.0;
            } else {
                Serial.println("Failed to fetch logs: " + String(code));
            }
            http.end();

            // Fetch maxNasi
            http.begin(baseUrl + "profile/get_maxnasi?profile1=" + profile1_code + "&profile2=" + profile2_code + "&profile3=" + profile3_code + "&profile4=" + profile4_code);
            http.addHeader("x-device-name", DEVICE_NAME);
            http.addHeader("x-device-password", DEVICE_PASSWORD);
            code = http.GET();
            if (code == 200) {
                String payload = http.getString();
                JsonDocument doc;
                deserializeJson(doc, payload);
                JsonObject data = doc["data"];
                maxNasi[0] = data["maxnasi_profile1"] | 0.0;
                maxNasi[1] = data["maxnasi_profile2"] | 0.0;
                maxNasi[2] = data["maxnasi_profile3"] | 0.0;
                maxNasi[3] = data["maxnasi_profile4"] | 0.0;
            } else {
                Serial.println("Failed to fetch maxnasi: " + String(code));
            }
            http.end();

            vTaskDelay(pdMS_TO_TICKS(1000));
        }

        // Handle HTTP commands
        HttpCommand cmd;
        if (xQueueReceive(xHttpQueue, &cmd, pdMS_TO_TICKS(1000)) == pdTRUE) {
            if (WiFi.status() == WL_CONNECTED) {
                if (cmd.type == 0) {  // Create log
                    http.begin(baseUrl + "log/create");
                    http.addHeader("Content-Type", "application/json");
                    http.addHeader("x-device-name", DEVICE_NAME);
                    http.addHeader("x-device-password", DEVICE_PASSWORD);
                    String payload = "{\"weight\":" + String(cmd.weight, 2) + ",\"roll\":" + String(cmd.roll, 2) + ",\"pitch\":" + String(cmd.pitch, 2) + ",\"profile_code\":\"" + cmd.profile_code + "\"}";

                    int code = http.POST(payload);
                    if (code == 201) {
                        digitalWrite(PIN_BUZZER, HIGH);
                        vTaskDelay(pdMS_TO_TICKS(100));
                        digitalWrite(PIN_BUZZER, LOW);
                    } else {
                        for (int i = 0; i < 3; i++) {
                            digitalWrite(PIN_BUZZER, HIGH);
                            vTaskDelay(pdMS_TO_TICKS(150));
                            digitalWrite(PIN_BUZZER, LOW);
                            vTaskDelay(pdMS_TO_TICKS(100));
                        }
                    }
                    http.end();
                } else if (cmd.type == 1) {  // Delete latest
                    http.begin(baseUrl + "log/delete_latest?profile=" + cmd.profile_code);
                    http.addHeader("x-device-name", DEVICE_NAME);
                    http.addHeader("x-device-password", DEVICE_PASSWORD);
                    int code = http.sendRequest("DELETE");
                    if (code == 204) {
                        digitalWrite(PIN_BUZZER, HIGH);
                        vTaskDelay(pdMS_TO_TICKS(100));
                        digitalWrite(PIN_BUZZER, LOW);
                    } else {
                        for (int i = 0; i < 3; i++) {
                            digitalWrite(PIN_BUZZER, HIGH);
                            vTaskDelay(pdMS_TO_TICKS(150));
                            digitalWrite(PIN_BUZZER, LOW);
                            vTaskDelay(pdMS_TO_TICKS(100));
                        }
                    }
                    http.end();
                } else if (cmd.type == 2) {  // Delete all today
                    http.begin(baseUrl + "log/delete_all_today?profile=" + cmd.profile_code);
                    http.addHeader("x-device-name", DEVICE_NAME);
                    http.addHeader("x-device-password", DEVICE_PASSWORD);
                    int code = http.sendRequest("DELETE");
                    if (code == 204) {
                        digitalWrite(PIN_BUZZER, HIGH);
                        vTaskDelay(pdMS_TO_TICKS(100));
                        digitalWrite(PIN_BUZZER, LOW);
                    } else {
                        for (int i = 0; i < 3; i++) {
                            digitalWrite(PIN_BUZZER, HIGH);
                            vTaskDelay(pdMS_TO_TICKS(150));
                            digitalWrite(PIN_BUZZER, LOW);
                            vTaskDelay(pdMS_TO_TICKS(100));
                        }
                    }
                    http.end();
                } else if (cmd.type == 3) {  // research only for dataset
                    http.begin(baseUrl + "research/create");
                    http.addHeader("Content-Type", "application/json");
                    http.addHeader("x-device-name", DEVICE_NAME);
                    http.addHeader("x-device-password", DEVICE_PASSWORD);
                    String payload = "{\"weight\":" + String(cmd.weight, 2) + ",\"roll\":" + String(cmd.roll, 2) + ",\"pitch\":" + String(cmd.pitch, 2) + ",\"profile_code\":\"" + cmd.profile_code + "\"}";

                    int code = http.POST(payload);
                    if (code == 201) {
                        Serial.println("Research data sent successfully!");
                        ledcSetup(1, 5000, 8);  // Set up PWM channel with lower frequency
                        ledcAttachPin(PIN_BUZZER, 1);
                        ledcWrite(1, 100);  // Set duty cycle to a lower value for softer sound
                        vTaskDelay(pdMS_TO_TICKS(100));
                        ledcWrite(1, 0);  // Turn off the buzzer
                    } else {
                        for (int i = 0; i < 3; i++) {
                            digitalWrite(PIN_BUZZER, HIGH);
                            vTaskDelay(pdMS_TO_TICKS(150));
                            digitalWrite(PIN_BUZZER, LOW);
                            vTaskDelay(pdMS_TO_TICKS(100));
                        }
                    }
                    http.end();
                }
            } else {
                Serial.println("WiFi Disconnected!");
            }
        }
    }
}

void taskDeepSleepCode(void *pvParameters) {
    for (;;) {
        ButtonEvent event;
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if (h_taskMainLogic) vTaskSuspend(h_taskMainLogic);
        if (h_taskSensorRead) vTaskSuspend(h_taskSensorRead);
        if (h_taskButtonCheck) vTaskSuspend(h_taskButtonCheck);
        if (h_taskDisplayUpdate) vTaskSuspend(h_taskDisplayUpdate);
        if (h_taskHttpRequest) vTaskSuspend(h_taskHttpRequest);
        LoadCell.powerDown();
        Wire.end();
        Serial.end();
        digitalWrite(PIN_LCD_BL, LOW);
        digitalWrite(PIN_POWER_ON, LOW);
        detachInterrupt(digitalPinToInterrupt(PIN_HX711_DATA));
        esp_deep_sleep_start();
    }
}

void IRAM_ATTR resetESP() {
    detachInterrupt(digitalPinToInterrupt(PUSH_BUTTON_3));
    detachInterrupt(digitalPinToInterrupt(PIN_HX711_DATA));

    ESP.restart();
}

void IRAM_ATTR dataReadyISR() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(xDataReadySemaphore, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
}

String getStateName(AppState state) {
    switch (state) {
        case STATE_PROFILE_SELECTION:
            return "PROFILE SELECTION";
        case STATE_MEASURING:
            return "MEASURING";
        case STATE_GYRO_TEST:
            return "GYRO TEST";
        case STATE_CALIBRATION:
            return "CALIBRATION";
        case STATE_PAIRING:
            return "PAIRING";
        case STATE_ALARM:
            return "ALARM";
        case STATE_HISTORY:
            return "HISTORY";
        default:
            return "Unknown";
    }
}