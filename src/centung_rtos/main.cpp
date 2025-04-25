#include <Arduino.h>
#include "config.h"  // Updated from pin_config.h
#include "library.h" // Includes all necessary libraries and headers

// Firebase Configuration
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

// Global Variables
String kodeUnik1 = PROFILE_CODE_1;
String kodeUnik2 = PROFILE_CODE_2;
String kodeUnik3 = PROFILE_CODE_3;
String kodeUnik4 = PROFILE_CODE_4;

const int calVal_eepromAdress = 0;
const int tareOffsetVal_eepromAdress = 20;
float calibrationValue = 2230.0; // Example value; adjust based on calibration
float calibrationOffset = 0.0;
String devId;
String pengguna1 = "User1", pengguna2 = "User2", pengguna3 = "User3", pengguna4 = "User4"; // Example names
double KNasi1 = 0, KNasi2 = 0, KNasi3 = 0, KNasi4 = 0;
double cache1 = 0, cache2 = 0, cache3 = 0, cache4 = 0;
double KNasiSekarang = 0, KNasi_profile1 = 0, KNasi_profile2 = 0, KNasi_profile3 = 0, KNasi_profile4 = 0;
double max_nasi = 1000.0; // Example max weight in grams
double berat = 0;
int ulang = 0, state = 0, cek_berat = 0;
int timeout = 30;
int centong = 0, adc = 0, simpan = 0, done_query = 0, waitUntilClick = 0;
unsigned long stabilizingtime = 2000;
boolean _tare = true;

// MPU6050 Variables
const uint8_t MPU6050_DEFAULT_ADDRESS = 0x68;
const float MPU_ACCEL_SCALE = 4096.0;  // ±8g
const float MPU_GYRO_SCALE = 65.5;     // ±500°/s
float gyroBias[3] = {0, 0, 0};
float accelBias[3] = {0, 0, 0};

// Kalman Filter Variables
float KalmanAngleRoll = 0, KalmanAnglePitch = 0;
float KalmanUncertaintyAngleRoll = 4.0; // Initial uncertainty
float KalmanUncertaintyAnglePitch = 4.0;

// Smoothing Variables
const int SMOOTHING_WINDOW_SIZE = 10;
float rollWindow[SMOOTHING_WINDOW_SIZE];
float pitchWindow[SMOOTHING_WINDOW_SIZE];
int smoothingIndex = 0;

// Shared Variables (Protected by Mutex)
volatile float smoothedRoll = 0.0;
volatile float smoothedPitch = 0.0;
volatile float currentWeight = 0.0;
volatile bool newDataReady = false;

// RTOS Handles
TaskHandle_t mpuTaskHandle = NULL;
TaskHandle_t hx711TaskHandle = NULL;
TaskHandle_t buttonTaskHandle = NULL;
TaskHandle_t displayTaskHandle = NULL;
TaskHandle_t firebaseTaskHandle = NULL;

SemaphoreHandle_t hx711DataReadySemaphore = NULL;
SemaphoreHandle_t mutexSensorData = NULL;
QueueHandle_t buttonEventQueue = NULL;
QueueHandle_t displayDataQueue = NULL;
QueueHandle_t firebaseDataQueue = NULL;

// Hardware Instances
HX711_ADC LoadCell(PIN_HX711_DATA, PIN_HX711_CLOCK);
WiFiManager wm;
OneButton button1(PUSH_BUTTON_1, true);
OneButton button2(PUSH_BUTTON_2, true);
OneButton button3(PUSH_BUTTON_3, true);
OneButton button4(PUSH_BUTTON_4, true);
TFT_eSPI tft = TFT_eSPI();

// Sprite Instances (Defined in library.h screens)
TFT_eSprite loading_screen = TFT_eSprite(&tft);
TFT_eSprite select_profile = TFT_eSprite(&tft);
TFT_eSprite text_profile = TFT_eSprite(&tft);
TFT_eSprite timbangan_screen = TFT_eSprite(&tft);
TFT_eSprite text_timbangan = TFT_eSprite(&tft);
TFT_eSprite alarm_screen = TFT_eSprite(&tft);
TFT_eSprite alarm_text = TFT_eSprite(&tft);

// Utility Functions
void getLocalTime() {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) return;

    strftime(timeHour, 3, "%H", &timeinfo);
    strftime(timeMin, 3, "%M", &timeinfo);
    strftime(timeSec, 3, "%S", &timeinfo);
    strftime(timeWeekDay, 10, "%A", &timeinfo);

    String InWeek = String(timeWeekDay);
    for (int i = 0; i < 7; i++) {
        if (InWeek == SDay[i]) dayInWeek = i;
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
            if (tanggal <= 0) {
                tanggal = mm[i > 0 ? i - 1 : 11];
                bulan = i > 0 ? i : 12;
                if (bulan == 12 && i == 0) String(year) = "2023";
            }
        }
    }
}

String generateRandomString() {
    String randomString = "";
    const char characters[] = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
    for (int i = 0; i < 20; i++) {
        randomString += characters[random(sizeof(characters) - 1)];
    }
    return randomString;
}

// MPU6050 Functions
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
    Wire.beginTransmission(address);
    Wire.write(subAddress);
    Wire.write(data);
    Wire.endTransmission();
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t* dest) {
    Wire.beginTransmission(address);
    Wire.write(subAddress);
    Wire.endTransmission(false);
    Wire.requestFrom(address, count);
    for (uint8_t i = 0; i < count && Wire.available(); i++) {
        dest[i] = Wire.read();
    }
}

void initMPU6050() {
    writeByte(MPU6050_DEFAULT_ADDRESS, 0x6B, 0x00); // Wake up
    writeByte(MPU6050_DEFAULT_ADDRESS, 0x6B, 0x01); // PLL with X gyro
    writeByte(MPU6050_DEFAULT_ADDRESS, 0x1B, 0x08); // ±500°/s
    writeByte(MPU6050_DEFAULT_ADDRESS, 0x1C, 0x10); // ±8g
    writeByte(MPU6050_DEFAULT_ADDRESS, 0x1A, 0x00); // DLPF 260Hz
}

void calibrateMPU6050() {
    float RateRollSum = 0, RatePitchSum = 0, RateYawSum = 0;
    float AngleRoll, AnglePitch;
    const int samples = 2000;

    for (int i = 0; i < samples; i++) {
        float RateRoll, RatePitch, RateYaw;
        readMPU6050(RateRoll, RatePitch, RateYaw, AngleRoll, AnglePitch);
        RateRollSum += RateRoll;
        RatePitchSum += RatePitch;
        RateYawSum += RateYaw;
        delay(3);
    }

    gyroBias[0] = RateRollSum / samples;
    gyroBias[1] = RatePitchSum / samples;
    gyroBias[2] = RateYawSum / samples;
}

void readMPU6050(float &RateRoll, float &RatePitch, float &RateYaw, float &AngleRoll, float &AnglePitch) {
    uint8_t data[14];
    readBytes(MPU6050_DEFAULT_ADDRESS, 0x3B, 14, data);

    int16_t AcX = (data[0] << 8) | data[1];
    int16_t AcY = (data[2] << 8) | data[3];
    int16_t AcZ = (data[4] << 8) | data[5];
    int16_t GyX = (data[8] << 8) | data[9];
    int16_t GyY = (data[10] << 8) | data[11];
    int16_t GyZ = (data[12] << 8) | data[13];

    float AccX = (float)AcX / MPU_ACCEL_SCALE;
    float AccY = (float)AcY / MPU_ACCEL_SCALE;
    float AccZ = (float)AcZ / MPU_ACCEL_SCALE;

    AngleRoll = -atan2(AccX, sqrt(AccY * AccY + AccZ * AccZ)) * (180.0 / PI);
    AnglePitch = atan2(AccY, sqrt(AccX * AccX + AccZ * AccZ)) * (180.0 / PI);

    RateRoll = (float)GyY / MPU_GYRO_SCALE;  // Adjusted to match physical axes
    RatePitch = (float)GyX / MPU_GYRO_SCALE;
    RateYaw = (float)GyZ / MPU_GYRO_SCALE;
}

void kalman_1d(float &KalmanState, float &KalmanUncertainty, float Rate, float Measurement) {
    float dt = 0.004; // 4ms loop time
    KalmanState += dt * Rate;
    KalmanUncertainty += dt * dt * 4 * 4; // Process noise

    float R = 9.0; // Measurement noise
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

    if (xSemaphoreTake(mutexSensorData, pdMS_TO_TICKS(10)) == pdTRUE) {
        smoothedRoll = rollSum / SMOOTHING_WINDOW_SIZE;
        smoothedPitch = pitchSum / SMOOTHING_WINDOW_SIZE;
        xSemaphoreGive(mutexSensorData);
    }
}

// ISR
void IRAM_ATTR dataReadyISR() {
    if (LoadCell.update()) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(hx711DataReadySemaphore, &xHigherPriorityTaskWoken);
        if (xHigherPriorityTaskWoken) portYIELD_FROM_ISR();
    }
}

void IRAM_ATTR resetESP() {
    ESP.restart();
}

// FreeRTOS Tasks
void mpuTask(void *pvParameters) {
    initMPU6050();
    calibrateMPU6050();

    float RateRoll, RatePitch, RateYaw;
    float AngleRoll, AnglePitch;

    readMPU6050(RateRoll, RatePitch, RateYaw, AngleRoll, AnglePitch);
    KalmanAngleRoll = AngleRoll;
    KalmanAnglePitch = AnglePitch;

    for (int i = 0; i < SMOOTHING_WINDOW_SIZE; i++) {
        rollWindow[i] = KalmanAngleRoll;
        pitchWindow[i] = KalmanAnglePitch;
    }

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(4);

    for (;;) {
        readMPU6050(RateRoll, RatePitch, RateYaw, AngleRoll, AnglePitch);
        RateRoll -= gyroBias[0];
        RatePitch -= gyroBias[1];

        kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
        kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
        smoothAngles(KalmanAngleRoll, KalmanAnglePitch);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void hx711Task(void *pvParameters) {
    LoadCell.begin();
    LoadCell.start(stabilizingtime, _tare);
    LoadCell.setCalFactor(calibrationValue);

    for (;;) {
        if (xSemaphoreTake(hx711DataReadySemaphore, portMAX_DELAY) == pdTRUE) {
            if (LoadCell.update()) {
                float weight = LoadCell.getData();
                weight = round(weight * 100) / 100.0;
                if (weight < 0) weight = 0;

                if (xSemaphoreTake(mutexSensorData, pdMS_TO_TICKS(10)) == pdTRUE) {
                    currentWeight = weight;
                    xSemaphoreGive(mutexSensorData);
                }
            }
        }
    }
}

void buttonTask(void *pvParameters) {
    struct ButtonEvent {
        int button;
        int action; // 1 = click, 2 = long press, 3 = double click
    };

    button1.attachClick([]() {
        ButtonEvent evt = {1, 1};
        xQueueSend(buttonEventQueue, &evt, portMAX_DELAY);
    });
    button1.attachLongPressStart([]() {
        ButtonEvent evt = {1, 2};
        xQueueSend(buttonEventQueue, &evt, portMAX_DELAY);
    });

    button2.attachClick([]() {
        ButtonEvent evt = {2, 1};
        xQueueSend(buttonEventQueue, &evt, portMAX_DELAY);
    });

    button3.attachClick([]() {
        ButtonEvent evt = {3, 1};
        xQueueSend(buttonEventQueue, &evt, portMAX_DELAY);
    });

    button4.attachClick([]() {
        ButtonEvent evt = {4, 1};
        xQueueSend(buttonEventQueue, &evt, portMAX_DELAY);
    });
    button4.attachDoubleClick([]() {
        ButtonEvent evt = {4, 3};
        xQueueSend(buttonEventQueue, &evt, portMAX_DELAY);
    });

    for (;;) {
        ButtonEvent evt;
        if (xQueueReceive(buttonEventQueue, &evt, portMAX_DELAY) == pdTRUE) {
            if (evt.action == 1) { // Click
                if (evt.button == 1) devId = kodeUnik2;
                else if (evt.button == 2) devId = kodeUnik4;
                else if (evt.button == 3) devId = kodeUnik1;
                else if (evt.button == 4) devId = kodeUnik3;
                cek_berat = 1;
                ulang = 1;
            }
            // Expand for long press and double click as needed
        }
    }
}

void displayTask(void *pvParameters) {
    struct DisplayData {
        float weight;
        float roll;
        float pitch;
        float voltage;
    };

    tft.init();
    tft.setRotation(3);
    tft.setSwapBytes(true);
    loading_screen.createSprite(320, 170);
    select_profile.createSprite(320, 170);
    text_profile.createSprite(320, 170);
    timbangan_screen.createSprite(320, 170);
    text_timbangan.createSprite(320, 170);
    alarm_screen.createSprite(320, 170);
    alarm_text.createSprite(320, 170);

    for (;;) {
        DisplayData data;
        if (xQueueReceive(displayDataQueue, &data, pdMS_TO_TICKS(100)) == pdTRUE) {
            select_profile.fillSprite(PROFILE_BACKGROUND);
            text_profile.fillSprite(PROFILE_BACKGROUND);
            text_profile.drawCentreString(String(data.voltage), 246, 5, 1);
            text_profile.drawCentreString(pengguna1, 90, 27, 2);
            text_profile.pushToSprite(&select_profile, 0, 0, PROFILE_BACKGROUND);
            select_profile.pushSprite(0, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void firebaseTask(void *pvParameters) {
    struct FirebaseDataStruct {
        float weight;
        String deviceId;
    };

    // Placeholder Firebase config; define these in config.h or secrets file
    config.api_key = "YOUR_API_KEY";
    auth.user.email = "YOUR_EMAIL";
    auth.user.password = "YOUR_PASSWORD";
    config.database_url = "YOUR_DATABASE_URL";
    Firebase.begin(&config, &auth);

    for (;;) {
        FirebaseDataStruct data;
        if (xQueueReceive(firebaseDataQueue, &data, portMAX_DELAY) == pdTRUE) {
            FirebaseJson content;
            String doc_id = generateRandomString();
            String documentPath = "log_centung/" + doc_id;
            content.set("fields/berat_nasi/doubleValue", data.weight);
            content.set("fields/device_id/stringValue", data.deviceId);
            Firebase.Firestore.createDocument(&fbdo, "YOUR_PROJECT_ID", "", documentPath.c_str(), content.raw());
        }
    }
}

// Setup
void setup() {
    Serial.begin(115200);
    Wire.begin(PIN_IIC_SDA, PIN_IIC_SCL);
    Wire.setClock(400000);

    pinMode(PIN_POWER_ON, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_POWER_ON, HIGH);

    // Initialize RTOS objects
    hx711DataReadySemaphore = xSemaphoreCreateBinary();
    mutexSensorData = xSemaphoreCreateMutex();
    buttonEventQueue = xQueueCreate(10, sizeof(struct ButtonEvent));
    displayDataQueue = xQueueCreate(10, sizeof(struct DisplayData));
    firebaseDataQueue = xQueueCreate(10, sizeof(struct FirebaseDataStruct));

    // WiFi Setup
    wm.setConfigPortalTimeout(timeout);
    if (!wm.autoConnect("CENTUNG")) ESP.restart();

    // Attach Interrupts
    attachInterrupt(digitalPinToInterrupt(PIN_HX711_DATA), dataReadyISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(PUSH_BUTTON_3), resetESP, FALLING);

    // Create Tasks
    xTaskCreatePinnedToCore(mpuTask, "MPU Task", 4096, NULL, 2, &mpuTaskHandle, 1);
    xTaskCreatePinnedToCore(hx711Task, "HX711 Task", 4096, NULL, 3, &hx711TaskHandle, 1);
    xTaskCreatePinnedToCore(buttonTask, "Button Task", 2048, NULL, 1, &buttonTaskHandle, 1);
    xTaskCreatePinnedToCore(displayTask, "Display Task", 4096, NULL, 1, &displayTaskHandle, 0);
    xTaskCreatePinnedToCore(firebaseTask, "Firebase Task", 8192, NULL, 1, &firebaseTaskHandle, 0);

    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000)); // Minimal loop
}