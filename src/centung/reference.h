#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <freertos/task.h>

#include "library.h"  // Assuming this includes necessary libraries (e.g., HX711, Firebase, TFT_eSPI)
#include "pin_config.h"

// Firebase config
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

FirebaseJson query1;
FirebaseJson query2;

// Hardware instances
HX711_ADC LoadCell(PIN_HX711_DATA, PIN_HX711_CLOCK);
WiFiManager wm;
TFT_eSPI tft = TFT_eSPI();
OneButton button1(PUSH_BUTTON_1, true);
OneButton button2(PUSH_BUTTON_2, true);
OneButton button3(PUSH_BUTTON_3, true);
OneButton button4(PUSH_BUTTON_4, true);

// Sprite declarations (assuming defined in library.h)
extern TFT_eSprite loading_screen, progress_bar, select_profile, text_profile;
extern TFT_eSprite tunggu_nimbang, timbangan_screen, arc, text_timbangan;
extern TFT_eSprite pairing_mode, text_paring, calib_screen, calib_text;
extern TFT_eSprite position_screen, position_text, cek_data, alarm_screen, alarm_text;

// Global variables
String kodeUnik1 = "A1", kodeUnik2 = "B1", kodeUnik3 = "C1", kodeUnik4 = "D1";
const int calVal_eepromAdress = 0, tareOffsetVal_eepromAdress = 20;
volatile boolean newDataReady = false;
float calibrationValue, calibrationOffset;
String devId, pengguna1, pengguna2, pengguna3, pengguna4;
double KNasi1, KNasi2, KNasi3, KNasi4;
double cache1, cache2, cache3, cache4;
double KNasiSekarang, KNasi_profile1, KNasi_profile2, KNasi_profile3, KNasi_profile4;
double max_nasi, berat;
int ulang = 0, state = 0, cek_berat = 0, timeout = 30;
int centong = 0, adc = 0, simpan = 0, done_query = 0, waitUntilClick = 0;
int tanggal = 0, bulan = 0;
bool res;
unsigned long stabilizingtime = 2000;
boolean _tare = true;
float volt;
int brightness = 128;  // Example value

// IMU variables
float twoKp = twoKpDef, twoKi = twoKiDef;
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
volatile float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;
long sampling_timer;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
float axg, ayg, azg, gxrs, gyrs, gzrs;
float roll, pitch, yaw;
float SelfTest[6], gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};
float sampleFreq = 0.0f;
uint32_t lastUpdate = 0, firstUpdate = 0, Now = 0;

// RTOS handles
SemaphoreHandle_t xDataReadySemaphore, xSensorDataMutex, xStateMutex, xI2CMutex, xFirebaseMutex;
QueueHandle_t xButtonQueue, xFirebaseQueue;

// Enums and structs
enum AppState {
    STATE_PROFILE_SELECTION,
    STATE_CHECK_ORIENTATION,
    STATE_MEASURING,
    STATE_CALIBRATION,
    STATE_CHECK_WEIGHT,
    STATE_DELETE_DOC,
    STATE_PAIRING,
    STATE_ALARM,
    STATE_ERROR
};

struct SensorData {
    float weight;
    float roll;
    float pitch;
    float yaw;
};

struct ButtonEvent {
    int buttonId;
    int eventType;  // 0: click, 1: long press, 2: double click
};

struct FirebaseCommand {
    int type;  // 0: upload measurement, 1: initial query
    double weight;
    String deviceId;
};

// Shared variables
AppState currentState = STATE_PROFILE_SELECTION;
SensorData sensorData;
String selectedProfile;

// I2C read/write functions (unchanged from original)
void writeByte(uint8_t address, uint8_t subAddress, uint8_t data) { /* ... */ }
uint8_t readByte(uint8_t address, uint8_t subAddress) { /* ... */ }
void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t *dest) { /* ... */ }

// MPU6050 functions (unchanged)
void MPU6050_Init() { /* ... */ }
void mpu6050_GetData() { /* ... */ }
void mpu6050_updateQuaternion() { /* ... */ }
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) { /* ... */ }
void mpu6050_getRollPitchYaw() { /* ... */ }
void MPU6050SelfTest(float *destination) { /* ... */ }
void calibrateMPU6050(float *dest1, float *dest2) { /* ... */ }

// Utility functions
String generateRandomString() { /* ... */ }
String extractDocumentId(String documentName) { /* ... */ }
void deleteDocument(String documentId) { /* ... */ }
void getLocalTime() { /* ... */ }  // Simplified for RTOS, assume it updates global time variables

void get_query() {
    // This will be called by the Firebase task initially
    // Implementation as in original, updates pengguna1-4, KNasi1-4, KNasi_profile1-4
    // Assume it modifies global variables directly
}

void IRAM_ATTR dataReadyISR() {
    if (LoadCell.update()) {
        xSemaphoreGiveFromISR(xDataReadySemaphore, NULL);
    }
}

void IRAM_ATTR resetESP() {
    ESP.restart();
}

void taskSensorRead(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1) {
        if (xSemaphoreTake(xDataReadySemaphore, portMAX_DELAY) == pdTRUE) {
            float weight = LoadCell.getData();

            if (xSemaphoreTake(xI2CMutex, portMAX_DELAY) == pdTRUE) {
                mpu6050_GetData();
                mpu6050_updateQuaternion();
                Now = micros();
                sampleFreq = (1000000.0f / (Now - lastUpdate));
                lastUpdate = Now;
                MahonyAHRSupdateIMU(gxrs, gyrs, gzrs, axg, ayg, azg);
                mpu6050_getRollPitchYaw();
                xSemaphoreGive(xI2CMutex);
            }

            if (xSemaphoreTake(xSensorDataMutex, portMAX_DELAY) == pdTRUE) {
                sensorData.weight = weight < 0 ? 0 : weight;
                sensorData.roll = roll;
                sensorData.pitch = pitch;
                sensorData.yaw = yaw;
                xSemaphoreGive(xSensorDataMutex);
            }
        }
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));  // 10ms period
    }
}

void taskDisplayUpdate(void *pvParameters) {
    while (1) {
        AppState state;
        if (xSemaphoreTake(xStateMutex, portMAX_DELAY) == pdTRUE) {
            state = currentState;
            xSemaphoreGive(xStateMutex);
        }

        adc = analogRead(PIN_BAT_VOLT);
        volt = ((((float)adc / 4095.0) * 2.0 * 3.3 * (1100 / 1000)) - 3.2) * 100 / (4.2 - 3.2);
        if (volt > 100) volt = 100;

        switch (state) {
            case STATE_PROFILE_SELECTION:
                select_profile.pushImage(0, 0, 320, 170, profile);
                text_profile.fillSprite(PROFILE_BACKGROUND);
                text_profile.drawCentreString(String(volt), 246, 5, 1);
                text_profile.drawCentreString(pengguna1, 90, 27, 2);
                text_profile.drawCentreString(pengguna2, 250, 27, 2);
                text_profile.drawCentreString(pengguna3, 90, 125, 2);
                text_profile.drawCentreString(pengguna4, 250, 125, 2);
                text_profile.pushToSprite(&select_profile, 0, 0, PROFILE_BACKGROUND);
                select_profile.pushSprite(0, 0);
                break;

            case STATE_CHECK_ORIENTATION:
                position_screen.pushImage(0, 0, 320, 170, datar);
                position_text.fillSprite(0xE73C);
                position_text.drawCentreString(String(volt), 246, 5, 1);
                if (xSemaphoreTake(xSensorDataMutex, portMAX_DELAY) == pdTRUE) {
                    position_text.drawCentreString(String(sensorData.roll), 90, 95, 4);
                    position_text.drawCentreString(String(sensorData.pitch), 225, 95, 4);
                    xSemaphoreGive(xSensorDataMutex);
                }
                position_text.pushToSprite(&position_screen, 0, 0, 0xE73C);
                position_screen.pushSprite(0, 0);
                break;

            case STATE_MEASURING:
                if (state == 0) {  // Waiting state
                    tunggu_nimbang.pushImage(0, 0, 320, 170, timbangan);
                    text_timbangan.fillSprite(0xE73C);
                    text_timbangan.drawCentreString(devId, 15, 5, 2);
                    text_timbangan.drawCentreString("Ambil nasi sekarang", 160, 5, 2);
                    text_timbangan.drawCentreString(String(volt), 246, 5, 1);
                    text_timbangan.pushToSprite(&tunggu_nimbang, 0, 0, 0xE73C);
                    tunggu_nimbang.pushSprite(0, 0);
                } else {  // Measurement taken
                    int end_angle = (berat * 180 / 300) + 90;
                    timbangan_screen.pushImage(0, 0, 320, 170, timbangan);
                    arc.fillSprite(0xE73C);
                    text_timbangan.fillSprite(0xE73C);
                    arc.drawSmoothArc(160, 153, 90, 80, 89, end_angle, TFT_CYAN, 0xE73C, true);
                    text_timbangan.drawCentreString(String(berat), 160, 90, 6);
                    text_timbangan.drawCentreString("Penimbang Berat", 160, 5, 2);
                    text_timbangan.drawCentreString(devId, 15, 5, 2);
                    text_timbangan.drawCentreString(String(volt), 246, 5, 1);
                    arc.pushToSprite(&timbangan_screen, 0, 0, 0xE73C);
                    text_timbangan.pushToSprite(&timbangan_screen, 0, 0, 0xE73C);
                    timbangan_screen.pushSprite(0, 0);
                }
                break;

            // Add cases for STATE_CALIBRATION, STATE_CHECK_WEIGHT, etc., similarly
            case STATE_ALARM:
                alarm_screen.pushImage(0, 0, 320, 170, alarmScreen);
                alarm_text.fillSprite(0xE73C);
                alarm_text.drawCentreString("Simpan", 258, 35, 2);
                alarm_text.drawCentreString("Hapus", 258, 128, 2);
                alarm_text.drawCentreString(String(volt), 246, 5, 1);
                alarm_text.pushToSprite(&alarm_screen, 0, 0, 0xE73C);
                alarm_screen.pushSprite(0, 0);
                break;

                // Implement other states as needed
        }
        vTaskDelay(pdMS_TO_TICKS(100));  // Update every 100ms
    }
}

void taskButtonCheck(void *pvParameters) {
    while (1) {
        button1.tick();
        button2.tick();
        button3.tick();
        button4.tick();
        vTaskDelay(pdMS_TO_TICKS(10));  // Check every 10ms
    }
}

void taskFirebaseComms(void *pvParameters) {
    while (1) {
        FirebaseCommand cmd;
        if (xQueueReceive(xFirebaseQueue, &cmd, portMAX_DELAY) == pdTRUE) {
            if (xSemaphoreTake(xFirebaseMutex, portMAX_DELAY) == pdTRUE) {
                if (cmd.type == 0) {  // Upload measurement
                    String doc_id = generateRandomString();
                    String documentPath = "log_centung/" + doc_id;
                    FirebaseJson content;
                    content.set("fields/berat_nasi/doubleValue", cmd.weight);
                    content.set("fields/device_id/stringValue", cmd.deviceId);
                    if (Firebase.Firestore.createDocument(&fbdo, FIREBASE_PROJECT_ID, "", documentPath.c_str(), content.raw())) {
                        Serial.printf("ok\n%s\n\n", fbdo.payload().c_str());
                    } else {
                        Serial.println(fbdo.errorReason());
                    }
                    // Add timestamp transform as in original
                } else if (cmd.type == 1) {  // Initial query
                    get_query();
                    done_query = 1;
                }
                xSemaphoreGive(xFirebaseMutex);
            }
        }
    }
}

void taskMainLogic(void *pvParameters) {
    while (1) {
        ButtonEvent event;
        if (xQueueReceive(xButtonQueue, &event, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (xSemaphoreTake(xStateMutex, portMAX_DELAY) == pdTRUE) {
                switch (currentState) {
                    case STATE_PROFILE_SELECTION:
                        if (event.eventType == 0) {  // Click
                            if (event.buttonId == 1)
                                devId = kodeUnik2;
                            else if (event.buttonId == 2)
                                devId = kodeUnik4;
                            else if (event.buttonId == 3)
                                devId = kodeUnik1;
                            else if (event.buttonId == 4)
                                devId = kodeUnik3;
                            cek_berat = 1;
                            ulang = 1;
                            currentState = STATE_CHECK_ORIENTATION;
                        }
                        // Handle long press, double click similarly
                        break;

                    case STATE_CHECK_ORIENTATION:
                        if (xSemaphoreTake(xSensorDataMutex, portMAX_DELAY) == pdTRUE) {
                            if (sensorData.roll > -3 && sensorData.roll < 3 &&
                                sensorData.pitch > -3 && sensorData.pitch < 3) {
                                LoadCell.start(stabilizingtime, _tare);
                                currentState = STATE_MEASURING;
                            }
                            xSemaphoreGive(xSensorDataMutex);
                        }
                        break;

                    case STATE_MEASURING:
                        if (event.buttonId == 1 && event.eventType == 0) {  // Take measurement
                            if (xSemaphoreTake(xSensorDataMutex, portMAX_DELAY) == pdTRUE) {
                                berat = round(sensorData.weight * 100) / 100;
                                xSemaphoreGive(xSensorDataMutex);
                                vTaskDelay(pdMS_TO_TICKS(5000));  // Wait for stabilization

                                if (devId == kodeUnik1) {
                                    KNasi_profile1 += berat;
                                    cache1 += berat;
                                    if (cache1 >= (KNasi1 * 0.33)) currentState = STATE_ALARM;
                                }  // Similar for other profiles

                                FirebaseCommand cmd = {0, berat, devId};
                                xQueueSend(xFirebaseQueue, &cmd, 0);
                                state = 0;  // Reset sub-state
                            }
                        } else if (event.buttonId == 3 && event.eventType == 0) {
                            currentState = STATE_PROFILE_SELECTION;
                        }
                        break;

                    case STATE_ALARM:
                        if (event.buttonId == 1 && event.eventType == 0)
                            simpan = 1;
                        else if (event.buttonId == 2 && event.eventType == 0)
                            simpan = 0;
                        if (simpan != -1) currentState = STATE_MEASURING;
                        break;

                        // Add handlers for calibration, check weight, etc.
                }
                xSemaphoreGive(xStateMutex);
            }
        }
    }
}

void taskWiFi(void *pvParameters) {
    wm.setConfigPortalTimeout(timeout);
    res = wm.autoConnect("CENTUNG");
    if (!res) {
        Serial.println("WiFi failed");
        esp_deep_sleep_start();
    } else {
        screen_1();  // Loading screen
        Firebase.begin(&config, &auth);
        FirebaseCommand cmd = {1, 0, ""};  // Trigger initial query
        xQueueSend(xFirebaseQueue, &cmd, 0);
    }
    vTaskDelete(NULL);  // Delete task after connection
}

void setup() {
    // Basic hardware initialization
    Wire.begin(PIN_IIC_SDA, PIN_IIC_SCL);
    Wire.setClock(400000);
    Serial.begin(115200);
    pinMode(PIN_POWER_ON, OUTPUT);
    digitalWrite(PIN_POWER_ON, HIGH);
    // ... other pin setups ...

    // Load calibration from EEPROM
    EEPROM.begin(512);
    EEPROM.get(calVal_eepromAdress, calibrationValue);
    EEPROM.get(tareOffsetVal_eepromAdress, calibrationOffset);
    LoadCell.begin();
    LoadCell.start(stabilizingtime, _tare);
    LoadCell.setCalFactor(calibrationValue);
    LoadCell.setTareOffset(calibrationOffset);

    // TFT initialization
    tft.init();
    tft.setRotation(3);
    tft.setSwapBytes(true);
    tft.fillScreen(TFT_WHITE);

    // Sprite setup (assuming defined)
    loading_screen.createSprite(320, 170);
    // ... other sprites ...

    // RTOS initialization
    xDataReadySemaphore = xSemaphoreCreateBinary();
    xSensorDataMutex = xSemaphoreCreateMutex();
    xStateMutex = xSemaphoreCreateMutex();
    xI2CMutex = xSemaphoreCreateMutex();
    xFirebaseMutex = xSemaphoreCreateMutex();
    xButtonQueue = xQueueCreate(10, sizeof(ButtonEvent));
    xFirebaseQueue = xQueueCreate(10, sizeof(FirebaseCommand));

    // Button callbacks
    button1.attachClick([]() {
        ButtonEvent event = {1, 0};
        xQueueSend(xButtonQueue, &event, 0);
    });
    button1.attachLongPressStart([]() {
        ButtonEvent event = {1, 1};
        xQueueSend(xButtonQueue, &event, 0);
    });
    // ... other button callbacks similarly ...

    // Create tasks
    xTaskCreate(taskWiFi, "WiFi", 4096, NULL, 2, NULL);
    xTaskCreate(taskSensorRead, "Sensor", 4096, NULL, 3, NULL);
    xTaskCreate(taskDisplayUpdate, "Display", 4096, NULL, 1, NULL);
    xTaskCreate(taskButtonCheck, "Buttons", 2048, NULL, 2, NULL);
    xTaskCreate(taskFirebaseComms, "Firebase", 8192, NULL, 1, NULL);
    xTaskCreate(taskMainLogic, "Logic", 4096, NULL, 2, NULL);

    // Interrupts
    attachInterrupt(digitalPinToInterrupt(PIN_HX711_DATA), dataReadyISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(PUSH_BUTTON_3), resetESP, FALLING);
}

void loop() {
    vTaskDelete(NULL);  // Main loop not used in RTOS
}
