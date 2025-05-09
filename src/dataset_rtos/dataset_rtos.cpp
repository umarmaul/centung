#include <Arduino.h>
#include <HTTPClient.h>
#include <HX711_ADC.h>
#include <WiFi.h>
#include <Wire.h>

// --- Konfigurasi ---
// WiFi Credentials
const char *ssid = "Gedung MRPQ";   // Ganti dengan SSID Anda
const char *password = "umar1234";  // Ganti dengan Password Anda

// API Server
const char *serverName = "http://168.231.119.233:5000/research/create";
const char *deviceName = "prototype";
const char *devicePassword = "prototype";
const char *profile_code = "prototype";

// Pin Definitions
#define SDA_PIN 43
#define SCL_PIN 44
#define PUSH_BUTTON_1 21
#define PIN_HX711_DATA 18
#define PIN_HX711_CLOCK 17
#define PIN_BUZZER 3
#define PIN_LED_INDICATOR 15  // Asumsi pin 15 untuk indikator

// HX711 Settings
const int calVal_eepromAdress = 0;          // Alamat EEPROM (jika digunakan)
const int tareOffsetVal_eepromAdress = 20;  // Alamat EEPROM (jika digunakan)
float calibrationValue = 1408.25;           // Nilai kalibrasi Anda
unsigned long stabilizingtime = 2000;       // Waktu stabilisasi HX711
HX711_ADC LoadCell(PIN_HX711_DATA, PIN_HX711_CLOCK);

// MPU6050 Settings
const uint8_t MPU6050_ADDR = 0x68;
const float MPU_ACCEL_SCALE = 4096.0;  // Sesuai konfigurasi FS_SEL=1 (±8g)
const float MPU_GYRO_SCALE = 65.5;     // Sesuai konfigurasi FS_SEL=1 (±500°/s)
const int MPU_CALIBRATION_COUNT = 2000;
float GyroBiasRoll = 0, GyroBiasPitch = 0, GyroBiasYaw = 0;

// Kalman Filter Settings
float KalmanUncertaintyAngleRoll = 2 * 2;
float KalmanUncertaintyAnglePitch = 2 * 2;

// Smoothing Settings
const int SMOOTHING_WINDOW_SIZE = 10;
float rollWindow[SMOOTHING_WINDOW_SIZE];
float pitchWindow[SMOOTHING_WINDOW_SIZE];
int smoothingIndex = 0;

// --- Variabel Global (Shared) ---
// Lindungi akses ke variabel ini menggunakan mutexSensorData
volatile float berat = 0.0;
volatile float smoothedRoll = 0.0;
volatile float smoothedPitch = 0.0;
volatile float prediksi_berat = 0.0;

// --- RTOS Handles ---
TaskHandle_t hx711TaskHandle = NULL;
TaskHandle_t mpuTaskHandle = NULL;
TaskHandle_t httpTaskHandle = NULL;

SemaphoreHandle_t hx711DataReadySemaphore = NULL;
SemaphoreHandle_t buttonPressSemaphore = NULL;
SemaphoreHandle_t mutexSensorData = NULL;  // Mutex untuk melindungi data sensor

// --- Variabel State ---
volatile boolean newDataReady = false;  // Flag dari ISR HX711
volatile unsigned long lastButtonInterruptTime = 0;
const unsigned long buttonDebounceDelay = 200;  // Debounce untuk tombol (ms)

// --- Fungsi Prototypes ---
void connectWiFi();
void initMPU6050();
void calibrateMPU6050();
void readMPU6050(float &RateRoll, float &RatePitch, float &RateYaw, float &AngleRoll, float &AnglePitch);
void kalman_1d(float &KalmanState, float &KalmanUncertainty, float Rate, float Measurement);
void smoothAngles(float KalmanAngleRoll, float KalmanAnglePitch);  // Terima sudut sebagai argumen
void hx711Task(void *pvParameters);
void mpuTask(void *pvParameters);
void httpTask(void *pvParameters);
void IRAM_ATTR dataReadyISR();
void IRAM_ATTR buttonISR();

// --- Setup ---
void setup() {
    Serial.begin(115200);
    Serial.println("\nStarting ESP32 Centung Project with RTOS...");

    // Inisialisasi Pin
    pinMode(PUSH_BUTTON_1, INPUT_PULLUP);
    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PIN_LED_INDICATOR, OUTPUT);
    digitalWrite(PIN_LED_INDICATOR, LOW);  // Matikan LED awal
    digitalWrite(PIN_BUZZER, LOW);

    // Koneksi WiFi
    connectWiFi();

    // Inisialisasi I2C
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(400000);  // Fast I2C

    // Inisialisasi MPU6050
    initMPU6050();
    calibrateMPU6050();

    // Inisialisasi HX711
    LoadCell.begin();
    Serial.println("Starting HX711 Tare...");
    LoadCell.start(stabilizingtime, true);  // Mulai dengan proses tare
    if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag()) {
        Serial.println("HX711 Timeout! Check wiring.");
        // Handle error, mungkin restart atau masuk mode error
        while (1);  // Stop execution
    } else {
        LoadCell.setCalFactor(calibrationValue);
        Serial.println("HX711 Tare complete and ready.");
    }

    // --- Inisialisasi RTOS Objects ---
    hx711DataReadySemaphore = xSemaphoreCreateBinary();
    buttonPressSemaphore = xSemaphoreCreateBinary();
    mutexSensorData = xSemaphoreCreateMutex();

    if (hx711DataReadySemaphore == NULL || buttonPressSemaphore == NULL || mutexSensorData == NULL) {
        Serial.println("Failed to create RTOS objects!");
        while (1);  // Stop execution
    }

    // --- Attach Interrupts ---
    attachInterrupt(digitalPinToInterrupt(PIN_HX711_DATA), dataReadyISR, FALLING);
    attachInterrupt(digitalPinToInterrupt(PUSH_BUTTON_1), buttonISR, FALLING);

    // --- Buat Tasks ---
    // Core 0 biasanya lebih baik untuk WiFi/Network
    // Core 1 biasanya lebih baik untuk tugas real-time/IO
    xTaskCreatePinnedToCore(
        httpTask,         // Fungsi Task
        "HTTP Task",      // Nama Task
        8192,             // Ukuran Stack (HTTPClient bisa butuh lebih banyak)
        NULL,             // Parameter Task
        1,                // Prioritas Task (lebih rendah)
        &httpTaskHandle,  // Handle Task
        0);               // Pin ke Core 0

    xTaskCreatePinnedToCore(
        mpuTask,         // Fungsi Task
        "MPU Task",      // Nama Task
        4096,            // Ukuran Stack
        NULL,            // Parameter Task
        2,               // Prioritas Task (lebih tinggi dari HTTP)
        &mpuTaskHandle,  // Handle Task
        1);              // Pin ke Core 1

    xTaskCreatePinnedToCore(
        hx711Task,         // Fungsi Task
        "HX711 Task",      // Nama Task
        4096,              // Ukuran Stack
        NULL,              // Parameter Task
        3,                 // Prioritas Task (tertinggi, karena dipicu ISR)
        &hx711TaskHandle,  // Handle Task
        1);                // Pin ke Core 1

    Serial.println("Setup complete. Tasks running.");
    digitalWrite(PIN_LED_INDICATOR, HIGH);  // Nyalakan LED indikator bahwa sistem siap
}

// --- Loop Utama (biasanya tidak banyak digunakan dengan RTOS) ---
void loop() {
    // Kosongkan atau gunakan untuk tugas berprioritas sangat rendah
    vTaskDelay(pdMS_TO_TICKS(1000));  // Tunda 1 detik
}

// --- Implementasi Fungsi ---

void connectWiFi() {
    Serial.print("Connecting to Wi-Fi ");
    Serial.print(ssid);
    WiFi.begin(ssid, password);
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {  // Coba selama ~10 detik
        delay(500);
        Serial.print(".");
        attempts++;
    }
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nConnected to Wi-Fi");
        Serial.print("IP Address: ");
        Serial.println(WiFi.localIP());
    } else {
        Serial.println("\nFailed to connect to Wi-Fi. Check credentials or network.");
        // Handle error - mungkin restart atau mode offline?
    }
}

void initMPU6050() {
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(0x6B);  // PWR_MGMT_1 register
    Wire.write(0x00);  // Set to zero (wakes up the MPU-6050)
    Wire.endTransmission(true);
    // Konfigurasi tambahan (opsional, contoh: set sensitivity)
    // Wire.beginTransmission(MPU6050_ADDR);
    // Wire.write(0x1B); // GYRO_CONFIG
    // Wire.write(0x08); // Set FS_SEL=1 for ±500°/s
    // Wire.endTransmission(true);
    // Wire.beginTransmission(MPU6050_ADDR);
    // Wire.write(0x1C); // ACCEL_CONFIG
    // Wire.write(0x08); // Set AFS_SEL=1 for ±8g
    // Wire.endTransmission(true);
    delay(100);  // Tunggu sensor stabil
    Serial.println("MPU6050 Initialized.");
}

void calibrateMPU6050() {
    Serial.println("Calibrating MPU6050 Gyro... Keep it stable.");
    digitalWrite(PIN_LED_INDICATOR, LOW);  // Matikan LED selama kalibrasi

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

    Serial.println("Calibration Complete:");
    Serial.print("Gyro Bias Roll: ");
    Serial.println(GyroBiasRoll);
    Serial.print("Gyro Bias Pitch: ");
    Serial.println(GyroBiasPitch);
    Serial.print("Gyro Bias Yaw: ");
    Serial.println(GyroBiasYaw);
    digitalWrite(PIN_LED_INDICATOR, HIGH);  // Nyalakan LED lagi
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

void kalman_1d(float &KalmanState, float &KalmanUncertainty, float Rate, float Measurement) {
    // Predict
    float dt = 0.004;  // Waktu loop MPU task (sesuaikan jika berbeda)
    KalmanState += dt * Rate;
    KalmanUncertainty += dt * dt * 4 * 4;  // Q (Process Noise Variance)

    // Update
    float R = 3 * 3;  // R (Measurement Noise Variance)
    float KalmanGain = KalmanUncertainty / (KalmanUncertainty + R);
    KalmanState += KalmanGain * (Measurement - KalmanState);
    KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;
}

void smoothAngles(float KalmanAngleRoll, float KalmanAnglePitch) {
    // Masukkan nilai baru ke window
    rollWindow[smoothingIndex] = KalmanAngleRoll;
    pitchWindow[smoothingIndex] = KalmanAnglePitch;

    // Update index (circular buffer)
    smoothingIndex = (smoothingIndex + 1) % SMOOTHING_WINDOW_SIZE;

    // Hitung rata-rata
    float rollSum = 0, pitchSum = 0;
    for (int i = 0; i < SMOOTHING_WINDOW_SIZE; i++) {
        rollSum += rollWindow[i];
        pitchSum += pitchWindow[i];
    }

    // Update nilai smoothed (dilindungi mutex di luar fungsi ini)
    // Gunakan variabel lokal agar tidak perlu mutex di sini
    float tempSmoothedRoll = pitchSum / SMOOTHING_WINDOW_SIZE;  // Terbalik di kode asli?
    float tempSmoothedPitch = rollSum / SMOOTHING_WINDOW_SIZE;  // Terbalik di kode asli?

    // --- Critical Section Start ---
    if (xSemaphoreTake(mutexSensorData, pdMS_TO_TICKS(10)) == pdTRUE) {
        // Cek lagi, sepertinya di kode asli variabelnya tertukar
        // smoothedPitch = rollSum / smoothingWindow; -> harusnya pitchWindow
        // smoothedRoll = pitchSum / smoothingWindow;  -> harusnya rollWindow

        smoothedRoll = rollSum / SMOOTHING_WINDOW_SIZE;    // Menggunakan rollWindow
        smoothedPitch = pitchSum / SMOOTHING_WINDOW_SIZE;  // Menggunakan pitchWindow

        xSemaphoreGive(mutexSensorData);
    } else {
        Serial.println("MPU Task: Failed to get mutex for smoothing!");
    }
    // --- Critical Section End ---
}

// --- ISRs ---
void IRAM_ATTR dataReadyISR() {
    // Beri sinyal ke HX711 task bahwa data siap
    // Jangan lakukan pemrosesan berat di dalam ISR
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(hx711DataReadySemaphore, &xHigherPriorityTaskWoken);

    // Jika pemberian semaphore membangunkan task dengan prioritas lebih tinggi,
    // lakukan context switch
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

void IRAM_ATTR buttonISR() {
    unsigned long currentMillis = millis();
    // Debounce check
    if (currentMillis - lastButtonInterruptTime > buttonDebounceDelay) {
        lastButtonInterruptTime = currentMillis;
        // Beri sinyal ke HTTP task bahwa tombol ditekan
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xSemaphoreGiveFromISR(buttonPressSemaphore, &xHigherPriorityTaskWoken);

        if (xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
        }
    }
}

// --- Tasks ---

void hx711Task(void *pvParameters) {
    Serial.println("HX711 Task started.");
    float currentWeight;

    for (;;) {
        // Tunggu sinyal dari ISR bahwa data baru siap
        if (xSemaphoreTake(hx711DataReadySemaphore, portMAX_DELAY) == pdTRUE) {
            // Cek jika data benar-benar baru dan valid (opsional tapi bagus)
            if (LoadCell.update()) {
                currentWeight = LoadCell.getData();
                // Pembulatan dan clamping (opsional)
                currentWeight = round(currentWeight * 100) / 100.0;
                if (currentWeight < 0) {
                    currentWeight = 0;
                }

                // --- Critical Section Start ---
                // Ambil mutex sebelum mengubah variabel global 'berat'
                if (xSemaphoreTake(mutexSensorData, pdMS_TO_TICKS(10)) == pdTRUE) {
                    berat = currentWeight;
                    // Serial.print("Raw Weight Updated: "); Serial.println(berat); // Debugging
                    xSemaphoreGive(mutexSensorData);
                } else {
                    Serial.println("HX711 Task: Failed to get mutex!");
                }
                // --- Critical Section End ---
            } else {
                // Serial.println("HX711 Task: LoadCell.update() failed after ISR signal.");
            }
        }
    }
}

void mpuTask(void *pvParameters) {
    Serial.println("MPU Task started.");
    float RateRoll, RatePitch, RateYaw;
    float AngleRoll, AnglePitch;                      // Sudut dari accelerometer (raw)
    float KalmanAngleRoll = 0, KalmanAnglePitch = 0;  // State Kalman filter

    // Inisialisasi Kalman state awal (opsional, bisa dari pembacaan pertama)
    readMPU6050(RateRoll, RatePitch, RateYaw, AngleRoll, AnglePitch);
    KalmanAngleRoll = AngleRoll;
    KalmanAnglePitch = AnglePitch;

    // Inisialisasi smoothing window
    for (int i = 0; i < SMOOTHING_WINDOW_SIZE; i++) {
        rollWindow[i] = KalmanAngleRoll;
        pitchWindow[i] = KalmanAnglePitch;
    }

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(4);  // Target loop time 4ms

    for (;;) {
        // 1. Baca MPU6050
        readMPU6050(RateRoll, RatePitch, RateYaw, AngleRoll, AnglePitch);

        // 2. Koreksi Bias Gyro
        RateRoll -= GyroBiasRoll;
        RatePitch -= GyroBiasPitch;
        // RateYaw -= GyroBiasYaw; // Yaw tidak digunakan di Kalman/smoothing

        // 3. Jalankan Kalman Filter
        // Variabel Kalman state dan uncertainty di-pass by reference
        kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
        kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);

        // 4. Jalankan Smoothing Filter
        // Fungsi smoothAngles akan mengupdate variabel global (smoothedRoll, smoothedPitch)
        // dengan proteksi mutex di dalamnya.
        smoothAngles(KalmanAngleRoll, KalmanAnglePitch);

        // Debugging (opsional, bisa membebani CPU jika terlalu sering)
        // if (xSemaphoreTake(mutexSensorData, pdMS_TO_TICKS(5)) == pdTRUE) {
        //     Serial.printf("SR: %.2f, SP: %.2f\n", smoothedRoll, smoothedPitch);
        //     xSemaphoreGive(mutexSensorData);
        // }

        // 5. Tunda task untuk menjaga frekuensi loop
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void httpTask(void *pvParameters) {
    Serial.println("HTTP Task started.");
    float localBerat, localRoll, localPitch, localPrediksiBerat;

    for (;;) {
        // Tunggu sinyal dari ISR tombol
        if (xSemaphoreTake(buttonPressSemaphore, portMAX_DELAY) == pdTRUE) {
            Serial.println("Button pressed! Preparing to send data...");

            // Bunyikan buzzer sebagai indikasi
            digitalWrite(PIN_BUZZER, HIGH);
            vTaskDelay(pdMS_TO_TICKS(200));  // Tunda sebentar
            digitalWrite(PIN_BUZZER, LOW);

            // --- Critical Section Start ---
            // Ambil data sensor terbaru dari variabel global
            if (xSemaphoreTake(mutexSensorData, pdMS_TO_TICKS(50)) == pdTRUE) {
                localBerat = berat;
                localRoll = smoothedRoll;    // Now correctly physical roll
                localPitch = smoothedPitch;  // Now correctly physical pitch
                xSemaphoreGive(mutexSensorData);
            } else {
                Serial.println("HTTP Task: Failed to get mutex for reading sensor data!");
                // Putuskan apakah akan mengirim data lama atau tidak sama sekali
                continue;  // Lewati pengiriman kali ini jika tidak dapat data baru
            }
            // --- Critical Section End ---

            Serial.printf("Data to send -> Weight: %.2f, Roll: %.2f, Pitch: %.2f\n", localBerat, localRoll, localPitch);

            // Cek koneksi WiFi sebelum mengirim
            if (WiFi.status() == WL_CONNECTED) {
                HTTPClient http;
                bool requestSuccess = false;

                if (http.begin(serverName)) {  // Gunakan begin(const char*)
                    http.addHeader("Content-Type", "application/json");
                    http.addHeader("x-device-name", deviceName);
                    http.addHeader("x-device-password", devicePassword);

                    // Buat JSON payload
                    String requestBody = "{\"weight\":" + String(localBerat, 2) + "," +
                                         "\"roll\":" + String(localRoll, 2) + "," +
                                         "\"pitch\":" + String(localPitch, 2) + "," +
                                         "\"profile_code\":\"" + String(profile_code) + "\"}";

                    Serial.print("Sending POST: ");
                    Serial.println(requestBody);

                    // Kirim POST request
                    int httpResponseCode = http.POST(requestBody);

                    if (httpResponseCode > 0) {
                        Serial.print("HTTP Response code: ");
                        Serial.println(httpResponseCode);
                        String response = http.getString();
                        Serial.println("Response from server: " + response);
                        if (httpResponseCode == 200 || httpResponseCode == 201) {  // Sukses
                            requestSuccess = true;
                        }
                    } else {
                        Serial.print("HTTP POST failed, error: ");
                        Serial.println(http.errorToString(httpResponseCode).c_str());
                    }

                    http.end();  // Tutup koneksi

                } else {
                    Serial.println("HTTP Client begin failed!");
                }

                // Indikasi hasil pengiriman (misal: buzzer atau LED)
                if (requestSuccess) {
                    // Sukses, mungkin bunyi beep pendek
                    digitalWrite(PIN_BUZZER, HIGH);
                    vTaskDelay(pdMS_TO_TICKS(100));
                    digitalWrite(PIN_BUZZER, LOW);
                } else {
                    // Gagal, mungkin bunyi beep panjang atau beberapa kali
                    for (int i = 0; i < 3; i++) {
                        digitalWrite(PIN_BUZZER, HIGH);
                        vTaskDelay(pdMS_TO_TICKS(150));
                        digitalWrite(PIN_BUZZER, LOW);
                        vTaskDelay(pdMS_TO_TICKS(100));
                    }
                }

            } else {
                Serial.println("WiFi disconnected. Cannot send data.");
                // Gagal, mungkin bunyi beep panjang atau beberapa kali
                for (int i = 0; i < 3; i++) {
                    digitalWrite(PIN_BUZZER, HIGH);
                    vTaskDelay(pdMS_TO_TICKS(150));
                    digitalWrite(PIN_BUZZER, LOW);
                    vTaskDelay(pdMS_TO_TICKS(100));
                }
            }

            // Beri jeda setelah pengiriman untuk mencegah multiple send jika tombol ditahan
            vTaskDelay(pdMS_TO_TICKS(1000));  // Tunda 1 detik

        }  // End if semaphoreTake
    }  // End for(;;)
}