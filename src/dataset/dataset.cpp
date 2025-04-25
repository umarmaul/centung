#include <Arduino.h>
#include <HTTPClient.h>
#include <HX711_ADC.h>
#include <Wifi.h>
#include <Wire.h>

// Replace with your network credentials
const char* ssid = "Gedung MRPQ";
const char* password = "umar1234";

// Replace with your API server address and endpoint
const char* serverName = "http://145.223.117.210:5000/log/create-research";

// Device authentication headers
const char* deviceName = "centung-test";
const char* devicePassword = "centung-test";

const int calVal_eepromAdress = 0;
const int tareOffsetVal_eepromAdress = 20;
float calibrationValue = 1306.5337;
float calibrationOffset = 8434937.00;
float berat;
float prediksi_berat;

// Variables for gyro rates and accelerometer values
float RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float GyroBiasRoll = 0, GyroBiasPitch = 0, GyroBiasYaw = 0;
int CalibrationCount = 2000;  // Number of readings for calibration

// Kalman filter state and uncertainty variables
float KalmanAngleRoll = 0, KalmanUncertaintyAngleRoll = 2 * 2;
float KalmanAnglePitch = 0, KalmanUncertaintyAnglePitch = 2 * 2;
float Kalman1DOutput[] = {0, 0};

// Variables for smoothing
const int smoothingWindow = 10;
float rollWindow[smoothingWindow], pitchWindow[smoothingWindow];
int smoothingIndex = 0;
float smoothedRoll = 0, smoothedPitch = 0;

volatile boolean newDataReady;
unsigned long stabilizingtime = 2000;
boolean _tare = true;

// Pin definitions for ESP32
#define SDA_PIN 43
#define SCL_PIN 44
#define PUSH_BUTTON_1 21
#define PIN_HX711_DATA 18
#define PIN_HX711_CLOCK 17
#define PIN_BUZZER 3

// Loop timer for consistent timing
uint32_t LoopTimer;

HX711_ADC LoadCell(PIN_HX711_DATA, PIN_HX711_CLOCK);

void IRAM_ATTR dataReadyISR() {
    if (LoadCell.update()) {
        newDataReady = 1;
    }
}

// Improved Kalman filter function
void kalman_1d(float& KalmanState, float& KalmanUncertainty, float Rate, float Measurement) {
    // Predict
    KalmanState += 0.004 * Rate;                 // Update the estimated angle by the rate
    KalmanUncertainty += 0.004 * 0.004 * 4 * 4;  // Increase uncertainty based on rate noise

    // Update step
    float KalmanGain = KalmanUncertainty / (KalmanUncertainty + 3 * 3);  // Calculate Kalman gain
    KalmanState += KalmanGain * (Measurement - KalmanState);             // Update state with measurement
    KalmanUncertainty = (1 - KalmanGain) * KalmanUncertainty;            // Update uncertainty

    // Store the results
    Kalman1DOutput[0] = KalmanState;
    Kalman1DOutput[1] = KalmanUncertainty;
}

// Function to read accelerometer and gyroscope data
void readMPU6050() {
    // Request accelerometer data
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);  // Starting with ACCEL_XOUT_H register
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 14);  // Read 14 bytes of data

    // Read accelerometer data
    int16_t AccXLSB = Wire.read() << 8 | Wire.read();
    int16_t AccYLSB = Wire.read() << 8 | Wire.read();
    int16_t AccZLSB = Wire.read() << 8 | Wire.read();

    // Skip temperature readings
    Wire.read();
    Wire.read();

    // Read gyroscope data
    int16_t GyroX = Wire.read() << 8 | Wire.read();
    int16_t GyroY = Wire.read() << 8 | Wire.read();
    int16_t GyroZ = Wire.read() << 8 | Wire.read();

    // Scale values for gyro and accel
    RateRoll = (float)GyroX / 65.5;
    RatePitch = (float)GyroY / 65.5;
    RateYaw = (float)GyroZ / 65.5;

    AccX = (float)AccXLSB / 4096;
    AccY = (float)AccYLSB / 4096;
    AccZ = (float)AccZLSB / 4096;

    // Calculate roll and pitch angles from accelerometer
    AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * (180 / PI);
    AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * (180 / PI);
}

// Improved calibration function for gyro biases
void calibrateMPU6050() {
    float rollSum = 0, pitchSum = 0, yawSum = 0;

    // Collect multiple samples to average the gyro drift
    for (int i = 0; i < CalibrationCount; i++) {
        readMPU6050();
        rollSum += RateRoll;
        pitchSum += RatePitch;
        yawSum += RateYaw;
        delay(3);  // Slight delay to stabilize readings
    }

    GyroBiasRoll = rollSum / CalibrationCount;
    GyroBiasPitch = pitchSum / CalibrationCount;
    GyroBiasYaw = yawSum / CalibrationCount;

    Serial.println("Calibration complete");
}

// Smoothing function for roll and pitch
void smoothAngles() {
    rollWindow[smoothingIndex] = KalmanAngleRoll;
    pitchWindow[smoothingIndex] = KalmanAnglePitch;

    smoothingIndex = (smoothingIndex + 1) % smoothingWindow;

    float rollSum = 0, pitchSum = 0;
    for (int i = 0; i < smoothingWindow; i++) {
        rollSum += rollWindow[i];
        pitchSum += pitchWindow[i];
    }

    smoothedPitch = rollSum / smoothingWindow;
    smoothedRoll = pitchSum / smoothingWindow;
}

void setup() {
    Serial.begin(115200);
    WiFi.begin(ssid, password);

    // Connecting to WiFi
    Serial.print("Connecting to Wi-Fi");
    while (WiFi.status() != WL_CONNECTED) {
        delay(1000);
        Serial.print(".");
    }
    Serial.println("\nConnected to Wi-Fi");

    pinMode(PUSH_BUTTON_1, INPUT_PULLUP);
    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(15, OUTPUT);
    digitalWrite(15, HIGH);

    // Initialize I2C
    Wire.begin(SDA_PIN, SCL_PIN);  // Initialize I2C with custom SDA and SCL pins
    Wire.setClock(400000);         // Use fast I2C clock speed

    // Initialize MPU6050
    Wire.beginTransmission(0x68);
    Wire.write(0x6B);  // Power management register
    Wire.write(0x00);  // Wake up MPU6050
    Wire.endTransmission();

    delay(250);  // Wait for the sensor to stabilize

    // Initialize smoothing arrays
    for (int i = 0; i < smoothingWindow; i++) {
        rollWindow[i] = 0;
        pitchWindow[i] = 0;
    }

    // Calibrate the gyro
    calibrateMPU6050();

    LoadCell.begin();
    LoadCell.start(stabilizingtime, _tare);

    if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag()) {
        Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
        esp_deep_sleep_start();
    } else {
        LoadCell.setCalFactor(calibrationValue);  // set calibration value (float)
        Serial.println("Startup is complete");
    }

    attachInterrupt(digitalPinToInterrupt(PIN_HX711_DATA), dataReadyISR, FALLING);

    LoadCell.start(stabilizingtime, _tare);
    // LoadCell.tareNoDelay();
    if (LoadCell.getTareStatus() == true) {
        Serial.println("Tare complete");
    } else {
        Serial.println("Tare timeout");
    }

    LoopTimer = micros();  // Start loop timer
}

void loop() {
    readMPU6050();

    // Subtract the bias from gyro readings (gyro calibration)
    RateRoll -= GyroBiasRoll;
    RatePitch -= GyroBiasPitch;
    RateYaw -= GyroBiasYaw;

    // Apply Kalman filter for both roll and pitch
    kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
    KalmanAngleRoll = Kalman1DOutput[0];
    KalmanUncertaintyAngleRoll = Kalman1DOutput[1];

    kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
    KalmanAnglePitch = Kalman1DOutput[0];
    KalmanUncertaintyAnglePitch = Kalman1DOutput[1];

    // Smooth the output values
    smoothAngles();

    // Output the smoothed results
    Serial.print("Smoothed Roll: ");
    Serial.print(smoothedRoll);
    Serial.print(" Smoothed Pitch: ");
    Serial.println(smoothedPitch);

    // Ensure 4ms loop time
    while (micros() - LoopTimer < 4000);
    LoopTimer = micros();  // Reset loop timer

    if (digitalRead(PUSH_BUTTON_1) == LOW) {
        HTTPClient http;

        http.begin(serverName);
        http.addHeader("Content-Type", "application/json");
        http.addHeader("x-device-name", deviceName);
        http.addHeader("x-device-password", devicePassword);

        digitalWrite(PIN_BUZZER, HIGH);
        delay(500);
        digitalWrite(PIN_BUZZER, LOW);
        delay(5000);

        berat = LoadCell.getData();
        berat = round(berat * 100) / 100;
        if (berat < 0) {
            berat = 0;
        }
        Serial.print("Berat: ");
        Serial.println(berat);

        prediksi_berat = berat * cos(smoothedPitch * PI / 180);
        Serial.print("Prediksi Berat: ");
        Serial.println(prediksi_berat);

        String requestBody = "{\"weight\":" + String(berat, 2) + "," + "\"roll\":" + String(smoothedRoll, 2) + "," + "\"pitch\":" + String(smoothedPitch, 2) + "," + "\"weight_prediction\":" + String(prediksi_berat, 2) + "}";

        // Sending HTTP POST request
        int httpResponseCode = http.POST(requestBody);

        // Check the response
        if (httpResponseCode > 0) {
            Serial.print("HTTP Response code: ");
            Serial.println(httpResponseCode);
            String response = http.getString();
            Serial.println("Response from server: " + response);
        } else {
            Serial.print("Error code: ");
            Serial.println(httpResponseCode);
        }

        // End the connection
        http.end();
        delay(5000);
    }
}
