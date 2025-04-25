#include <Arduino.h>
#include <Wire.h>

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

// Pin definitions for ESP32
#define SDA_PIN 43
#define SCL_PIN 44

// Loop timer for consistent timing
uint32_t LoopTimer;

// Improved Kalman filter function
void kalman_1d(float &KalmanState, float &KalmanUncertainty, float Rate, float Measurement) {
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

    smoothedRoll = rollSum / smoothingWindow;
    smoothedPitch = pitchSum / smoothingWindow;
}

void setup() {
    Serial.begin(57600);
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

    LoopTimer = micros();  // Start loop timer
}

void loop() {
    // Read sensor data
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
}
