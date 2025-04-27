#include <Arduino.h>

#include "library.h"
#include "pin_config.h"

// Firebase config
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

FirebaseJson query1;
FirebaseJson query2;

// variables
String kodeUnik1 = "A1";
String kodeUnik2 = "B1";
String kodeUnik3 = "C1";
String kodeUnik4 = "D1";

const int calVal_eepromAdress = 0;
const int tareOffsetVal_eepromAdress = 20;
volatile boolean newDataReady;
float calibrationValue, calibrationOffset;
String devId;
String pengguna1, pengguna2, pengguna3, pengguna4;
double KNasi1, KNasi2, KNasi3, KNasi4;
double cache1, cache2, cache3, cache4;
double KNasiSekarang, KNasi_profile1, KNasi_profile2, KNasi_profile3, KNasi_profile4;
double max_nasi;
double berat;
int ulang = 0, state = 0, cek_berat = 0;
int timeout = 30;
int centong = 0, adc = 0, simpan = 0, done_query = 0, waitUntilClick = 0;
int tanggal = 0, bulan = 0;
bool res;
unsigned long stabilizingtime = 2000;
boolean _tare = true;

// Varibel untuk IMU
volatile float twoKp = twoKpDef;                                            // 2 * proportional gain (Kp)
volatile float twoKi = twoKiDef;                                            // 2 * integral gain (Ki)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;                  // quaternion of sensor frame relative to auxiliary frame
volatile float integralFBx = 0.0f, integralFBy = 0.0f, integralFBz = 0.0f;  // integral error terms scaled by Ki

long sampling_timer;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
float axg, ayg, azg, gxrs, gyrs, gzrs;
float roll, pitch, yaw;

float SelfTest[6];

float gyroBias[3] = {0, 0, 0}, accelBias[3] = {0, 0, 0};  // Bias corrections for gyro and accelerometer

float sampleFreq = 0.0f;                   // integration interval for both filter schemes
uint32_t lastUpdate = 0, firstUpdate = 0;  // used to calculate integration interval
uint32_t Now = 0;                          // used to calculate integration interval

// inisialisasi loadcell dan wifi
HX711_ADC LoadCell(PIN_HX711_DATA, PIN_HX711_CLOCK);
WiFiManager wm;

// One Button setup
OneButton button1(PUSH_BUTTON_1, true);
OneButton button2(PUSH_BUTTON_2, true);
OneButton button3(PUSH_BUTTON_3, true);
OneButton button4(PUSH_BUTTON_4, true);

void getLocalTime() {
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

char getRandomChar() {
    char characters[] = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789";
    int randomIndex = random(sizeof(characters) - 1);
    return characters[randomIndex];
}

String generateRandomString() {
    String randomString = "";
    for (int i = 0; i < 20; i++) {
        randomString += getRandomChar();
    }
    return randomString;
}

void screen_1() {
    int blocks = 0;
    while (blocks < 20) {
        loading_screen.pushImage(0, 0, 320, 170, loading);
        progress_bar.fillSprite(0xE73B);
        blocks++;
        progress_bar.drawRoundRect(98, 122, 124, 16, 3, TFT_BLACK);

        for (int i = 0; i < blocks; i++) {
            progress_bar.fillRect(i * 5 + (98 + 2) + (i * 1), 122 + 4, 5, 10, TFT_BLACK);
        }

        progress_bar.pushToSprite(&loading_screen, 0, 0, 0xE73B);
        loading_screen.pushSprite(0, 0);
    }
}

void IRAM_ATTR dataReadyISR() {
    if (LoadCell.update()) {
        newDataReady = 1;
    }
}

void IRAM_ATTR resetESP() {
    ESP.restart();
}

void writeByte(uint8_t address, uint8_t subAddress, uint8_t data) {
    Wire.beginTransmission(address);  // Initialize the Tx buffer
    Wire.write(subAddress);           // Put slave register address in Tx buffer
    Wire.write(data);                 // Put data in Tx buffer
    Wire.endTransmission();           // Send the Tx buffer
}

void MPU6050_Init() {
    // MPU6050 Initializing & Reset
    writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x00);  // set to zero (wakes up the MPU-6050)

    // MPU6050 Clock Type
    writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x01);  // Selection Clock 'PLL with X axis gyroscope reference'

    // MPU6050 Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV) for DMP
    // writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SMPLRT_DIV, 0x00); // Default is 1KHz // example 0x04 is 200Hz

    // MPU6050 Gyroscope Configuration Setting
    /* Wire.write(0x00); // FS_SEL=0, Full Scale Range = +/- 250 [degree/sec]
       Wire.write(0x08); // FS_SEL=1, Full Scale Range = +/- 500 [degree/sec]
       Wire.write(0x10); // FS_SEL=2, Full Scale Range = +/- 1000 [degree/sec]
       Wire.write(0x18); // FS_SEL=3, Full Scale Range = +/- 2000 [degree/sec]   */
    writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, 0x18);  // FS_SEL=3

    // MPU6050 Accelerometer Configuration Setting
    /* Wire.write(0x00); // AFS_SEL=0, Full Scale Range = +/- 2 [g]
       Wire.write(0x08); // AFS_SEL=1, Full Scale Range = +/- 4 [g]
       Wire.write(0x10); // AFS_SEL=2, Full Scale Range = +/- 8 [g]
       Wire.write(0x18); // AFS_SEL=3, Full Scale Range = +/- 10 [g] */
    writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 0x10);  // AFS_SEL=2

    // MPU6050 DLPF(Digital Low Pass Filter)
    /*Wire.write(0x00);     // Accel BW 260Hz, Delay 0ms / Gyro BW 256Hz, Delay 0.98ms, Fs 8KHz
      Wire.write(0x01);     // Accel BW 184Hz, Delay 2ms / Gyro BW 188Hz, Delay 1.9ms, Fs 1KHz
      Wire.write(0x02);     // Accel BW 94Hz, Delay 3ms / Gyro BW 98Hz, Delay 2.8ms, Fs 1KHz
      Wire.write(0x03);     // Accel BW 44Hz, Delay 4.9ms / Gyro BW 42Hz, Delay 4.8ms, Fs 1KHz
      Wire.write(0x04);     // Accel BW 21Hz, Delay 8.5ms / Gyro BW 20Hz, Delay 8.3ms, Fs 1KHz
      Wire.write(0x05);     // Accel BW 10Hz, Delay 13.8ms / Gyro BW 10Hz, Delay 13.4ms, Fs 1KHz
      Wire.write(0x06);     // Accel BW 5Hz, Delay 19ms / Gyro BW 5Hz, Delay 18.6ms, Fs 1KHz */
    writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_CONFIG, 0x00);  // Accel BW 260Hz, Delay 0ms / Gyro BW 256Hz, Delay 0.98ms, Fs 8KHz
}

uint8_t readByte(uint8_t address, uint8_t subAddress) {
    uint8_t data;                           // `data` will store the register data
    Wire.beginTransmission(address);        // Initialize the Tx buffer
    Wire.write(subAddress);                 // Put slave register address in Tx buffer
    Wire.endTransmission(false);            // Send the Tx buffer, but send a restart to keep connection alive
    Wire.requestFrom(address, (uint8_t)1);  // Read one byte from slave register address
    data = Wire.read();                     // Fill Rx buffer with result
    return data;                            // Return data read from slave register
}

void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t* dest) {
    Wire.beginTransmission(address);  // Initialize the Tx buffer
    Wire.write(subAddress);           // Put slave register address in Tx buffer
    Wire.endTransmission(false);      // Send the Tx buffer, but send a restart to keep connection alive
    uint8_t i = 0;
    Wire.requestFrom(address, count);  // Read bytes from slave register address
    while (Wire.available()) {
        dest[i++] = Wire.read();
    }  // Put read results in the Rx buffer
}

void mpu6050_GetData() {
    uint8_t data_org[14];  // original data of accelerometer and gyro
    readBytes(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, 14, &data_org[0]);

    AcX = data_org[0] << 8 | data_org[1];    // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    AcY = data_org[2] << 8 | data_org[3];    // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ = data_org[4] << 8 | data_org[5];    // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp = data_org[6] << 8 | data_org[7];    // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX = data_org[8] << 8 | data_org[9];    // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY = data_org[10] << 8 | data_org[11];  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ = data_org[12] << 8 | data_org[13];  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

void mpu6050_updateQuaternion() {
    axg = (float)(AcX - MPU6050_AXOFFSET) / MPU6050_AXGAIN;
    ayg = (float)(AcY - MPU6050_AYOFFSET) / MPU6050_AYGAIN;
    azg = (float)(AcZ - MPU6050_AZOFFSET) / MPU6050_AZGAIN;
    gxrs = (float)(GyX - MPU6050_GXOFFSET) / MPU6050_GXGAIN * 0.01745329;  // degree to radians
    gyrs = (float)(GyY - MPU6050_GYOFFSET) / MPU6050_GYGAIN * 0.01745329;  // degree to radians
    gzrs = (float)(GyZ - MPU6050_GZOFFSET) / MPU6050_GZGAIN * 0.01745329;  // degree to radians
    // Degree to Radians Pi / 180 = 0.01745329 0.01745329251994329576923690768489
}

void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
    float norm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // Normalise accelerometer measurement
        norm = sqrt(ax * ax + ay * ay + az * az);
        ax /= norm;
        ay /= norm;
        az /= norm;

        // Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        // Error is sum of cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Compute and apply integral feedback if enabled
        if (twoKi > 0.0f) {
            integralFBx += twoKi * halfex * (1.0f / sampleFreq);  // integral error scaled by Ki
            integralFBy += twoKi * halfey * (1.0f / sampleFreq);
            integralFBz += twoKi * halfez * (1.0f / sampleFreq);
            gx += integralFBx;  // apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        } else {
            integralFBx = 0.0f;  // prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += twoKp * halfex;
        gy += twoKp * halfey;
        gz += twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * (1.0f / sampleFreq));  // pre-multiply common factors
    gy *= (0.5f * (1.0f / sampleFreq));
    gz *= (0.5f * (1.0f / sampleFreq));
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 /= norm;
    q1 /= norm;
    q2 /= norm;
    q3 /= norm;
}

void mpu6050_getRollPitchYaw() {
    //  yaw = atan2(2*q1*q2 - 2*q0*q3, 2*q0*q0 + 2*q1*q1 - 1) * 57.29577951;
    //  pitch = -asin(2*q1*q3 + 2*q0*q2) * 57.29577951;
    //  roll = atan2(2*q2*q3 - 2*q0*q1, 2*q0*q0 + 2*q3*q3 - 1) * 57.29577951;
    //  roll = atan2(2*q0*q1 + 2*q2*q3, 1 - 2*q1*q1 - 2*q2*q2) * 57.29577951;
    //  pitch = asin(2*q0*q2 - 2*q3*q1) * 57.29577951;
    //  yaw = atan2(2*q0*q3 + 2*q1*q2, 1 - 2*q2*q2 - 2*q3*q3) * 57.29577951;
    yaw = -atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.29577951;
    pitch = asin(2.0f * (q1 * q3 - q0 * q2)) * 57.29577951;
    roll = atan2(2.0f * (q0 * q1 + q2 * q3), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3) * 57.29577951;
}

// Accelerometer and gyroscope self test; check calibration wrt factory settings
void MPU6050SelfTest(float* destination)  // Should return percent deviation from factory trim values, +/- 14 or less deviation is a pass
{
    uint8_t rawData[4];
    uint8_t selfTest[6];
    float factoryTrim[6];

    // Configure the accelerometer for self-test
    writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 0xF0);       // Enable self test on all three axes and set accelerometer range to +/- 8 g
    writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, 0xE0);        // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
    delay(250);                                                              // Delay a while to let the device execute the self-test
    rawData[0] = readByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SELF_TEST_X);  // X-axis self-test results
    rawData[1] = readByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SELF_TEST_Y);  // Y-axis self-test results
    rawData[2] = readByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SELF_TEST_Z);  // Z-axis self-test results
    rawData[3] = readByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SELF_TEST_A);  // Mixed-axis self-test results
    // Extract the acceleration test results first
    selfTest[0] = (rawData[0] >> 3) | (rawData[3] & 0x30) >> 4;  // XA_TEST result is a five-bit unsigned integer
    selfTest[1] = (rawData[1] >> 3) | (rawData[3] & 0x0C) >> 2;  // YA_TEST result is a five-bit unsigned integer
    selfTest[2] = (rawData[2] >> 3) | (rawData[3] & 0x03);       // ZA_TEST result is a five-bit unsigned integer
    // Extract the gyration test results first
    selfTest[3] = rawData[0] & 0x1F;  // XG_TEST result is a five-bit unsigned integer
    selfTest[4] = rawData[1] & 0x1F;  // YG_TEST result is a five-bit unsigned integer
    selfTest[5] = rawData[2] & 0x1F;  // ZG_TEST result is a five-bit unsigned integer
    // Process results to allow final comparison with factory set values
    factoryTrim[0] = (4096.0 * 0.34) * (pow((0.92 / 0.34), (((float)selfTest[0] - 1.0) / 30.0)));  // FT[Xa] factory trim calculation
    factoryTrim[1] = (4096.0 * 0.34) * (pow((0.92 / 0.34), (((float)selfTest[1] - 1.0) / 30.0)));  // FT[Ya] factory trim calculation
    factoryTrim[2] = (4096.0 * 0.34) * (pow((0.92 / 0.34), (((float)selfTest[2] - 1.0) / 30.0)));  // FT[Za] factory trim calculation
    factoryTrim[3] = (25.0 * 131.0) * (pow(1.046, ((float)selfTest[3] - 1.0)));                    // FT[Xg] factory trim calculation
    factoryTrim[4] = (-25.0 * 131.0) * (pow(1.046, ((float)selfTest[4] - 1.0)));                   // FT[Yg] factory trim calculation
    factoryTrim[5] = (25.0 * 131.0) * (pow(1.046, ((float)selfTest[5] - 1.0)));                    // FT[Zg] factory trim calculation

    //  Output self-test results and factory trim calculation if desired
    //  Serial.println(selfTest[0]); Serial.println(selfTest[1]); Serial.println(selfTest[2]);
    //  Serial.println(selfTest[3]); Serial.println(selfTest[4]); Serial.println(selfTest[5]);
    //  Serial.println(factoryTrim[0]); Serial.println(factoryTrim[1]); Serial.println(factoryTrim[2]);
    //  Serial.println(factoryTrim[3]); Serial.println(factoryTrim[4]); Serial.println(factoryTrim[5]);

    // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
    // To get to percent, must multiply by 100 and subtract result from 100
    for (int i = 0; i < 6; i++) {
        destination[i] = 100.0 + 100.0 * ((float)selfTest[i] - factoryTrim[i]) / factoryTrim[i];  // Report percent differences
    }
}

// Function which accumulates gyro and accelerometer data after device initialization. It calculates the average
// of the at-rest readings and then loads the resulting offsets into accelerometer and gyro bias registers.
void calibrateMPU6050(float* dest1, float* dest2) {
    uint8_t data[12];  // data array to hold accelerometer and gyro x, y, z, data
    uint16_t ii, packet_count, fifo_count;
    int32_t gyro_bias[3] = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

    // reset device, reset all registers, clear gyro and accelerometer bias registers
    writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x80);  // Write a one to bit 7 reset bit; toggle reset device
    delay(100);

    // get stable time source
    // Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
    writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x01);
    writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_2, 0x00);
    delay(200);

    // Configure device for bias calculation
    writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_INT_ENABLE, 0x00);    // Disable all interrupts
    writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_FIFO_EN, 0x00);       // Disable FIFO
    writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, 0x00);    // Turn on internal clock source
    writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_I2C_MST_CTRL, 0x00);  // Disable I2C master
    writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_USER_CTRL, 0x00);     // Disable FIFO and I2C master modes
    writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_USER_CTRL, 0x0C);     // Reset FIFO and DMP
    delay(15);

    // Configure MPU6050 gyro and accelerometer for bias calculation
    writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_CONFIG, 0x01);        // Set low-pass filter to 188 Hz
    writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_SMPLRT_DIV, 0x00);    // Set sample rate to 1 kHz
    writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, 0x00);   // Set gyro full-scale to 250 degrees per second, maximum sensitivity
    writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, 0x00);  // Set accelerometer full-scale to 2 g, maximum sensitivity

    uint16_t gyrosensitivity = 131;     // = 131 LSB/degrees/sec
    uint16_t accelsensitivity = 16384;  // = 16384 LSB/g

    // Configure FIFO to capture accelerometer and gyro data for bias calculation
    writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_USER_CTRL, 0x40);  // Enable FIFO
    writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_FIFO_EN, 0x78);    // Enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes in MPU-6050)
    delay(80);                                                       // accumulate 80 samples in 80 milliseconds = 960 bytes

    // At end of sample accumulation, turn off FIFO sensor read
    writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_FIFO_EN, 0x00);             // Disable gyro and accelerometer sensors for FIFO
    readBytes(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_FIFO_COUNTH, 2, &data[0]);  // read FIFO sample count
    fifo_count = ((uint16_t)data[0] << 8) | data[1];
    packet_count = fifo_count / 12;  // How many sets of full gyro and accelerometer data for averaging

    for (ii = 0; ii < packet_count; ii++) {
        int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
        readBytes(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_FIFO_R_W, 12, &data[0]);  // read data for averaging
        accel_temp[0] = (int16_t)(((int16_t)data[0] << 8) | data[1]);           // Form signed 16-bit integer for each sample in FIFO
        accel_temp[1] = (int16_t)(((int16_t)data[2] << 8) | data[3]);
        accel_temp[2] = (int16_t)(((int16_t)data[4] << 8) | data[5]);
        gyro_temp[0] = (int16_t)(((int16_t)data[6] << 8) | data[7]);
        gyro_temp[1] = (int16_t)(((int16_t)data[8] << 8) | data[9]);
        gyro_temp[2] = (int16_t)(((int16_t)data[10] << 8) | data[11]);

        accel_bias[0] += (int32_t)accel_temp[0];  // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        accel_bias[1] += (int32_t)accel_temp[1];
        accel_bias[2] += (int32_t)accel_temp[2];
        gyro_bias[0] += (int32_t)gyro_temp[0];
        gyro_bias[1] += (int32_t)gyro_temp[1];
        gyro_bias[2] += (int32_t)gyro_temp[2];
    }
    accel_bias[0] /= (int32_t)packet_count;  // Normalize sums to get average count biases
    accel_bias[1] /= (int32_t)packet_count;
    accel_bias[2] /= (int32_t)packet_count;
    gyro_bias[0] /= (int32_t)packet_count;
    gyro_bias[1] /= (int32_t)packet_count;
    gyro_bias[2] /= (int32_t)packet_count;

    if (accel_bias[2] > 0L) {
        accel_bias[2] -= (int32_t)accelsensitivity;
    }  // Remove gravity from the z-axis accelerometer bias calculation
    else {
        accel_bias[2] += (int32_t)accelsensitivity;
    }

    // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
    data[0] = (-gyro_bias[0] / 4 >> 8) & 0xFF;  // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
    data[1] = (-gyro_bias[0] / 4) & 0xFF;       // Biases are additive, so change sign on calculated average gyro biases
    data[2] = (-gyro_bias[1] / 4 >> 8) & 0xFF;
    data[3] = (-gyro_bias[1] / 4) & 0xFF;
    data[4] = (-gyro_bias[2] / 4 >> 8) & 0xFF;
    data[5] = (-gyro_bias[2] / 4) & 0xFF;

    // Push gyro biases to hardware registers
    writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_XG_OFFS_USRH, data[0]);  // might not be supported in MPU6050
    writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_XG_OFFS_USRL, data[1]);
    writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_YG_OFFS_USRH, data[2]);
    writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_YG_OFFS_USRL, data[3]);
    writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ZG_OFFS_USRH, data[4]);
    writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ZG_OFFS_USRL, data[5]);

    dest1[0] = (float)gyro_bias[0] / (float)gyrosensitivity;  // construct gyro bias in deg/s for later manual subtraction
    dest1[1] = (float)gyro_bias[1] / (float)gyrosensitivity;
    dest1[2] = (float)gyro_bias[2] / (float)gyrosensitivity;

    // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
    // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
    // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
    // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
    // the accelerometer biases calculated above must be divided by 8.

    int32_t accel_bias_reg[3] = {0, 0, 0};                                  // A place to hold the factory accelerometer trim biases
    readBytes(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_XA_OFFS_H, 2, &data[0]);  // Read factory accelerometer trim values
    accel_bias_reg[0] = (int16_t)((int16_t)data[0] << 8) | data[1];
    readBytes(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_YA_OFFS_H, 2, &data[0]);
    accel_bias_reg[1] = (int16_t)((int16_t)data[0] << 8) | data[1];
    readBytes(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ZA_OFFS_H, 2, &data[0]);
    accel_bias_reg[2] = (int16_t)((int16_t)data[0] << 8) | data[1];

    uint32_t mask = 1uL;              // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
    uint8_t mask_bit[3] = {0, 0, 0};  // Define array to hold mask bit for each accelerometer bias axis

    for (ii = 0; ii < 3; ii++) {
        if (accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01;  // If temperature compensation bit is set, record that fact in mask_bit
    }

    // Construct total accelerometer bias, including calculated average accelerometer bias from above
    accel_bias_reg[0] -= (accel_bias[0] / 8);  // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
    accel_bias_reg[1] -= (accel_bias[1] / 8);
    accel_bias_reg[2] -= (accel_bias[2] / 8);

    data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
    data[1] = (accel_bias_reg[0]) & 0xFF;
    data[1] = data[1] | mask_bit[0];  // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
    data[3] = (accel_bias_reg[1]) & 0xFF;
    data[3] = data[3] | mask_bit[1];  // preserve temperature compensation bit when writing back to accelerometer bias registers
    data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
    data[5] = (accel_bias_reg[2]) & 0xFF;
    data[5] = data[5] | mask_bit[2];  // preserve temperature compensation bit when writing back to accelerometer bias registers

    // Push accelerometer biases to hardware registers
    writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_XA_OFFS_H, data[0]);  // might not be supported in MPU6050
    writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_XA_OFFS_L_TC, data[1]);
    writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_YA_OFFS_H, data[2]);
    writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_YA_OFFS_L_TC, data[3]);
    writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ZA_OFFS_H, data[4]);
    writeByte(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ZA_OFFS_L_TC, data[5]);

    // Output scaled accelerometer biases for manual subtraction in the main program
    dest2[0] = (float)accel_bias[0] / (float)accelsensitivity;
    dest2[1] = (float)accel_bias[1] / (float)accelsensitivity;
    dest2[2] = (float)accel_bias[2] / (float)accelsensitivity;
}

String extractDocumentId(String documentName) {
    // Extract the document ID from the document name
    int lastSlashIndex = documentName.lastIndexOf('/');
    return documentName.substring(lastSlashIndex + 1);
}

void deleteDocument(String documentId) {
    String path = "/log_centung/" + documentId;
    if (Firebase.Firestore.deleteDocument(&fbdo, FIREBASE_PROJECT_ID, "", path.c_str())) {
        Serial.printf("Document %s deleted successfully.\n", documentId.c_str());
    } else {
        Serial.printf("Error deleting document %s: %s\n", documentId.c_str(), fbdo.errorReason());
    }
}

void get_query() {
    Serial.print("Query a Firestore database... ");
    // inisialisasi query 1 dan query 2 ===================================================
    query1.set("select/fields/[0]/fieldPath", "device_id");
    query1.set("select/fields/[1]/fieldPath", "nama_lengkap");
    query1.set("select/fields/[2]/fieldPath", "KNasi");
    query1.set("from/collectionId", "profile");
    query1.set("from/allDescendants", false);

    query2.set("select/fields/[0]/fieldPath", "device_id");
    query2.set("select/fields/[1]/fieldPath", "timestamp");
    query2.set("select/fields/[2]/fieldPath", "berat_nasi");
    query2.set("from/collectionId", "log_centung");
    query2.set("from/allDescendants", false);

    // Ambil Pengguna 1==============================================================
    // ambil data nama dan berat maximum
    query1.set("where/fieldFilter/field/fieldPath", "device_id");
    query1.set("where/fieldFilter/op", "EQUAL");
    query1.set("where/fieldFilter/value/stringValue", kodeUnik1);
    query1.set("limit", 1);

    Firebase.Firestore.runQuery(&fbdo, FIREBASE_PROJECT_ID, "", "/", &query1);
    String payload1 = fbdo.payload().c_str();
    DynamicJsonDocument doc1(2048);
    deserializeJson(doc1, payload1);
    String nama1 = doc1[0]["document"]["fields"]["nama_lengkap"]["stringValue"];
    if (nama1 == "null") {
        nama1 = "pengguna 1";
    }
    pengguna1 = nama1;
    double Nasi1 = doc1[0]["document"]["fields"]["KNasi"]["doubleValue"];
    KNasi1 = Nasi1;

    // ambil data berat nasi hari ini
    query2.set("where/compositeFilter/op", "AND");

    query2.set("where/compositeFilter/filters/[0]/fieldFilter/field/fieldPath", "device_id");
    query2.set("where/compositeFilter/filters/[0]/fieldFilter/op", "EQUAL");
    query2.set("where/compositeFilter/filters/[0]/fieldFilter/value/stringValue", kodeUnik1);

    query2.set("where/compositeFilter/filters/[1]/fieldFilter/field/fieldPath", "timestamp");
    query2.set("where/compositeFilter/filters/[1]/fieldFilter/op", "GREATER_THAN_OR_EQUAL");
    query2.set("where/compositeFilter/filters/[1]/fieldFilter/value/timestampValue", String(year) + "-" + String(bulan) + "-" + String(tanggal) + "T17:00:00.000000Z");

    query2.set("orderBy/field/fieldPath", "timestamp");
    query2.set("orderBy/direction", "ASCENDING");

    Firebase.Firestore.runQuery(&fbdo, FIREBASE_PROJECT_ID, "", "/", &query2);
    String payload2 = fbdo.payload().c_str();
    DynamicJsonDocument doc2(50000);
    deserializeJson(doc2, payload2);

    KNasi_profile1 = 0;
    for (int i = 0; i < doc2.size(); i++) {
        double Berat = doc2[i]["document"]["fields"]["berat_nasi"]["doubleValue"];
        KNasi_profile1 += Berat;
    }
    Serial.printf("Total Documents: %d\n", doc2.size());

    // Ambil Pengguna 2==============================================================
    query1.set("where/fieldFilter/field/fieldPath", "device_id");
    query1.set("where/fieldFilter/op", "EQUAL");
    query1.set("where/fieldFilter/value/stringValue", kodeUnik2);
    query1.set("limit", 1);

    Firebase.Firestore.runQuery(&fbdo, FIREBASE_PROJECT_ID, "", "/", &query1);
    payload1 = fbdo.payload().c_str();
    deserializeJson(doc1, payload1);
    String nama2 = doc1[0]["document"]["fields"]["nama_lengkap"]["stringValue"];
    if (nama2 == "null") {
        nama2 = "pengguna 2";
    }
    pengguna2 = nama2;
    double Nasi2 = doc1[0]["document"]["fields"]["KNasi"]["doubleValue"];
    KNasi2 = Nasi2;

    // ambil data berat nasi hari ini
    query2.set("where/compositeFilter/op", "AND");

    query2.set("where/compositeFilter/filters/[0]/fieldFilter/field/fieldPath", "device_id");
    query2.set("where/compositeFilter/filters/[0]/fieldFilter/op", "EQUAL");
    query2.set("where/compositeFilter/filters/[0]/fieldFilter/value/stringValue", kodeUnik2);

    query2.set("where/compositeFilter/filters/[1]/fieldFilter/field/fieldPath", "timestamp");
    query2.set("where/compositeFilter/filters/[1]/fieldFilter/op", "GREATER_THAN_OR_EQUAL");
    query2.set("where/compositeFilter/filters/[1]/fieldFilter/value/timestampValue", String(year) + "-" + String(bulan) + "-" + String(tanggal) + "T17:00:00.000000Z");

    query2.set("orderBy/field/fieldPath", "timestamp");
    query2.set("orderBy/direction", "ASCENDING");

    Firebase.Firestore.runQuery(&fbdo, FIREBASE_PROJECT_ID, "", "/", &query2);
    payload2 = fbdo.payload().c_str();
    deserializeJson(doc2, payload2);

    KNasi_profile2 = 0;
    for (int i = 0; i < doc2.size(); i++) {
        double Berat = doc2[i]["document"]["fields"]["berat_nasi"]["doubleValue"];
        KNasi_profile2 += Berat;
    }
    Serial.printf("Total Documents: %d\n", doc2.size());

    // Ambil Pengguna 3 ============================================================
    query1.set("where/fieldFilter/field/fieldPath", "device_id");
    query1.set("where/fieldFilter/op", "EQUAL");
    query1.set("where/fieldFilter/value/stringValue", kodeUnik3);
    query1.set("limit", 1);

    Firebase.Firestore.runQuery(&fbdo, FIREBASE_PROJECT_ID, "", "/", &query1);
    payload1 = fbdo.payload().c_str();
    deserializeJson(doc1, payload1);
    String nama3 = doc1[0]["document"]["fields"]["nama_lengkap"]["stringValue"];
    if (nama3 == "null") {
        nama3 = "pengguna 3";
    }
    pengguna3 = nama3;
    double Nasi3 = doc1[0]["document"]["fields"]["KNasi"]["doubleValue"];
    KNasi3 = Nasi3;

    // ambil data berat nasi hari ini
    query2.set("where/compositeFilter/op", "AND");

    query2.set("where/compositeFilter/filters/[0]/fieldFilter/field/fieldPath", "device_id");
    query2.set("where/compositeFilter/filters/[0]/fieldFilter/op", "EQUAL");
    query2.set("where/compositeFilter/filters/[0]/fieldFilter/value/stringValue", kodeUnik3);

    query2.set("where/compositeFilter/filters/[1]/fieldFilter/field/fieldPath", "timestamp");
    query2.set("where/compositeFilter/filters/[1]/fieldFilter/op", "GREATER_THAN_OR_EQUAL");
    query2.set("where/compositeFilter/filters/[1]/fieldFilter/value/timestampValue", String(year) + "-" + String(bulan) + "-" + String(tanggal) + "T17:00:00.000000Z");

    query2.set("orderBy/field/fieldPath", "timestamp");
    query2.set("orderBy/direction", "ASCENDING");

    Firebase.Firestore.runQuery(&fbdo, FIREBASE_PROJECT_ID, "", "/", &query2);
    payload2 = fbdo.payload().c_str();
    deserializeJson(doc2, payload2);

    KNasi_profile3 = 0;
    for (int i = 0; i < doc2.size(); i++) {
        double Berat = doc2[i]["document"]["fields"]["berat_nasi"]["doubleValue"];
        KNasi_profile3 += Berat;
    }
    Serial.printf("Total Documents: %d\n", doc2.size());

    // Ambil Pengguna 4 =====================================================================
    query1.set("where/fieldFilter/field/fieldPath", "device_id");
    query1.set("where/fieldFilter/op", "EQUAL");
    query1.set("where/fieldFilter/value/stringValue", kodeUnik4);
    query1.set("limit", 1);

    Firebase.Firestore.runQuery(&fbdo, FIREBASE_PROJECT_ID, "", "/", &query1);
    payload1 = fbdo.payload().c_str();
    deserializeJson(doc1, payload1);
    String nama4 = doc1[0]["document"]["fields"]["nama_lengkap"]["stringValue"];
    if (nama4 == "null") {
        nama4 = "pengguna 4";
    }
    pengguna4 = nama4;
    double Nasi4 = doc1[0]["document"]["fields"]["KNasi"]["doubleValue"];
    KNasi4 = Nasi4;

    // ambil data berat nasi hari ini
    query2.set("where/compositeFilter/op", "AND");

    query2.set("where/compositeFilter/filters/[0]/fieldFilter/field/fieldPath", "device_id");
    query2.set("where/compositeFilter/filters/[0]/fieldFilter/op", "EQUAL");
    query2.set("where/compositeFilter/filters/[0]/fieldFilter/value/stringValue", kodeUnik4);

    query2.set("where/compositeFilter/filters/[1]/fieldFilter/field/fieldPath", "timestamp");
    query2.set("where/compositeFilter/filters/[1]/fieldFilter/op", "GREATER_THAN_OR_EQUAL");
    query2.set("where/compositeFilter/filters/[1]/fieldFilter/value/timestampValue", String(year) + "-" + String(bulan) + "-" + String(tanggal) + "T17:00:00.000000Z");

    query2.set("orderBy/field/fieldPath", "timestamp");
    query2.set("orderBy/direction", "ASCENDING");

    Firebase.Firestore.runQuery(&fbdo, FIREBASE_PROJECT_ID, "", "/", &query2);
    payload2 = fbdo.payload().c_str();
    deserializeJson(doc2, payload2);

    KNasi_profile4 = 0;
    for (int i = 0; i < doc2.size(); i++) {
        double Berat = doc2[i]["document"]["fields"]["berat_nasi"]["doubleValue"];
        KNasi_profile4 += Berat;
    }
    Serial.printf("Total Documents: %d\n", doc2.size());
}

void alarm_notifikasi() {
    waitUntilClick = 0;

    adc = analogRead(PIN_BAT_VOLT);
    volt = ((((float)adc / 4095.0) * 2.0 * 3.3 * (1100 / 1000)) - 3.2) * 100 / (4.2 - 3.2);
    if (volt > 100) {
        volt = 100;
    }

    alarm_screen.pushImage(0, 0, 320, 170, alarmScreen);
    alarm_text.fillSprite(0xE73C);
    alarm_text.drawCentreString("Simpan", 258, 35, 2);
    alarm_text.drawCentreString("Hapus", 258, 128, 2);
    alarm_text.drawCentreString(String(volt), 246, 5, 1);
    alarm_text.pushToSprite(&alarm_screen, 0, 0, 0xE73C);
    alarm_screen.pushSprite(0, 0);

    for (int i = 0; i < 3; i++) {
        digitalWrite(PIN_BUZZER, HIGH);
        delay(1000);
        digitalWrite(PIN_BUZZER, LOW);
        delay(1000);
    }

    do {
        adc = analogRead(PIN_BAT_VOLT);
        volt = ((((float)adc / 4095.0) * 2.0 * 3.3 * (1100 / 1000)) - 3.2) * 100 / (4.2 - 3.2);
        if (volt > 100) {
            volt = 100;
        }

        alarm_screen.pushImage(0, 0, 320, 170, alarmScreen);
        alarm_text.fillSprite(0xE73C);
        alarm_text.drawCentreString("Simpan", 258, 35, 2);
        alarm_text.drawCentreString("Hapus", 258, 128, 2);
        alarm_text.drawCentreString(String(volt), 246, 5, 1);
        alarm_text.pushToSprite(&alarm_screen, 0, 0, 0xE73C);
        alarm_screen.pushSprite(0, 0);

        if (digitalRead(PUSH_BUTTON_1) == LOW) {
            simpan = 1;
            waitUntilClick = 1;
            delay(500);
        } else if (digitalRead(PUSH_BUTTON_2) == LOW) {
            simpan = 0;
            waitUntilClick = 1;
            delay(500);
        }
    } while (waitUntilClick == 0);

    delay(500);
}

void setup() {
    Wire.begin(PIN_IIC_SDA, PIN_IIC_SCL);
    Wire.setClock(400000);  // 400kHz I2C clock. Comment this line if having compilation difficulties
    Serial.begin(115200);
    Serial.println("\nStarting CENTUNG");

    pinMode(PIN_POWER_ON, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);
    pinMode(PUSH_BUTTON_1, INPUT_PULLUP);
    pinMode(PUSH_BUTTON_2, INPUT_PULLUP);
    pinMode(PUSH_BUTTON_3, INPUT_PULLUP);
    pinMode(PUSH_BUTTON_4, INPUT_PULLUP);
    pinMode(PIN_BAT_VOLT, INPUT);
    digitalWrite(PIN_POWER_ON, HIGH);

    rtc_gpio_pullup_en((gpio_num_t)PIN_HX711_DATA);
    rtc_gpio_hold_en((gpio_num_t)PIN_HX711_DATA);

    rtc_gpio_pullup_en((gpio_num_t)PUSH_BUTTON_3);
    rtc_gpio_hold_en((gpio_num_t)PUSH_BUTTON_3);
    esp_sleep_enable_ext0_wakeup((gpio_num_t)PUSH_BUTTON_3, 0);

    EEPROM.begin(512);
    EEPROM.get(calVal_eepromAdress, calibrationValue);
    Serial.print("Calibration value: ");
    Serial.println(calibrationValue);

    EEPROM.begin(512);
    EEPROM.get(tareOffsetVal_eepromAdress, calibrationOffset);
    Serial.print("Zero offset value: ");
    Serial.println(calibrationOffset);

    ledcSetup(0, 10000, 8);
    ledcAttachPin(38, 0);
    ledcWrite(0, brightness);

    tft.init();
    tft.setRotation(3);
    tft.setSwapBytes(true);
    tft.fillScreen(TFT_WHITE);
    tft.setTextColor(TFT_BLACK, 0xE73C);
    tft.pushImage(0, 0, 320, 170, loading);

    randomSeed(analogRead(0));  // inisiasi generator random value

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

    LoadCell.begin();
    LoadCell.start(stabilizingtime, _tare);

    if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag()) {
        Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
        esp_deep_sleep_start();
    } else {
        LoadCell.setCalFactor(calibrationValue);  // set calibration value (float)
        Serial.println("Startup is complete");
    }

    // Setup SD Card
    if (!SD.begin(PIN_SD_CS)) {
        Serial.println("Inisialisasi SD Card gagal!");
    }
    Serial.println("Inisialisasi SD Card selesai");

    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize);

    attachInterrupt(digitalPinToInterrupt(PUSH_BUTTON_3), resetESP, FALLING);
    attachInterrupt(digitalPinToInterrupt(PIN_HX711_DATA), dataReadyISR, FALLING);

    // WIFI SETUP
    Serial.print("Connecting to Wi-Fi");
    wm.setConfigPortalTimeout(timeout);
    res = wm.autoConnect("CENTUNG");

    if (!res) {
        Serial.println("failed to connect and hit timeout");
        delay(3000);
        esp_deep_sleep_start();
    } else {
        screen_1();
        Serial.println("connected...yeey");
        Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);

        config.api_key = API_KEY;
        auth.user.email = USER_EMAIL;
        auth.user.password = USER_PASSWORD;
        config.token_status_callback = tokenStatusCallback;

        Firebase.reconnectNetwork(false);
        fbdo.setBSSLBufferSize(4096, 1024);
        fbdo.setResponseSize(2048);

        Serial.print("Pinging ip ");
        Serial.println(remote_ip);

        if (Ping.ping(remote_ip)) {
            Serial.println("Success!!");
            Firebase.begin(&config, &auth);
        } else {
            Serial.println("Error");

            adc = analogRead(PIN_BAT_VOLT);
            volt = ((((float)adc / 4095.0) * 2.0 * 3.3 * (1100 / 1000)) - 3.2) * 100 / (4.2 - 3.2);
            if (volt > 100) {
                volt = 100;
            }

            wm.setConfigPortalTimeout(timeout);
            pairing_mode.pushImage(0, 0, 320, 170, kotak);
            text_paring.fillSprite(0xE73C);
            text_paring.drawCentreString("Koneksi Terputus", 160, 55, 4);
            text_paring.drawCentreString("WiFi: CENTUNG", 160, 90, 4);
            text_paring.drawCentreString(String(volt), 246, 5, 1);
            text_paring.pushToSprite(&pairing_mode, 0, 0, 0xE73C);
            pairing_mode.pushSprite(0, 0);

            if (!wm.startConfigPortal("CENTUNG")) {
                Serial.println("failed to connect and hit timeout");
                delay(3000);
                esp_deep_sleep_start();
            } else {
                Serial.println("connected...yeey :)");
                pairing_mode.pushImage(0, 0, 320, 170, kotak);
                text_paring.fillSprite(0xE73C);
                text_paring.drawCentreString("Koneksi Terhubung ", 160, 55, 4);
                text_paring.drawCentreString("WiFi: CENTUNG", 160, 90, 4);
                text_paring.pushToSprite(&pairing_mode, 0, 0, 0xE73C);
                pairing_mode.pushSprite(0, 0);
                delay(3000);
                esp_deep_sleep_start();
            }
        }
    }

    // Initialize MPU6050
    MPU6050SelfTest(SelfTest);
    MPU6050_Init();
    sampling_timer = micros();

    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);

    detachInterrupt(digitalPinToInterrupt(PUSH_BUTTON_3));

    cache1 = 0;
    cache2 = 0;
    cache3 = 0;
    cache4 = 0;

    button1.attachClick([]() {
        if (cek_berat == 0) {
            devId = kodeUnik2;
            ulang = 1;
            cek_berat = 1;
        } else {
            state = 1;
        }
    });

    button1.attachLongPressStart([]() {
        // mode cek berat nasi saat ini
        int keluar_kalender = 0;
        do {
            button4.tick();
            max_nasi = 0;
            getLocalTime();

            adc = analogRead(PIN_BAT_VOLT);
            volt = ((((float)adc / 4095.0) * 2.0 * 3.3 * (1100 / 1000)) - 3.2) * 100 / (4.2 - 3.2);
            if (volt > 100) {
                volt = 100;
            }

            if (devId == kodeUnik1) {
                max_nasi = KNasi1;
                KNasiSekarang = KNasi_profile1;
            } else if (devId == kodeUnik2) {
                max_nasi = KNasi2;
                KNasiSekarang = KNasi_profile2;
            } else if (devId == kodeUnik3) {
                max_nasi = KNasi3;
                KNasiSekarang = KNasi_profile3;
            } else if (devId == kodeUnik4) {
                max_nasi = KNasi4;
                KNasiSekarang = KNasi_profile4;
            }

            cek_data.fillSprite(TFT_BLACK);

            cek_data.drawRoundRect(2, 2, 118, 166, 5, TFT_WHITE);  /// border
            cek_data.fillRoundRect(fromLeft, 61, 100, 24, 5, color2);
            cek_data.fillRoundRect(fromLeft, 110, 100, 24, 5, color2);

            cek_data.setTextColor(TFT_WHITE, color2);
            cek_data.drawCentreString(String(KNasiSekarang) + String(" gram"), 62, 64, 2);
            cek_data.drawCentreString(String(max_nasi) + " gram", 62, 113, 2);

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

            cek_data.setTextColor(gray, TFT_BLACK);
            cek_data.setTextFont(0);
            cek_data.drawString("BATTERY:", 250, 34);

            cek_data.drawString(String(volt) + "%", 250, 46);
            cek_data.drawRoundRect(304, 30, 12, 136, 2, TFT_SILVER);

            seg = brightness / 24;
            for (int i = 0; i < seg; i++)
                cek_data.fillRect(308, 150 - (i * 13), 4, 11, 0x35F9);
            cek_data.drawLine(cax - 10, cay - 10, cax + 152, cay - 10, gray);

            cek_data.pushImage(298, 0, 26, 26, bright);
            cek_data.pushSprite(0, 0);

            if (digitalRead(PUSH_BUTTON_1) == LOW) {
                if (brightness < 240) {
                    brightness++;
                    ledcSetup(0, 10000, 8);
                    ledcAttachPin(38, 0);
                    ledcWrite(0, brightness);
                }
            }

            if (digitalRead(PUSH_BUTTON_2) == LOW) {
                if (brightness > 24) {
                    brightness--;
                    ledcSetup(0, 10000, 8);
                    ledcAttachPin(38, 0);
                    ledcWrite(0, brightness);
                }
            }

            if (digitalRead(PUSH_BUTTON_3) == LOW) {
                keluar_kalender = 1;
                delay(500);
            }
        } while (keluar_kalender == 0);
    });

    button2.attachClick([]() {
        if (cek_berat == 0) {
            devId = kodeUnik4;
            cek_berat = 1;
            ulang = 1;
        }
    });

    button2.attachLongPressStart([]() {
        // calibration mode
        int keluar = 0;
        int mulai_calib = 0;

        Serial.println("***");
        Serial.println("Start calibration");
        Serial.println("Pastikan Centung dalam kondisi datar");

        do {
            adc = analogRead(PIN_BAT_VOLT);
            volt = ((((float)adc / 4095.0) * 2.0 * 3.3 * (1100 / 1000)) - 3.2) * 100 / (4.2 - 3.2);
            if (volt > 100) {
                volt = 100;
            }

            calib_screen.pushImage(0, 0, 320, 170, kotak);
            calib_text.fillSprite(0xE73C);
            calib_text.drawCentreString("Mode Kalibrasi", 160, 55, 4);
            calib_text.drawCentreString("Lepaskan beban!", 160, 90, 4);
            calib_text.drawCentreString("Kembali", 55, 20, 2);
            calib_text.drawCentreString("Reset", 55, 135, 2);
            calib_text.drawCentreString("Mulai", 265, 20, 2);
            calib_text.drawCentreString(String(volt), 246, 5, 1);
            calib_text.pushToSprite(&calib_screen, 0, 0, 0xE73C);
            calib_screen.pushSprite(0, 0);

            if (digitalRead(PUSH_BUTTON_1) == LOW) {
                detachInterrupt(digitalPinToInterrupt(PIN_HX711_DATA));

                do {
                    adc = analogRead(PIN_BAT_VOLT);
                    volt = ((((float)adc / 4095.0) * 2.0 * 3.3 * (1100 / 1000)) - 3.2) * 100 / (4.2 - 3.2);
                    if (volt > 100) {
                        volt = 100;
                    }

                    mpu6050_GetData();
                    mpu6050_updateQuaternion();

                    Now = micros();
                    sampleFreq = (1000000.0f / (Now - lastUpdate));  // set integration time by time elapsed since last filter update
                    lastUpdate = Now;

                    MahonyAHRSupdateIMU(gxrs, gyrs, gzrs, axg, ayg, azg);
                    mpu6050_getRollPitchYaw();

                    Serial.print(roll);
                    Serial.print("\t");
                    Serial.print(pitch);
                    Serial.print("\t");
                    Serial.println(yaw);

                    position_screen.pushImage(0, 0, 320, 170, datar);
                    position_text.fillSprite(0xE73C);
                    position_text.drawCentreString(String(volt), 246, 5, 1);
                    position_text.drawCentreString(String(roll), 90, 95, 4);
                    position_text.drawCentreString(String(pitch), 225, 95, 4);
                    position_text.pushToSprite(&position_screen, 0, 0, 0xE73C);
                    position_screen.pushSprite(0, 0);

                    if ((roll > -1 && roll < 1) && (pitch > -1 && pitch < 1)) {
                        mulai_calib = 1;
                        LoadCell.update();
                        LoadCell.tareNoDelay();

                        if (LoadCell.getTareStatus() == true) {
                            Serial.println("Tare complete");
                        }

                        calibrationOffset = 8434937.00;
                        EEPROM.begin(512);
                        EEPROM.put(tareOffsetVal_eepromAdress, calibrationOffset);
                        EEPROM.commit();

                        LoadCell.setTareOffset(calibrationOffset);
                        Serial.print("Tare offset value:");
                        Serial.print(calibrationOffset);
                        Serial.print(", saved to EEprom adr:");
                        Serial.println(tareOffsetVal_eepromAdress);

                        LoadCell.setCalFactor(1.0);
                    }
                } while (mulai_calib == 0);

                Serial.println("Letakkan beban 20 gram");
                mulai_calib = 0;

                do {
                    adc = analogRead(PIN_BAT_VOLT);
                    volt = ((((float)adc / 4095.0) * 2.0 * 3.3 * (1100 / 1000)) - 3.2) * 100 / (4.2 - 3.2);
                    if (volt > 100) {
                        volt = 100;
                    }

                    calib_screen.pushImage(0, 0, 320, 170, kotak);
                    calib_text.fillSprite(0xE73C);
                    calib_text.drawCentreString("Letakkan beban", 160, 55, 4);
                    calib_text.drawCentreString("20 gram", 160, 90, 4);
                    calib_text.drawCentreString("Mulai", 265, 20, 2);
                    calib_text.drawCentreString(String(volt), 246, 5, 1);
                    calib_text.pushToSprite(&calib_screen, 0, 0, 0xE73C);
                    calib_screen.pushSprite(0, 0);

                    if (digitalRead(PUSH_BUTTON_1) == LOW) {
                        mulai_calib = 1;
                        Serial.println("Menentukan besar nilai kalibrasi");
                    }
                } while (mulai_calib == 0);

                LoadCell.update();
                float known_mass = 20.0;
                delay(3200);

                LoadCell.refreshDataSet();
                calibrationValue = LoadCell.getNewCalibration(known_mass);

                Serial.print("New calibration value has been set to: ");
                Serial.println(calibrationValue);
                mulai_calib = 0;

                do {
                    adc = analogRead(PIN_BAT_VOLT);
                    volt = ((((float)adc / 4095.0) * 2.0 * 3.3 * (1100 / 1000)) - 3.2) * 100 / (4.2 - 3.2);
                    if (volt > 100) {
                        volt = 100;
                    }

                    calib_screen.pushImage(0, 0, 320, 170, kotak);
                    calib_text.fillSprite(0xE73C);
                    calib_text.drawCentreString("Calibration value:", 160, 55, 4);
                    calib_text.drawCentreString(String(calibrationValue), 160, 90, 4);
                    calib_text.drawCentreString("Selesai", 265, 20, 2);
                    calib_text.drawCentreString(String(volt), 246, 5, 1);
                    calib_text.pushToSprite(&calib_screen, 0, 0, 0xE73C);
                    calib_screen.pushSprite(0, 0);

                    if (digitalRead(PUSH_BUTTON_1) == LOW) {
                        mulai_calib = 1;
                        keluar = 1;
                        state = 0;
                        Serial.println("Menyimpan nilai ke EEPROM");
                        delay(500);
                    }
                } while (mulai_calib == 0);

                EEPROM.begin(512);
                EEPROM.put(calVal_eepromAdress, calibrationValue);
                EEPROM.commit();

                Serial.print("Value: ");
                Serial.println(calibrationValue);
                Serial.print("saved to EEPROM address: ");
                Serial.println(calVal_eepromAdress);
                LoadCell.setCalFactor(calibrationValue);

                Serial.println("End calibration");
                Serial.println("***");
                attachInterrupt(digitalPinToInterrupt(PIN_HX711_DATA), dataReadyISR, FALLING);
            }

            else if (digitalRead(PUSH_BUTTON_3) == LOW) {
                keluar = 1;
                delay(500);
            }

            else if (digitalRead(PUSH_BUTTON_4) == LOW) {
                Serial.println("Setelan pabrik");

                calibrationValue = 1306.5337;
                EEPROM.begin(512);
                EEPROM.put(calVal_eepromAdress, calibrationValue);
                EEPROM.commit();

                calibrationOffset = 8434937.00;
                EEPROM.begin(512);
                EEPROM.put(tareOffsetVal_eepromAdress, calibrationOffset);
                EEPROM.commit();

                Serial.print("Calibration value: ");
                Serial.println(calibrationValue);
                Serial.print("Zero offset value: ");
                Serial.println(calibrationOffset);

                LoadCell.setTareOffset(calibrationOffset);
                LoadCell.setCalFactor(calibrationValue);

                keluar = 1;
                delay(500);
            }
        } while (keluar == 0);
    });

    button3.attachClick([]() {
        if (cek_berat == 0) {
            devId = kodeUnik1;
            cek_berat = 1;
            ulang = 1;
        } else {
            state = 3;
        }
    });

    button3.attachLongPressStart([]() {
        pinMode(PIN_POWER_ON, OUTPUT);
        pinMode(PIN_LCD_BL, OUTPUT);
        digitalWrite(PIN_POWER_ON, LOW);
        digitalWrite(PIN_LCD_BL, LOW);
        delay(1000);
        esp_deep_sleep_start();
    });

    button4.attachClick([]() {
        if (cek_berat == 0) {
            devId = kodeUnik3;
            cek_berat = 1;
            ulang = 1;
        }
    });

    button4.attachDoubleClick([]() {
        int delete_data = 0;
        do {
            adc = analogRead(PIN_BAT_VOLT);
            volt = ((((float)adc / 4095.0) * 2.0 * 3.3 * (1100 / 1000)) - 3.2) * 100 / (4.2 - 3.2);
            if (volt > 100) {
                volt = 100;
            }

            position_screen.pushImage(0, 0, 320, 170, kotak);
            position_text.fillSprite(0xE73C);
            position_text.drawCentreString("Hapus Dokumen", 160, 55, 4);
            position_text.drawCentreString("Device Id: " + devId, 160, 90, 4);
            position_text.drawCentreString("Kembali", 55, 20, 2);
            position_text.drawCentreString("Hari Ini", 265, 20, 2);
            position_text.drawCentreString("Terakhir", 265, 135, 2);
            position_text.drawCentreString(String(volt), 246, 5, 1);
            position_text.pushToSprite(&position_screen, 0, 0, 0xE73C);
            position_screen.pushSprite(0, 0);

            if (digitalRead(PUSH_BUTTON_1) == LOW) {
                delete_data = 1;

                FirebaseJson deleteDoc;

                deleteDoc.set("select/fields/[0]/fieldPath", "timestamp");
                deleteDoc.set("select/fields/[1]/fieldPath", "__name__");
                deleteDoc.set("select/fields/[2]/fieldPath", "device_id");

                deleteDoc.set("from/collectionId", "log_centung");
                deleteDoc.set("from/allDescendants", false);

                deleteDoc.set("where/compositeFilter/op", "AND");

                deleteDoc.set("where/compositeFilter/filters/[0]/fieldFilter/field/fieldPath", "device_id");
                deleteDoc.set("where/compositeFilter/filters/[0]/fieldFilter/op", "EQUAL");
                deleteDoc.set("where/compositeFilter/filters/[0]/fieldFilter/value/stringValue", devId);

                deleteDoc.set("where/compositeFilter/filters/[1]/fieldFilter/field/fieldPath", "timestamp");
                deleteDoc.set("where/compositeFilter/filters/[1]/fieldFilter/op", "GREATER_THAN_OR_EQUAL");
                deleteDoc.set("where/compositeFilter/filters/[1]/fieldFilter/value/timestampValue", String(year) + "-" + String(bulan) + "-" + String(tanggal) + "T17:00:00.000000Z");

                deleteDoc.set("orderBy/field/fieldPath", "timestamp");
                deleteDoc.set("orderBy/direction", "ASCENDING");

                Firebase.Firestore.runQuery(&fbdo, FIREBASE_PROJECT_ID, "", "/", &deleteDoc);
                Serial.printf("ok\n%s\n\n", fbdo.payload().c_str());

                String payload3 = fbdo.payload().c_str();
                DynamicJsonDocument doc3(50000);
                deserializeJson(doc3, payload3);

                for (int i = 0; i < doc3.size(); i++) {
                    String documentName = doc3[i]["document"]["name"].as<String>();
                    String documentId = extractDocumentId(documentName);
                    Serial.printf("Document ID: %s\n", documentId.c_str());

                    // Delete the document
                    deleteDocument(documentId);
                }
                Serial.printf("Total Documents: %d\n", doc3.size());

                if (devId == kodeUnik1) {
                    KNasi_profile1 = 0;
                } else if (devId == kodeUnik2) {
                    KNasi_profile2 = 0;
                } else if (devId == kodeUnik3) {
                    KNasi_profile3 = 0;
                } else if (devId == kodeUnik4) {
                    KNasi_profile4 = 0;
                }

                adc = analogRead(PIN_BAT_VOLT);
                volt = ((((float)adc / 4095.0) * 2.0 * 3.3 * (1100 / 1000)) - 3.2) * 100 / (4.2 - 3.2);
                if (volt > 100) {
                    volt = 100;
                }

                position_screen.pushImage(0, 0, 320, 170, kotak);
                position_text.fillSprite(0xE73C);
                position_text.drawCentreString("Dokumen hari ini", 160, 55, 4);
                position_text.drawCentreString("Telah dihapus", 160, 90, 4);
                position_text.drawCentreString(String(volt), 246, 5, 1);
                position_text.pushToSprite(&position_screen, 0, 0, 0xE73C);
                position_screen.pushSprite(0, 0);
                delay(2000);
            }

            else if (digitalRead(PUSH_BUTTON_2) == LOW) {
                delete_data = 1;

                FirebaseJson deleteDoc;

                deleteDoc.set("select/fields/[0]/fieldPath", "timestamp");
                deleteDoc.set("select/fields/[1]/fieldPath", "__name__");
                deleteDoc.set("select/fields/[2]/fieldPath", "device_id");

                deleteDoc.set("from/collectionId", "log_centung");
                deleteDoc.set("from/allDescendants", false);

                deleteDoc.set("where/compositeFilter/op", "AND");

                deleteDoc.set("where/compositeFilter/filters/[0]/fieldFilter/field/fieldPath", "device_id");
                deleteDoc.set("where/compositeFilter/filters/[0]/fieldFilter/op", "EQUAL");
                deleteDoc.set("where/compositeFilter/filters/[0]/fieldFilter/value/stringValue", devId);

                deleteDoc.set("where/compositeFilter/filters/[1]/fieldFilter/field/fieldPath", "timestamp");
                deleteDoc.set("where/compositeFilter/filters/[1]/fieldFilter/op", "GREATER_THAN_OR_EQUAL");
                deleteDoc.set("where/compositeFilter/filters/[1]/fieldFilter/value/timestampValue", String(year) + "-" + String(bulan) + "-" + String(tanggal) + "T17:00:00.000000Z");

                deleteDoc.set("orderBy/field/fieldPath", "timestamp");
                deleteDoc.set("orderBy/direction", "DESCENDING");

                deleteDoc.set("limit", 1);

                Firebase.Firestore.runQuery(&fbdo, FIREBASE_PROJECT_ID, "", "/", &deleteDoc);
                Serial.printf("ok\n%s\n\n", fbdo.payload().c_str());

                String payload3 = fbdo.payload().c_str();
                DynamicJsonDocument doc3(2048);
                deserializeJson(doc3, payload3);

                String documentName = doc3[0]["document"]["name"].as<String>();
                String documentId = extractDocumentId(documentName);
                Serial.printf("Document ID: %s\n", documentId.c_str());
                deleteDocument(documentId);

                if (devId == kodeUnik1) {
                    KNasi_profile1 = KNasi_profile1 - berat;
                } else if (devId == kodeUnik2) {
                    KNasi_profile2 = KNasi_profile2 - berat;
                } else if (devId == kodeUnik3) {
                    KNasi_profile3 = KNasi_profile3 - berat;
                } else if (devId == kodeUnik4) {
                    KNasi_profile4 = KNasi_profile4 - berat;
                }

                adc = analogRead(PIN_BAT_VOLT);
                volt = ((((float)adc / 4095.0) * 2.0 * 3.3 * (1100 / 1000)) - 3.2) * 100 / (4.2 - 3.2);
                if (volt > 100) {
                    volt = 100;
                }

                position_screen.pushImage(0, 0, 320, 170, kotak);
                position_text.fillSprite(0xE73C);
                position_text.drawCentreString("Dokumen terakhir", 160, 55, 4);
                position_text.drawCentreString("Telah dihapus", 160, 90, 4);
                position_text.drawCentreString(String(volt), 246, 5, 1);
                position_text.pushToSprite(&position_screen, 0, 0, 0xE73C);
                position_screen.pushSprite(0, 0);
                delay(2000);
            }

            else if (digitalRead(PUSH_BUTTON_3) == LOW) {
                delete_data = 1;
                delay(1000);
            }

        } while (delete_data == 0);
    });

    button4.attachLongPressStart([]() {
        wm.setConfigPortalTimeout(timeout);
        int selesai_wifi = 0;

        do {
            adc = analogRead(PIN_BAT_VOLT);
            volt = ((((float)adc / 4095.0) * 2.0 * 3.3 * (1100 / 1000)) - 3.2) * 100 / (4.2 - 3.2);
            if (volt > 100) {
                volt = 100;
            }

            pairing_mode.pushImage(0, 0, 320, 170, kotak);
            text_paring.fillSprite(0xE73C);
            text_paring.drawCentreString("Pairing Mode", 160, 55, 4);
            text_paring.drawCentreString("Kembali", 55, 20, 2);
            text_paring.drawCentreString("Mulai", 265, 20, 2);
            text_paring.drawCentreString(String(volt), 246, 5, 1);
            text_paring.pushToSprite(&pairing_mode, 0, 0, 0xE73C);
            pairing_mode.pushSprite(0, 0);

            if (digitalRead(PUSH_BUTTON_1) == LOW) {
                adc = analogRead(PIN_BAT_VOLT);
                volt = ((((float)adc / 4095.0) * 2.0 * 3.3 * (1100 / 1000)) - 3.2) * 100 / (4.2 - 3.2);
                if (volt > 100) {
                    volt = 100;
                }

                pairing_mode.pushImage(0, 0, 320, 170, kotak);
                text_paring.fillSprite(0xE73C);
                text_paring.drawCentreString("Pairing Mode", 160, 55, 4);
                text_paring.drawCentreString("WiFi: CENTUNG", 160, 90, 4);
                text_paring.drawCentreString(String(volt), 246, 5, 1);
                text_paring.pushToSprite(&pairing_mode, 0, 0, 0xE73C);
                pairing_mode.pushSprite(0, 0);

                if (!wm.startConfigPortal("CENTUNG")) {
                    Serial.println("failed to connect and hit timeout");
                    delay(3000);
                    esp_deep_sleep_start();
                } else {
                    Serial.println("connected...yeey :)");
                    adc = analogRead(PIN_BAT_VOLT);
                    volt = ((((float)adc / 4095.0) * 2.0 * 3.3 * (1100 / 1000)) - 3.2) * 100 / (4.2 - 3.2);
                    if (volt > 100) {
                        volt = 100;
                    }

                    pairing_mode.pushImage(0, 0, 320, 170, kotak);
                    text_paring.fillSprite(0xE73C);
                    text_paring.drawCentreString("Pairing Mode", 160, 55, 4);
                    text_paring.drawCentreString("Terhubung!", 160, 90, 4);
                    text_paring.drawCentreString(String(volt), 246, 5, 1);
                    text_paring.pushToSprite(&pairing_mode, 0, 0, 0xE73C);
                    pairing_mode.pushSprite(0, 0);
                    delay(3000);
                    esp_deep_sleep_start();
                }
            }

            else if (digitalRead(PUSH_BUTTON_3) == LOW) {
                selesai_wifi = 1;
                delay(500);
            }
        } while (selesai_wifi == 0);
    });
}

//================================================================
void loop() {
    getLocalTime();
    cek_berat = 0;
    state = 0;
    ulang = 0;

    if (done_query == 0) {
        Serial.print("Query a Firestore database... ");
        get_query();
        done_query = 1;
    }

    do {
        adc = analogRead(PIN_BAT_VOLT);
        volt = ((((float)adc / 4095.0) * 2.0 * 3.3 * (1100 / 1000)) - 3.2) * 100 / (4.2 - 3.2);

        if (volt > 100) {
            volt = 100;
        }

        centong = 0;

        select_profile.pushImage(0, 0, 320, 170, profile);
        text_profile.fillSprite(PROFILE_BACKGROUND);
        text_profile.drawCentreString(String(volt), 246, 5, 1);
        text_profile.drawCentreString(pengguna1, 90, 27, 2);
        text_profile.drawCentreString(pengguna2, 250, 27, 2);
        text_profile.drawCentreString(pengguna3, 90, 125, 2);
        text_profile.drawCentreString(pengguna4, 250, 125, 2);

        text_profile.pushToSprite(&select_profile, 0, 0, PROFILE_BACKGROUND);
        select_profile.pushSprite(0, 0);

        button1.tick();
        button2.tick();
        button3.tick();
        button4.tick();
    } while (ulang == 0);

    do {
        adc = analogRead(PIN_BAT_VOLT);
        volt = ((((float)adc / 4095.0) * 2.0 * 3.3 * (1100 / 1000)) - 3.2) * 100 / (4.2 - 3.2);

        if (volt > 100) {
            volt = 100;
        }

        ulang = 0;

        mpu6050_GetData();
        mpu6050_updateQuaternion();

        Now = micros();
        sampleFreq = (1000000.0f / (Now - lastUpdate));  // set integration time by time elapsed since last filter update
        lastUpdate = Now;

        MahonyAHRSupdateIMU(gxrs, gyrs, gzrs, axg, ayg, azg);
        mpu6050_getRollPitchYaw();

        Serial.print(roll);
        Serial.print("\t");
        Serial.print(pitch);
        Serial.print("\t");
        Serial.println(yaw);

        if ((roll > -3 && roll < 3) && (pitch > -3 && pitch < 3)) {
            ulang = 1;
            LoadCell.start(stabilizingtime, _tare);
            // LoadCell.tareNoDelay();
            if (LoadCell.getTareStatus() == true) {
                Serial.println("Tare complete");
            }
        } else {
            ulang = 0;
            position_screen.pushImage(0, 0, 320, 170, datar);
            position_text.fillSprite(0xE73C);
            position_text.drawCentreString(String(volt), 246, 5, 1);
            position_text.drawCentreString(String(roll), 90, 95, 4);
            position_text.drawCentreString(String(pitch), 225, 95, 4);
            position_text.pushToSprite(&position_screen, 0, 0, 0xE73C);
            position_screen.pushSprite(0, 0);
        }
    } while (ulang == 0);

    do {
        ulang = 0;
        cek_berat = 1;
        int mulai = 0;
        String doc_id = generateRandomString();

        button1.tick();
        button2.tick();
        button3.tick();
        button4.tick();

        switch (state) {
            default:
                adc = analogRead(PIN_BAT_VOLT);
                volt = ((((float)adc / 4095.0) * 2.0 * 3.3 * (1100 / 1000)) - 3.2) * 100 / (4.2 - 3.2);

                if (volt > 100) {
                    volt = 100;
                }

                tunggu_nimbang.pushImage(0, 0, 320, 170, timbangan);
                text_timbangan.fillSprite(0xE73C);
                text_timbangan.setTextColor(TFT_BLACK, 0xE73C);
                text_timbangan.drawCentreString(devId, 15, 5, 2);
                text_timbangan.drawCentreString("Ambil nasi sekarang", 160, 5, 2);
                text_timbangan.drawCentreString(String(volt), 246, 5, 1);
                text_timbangan.pushToSprite(&tunggu_nimbang, 0, 0, 0xE73C);
                tunggu_nimbang.pushSprite(0, 0);
                break;

            case 1:
                do {
                    mpu6050_GetData();
                    mpu6050_updateQuaternion();

                    Now = micros();
                    sampleFreq = (1000000.0f / (Now - lastUpdate));  // set integration time by time elapsed since last filter update
                    lastUpdate = Now;

                    MahonyAHRSupdateIMU(gxrs, gyrs, gzrs, axg, ayg, azg);
                    mpu6050_getRollPitchYaw();

                    Serial.print(roll);
                    Serial.print("\t");
                    Serial.print(pitch);
                    Serial.print("\t");
                    Serial.println(yaw);

                    if ((roll > -3 && roll < 3) && (pitch > -3 && pitch < 3)) {
                        mulai = 1;
                    } else {
                        mulai = 0;
                        position_screen.pushImage(0, 0, 320, 170, datar);
                        position_text.fillSprite(0xE73C);
                        position_text.drawCentreString(String(volt), 246, 5, 1);
                        position_text.drawCentreString(String(roll), 90, 95, 4);
                        position_text.drawCentreString(String(pitch), 225, 95, 4);
                        position_text.pushToSprite(&position_screen, 0, 0, 0xE73C);
                        position_screen.pushSprite(0, 0);
                    }
                } while (mulai == 0);

                if (newDataReady && mulai == 1) {
                    delay(5000);
                    berat = LoadCell.getData();
                    berat = round(berat * 100) / 100;
                    if (berat < 0) {
                        berat = 0;
                    }
                    /*
                              appendFileCalib(SD, "/Calibration.txt", calibrationValue, berat);
                              readFile(SD, "/Calibration.txt");
                              Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));

                              double error = (abs(berat - 20.0) * 100) / 20.0;
                              error = round(error * 100) / 100;
                              String persentase = String(error);
                              appendFileaccurate(SD, "/Akurasi.txt", berat, persentase + "%");
                              readFile(SD, "/Akurasi.txt");
                              Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
                    */
                    centong++;
                    simpan = 1;
                    newDataReady = 0;
                    Serial.print("Load_cell output val: ");
                    Serial.println(berat);

                    // PROFILE 1
                    if (devId == kodeUnik1) {
                        KNasi_profile1 += berat;
                        cache1 += berat;
                        if (cache1 >= (KNasi1 * 0.33)) {
                            alarm_notifikasi();
                        }
                    }

                    // PROFILE 2
                    else if (devId == kodeUnik2) {
                        KNasi_profile2 += berat;
                        cache2 += berat;
                        if (cache2 >= (KNasi2 * 0.33)) {
                            alarm_notifikasi();
                        }
                    }

                    // PROFILE 3
                    else if (devId == kodeUnik3) {
                        KNasi_profile3 += berat;
                        cache3 += berat;
                        if (cache3 >= (KNasi3 * 0.33)) {
                            alarm_notifikasi();
                        }
                    }

                    // PROFILE 4
                    else if (devId == kodeUnik4) {
                        KNasi_profile4 += berat;
                        cache4 += berat;
                        if (cache4 >= (KNasi4 * 0.33)) {
                            alarm_notifikasi();
                        }
                    }

                    int end_angle = (berat * 180 / 300) + 90;
                    adc = analogRead(PIN_BAT_VOLT);
                    volt = ((((float)adc / 4095.0) * 2.0 * 3.3 * (1100 / 1000)) - 3.2) * 100 / (4.2 - 3.2);

                    if (volt > 100) {
                        volt = 100;
                    }

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
                    delay(3000);

                    if (Firebase.ready() && simpan == 1) {
                        Serial.print("Commit a document (set server value, update document)... ");

                        FirebaseJson content;
                        String documentPath = "log_centung/" + doc_id;
                        content.set("fields/berat_nasi/doubleValue", berat);
                        content.set("fields/device_id/stringValue", devId);
                        Serial.print("Create a document... ");
                        if (Firebase.Firestore.createDocument(&fbdo, FIREBASE_PROJECT_ID, "" /* databaseId can be (default) or empty */, documentPath.c_str(), content.raw()))
                            Serial.printf("ok\n%s\n\n", fbdo.payload().c_str());
                        else
                            Serial.println(fbdo.errorReason());

                        std::vector<struct firebase_firestore_document_write_t> writes;
                        struct firebase_firestore_document_write_t transform_write;
                        transform_write.type = firebase_firestore_document_write_type_transform;
                        transform_write.document_transform.transform_document_path = documentPath;
                        struct firebase_firestore_document_write_field_transforms_t field_transforms;
                        field_transforms.fieldPath = "timestamp";
                        field_transforms.transform_type = firebase_firestore_transform_type_set_to_server_value;
                        field_transforms.transform_content = "REQUEST_TIME";
                        transform_write.document_transform.field_transforms.push_back(field_transforms);
                        writes.push_back(transform_write);

                        Serial.print("Create a document... ");

                        if (Firebase.Firestore.commitDocument(&fbdo, FIREBASE_PROJECT_ID, "" /* databaseId can be (default) or empty */, writes /* dynamic array of firebase_firestore_document_write_t */, "" /* transaction */))
                            Serial.printf("ok\n%s\n\n", fbdo.payload().c_str());
                        else
                            Serial.println(fbdo.errorReason());
                    }
                }

                state = 0;
                break;

            case 3:
                ulang = 1;
                cek_berat = 0;
                state = 0;
                break;
        }
    } while (ulang == 0);
}
